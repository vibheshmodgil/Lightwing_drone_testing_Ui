/* ============================================================================
   LiteWing Bench  —  hardware test rig for LiteWing V1.2 (ESP32-S3)
   ----------------------------------------------------------------------------
   Replaces flight firmware with a self-hosted test dashboard:
     - I2C0 / I2C1 bus scan + sensor identification (MPU6050, VL53L1X,
       MS5611, HMC5883, PMW3901 over SPI)
     - Live IMU: raw accel/gyro, gyro-bias calibration, complementary attitude
     - Per-motor PWM control with master arm + comms watchdog
     - Health sweep: runs each motor alone, logs battery sag -> finds dead
       motors / blown IRLML6344 MOSFETs
     - All GPIO assignments editable from the UI, persisted in NVS

   USE:  Board = "ESP32S3 Dev Module", USB CDC On Boot = DISABLED.
         (V1.2 bridges USB to UART0 via a CH340K - VID 1A86 PID 7522 -
          so Serial must stay on UART0, not the native USB peripheral.)
         Power on, join WiFi "LiteWing-Bench" (pass: litewing), open
         http://192.168.4.1

   WARNING: motors spin. Remove propellers before arming. Flashing this
   erases the stock Crazyflie/ESP-Drone firmware; restore it with the
   CircuitDigest web flasher.

   PIN MAP: I2C0/I2C1/SPI/buzzer pins are confirmed from the LiteWing wiki.
   Motor pins are confirmed against LiteWing-Arduino/.../motors.h; the
   battery ADC pin and divider are traced from the ADC_BAT net in
   hardware/LiteWingV2.5C/LiteWingV2.5C.kicad_sch (both in the
   Circuit-Digest/LiteWing repo). The Pins panel can still override them.

   NOTE: GPIO 7/8/9 are the RGB status LEDs and GPIO 12 is MPU_INT — an
   earlier revision of this file used those as motor pins by mistake. If
   you ever pressed "Save pins", those bad values are still in NVS and will
   override the defaults below; fix them in the Pins panel or reflash with
   "Erase All Flash Before Sketch Upload" enabled.
   ========================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>

// ---------------------------------------------------------------- config ---
static const char* AP_SSID = "LiteWing-Bench";
static const char* AP_PASS = "litewing";

// Confirmed from LiteWing hardware docs
#define I2C0_SDA   11
#define I2C0_SCL   10
#define I2C1_SDA   40
#define I2C1_SCL   41
#define SPI_MISO   37
#define SPI_SCK    36
#define SPI_MOSI   35
#define SPI_CS     42
#define BUZZER     39

#define PWM_FREQ   8000      // brushed coreless; matches Betaflight setup
#define PWM_BITS   10
#define PWM_MAX    1023
#define WD_MS      1500      // disarm if UI goes quiet this long

struct Cfg {
  int m[4] = { 5, 6, 3, 4 };    // MOT_1..MOT_4, from LiteWingV2.5C.kicad_sch
  int vbatPin = 2;              // ADC_BAT (ADC1_CH1)
  float vbatDiv = 2.0f;         // R26/R27 = 100K/100K -> 1:2
} cfg;

// Roll/pitch stabiliser gains. Index 0 = roll, 1 = pitch.
struct PidCfg {
  float kp[2]   = { 0.60f, 0.60f };
  float ki[2]   = { 0.00f, 0.00f };
  float kd[2]   = { 0.03f, 0.03f };
  int   base    = 0;          // base throttle %, all four motors
  int   mode    = 0;          // 0 off, 1 roll only, 2 pitch only, 3 both
  float iLim    = 15.0f;      // integral clamp, output %
  float oLim    = 30.0f;      // per-axis authority clamp, output %
  int   inv[2]  = { 0, 0 };   // flip correction sign per axis
  // --- flight / hover ---
  int   flight  = 0;          // 0 bench (gimbal), 1 flight (free hover)
  float yawKp   = 0.25f;      // yaw RATE damper. resists spin, does not hold heading
  float yawLim  = 12.0f;      // yaw authority clamp, output %
  float tiltCut = 60.0f;      // disarm above this |angle|, degrees
  int   thrMin  = 12;         // below this throttle, hold integrators at zero
  float slew    = 45.0f;      // throttle ramp limit, % per second
} pid;

Preferences prefs;
WebServer server(80);

// ---------------------------------------------------------------- state ----
volatile int  duty[4] = {0,0,0,0};
volatile bool armed = false;
uint32_t lastCmd = 0;

float gBias[3] = {0,0,0};
float roll = 0, pitch = 0;      // complementary-filtered
float aRoll = 0, aPitch = 0;    // accelerometer-only, unfiltered
int16_t ax,ay,az,gx,gy,gz; int16_t traw;
uint32_t lastImu = 0;
bool imuOk = false;

String sweepLog = "";
bool sweepRunning = false;

// stabiliser runtime
bool     pidRun = false;
float    pidSp[2]  = {0,0};                       // setpoints, degrees
float    pTerm[2]={0,0}, iTerm[2]={0,0}, dTerm[2]={0,0}, pidOut[2]={0,0};
uint32_t lastPid = 0;
float    pidDt = 0;
float    yawOut = 0;            // yaw rate damper output, %
float    baseNow = 0;           // slew-limited throttle actually applied
int      baseTgt = 0;           // throttle the UI asked for
int      pidFault = 0;          // 0 none, 1 tilt cutoff, 2 IMU lost
static void pidStop();          // used by the sweep and manual-motor paths below

// ---------------------------------------------------------------- pwm ------
static void pwmAttach(int pin) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(pin, PWM_FREQ, PWM_BITS);
#else
  static int ch = 0;
  ledcSetup(ch, PWM_FREQ, PWM_BITS);
  ledcAttachPin(pin, ch);
  ch++;
#endif
}
static void pwmWrite(int pin, int v) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, v);
#else
  ledcWrite(pin, v);   // v2.x: channel==index order matches attach order
#endif
}
static void applyMotors() {
  for (int i = 0; i < 4; i++) {
    int v = armed ? map(duty[i], 0, 100, 0, PWM_MAX) : 0;
    pwmWrite(cfg.m[i], v);
  }
}
static void allStop() { for (int i=0;i<4;i++) duty[i]=0; armed=false; applyMotors(); }

// ---------------------------------------------------------------- mpu6050 --
#define MPU 0x68
static void wr(uint8_t r, uint8_t v){ Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
static bool imuInit() {
  Wire.beginTransmission(MPU);
  if (Wire.endTransmission() != 0) return false;
  wr(0x6B, 0x80); delay(100);          // reset
  wr(0x6B, 0x01); delay(50);           // wake, PLL gyro-X
  wr(0x19, 0x00);                      // 1 kHz
  wr(0x1A, 0x03);                      // DLPF 44 Hz
  wr(0x1B, 0x18);                      // gyro +-2000 dps
  wr(0x1C, 0x10);                      // accel +-8 g
  return true;
}
static void imuRead() {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) { imuOk = false; return; }
  if (Wire.requestFrom(MPU, 14) != 14) { imuOk = false; return; }
  ax=(Wire.read()<<8)|Wire.read(); ay=(Wire.read()<<8)|Wire.read(); az=(Wire.read()<<8)|Wire.read();
  traw=(Wire.read()<<8)|Wire.read();
  gx=(Wire.read()<<8)|Wire.read(); gy=(Wire.read()<<8)|Wire.read(); gz=(Wire.read()<<8)|Wire.read();
  imuOk = true;

  float axg=ax/4096.0f, ayg=ay/4096.0f, azg=az/4096.0f;
  float gxd=(gx/16.4f)-gBias[0], gyd=(gy/16.4f)-gBias[1];

  aRoll  = atan2f(ayg, azg)*57.2958f;
  aPitch = atan2f(-axg, sqrtf(ayg*ayg+azg*azg))*57.2958f;

  uint32_t now=micros(); float dt=(now-lastImu)/1e6f; lastImu=now;
  if (dt<=0 || dt>0.2f) return;
  const float k = 0.98f;
  roll  = k*(roll  + gxd*dt) + (1-k)*aRoll;
  pitch = k*(pitch + gyd*dt) + (1-k)*aPitch;
}
static void calibGyro() {
  long s[3]={0,0,0};
  for (int i=0;i<400;i++){ imuRead(); s[0]+=gx; s[1]+=gy; s[2]+=gz; delay(3); }
  for (int i=0;i<3;i++) gBias[i] = (s[i]/400.0f)/16.4f;
  roll=0; pitch=0;
}

// ---------------------------------------------------------------- battery --
static float vbat() {
  uint32_t s=0; for (int i=0;i<16;i++) s+=analogReadMilliVolts(cfg.vbatPin);
  return (s/16.0f)/1000.0f*cfg.vbatDiv;
}

// ---------------------------------------------------------------- scan -----
static String scanBus(TwoWire &w, const char* name) {
  String o = "\"" + String(name) + "\":[";
  bool first = true;
  for (uint8_t a=1; a<127; a++) {
    w.beginTransmission(a);
    if (w.endTransmission()==0) {
      if(!first) o += ","; first=false;
      o += "\"0x" + String(a,HEX) + "\"";
    }
  }
  return o + "]";
}
static uint8_t pmwRead(uint8_t reg) {
  digitalWrite(SPI_CS, LOW);
  SPI.transfer(reg & 0x7F); delayMicroseconds(35);
  uint8_t v = SPI.transfer(0x00);
  delayMicroseconds(1); digitalWrite(SPI_CS, HIGH); delayMicroseconds(20);
  return v;
}

// ---------------------------------------------------------------- sweep ----
// Runs each motor alone at 30/50/70% for 700 ms, records battery sag.
// A motor that draws nothing (no sag) has a dead winding or blown MOSFET.
//
// Non-blocking: driven from loop() so the web server keeps answering and the
// browser can log battery voltage across the whole sweep. The earlier blocking
// version froze the UI for ~10 s and lost exactly the samples worth plotting.
enum { SW_IDLE = 0, SW_RUN, SW_SETTLE };
static int      swState = SW_IDLE;
static int      swMotor = 0, swLevel = 0;
static uint32_t swT0 = 0;
static float    swRest = 0, swLo = 9.9f;
static String   swLine = "";
static const int      SW_LEVELS[3] = { 30, 50, 70 };
static const uint32_t SW_HOLD = 700, SW_SETTLE_MS = 400;

static void swStartLevel() {
  for (int k = 0; k < 4; k++) duty[k] = 0;
  duty[swMotor] = SW_LEVELS[swLevel];
  armed = true;
  applyMotors();
  swLo = 9.9f;
  swT0 = millis();
  swState = SW_RUN;
}
static void sweepStart() {
  if (sweepRunning) return;
  if (pidRun) pidStop();
  sweepLog = "";
  swRest = vbat();
  sweepLog += "rest " + String(swRest,2) + "V\\n";
  swMotor = 0; swLevel = 0;
  swLine = "M1 ";
  sweepRunning = true;
  swStartLevel();
}
static void sweepTick() {
  if (!sweepRunning) return;
  uint32_t now = millis();

  if (swState == SW_RUN) {
    float v = vbat();
    if (v < swLo) swLo = v;
    if (now - swT0 < SW_HOLD) return;

    swLine += String(SW_LEVELS[swLevel]) + "%:" + String(swRest - swLo, 3) + "V  ";
    swLevel++;
    if (swLevel < 3) { swStartLevel(); return; }

    allStop();
    sweepLog += swLine + "\\n";
    swT0 = now;
    swState = SW_SETTLE;
    return;
  }

  if (swState == SW_SETTLE) {
    if (now - swT0 < SW_SETTLE_MS) return;
    swMotor++;
    if (swMotor < 4) {
      swLevel = 0;
      swLine = "M" + String(swMotor + 1) + " ";
      swStartLevel();
    } else {
      allStop();
      sweepRunning = false;
      swState = SW_IDLE;
      lastCmd = millis();          // don't trip the watchdog on the way out
    }
  }
}

// ---------------------------------------------------------------- pid ------
// Roll/pitch angle stabiliser for a two-axis test gimbal. The frame is free to
// rotate about roll and pitch but cannot translate, so a wrong gain shakes the
// rig instead of flying it into a wall.
//
// D acts on the gyro rate, not on the differentiated angle. Differentiating a
// noisy angle amplifies the noise and kicks hard the instant the setpoint
// moves; rate-on-measurement has neither problem.
static void pidStop() {
  pidRun = false;
  for (int a = 0; a < 2; a++) { pTerm[a]=iTerm[a]=dTerm[a]=pidOut[a]=0; }
  yawOut = 0;
  baseNow = 0; baseTgt = 0; pid.base = 0;
  allStop();
}
static void pidStart() {
  if (sweepRunning) return;
  for (int a = 0; a < 2; a++) iTerm[a] = 0;   // never inherit stale windup
  yawOut = 0;
  pidFault = 0;
  // Flight always starts at zero throttle and must be raised deliberately.
  // Bench ramps up to whatever base throttle was dialled in.
  baseTgt = pid.flight ? 0 : pid.base;
  baseNow = 0;
  lastPid = micros();
  pidRun = true;
  armed = true;
}
static void pidTick() {
  if (!pidRun) return;

  uint32_t now = micros();
  float dt = (now - lastPid) / 1e6f;
  lastPid = now;
  if (dt <= 0 || dt > 0.1f) return;       // first pass, or we stalled
  pidDt = dt;

  // ---- failsafes, before anything drives a motor --------------------------
  if (!imuOk) { pidFault = 2; pidStop(); return; }
  if (pid.flight &&
      (fabsf(roll) > pid.tiltCut || fabsf(pitch) > pid.tiltCut)) {
    pidFault = 1; pidStop(); return;      // flipped or crashed
  }

  // ---- throttle slew: a step from 0 to hover is violent -------------------
  float lim = pid.slew * dt;
  if      (baseNow < baseTgt) baseNow = min((float)baseTgt, baseNow + lim);
  else if (baseNow > baseTgt) baseNow = max((float)baseTgt, baseNow - lim);

  const float ang[2]  = { roll, pitch };
  const float rate[2] = { gx/16.4f - gBias[0], gy/16.4f - gBias[1] };

  // Sitting on the ground the error is constant and the integrator would wind
  // up, so the drone would leap sideways the instant it left the floor.
  bool grounded = pid.flight && baseNow < pid.thrMin;

  for (int a = 0; a < 2; a++) {
    bool active = (pid.mode == 3) || (pid.mode == 1 && a == 0) || (pid.mode == 2 && a == 1);
    if (!active) { pTerm[a]=iTerm[a]=dTerm[a]=pidOut[a]=0; continue; }

    float e = pidSp[a] - ang[a];
    pTerm[a] = pid.kp[a] * e;
    if (grounded) {
      iTerm[a] = 0;
    } else {
      iTerm[a] += pid.ki[a] * e * dt;
      iTerm[a] = constrain(iTerm[a], -pid.iLim, pid.iLim);
    }
    dTerm[a] = -pid.kd[a] * rate[a];
    pidOut[a] = constrain(pTerm[a] + iTerm[a] + dTerm[a], -pid.oLim, pid.oLim);
  }

  // Yaw is a RATE DAMPER, not a heading hold. It resists spin; it will not
  // return to a heading, because nothing fuses the magnetometer.
  if (pid.flight && !grounded) {
    float yr = gz/16.4f - gBias[2];
    yawOut = constrain(-pid.yawKp * yr, -pid.yawLim, pid.yawLim);
  } else {
    yawOut = 0;
  }

  // X-quad mix. M1 front-right, M2 back-left, M3 back-right, M4 front-left.
  // Yaw acts on the diagonals: M1/M2 share one rotation, M3/M4 the other.
  float r = pid.inv[0] ? -pidOut[0] : pidOut[0];
  float q = pid.inv[1] ? -pidOut[1] : pidOut[1];
  float y = yawOut;
  float b = baseNow;
  duty[0] = constrain((int)lroundf(b - r - q - y), 0, 100);
  duty[1] = constrain((int)lroundf(b + r + q - y), 0, 100);
  duty[2] = constrain((int)lroundf(b - r + q + y), 0, 100);
  duty[3] = constrain((int)lroundf(b + r - q + y), 0, 100);
  applyMotors();
}

static void pidLoad() {
  prefs.begin("bench", true);
  pid.kp[0]=prefs.getFloat("pkp0",pid.kp[0]); pid.kp[1]=prefs.getFloat("pkp1",pid.kp[1]);
  pid.ki[0]=prefs.getFloat("pki0",pid.ki[0]); pid.ki[1]=prefs.getFloat("pki1",pid.ki[1]);
  pid.kd[0]=prefs.getFloat("pkd0",pid.kd[0]); pid.kd[1]=prefs.getFloat("pkd1",pid.kd[1]);
  pid.iLim  = prefs.getFloat("pil", pid.iLim);
  pid.oLim  = prefs.getFloat("pol", pid.oLim);
  pid.inv[0]= prefs.getInt("piv0", pid.inv[0]);
  pid.inv[1]= prefs.getInt("piv1", pid.inv[1]);
  pid.yawKp = prefs.getFloat("pykp", pid.yawKp);
  pid.yawLim= prefs.getFloat("pyl",  pid.yawLim);
  pid.tiltCut=prefs.getFloat("ptc",  pid.tiltCut);
  prefs.end();
}
static void pidSave() {
  prefs.begin("bench", false);
  prefs.putFloat("pkp0",pid.kp[0]); prefs.putFloat("pkp1",pid.kp[1]);
  prefs.putFloat("pki0",pid.ki[0]); prefs.putFloat("pki1",pid.ki[1]);
  prefs.putFloat("pkd0",pid.kd[0]); prefs.putFloat("pkd1",pid.kd[1]);
  prefs.putFloat("pil", pid.iLim);  prefs.putFloat("pol", pid.oLim);
  prefs.putInt("piv0", pid.inv[0]); prefs.putInt("piv1", pid.inv[1]);
  prefs.putFloat("pykp", pid.yawKp); prefs.putFloat("pyl", pid.yawLim);
  prefs.putFloat("ptc", pid.tiltCut);
  prefs.end();
}

// ---------------------------------------------------------------- html -----
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>LiteWing Bench</title><style>
:root{--bg:#0e1116;--pnl:#161b22;--ln:#2a323d;--fg:#d7dee8;--dim:#7d8998;--ok:#4fd08a;--warn:#e0b341;--hot:#e05a4f;--m1:#4fd08a;--m2:#5aa9e0;--m3:#e0b341;--m4:#c77ae0}
*{box-sizing:border-box}body{margin:0 auto;max-width:600px;background:var(--bg);
color:var(--fg);font:13px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace;padding:10px}
h1{font-size:14px;letter-spacing:.14em;text-transform:uppercase;margin:2px 0 12px;color:var(--dim)}
.p{background:var(--pnl);border:1px solid var(--ln);padding:10px;margin-bottom:10px}
.p>h2{font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--dim);margin:0 0 8px;font-weight:500}
.row{display:flex;justify-content:space-between;padding:2px 0}
.row b{font-weight:600}
button{font:inherit;background:#222a35;color:var(--fg);border:1px solid var(--ln);
padding:7px 12px;cursor:pointer}button:active{background:#2e3846}
button:focus-visible{outline:2px solid var(--ok);outline-offset:1px}
button[disabled]{opacity:.4;cursor:default}
.arm{width:100%;padding:14px;font-size:14px;letter-spacing:.1em;border-color:var(--hot);color:var(--hot)}
.arm.on{background:var(--hot);color:#0e1116;font-weight:700}
input[type=range]{width:100%;accent-color:var(--ok)}
input[type=number]{width:70px;background:#0e1116;color:var(--fg);border:1px solid var(--ln);padding:4px;font:inherit}
.m{border-top:1px solid var(--ln);padding-top:8px;margin-top:8px}
.m:first-of-type{border:0;margin:0;padding:0}
pre{margin:0;white-space:pre-wrap;color:var(--dim);font-size:12px}
.tag{color:var(--ok)}.bad{color:var(--hot)}.note{color:var(--warn);font-size:12px;margin-bottom:10px}
.cap{color:var(--dim);font-size:11px;line-height:1.4;margin-top:8px}
.g{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}

/* ---------------------------------------------------- 3d attitude view --- */
.scene{height:232px;display:flex;align-items:center;justify-content:center;
perspective:640px;perspective-origin:50% 42%;margin:2px 0 6px;overflow:hidden}
.rig{width:170px;height:170px;position:relative;transform-style:preserve-3d;transform:rotateX(64deg)}
.grid{position:absolute;inset:-44px;transform:translateZ(-48px);border:1px solid #232a34;
background:
repeating-linear-gradient(0deg,transparent 0 24px,rgba(125,137,152,.13) 24px 25px),
repeating-linear-gradient(90deg,transparent 0 24px,rgba(125,137,152,.13) 24px 25px),
radial-gradient(circle at 50% 50%,rgba(79,208,138,.05),transparent 65%)}
.shadow{position:absolute;left:50%;top:50%;width:150px;height:150px;margin:-75px 0 0 -75px;
border-radius:50%;transform:translateZ(-47px);filter:blur(9px);
background:radial-gradient(circle,rgba(0,0,0,.6) 0,rgba(0,0,0,.28) 45%,transparent 70%)}
.drone{position:absolute;inset:0;transform-style:preserve-3d}

.bm{position:absolute;left:50%;top:50%;width:150px;height:9px;margin:-4.5px 0 0 -75px;
transform-style:preserve-3d}
.bm>i{position:absolute;inset:0;border-radius:4px;
background:linear-gradient(180deg,#535f70,#39434f);box-shadow:0 0 0 1px #1b212a}
.bm>u{position:absolute;left:0;right:0;top:100%;height:6px;transform-origin:top;
transform:rotateX(-90deg);background:#232b35;border-radius:0 0 3px 3px}
.b1{transform:rotate(45deg) translateZ(2px)}
.b2{transform:rotate(-45deg) translateZ(2px)}

.bd{position:absolute;left:50%;top:50%;width:48px;height:38px;margin:-19px 0 0 -24px;
transform-style:preserve-3d}
.bd>i{position:absolute;left:50%;top:50%;background:#2f3945;box-shadow:0 0 0 1px #1b212a}
.bd .tp{width:48px;height:38px;margin:-19px 0 0 -24px;transform:translateZ(9px);border-radius:5px;
background:linear-gradient(155deg,#63728a,#3c4757 60%,#333e4c)}
.bd .bt{width:48px;height:38px;margin:-19px 0 0 -24px;transform:translateZ(-9px);background:#191f27}
.bd .fr{width:48px;height:18px;margin:-9px 0 0 -24px;transform:translateY(-19px) rotateX(90deg);background:#3e4958}
.bd .bk{width:48px;height:18px;margin:-9px 0 0 -24px;transform:translateY(19px) rotateX(90deg);background:#232b35}
.bd .lf{width:18px;height:38px;margin:-19px 0 0 -9px;transform:translateX(-24px) rotateY(90deg);background:#2a333e}
.bd .rt{width:18px;height:38px;margin:-19px 0 0 -9px;transform:translateX(24px) rotateY(90deg);background:#2a333e}
.nose{position:absolute;left:50%;top:50%;width:0;height:0;margin:-33px 0 0 -7px;
border-left:7px solid transparent;border-right:7px solid transparent;
border-bottom:13px solid var(--ok);transform:translateZ(10px);
filter:drop-shadow(0 0 4px rgba(79,208,138,.55))}
.led{position:absolute;left:50%;top:50%;width:9px;height:9px;margin:6px 0 0 -4.5px;
border-radius:50%;transform:translateZ(10px);background:var(--ok);
box-shadow:0 0 7px var(--ok)}
.led.hot{background:var(--hot);box-shadow:0 0 9px var(--hot)}

.pod{position:absolute;transform-style:preserve-3d}
.pod>i{position:absolute;left:50%;top:50%;border-radius:50%;box-shadow:0 0 0 1px #1b212a}
.pod .lo{width:19px;height:19px;margin:-9.5px 0 0 -9.5px;transform:translateZ(3px);background:#252d38}
.pod .hi{width:16px;height:16px;margin:-8px 0 0 -8px;transform:translateZ(10px);
background:radial-gradient(circle at 34% 28%,#7a8a9e,#39434f)}
.pod .disc{width:58px;height:58px;margin:-29px 0 0 -29px;transform:translateZ(13px);opacity:0;
transition:opacity .25s;box-shadow:none;
background:radial-gradient(circle,transparent 26%,rgba(79,208,138,.10) 45%,rgba(79,208,138,.27) 68%,transparent 71%)}
.prop{position:absolute;width:2px;height:2px;transform:translateZ(13px);transform-style:preserve-3d}
.prop>b{position:absolute;left:50%;top:50%;width:56px;height:6px;margin:-3px 0 0 -28px;border-radius:4px;
background:linear-gradient(90deg,rgba(150,170,195,.08),rgba(176,196,222,.72),rgba(150,170,195,.08))}
.prop>b:nth-child(2){transform:rotate(90deg)}
.prop.sp{animation:spin .4s linear infinite}
@keyframes spin{from{transform:translateZ(13px) rotate(0)}to{transform:translateZ(13px) rotate(360deg)}}
.p1{left:calc(50% + 53px);top:calc(50% - 53px)}
.p2{left:calc(50% - 53px);top:calc(50% - 53px)}
.p3{left:calc(50% - 53px);top:calc(50% + 53px)}
.p4{left:calc(50% + 53px);top:calc(50% + 53px)}
.att{display:flex;gap:20px;justify-content:center;color:var(--dim)}
.att b{color:var(--fg);font-variant-numeric:tabular-nums}

/* ---------------------------------------------------- tables and charts -- */
.t{width:100%;border-collapse:collapse;font-size:12px}
.t th{color:var(--dim);font-weight:500;text-align:right;padding:0 0 5px;
font-size:10px;letter-spacing:.12em;text-transform:uppercase}
.t th:first-child{text-align:left}
.t td{padding:2px 0;text-align:right;font-variant-numeric:tabular-nums}
.t td:first-child{text-align:left;color:var(--dim)}
.t td.r{color:var(--dim)}
.t tr.sep td{border-top:1px solid var(--ln);padding-top:6px}
.lt{width:100%;border-collapse:collapse;font-size:11px;margin-top:6px}
.lt th{color:var(--dim);font-weight:500;text-align:right;padding:3px 4px;
border-bottom:1px solid var(--ln);font-size:10px;letter-spacing:.08em;text-transform:uppercase;
position:sticky;top:0;background:var(--pnl)}
.lt th:first-child,.lt td:first-child{text-align:left}
.lt td{padding:3px 4px;text-align:right;font-variant-numeric:tabular-nums;border-bottom:1px solid #1e242c}
.lt tr:hover td{background:#1b212a}
.wrap{max-height:200px;overflow:auto}
canvas{width:100%;height:96px;display:block;cursor:crosshair}
.ch{margin-bottom:9px}
.ch h3{font-size:10px;letter-spacing:.12em;text-transform:uppercase;color:var(--dim);
margin:0 0 3px;font-weight:500;display:flex;justify-content:space-between}
.ch h3 em{font-style:normal;color:var(--fg);font-variant-numeric:tabular-nums}
.key{display:flex;gap:11px;font-size:10px;color:var(--dim);margin-top:3px}
.key i{display:inline-block;width:9px;height:2px;vertical-align:middle;margin-right:3px}
.rec{border-color:var(--ok);color:var(--ok)}
.rec.on{background:var(--hot);border-color:var(--hot);color:#0e1116;font-weight:700}
.stat{display:grid;grid-template-columns:1fr 1fr;gap:1px 14px;margin-bottom:9px;font-size:11px}
.stat span{color:var(--dim)}
.stat b{float:right;color:var(--fg);font-variant-numeric:tabular-nums}
</style></head><body>
<h1>LiteWing Bench</h1>
<div class=note>Remove propellers before arming.</div>

<div class=p><h2>Status</h2>
<div class=row><span>Battery</span><b id=vb>--</b></div>
<div class=row><span>IMU</span><b id=io>--</b></div>
<div class=row><span>Armed</span><b id=ar>--</b></div>
</div>

<div class=p><h2>Motors</h2>
<button class=arm id=armb onclick=toggleArm()>ARM</button>
<div id=mots></div>
<div class=g style=margin-top:10px>
<button onclick=stop()>Stop all</button>
<button id=swb onclick=sweep()>Health sweep</button>
</div>
<pre id=sw style=margin-top:8px></pre>
</div>

<div class=p><h2>Attitude</h2>
<div class=scene><div class=rig>
  <div class=grid></div>
  <div class=shadow id=shd></div>
  <div class=drone id=drn>
    <div class="bm b1"><i></i><u></u></div>
    <div class="bm b2"><i></i><u></u></div>
    <div class="pod p1"><i class=lo></i><i class=hi></i><i class=disc id=dc0></i></div>
    <div class="pod p2"><i class=lo></i><i class=hi></i><i class=disc id=dc1></i></div>
    <div class="pod p3"><i class=lo></i><i class=hi></i><i class=disc id=dc2></i></div>
    <div class="pod p4"><i class=lo></i><i class=hi></i><i class=disc id=dc3></i></div>
    <div class="prop p1" id=pr0><b></b><b></b></div>
    <div class="prop p2" id=pr1><b></b><b></b></div>
    <div class="prop p3" id=pr2><b></b><b></b></div>
    <div class="prop p4" id=pr3><b></b><b></b></div>
    <div class=bd><i class=bt></i><i class=fr></i><i class=bk></i><i class=lf></i><i class=rt></i><i class=tp></i></div>
    <div class=led id=ledd></div>
    <div class=nose></div>
  </div>
</div></div>
<div class=att><span>roll <b id=abr>--</b></span><span>pitch <b id=abp>--</b></span></div>
</div>

<div class=p><h2>Session log</h2>
<div class=g3>
<button class=rec id=recb onclick=toggleRec()>RECORD</button>
<button onclick=clearLog()>Clear</button>
<button onclick=exportAll()>Export CSV</button>
</div>
<div class=stat style=margin-top:10px>
<span>elapsed <b id=lgT>--</b></span><span>samples <b id=lgN>--</b></span>
<span>V start <b id=lgV0>--</b></span><span>V now <b id=lgV1>--</b></span>
<span>V min <b id=lgVm>--</b></span><span>total drop <b id=lgDr>--</b></span>
</div>
<div class=g3 style=margin-bottom:9px>
<button onclick=setWin(30)>30 s</button>
<button onclick=setWin(120)>2 min</button>
<button onclick=setWin(0)>All</button>
</div>

<div class=ch><h3><span>battery volts</span><em id=cv1>--</em></h3>
<canvas id=c1></canvas></div>

<div class=ch><h3><span>motor duty %</span><em id=cv2>--</em></h3>
<canvas id=c2></canvas>
<div class=key><span><i style=background:var(--m1)></i>M1</span><span><i style=background:var(--m2)></i>M2</span>
<span><i style=background:var(--m3)></i>M3</span><span><i style=background:var(--m4)></i>M4</span></div></div>

<div class=ch><h3><span>attitude deg</span><em id=cv3>--</em></h3>
<canvas id=c3></canvas>
<div class=key><span><i style=background:var(--ok)></i>roll</span><span><i style=background:var(--m2)></i>pitch</span></div></div>

<div class=ch><h3><span>gyro dps</span><em id=cv4>--</em></h3>
<canvas id=c4></canvas>
<div class=key><span><i style=background:var(--m1)></i>x</span><span><i style=background:var(--m2)></i>y</span>
<span><i style=background:var(--m3)></i>z</span></div></div>

<h2 style=margin-top:14px>Load segments</h2>
<div class=cap style=margin:0>Every period of constant motor output, with the
battery sag it caused. Sag is measured against the resting voltage just before
the segment started.</div>
<div class=wrap>
<table class=lt><thead><tr><th>t</th><th>output</th><th>dur</th>
<th>V rest</th><th>V min</th><th>sag</th></tr></thead>
<tbody id=segs><tr><td colspan=6 style=color:var(--dim)>not recording</td></tr></tbody></table>
</div>
</div>

<div class=p><h2>IMU</h2>
<table class=t>
<tr><th>&nbsp;</th><th>raw</th><th>filtered</th></tr>
<tr><td>accel x</td><td class=r id=rax>--</td><td id=fax>--</td></tr>
<tr><td>accel y</td><td class=r id=ray>--</td><td id=fay>--</td></tr>
<tr><td>accel z</td><td class=r id=raz>--</td><td id=faz>--</td></tr>
<tr class=sep><td>gyro x</td><td class=r id=rgx>--</td><td id=fgx>--</td></tr>
<tr><td>gyro y</td><td class=r id=rgy>--</td><td id=fgy>--</td></tr>
<tr><td>gyro z</td><td class=r id=rgz>--</td><td id=fgz>--</td></tr>
<tr class=sep><td>roll</td><td class=r id=rrl>--</td><td id=frl>--</td></tr>
<tr><td>pitch</td><td class=r id=rpt>--</td><td id=fpt>--</td></tr>
<tr class=sep><td>gyro bias</td><td class=r colspan=2 id=gbs>--</td></tr>
<tr><td>die temp</td><td class=r colspan=2 id=tp>--</td></tr>
</table>
<div class=cap>raw accel/gyro are LSB counts; raw roll/pitch are
accelerometer-only. filtered = scaled, bias-corrected, complementary.</div>
<button style=margin-top:8px onclick=calib()>Calibrate gyro bias</button>
</div>

<div class=p><h2>Sensors</h2>
<pre id=sc>tap scan</pre>
<button style=margin-top:8px onclick=scan()>Scan buses</button>
</div>

<div class=p><h2>Pins</h2>
<div id=pins></div>
<button style=margin-top:8px onclick=savePins()>Save pins and reboot</button>
</div>

<script>
const SGN={r:1,p:-1};        // attitude sign convention, see docs/UI.md
const MC=['#4fd08a','#5aa9e0','#e0b341','#c77ae0'];
let armed=false, sweeping=false;

const mots=document.getElementById('mots');
for(let i=0;i<4;i++){mots.insertAdjacentHTML('beforeend',
`<div class=m><div class=row><span>M${i+1}</span><b id=d${i}>0%</b></div>
<input type=range min=0 max=100 value=0 id=s${i} oninput=setM(${i},this.value)>
<div class=g><button onclick=pulse(${i},30)>30% 1s</button><button onclick=pulse(${i},60)>60% 1s</button></div></div>`);}

async function j(u,o){try{const r=await fetch(u,o);return await r.json()}catch(e){return null}}
function setM(i,v){document.getElementById('d'+i).textContent=v+'%';j('/api/motor?i='+i+'&v='+v)}
async function pulse(i,v){document.getElementById('s'+i).value=v;setM(i,v);
mark('pulse M'+(i+1)+' '+v+'%');
setTimeout(()=>{document.getElementById('s'+i).value=0;setM(i,0)},1000)}
function toggleArm(){armed=!armed;mark(armed?'arm':'disarm');j('/api/arm?v='+(armed?1:0))}
function stop(){armed=false;for(let i=0;i<4;i++){document.getElementById('s'+i).value=0;
document.getElementById('d'+i).textContent='0%'}mark('stop');j('/api/stop')}
function calib(){document.getElementById('gbs').textContent='calibrating...';mark('calibrate');j('/api/calib')}

async function sweep(){
  if(sweeping)return;
  mark('sweep start');
  document.getElementById('sw').textContent='starting...';
  await j('/api/sweep');
}
async function sweepDone(){
  const r=await j('/api/sweeplog');
  document.getElementById('sw').textContent=r?r.log:'failed';
  mark('sweep end');
}

async function scan(){const r=await j('/api/scan');if(!r){document.getElementById('sc').textContent='scan failed';return}
let t='I2C0 (IMU bus): '+(r.i2c0.length?r.i2c0.join(' '):'nothing found')+'\n';
t+='I2C1 (aux bus): '+(r.i2c1.length?r.i2c1.join(' '):'nothing found')+'\n\n';
for(const[k,v]of Object.entries(r.id))t+=(v?'[ok]  ':'[--]  ')+k+'\n';
document.getElementById('sc').textContent=t}

async function loadPins(){const p=await j('/api/pins');if(!p)return;
let h='';for(let i=0;i<4;i++)h+=`<div class=row><span>M${i+1} gpio</span><input type=number id=p${i} value=${p.m[i]}></div>`;
h+=`<div class=row><span>vbat adc gpio</span><input type=number id=pv value=${p.vbatPin}></div>`;
h+=`<div class=row><span>vbat divider</span><input type=number step=0.01 id=pd value=${p.vbatDiv}></div>`;
document.getElementById('pins').innerHTML=h}
function savePins(){let q='/api/pins?vbat='+pv.value+'&div='+pd.value;
for(let i=0;i<4;i++)q+='&m'+i+'='+document.getElementById('p'+i).value;
j(q).then(()=>document.getElementById('pins').insertAdjacentHTML('beforeend','<div class=note>Saved. Rebooting.</div>'))}

/* ------------------------------------------------------------- logging --- */
const LOG={on:false,t0:0,s:[],ev:[],seg:[],cur:null,rest:null,win:120,full:false};
const CAP=36000;                      // ~72 min at 120 ms, then stop

function mark(text){ if(LOG.on) LOG.ev.push({t:now(),text:text}); }
function now(){ return (Date.now()-LOG.t0)/1000; }

function toggleRec(){
  LOG.on=!LOG.on;
  if(LOG.on && !LOG.s.length) LOG.t0=Date.now();
  recb.classList.toggle('on',LOG.on);
  recb.textContent=LOG.on?'RECORDING':'RECORD';
  mark(LOG.on?'record start':'record stop');
}
function clearLog(){
  LOG.s=[];LOG.ev=[];LOG.seg=[];LOG.cur=null;LOG.rest=null;LOG.full=false;
  LOG.t0=Date.now();drawSegs();
}
function setWin(w){LOG.win=w}

function sample(s){
  if(!LOG.on)return;
  if(LOG.s.length>=CAP){ if(!LOG.full){LOG.full=true;LOG.on=false;
    recb.classList.remove('on');recb.textContent='RECORD';
    mark('buffer full - recording stopped');} return; }
  const t=now();
  LOG.s.push({t:t,vb:s.vbat,d:s.duty.slice(),ro:s.roll,pi:s.pitch,
              gx:s.g[0],gy:s.g[1],gz:s.g[2],ax:s.a[0],ay:s.a[1],az:s.a[2],
              tp:s.temp,ar:s.armed?1:0});
  segment(t,s);
}

/* one row per period of constant motor output */
function segment(t,s){
  const key=s.duty.join(',');
  const live=s.duty.some(v=>v>0);
  if(LOG.cur && LOG.cur.key!==key){ closeSeg(t); }
  if(!LOG.cur){
    if(!live){ LOG.rest=s.vbat; return; }         // idle: track resting volts
    LOG.cur={key:key,t0:t,duty:s.duty.slice(),
             rest:(LOG.rest!=null?LOG.rest:s.vbat),vmin:s.vbat};
    return;
  }
  if(s.vbat<LOG.cur.vmin)LOG.cur.vmin=s.vbat;
}
function closeSeg(t){
  const c=LOG.cur; LOG.cur=null;
  if(!c)return;
  const dur=t-c.t0;
  if(dur<0.05)return;
  LOG.seg.push({t:c.t0,dur:dur,duty:c.duty,rest:c.rest,vmin:c.vmin,sag:c.rest-c.vmin});
  drawSegs();
}
function dutyLabel(d){
  const on=[];for(let i=0;i<4;i++)if(d[i]>0)on.push('M'+(i+1)+' '+d[i]+'%');
  return on.length?on.join(' + '):'idle';
}
function mmss(t){const m=Math.floor(t/60),s=t-m*60;return m+':'+(s<10?'0':'')+s.toFixed(1)}
function drawSegs(){
  const b=document.getElementById('segs');
  if(!LOG.seg.length){b.innerHTML='<tr><td colspan=6 style=color:var(--dim)>'+
    (LOG.on?'waiting for motor output':'not recording')+'</td></tr>';return}
  let h='';
  for(let i=LOG.seg.length-1;i>=0;i--){const g=LOG.seg[i];
    const bad=g.sag<0.005;
    h+='<tr><td>'+mmss(g.t)+'</td><td>'+dutyLabel(g.duty)+'</td><td>'+g.dur.toFixed(2)+'s</td>'+
       '<td>'+g.rest.toFixed(3)+'</td><td>'+g.vmin.toFixed(3)+'</td>'+
       '<td style="color:'+(bad?'var(--hot)':'var(--ok)')+'">'+g.sag.toFixed(3)+'</td></tr>';}
  b.innerHTML=h;
}

/* ---------------------------------------------------------- csv export --- */
function dl(name,text){
  const a=document.createElement('a');
  a.href=URL.createObjectURL(new Blob([text],{type:'text/csv'}));
  a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),4000);
}
function exportAll(){
  if(!LOG.s.length){alert('nothing recorded yet');return}
  let c='t_s,vbat_v,m1_pct,m2_pct,m3_pct,m4_pct,armed,roll_deg,pitch_deg,'+
        'gx_dps,gy_dps,gz_dps,ax_g,ay_g,az_g,temp_c\n';
  for(const r of LOG.s)c+=[r.t.toFixed(3),r.vb.toFixed(4),r.d[0],r.d[1],r.d[2],r.d[3],r.ar,
    r.ro.toFixed(2),r.pi.toFixed(2),r.gx.toFixed(2),r.gy.toFixed(2),r.gz.toFixed(2),
    r.ax.toFixed(4),r.ay.toFixed(4),r.az.toFixed(4),r.tp.toFixed(1)].join(',')+'\n';
  dl('litewing_samples.csv',c);

  let g='t_s,duration_s,m1_pct,m2_pct,m3_pct,m4_pct,v_rest,v_min,sag_v\n';
  for(const s of LOG.seg)g+=[s.t.toFixed(3),s.dur.toFixed(3),s.duty[0],s.duty[1],s.duty[2],
    s.duty[3],s.rest.toFixed(4),s.vmin.toFixed(4),s.sag.toFixed(4)].join(',')+'\n';
  let e='t_s,event\n';
  for(const v of LOG.ev)e+=v.t.toFixed(3)+',"'+v.text+'"\n';
  setTimeout(()=>dl('litewing_segments.csv',g),350);
  setTimeout(()=>dl('litewing_events.csv',e),700);
}

/* -------------------------------------------------------------- charts --- */
let hoverT=null;
function view(){
  if(!LOG.s.length)return[];
  if(!LOG.win)return LOG.s;
  const cut=LOG.s[LOG.s.length-1].t-LOG.win;
  let i=LOG.s.length-1;while(i>0&&LOG.s[i-1].t>=cut)i--;
  return LOG.s.slice(i);
}
function plot(id,rows,lines,opts){
  const cv=document.getElementById(id),dpr=devicePixelRatio||1;
  const w=cv.clientWidth,h=cv.clientHeight;
  if(cv.width!==Math.round(w*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr)}
  const x=cv.getContext('2d');x.setTransform(dpr,0,0,dpr,0,0);
  x.clearRect(0,0,w,h);
  x.fillStyle='#12161c';x.fillRect(0,0,w,h);
  if(rows.length<2){x.fillStyle='#7d8998';x.font='11px ui-monospace,monospace';
    x.fillText('no data',8,h/2);return}

  const PL=40,PR=4,PT=6,PB=13,gw=w-PL-PR,gh=h-PT-PB;
  const t0=rows[0].t,t1=rows[rows.length-1].t,dt=(t1-t0)||1;
  let lo=opts.lo,hi=opts.hi;
  if(lo==null){lo=Infinity;hi=-Infinity;
    const sk=Math.max(1,Math.ceil(rows.length/900));
    for(let i=0;i<rows.length;i+=sk){const r=rows[i];
      for(const L of lines){const v=L.f(r);if(v<lo)lo=v;if(v>hi)hi=v}}}
  if(!isFinite(lo)){lo=0;hi=1}
  if(hi-lo<opts.min){const c=(hi+lo)/2;lo=c-opts.min/2;hi=c+opts.min/2}
  if(opts.lo==null){const pad=(hi-lo)*0.12;lo-=pad;hi+=pad}
  const X=t=>PL+((t-t0)/dt)*gw, Y=v=>PT+gh-((v-lo)/(hi-lo))*gh;

  x.strokeStyle='#232a34';x.lineWidth=1;x.fillStyle='#7d8998';
  x.font='9px ui-monospace,monospace';x.textAlign='right';
  for(let k=0;k<=3;k++){const v=lo+(hi-lo)*k/3,y=Math.round(Y(v))+.5;
    x.beginPath();x.moveTo(PL,y);x.lineTo(w-PR,y);x.stroke();
    x.fillText(v.toFixed(opts.dp),PL-5,y+3)}

  for(const v of LOG.ev){ if(v.t<t0||v.t>t1)continue;
    const px=Math.round(X(v.t))+.5;
    x.strokeStyle='rgba(224,179,65,.45)';x.beginPath();
    x.moveTo(px,PT);x.lineTo(px,PT+gh);x.stroke() }

  // decimate: a long session holds far more samples than the canvas has
  // pixels, and redrawing all of them 7x a second stalls the poll loop
  const step=Math.max(1,Math.ceil(rows.length/900));
  for(const L of lines){
    x.strokeStyle=L.c;x.lineWidth=1.5;x.lineJoin='round';x.beginPath();
    let started=false;
    for(let i=0;i<rows.length;i+=step){const r=rows[i],v=L.f(r);if(v==null)continue;
      const px=X(r.t),py=Y(v);
      if(!started){x.moveTo(px,py);started=true}else x.lineTo(px,py)}
    const last=rows[rows.length-1],lv=L.f(last);
    if(started&&lv!=null)x.lineTo(X(last.t),Y(lv));
    x.stroke();
  }

  x.textAlign='left';x.fillStyle='#7d8998';
  x.fillText(mmss(t0),PL,h-3);
  x.textAlign='right';x.fillText(mmss(t1),w-PR,h-3);

  if(hoverT!=null&&hoverT>=t0&&hoverT<=t1){
    const px=Math.round(X(hoverT))+.5;
    x.strokeStyle='#7d8998';x.setLineDash([2,3]);x.beginPath();
    x.moveTo(px,PT);x.lineTo(px,PT+gh);x.stroke();x.setLineDash([])}
}
function nearest(t){
  const r=view();if(!r.length)return null;
  let best=r[0],bd=1e9;
  for(const s of r){const d=Math.abs(s.t-t);if(d<bd){bd=d;best=s}}
  return best;
}
for(const id of ['c1','c2','c3','c4']){
  const cv=document.getElementById(id);
  cv.addEventListener('mousemove',e=>{
    const rows=view();if(rows.length<2)return;
    const b=cv.getBoundingClientRect(),PL=40,PR=4;
    const f=(e.clientX-b.left-PL)/(b.width-PL-PR);
    hoverT=rows[0].t+f*(rows[rows.length-1].t-rows[0].t);
  });
  cv.addEventListener('mouseleave',()=>{hoverT=null});
}

function redraw(){
  const rows=view();
  plot('c1',rows,[{c:'#4fd08a',f:r=>r.vb}],{min:0.05,dp:2});
  plot('c2',rows,[{c:MC[0],f:r=>r.d[0]},{c:MC[1],f:r=>r.d[1]},
                  {c:MC[2],f:r=>r.d[2]},{c:MC[3],f:r=>r.d[3]}],{lo:0,hi:100,min:1,dp:0});
  plot('c3',rows,[{c:'#4fd08a',f:r=>r.ro},{c:'#5aa9e0',f:r=>r.pi}],{min:10,dp:0});
  plot('c4',rows,[{c:MC[0],f:r=>r.gx},{c:MC[1],f:r=>r.gy},{c:MC[2],f:r=>r.gz}],{min:20,dp:0});

  const h=hoverT!=null?nearest(hoverT):(LOG.s.length?LOG.s[LOG.s.length-1]:null);
  if(h){
    cv1.textContent=h.vb.toFixed(3)+' V';
    cv2.textContent=h.d.map(v=>v+'%').join(' ');
    cv3.textContent=h.ro.toFixed(1)+' / '+h.pi.toFixed(1);
    cv4.textContent=[h.gx,h.gy,h.gz].map(v=>v.toFixed(0)).join(' ');
  }
  if(LOG.s.length){
    const f=LOG.s[0],l=LOG.s[LOG.s.length-1];
    let vm=Infinity;for(const r of LOG.s)if(r.vb<vm)vm=r.vb;
    lgT.textContent=mmss(l.t);lgN.textContent=LOG.s.length+(LOG.full?' (full)':'');
    lgV0.textContent=f.vb.toFixed(3);lgV1.textContent=l.vb.toFixed(3);
    lgVm.textContent=vm.toFixed(3);lgDr.textContent=(f.vb-l.vb).toFixed(3)+' V';
  }
}
setInterval(redraw,140);

/* ------------------------------------------------------- attitude view --- */
const A={r:0,p:0,tr:0,tp:0};
function frame(){
  A.r+=(A.tr-A.r)*0.22; A.p+=(A.tp-A.p)*0.22;
  drn.style.transform='rotateY('+(SGN.r*A.r)+'deg) rotateX('+(SGN.p*A.p)+'deg)';
  shd.style.transform='translateZ(-47px) translate('+(A.r*0.5)+'px,'+(-A.p*0.5)+'px)';
  requestAnimationFrame(frame);
}
requestAnimationFrame(frame);

function props(duty){
  for(let i=0;i<4;i++){
    const d=duty[i],el=document.getElementById('pr'+i),dc=document.getElementById('dc'+i);
    if(d>0){el.classList.add('sp');el.style.animationDuration=(0.62-0.0052*d).toFixed(3)+'s'}
    else el.classList.remove('sp');
    dc.style.opacity=d>0?Math.min(1,0.25+d/100).toFixed(2):0;
  }
}

/* ---------------------------------------------------------------- poll --- */
async function tick(){
  const s=await j('/api/state');if(!s)return;
  vb.textContent=s.vbat.toFixed(2)+' V';
  io.innerHTML=s.imu?'<span class=tag>online</span>':'<span class=bad>no response</span>';
  ar.innerHTML=s.armed?'<span class=bad>ARMED</span>':'<span class=tag>safe</span>';
  armed=s.armed;armb.textContent=s.armed?'DISARM':'ARM';armb.className=s.armed?'arm on':'arm';
  ledd.className=s.armed?'led hot':'led';

  const R=['rax','ray','raz'],F=['fax','fay','faz'];
  for(let i=0;i<3;i++){document.getElementById(R[i]).textContent=s.ar[i];
  document.getElementById(F[i]).textContent=s.a[i].toFixed(3)+' g'}
  const RG=['rgx','rgy','rgz'],FG=['fgx','fgy','fgz'];
  const D=String.fromCharCode(176);
  for(let i=0;i<3;i++){document.getElementById(RG[i]).textContent=s.gr[i];
  document.getElementById(FG[i]).textContent=s.g[i].toFixed(2)+D+'/s'}
  rrl.textContent=s.aroll.toFixed(1)+D; frl.textContent=s.roll.toFixed(1)+D;
  rpt.textContent=s.apitch.toFixed(1)+D; fpt.textContent=s.pitch.toFixed(1)+D;
  gbs.textContent=s.bias.map(v=>v.toFixed(2)).join('  ')+' '+D+'/s';
  tp.textContent=s.temp.toFixed(1)+' C';
  abr.textContent=s.roll.toFixed(1)+D; abp.textContent=s.pitch.toFixed(1)+D;

  A.tr=s.roll; A.tp=s.pitch;
  props(s.duty);

  if(s.sweep){ for(let i=0;i<4;i++){document.getElementById('s'+i).value=s.duty[i];
    document.getElementById('d'+i).textContent=s.duty[i]+'%'}
    document.getElementById('sw').textContent='sweeping motor '+s.swm+' ...'; }
  if(s.sweep!==sweeping){ sweeping=s.sweep; swb.disabled=sweeping;
    if(!sweeping) sweepDone(); }

  sample(s);
}
// self-scheduling rather than setInterval: if the ESP32 is slow to answer,
// fixed-interval polling queues up calls instead of backing off
async function poll(){ await tick(); setTimeout(poll,120); }
loadPins();drawSegs();poll();
</script></body></html>)HTML";

static const char PAGE_PID[] PROGMEM = R"PIDHTML(<!doctype html><html><head>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>LiteWing PID</title><style>
:root{--bg:#0e1116;--pnl:#161b22;--ln:#2a323d;--fg:#d7dee8;--dim:#7d8998;--ok:#4fd08a;
--warn:#e0b341;--hot:#e05a4f;--sp:#8b96a5;--kp:#4fd08a;--ki:#e0b341;--kd:#5aa9e0}
*{box-sizing:border-box}body{margin:0 auto;max-width:620px;background:var(--bg);
color:var(--fg);font:13px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace;padding:10px}
h1{font-size:14px;letter-spacing:.14em;text-transform:uppercase;margin:2px 0 10px;
color:var(--dim);display:flex;justify-content:space-between;align-items:baseline}
h1 a{color:var(--dim);font-size:11px;letter-spacing:.08em;text-decoration:none;border-bottom:1px solid var(--ln)}
h1 a:hover{color:var(--fg)}
.p{background:var(--pnl);border:1px solid var(--ln);padding:10px;margin-bottom:10px}
.p>h2{font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--dim);
margin:0 0 8px;font-weight:500}
.row{display:flex;justify-content:space-between;padding:2px 0}
button{font:inherit;background:#222a35;color:var(--fg);border:1px solid var(--ln);
padding:7px 12px;cursor:pointer}button:active{background:#2e3846}
button:focus-visible{outline:2px solid var(--ok);outline-offset:1px}
button.sel{background:#2e3846;border-color:var(--ok);color:var(--ok)}
button[disabled]{opacity:.4;cursor:default}
.stopb{width:100%;padding:16px;font-size:15px;letter-spacing:.12em;font-weight:700;
background:var(--hot);border-color:var(--hot);color:#0e1116;margin-bottom:8px}
.runb{width:100%;padding:14px;font-size:14px;letter-spacing:.1em;
border-color:var(--ok);color:var(--ok)}
.runb.on{background:var(--ok);border-color:var(--ok);color:#0e1116;font-weight:700}
.g{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.g3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
.g4{display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:6px}
input[type=range]{width:100%;accent-color:var(--ok)}
input[type=number]{width:100%;background:#0e1116;color:var(--fg);border:1px solid var(--ln);
padding:5px;font:inherit;font-size:12px}
.note{color:var(--warn);font-size:12px;margin-bottom:10px}
.cap{color:var(--dim);font-size:11px;line-height:1.45;margin-top:8px}
canvas{width:100%;height:118px;display:block;cursor:crosshair}
.ch{margin-bottom:9px}
.ch h3{font-size:10px;letter-spacing:.12em;text-transform:uppercase;color:var(--dim);
margin:0 0 3px;font-weight:500;display:flex;justify-content:space-between}
.ch h3 em{font-style:normal;color:var(--fg);font-variant-numeric:tabular-nums}
.key{display:flex;gap:11px;font-size:10px;color:var(--dim);margin-top:3px}
.key i{display:inline-block;width:9px;height:2px;vertical-align:middle;margin-right:3px}
.gain{border-top:1px solid var(--ln);margin-top:9px;padding-top:9px}
.gain:first-of-type{border:0;margin:0;padding:0}
.gl{display:grid;grid-template-columns:26px 1fr 76px;gap:7px;align-items:center;margin-bottom:5px}
.gl label{color:var(--dim);font-size:11px}
.mx{display:grid;grid-template-columns:1fr 1fr;gap:5px;margin-top:8px}
.mo{background:#12161c;border:1px solid var(--ln);padding:5px 7px;font-size:11px}
.mo b{float:right;font-variant-numeric:tabular-nums}
.bar{height:3px;background:#232a34;margin-top:4px}
.bar i{display:block;height:100%;background:var(--ok)}
.mt{width:100%;border-collapse:collapse;font-size:11px;margin-top:6px}
.mt th{color:var(--dim);font-weight:500;text-align:right;padding:3px 4px;
border-bottom:1px solid var(--ln);font-size:10px;letter-spacing:.08em;text-transform:uppercase}
.mt th:first-child,.mt td:first-child{text-align:left}
.mt td{padding:3px 4px;text-align:right;font-variant-numeric:tabular-nums;border-bottom:1px solid #1e242c}
.st{display:grid;grid-template-columns:1fr 1fr;gap:1px 14px;font-size:11px}
.st span{color:var(--dim)}
.st b{float:right;color:var(--fg);font-variant-numeric:tabular-nums}
.warn{background:#2a1e1a;border:1px solid #5a3630;color:var(--warn);
padding:8px;font-size:11px;line-height:1.45;margin-bottom:10px}
.warn.fly{background:#2e1a18;border-color:#7a3a30;color:#f0a08f}
.thr{width:100%;height:34px;accent-color:var(--hot)}
.big{font-size:26px;font-weight:700;font-variant-numeric:tabular-nums;text-align:center;
padding:4px 0 2px}
.fault{background:#3a1c18;border:1px solid var(--hot);color:var(--hot);padding:7px;
font-size:12px;margin-bottom:8px;display:none}
.fault.on{display:block}
textarea{width:100%;height:150px;background:#0e1116;color:var(--dim);border:1px solid var(--ln);
font:11px/1.35 ui-monospace,SFMono-Regular,Menlo,monospace;padding:7px;resize:vertical}
</style></head><body>
<h1><span>LiteWing PID</span><a href="/">&larr; bench</a></h1>

<div class=warn id=banner><b>Propellers off. Gimbal mounted.</b> This closes a feedback
loop from the IMU onto the motors. A wrong gain oscillates the frame hard and
an unrestrained airframe will flip itself off the bench. Mount it in a two-axis
gimbal that lets it rotate but not translate, and keep a hand near STOP.</div>

<div class=p><h2>Mode</h2>
<div class=g>
<button id=fm0 onclick=setFlight(0)>Bench / gimbal</button>
<button id=fm1 onclick=setFlight(1)>Flight / hover</button>
</div>
<div class=cap>Flight mode raises the throttle ceiling to 90%, enables the yaw
rate damper, holds the integrators down until throttle passes the takeoff
threshold, and arms the tilt cutoff. Switching modes always stops the loop.</div>
</div>

<div class=p>
<div class=fault id=fault></div>
<button class=stopb onclick=estop()>STOP</button>
<button class=runb id=runb onclick=toggleRun()>START LOOP</button>
<div class=st style=margin-top:10px>
<span>loop <b id=stRun>--</b></span><span>rate <b id=stHz>--</b></span>
<span>IMU <b id=stImu>--</b></span><span>battery <b id=stV>--</b></span>
<span>yaw rate <b id=stYaw>--</b></span><span>tilt <b id=stTilt>--</b></span>
</div>
</div>

<div class=p id=thrPanel style=display:none><h2>Throttle</h2>
<div class=big id=thrBig>0%</div>
<input type=range min=0 max=90 value=0 id=thrR class=thr oninput=setThr(this.value)>
<div class=row style=margin-top:4px><span>commanded</span><b id=thrCmd>0%</b></div>
<div class=row><span>applied (ramped)</span><b id=thrNow>0.0%</b></div>
<div class=g3 style=margin-top:8px>
<button onclick=thrStep(-5)>&minus;5%</button>
<button onclick=thrStep(5)>+5%</button>
<button onclick=setThr(0)>Idle</button>
</div>
<div class=cap>Throttle ramps at 45%/s rather than stepping, so the frame does
not jump. Hover on a charged pack is usually 40&ndash;55%. Raise in small
increments until it gets light on the bench, then commit.</div>
</div>

<div class=p><h2>Axis</h2>
<div class=g4>
<button id=md0 onclick=setMode(0)>Off</button>
<button id=md1 onclick=setMode(1)>Roll</button>
<button id=md2 onclick=setMode(2)>Pitch</button>
<button id=md3 onclick=setMode(3)>Both</button>
</div>
<div class=cap>Tune one axis at a time. Lock the other in the gimbal, get roll
stable, then repeat for pitch. Both together only once each is settled.</div>
</div>

<div class=p id=basePanel><h2>Base throttle</h2>
<div class=row><span>all four motors</span><b id=baseV>0%</b></div>
<input type=range min=0 max=60 value=0 id=baseR oninput=setBase(this.value)>
<div class=cap>The stabiliser trims around this. At 0% it can only ever
subtract, so it cannot correct &mdash; raise it until the motors just spin up,
typically 12&ndash;20%, then tune.</div>
</div>

<div class=p><h2>Gains</h2>
<div class=gain id=gr>
<div class=row><span style=color:var(--dim)>roll</span><b id=eR>--</b></div>
<div class=gl><label>Kp</label><input type=range min=0 max=4 step=0.01 id=kp0r oninput=gain(0,'kp',this.value)><input type=number step=0.01 id=kp0 onchange=gain(0,'kp',this.value)></div>
<div class=gl><label>Ki</label><input type=range min=0 max=2 step=0.005 id=ki0r oninput=gain(0,'ki',this.value)><input type=number step=0.005 id=ki0 onchange=gain(0,'ki',this.value)></div>
<div class=gl><label>Kd</label><input type=range min=0 max=0.5 step=0.001 id=kd0r oninput=gain(0,'kd',this.value)><input type=number step=0.001 id=kd0 onchange=gain(0,'kd',this.value)></div>
</div>
<div class=gain id=gp>
<div class=row><span style=color:var(--dim)>pitch</span><b id=eP>--</b></div>
<div class=gl><label>Kp</label><input type=range min=0 max=4 step=0.01 id=kp1r oninput=gain(1,'kp',this.value)><input type=number step=0.01 id=kp1 onchange=gain(1,'kp',this.value)></div>
<div class=gl><label>Ki</label><input type=range min=0 max=2 step=0.005 id=ki1r oninput=gain(1,'ki',this.value)><input type=number step=0.005 id=ki1 onchange=gain(1,'ki',this.value)></div>
<div class=gl><label>Kd</label><input type=range min=0 max=0.5 step=0.001 id=kd1r oninput=gain(1,'kd',this.value)><input type=number step=0.001 id=kd1 onchange=gain(1,'kd',this.value)></div>
</div>
<div class=g3 style=margin-top:10px>
<button onclick=copyAxis()>Roll &rarr; pitch</button>
<button onclick=zeroGains()>Zero all</button>
<button onclick=saveGains()>Save to NVS</button>
</div>
<div class=g style=margin-top:6px>
<button onclick=resetI()>Reset I</button>
<button onclick=invert()>Invert axis</button>
</div>
<div class=gain id=gy style=display:none>
<div class=row><span style=color:var(--dim)>yaw rate damper</span><b id=eY>--</b></div>
<div class=gl><label>Kp</label><input type=range min=0 max=1.5 step=0.01 id=ykpr oninput=yawGain(this.value)><input type=number step=0.01 id=ykp onchange=yawGain(this.value)></div>
<div class=cap style=margin-top:4px>Damps rotation only. It will not hold a
heading &mdash; nothing fuses the magnetometer, so slow yaw drift is expected
and normal.</div>
</div>
<div id=saved class=cap></div>
</div>

<div class=p><h2>Setpoint</h2>
<div class=row><span id=spAx>roll target</span><b id=spV>0.0&deg;</b></div>
<input type=range min=-20 max=20 step=0.5 value=0 id=spR oninput=setSp(this.value)>
<div class=g3 style=margin-top:8px>
<button onclick=level()>Level (0&deg;)</button>
<button onclick=step(10)>Step +10&deg;</button>
<button onclick=step(-10)>Step -10&deg;</button>
</div>
<div class=cap>A step command is how you read the tune. Watch the response
curve below: overshoot means Kp or Ki is high, a slow crawl means Kp is low, a
fuzzy trace means Kd is high and amplifying gyro noise.</div>
</div>

<div class=p><h2>Response</h2>
<div class=ch><h3><span>setpoint vs angle, deg</span><em id=cvA>--</em></h3>
<canvas id=cA></canvas>
<div class=key><span><i style=background:var(--sp)></i>setpoint</span>
<span><i style=background:var(--ok)></i>angle</span></div></div>

<div class=ch><h3><span>term contributions, % output</span><em id=cvT>--</em></h3>
<canvas id=cT></canvas>
<div class=key><span><i style=background:var(--kp)></i>P</span>
<span><i style=background:var(--ki)></i>I</span>
<span><i style=background:var(--kd)></i>D</span>
<span><i style=background:#d7dee8></i>total</span></div></div>

<div class=ch><h3><span>motor duty %</span><em id=cvM>--</em></h3>
<canvas id=cM></canvas>
<div class=key><span><i style=background:#4fd08a></i>M1</span><span><i style=background:#5aa9e0></i>M2</span>
<span><i style=background:#e0b341></i>M3</span><span><i style=background:#c77ae0></i>M4</span></div></div>

<div class=g3>
<button onclick=setWin(10)>10 s</button>
<button onclick=setWin(30)>30 s</button>
<button onclick=setWin(0)>All</button>
</div>
</div>

<div class=p><h2>Step response <span id=stepSt style="float:right;color:var(--dim);
text-transform:none;letter-spacing:0">idle</span></h2>
<div class=st>
<span>rise 10-90% <b id=mRise>--</b></span><span>overshoot <b id=mOvr>--</b></span>
<span>settling &plusmn;2% <b id=mSet>--</b></span><span>steady error <b id=mSse>--</b></span>
<span>peak <b id=mPk>--</b></span><span>RMS error <b id=mRms>--</b></span>
</div>
<div class=cap>Measured from the most recent step command. Rise, settling and
peak are read off the logged response; steady error is the mean of the last
second. A tune is good when overshoot is under about 15%, settling is under a
second, and steady error is near zero.</div>
<table class=mt><thead><tr><th>t</th><th>axis</th><th>step</th><th>rise</th>
<th>over</th><th>settle</th></tr></thead>
<tbody id=hist><tr><td colspan=6 style=color:var(--dim)>no steps yet</td></tr></tbody></table>
<div class=g style=margin-top:8px>
<button onclick=exportCsv()>Export CSV</button>
<button onclick=clearLog()>Clear trace</button>
</div>
</div>

<div class=p><h2>Shareable log</h2>
<div class=cap style=margin:0>A compact summary plus a downsampled trace, small
enough to paste into a chat. Includes the gains, the flight statistics and an
oscillation estimate &mdash; everything needed to diagnose a tune.</div>
<div class=g style="margin:8px 0">
<button onclick=makeLog()>Generate</button>
<button onclick=copyLog()>Copy</button>
</div>
<textarea id=logOut readonly placeholder="press Generate after a flight"></textarea>
<div class=cap id=logMsg></div>
</div>

<script>
const S={run:false,mode:0,base:0,ax:0,win:30,flight:0,yawkp:0,tilt:60,
 kp:[0,0],ki:[0,0],kd:[0,0],inv:[0,0],sp:[0,0]};
const LOG={t0:Date.now(),s:[],steps:[],cur:null,hist:[]};
const CAP=30000;

async function j(u){try{const r=await fetch(u);return await r.json()}catch(e){return null}}
function now(){return (Date.now()-LOG.t0)/1000}
function mmss(t){const m=Math.floor(t/60),s=t-m*60;return m+':'+(s<10?'0':'')+s.toFixed(1)}

/* --------------------------------------------------------------- config -- */
async function loadCfg(){
  const c=await j('/api/pid');if(!c)return;
  S.kp=c.kp;S.ki=c.ki;S.kd=c.kd;S.inv=c.inv;S.base=c.base;S.mode=c.mode;
  for(let a=0;a<2;a++){
    document.getElementById('kp'+a).value=c.kp[a];document.getElementById('kp'+a+'r').value=c.kp[a];
    document.getElementById('ki'+a).value=c.ki[a];document.getElementById('ki'+a+'r').value=c.ki[a];
    document.getElementById('kd'+a).value=c.kd[a];document.getElementById('kd'+a+'r').value=c.kd[a];
  }
  baseR.value=c.base;baseV.textContent=c.base+'%';
  S.flight=c.flight;S.yawkp=c.yawkp;S.tilt=c.tilt;
  ykp.value=c.yawkp;ykpr.value=c.yawkp;
  paintMode();paintFlight();
}
function gain(a,k,v){
  v=parseFloat(v)||0;S[k][a]=v;
  document.getElementById(k+a).value=v;document.getElementById(k+a+'r').value=v;
  j('/api/pidset?'+k+a+'='+v);
}
function setBase(v){S.base=+v;baseV.textContent=v+'%';j('/api/pidset?base='+v)}
function setFlight(f){S.flight=f;paintFlight();j('/api/pidset?flight='+f)}
function paintFlight(){
  fm0.className=S.flight?'':'sel';fm1.className=S.flight?'sel':'';
  thrPanel.style.display=S.flight?'':'none';
  basePanel.style.display=S.flight?'none':'';   // same value, one control each mode
  gy.style.display=S.flight?'':'none';
  document.getElementById('banner').className=S.flight?'warn fly':'warn';
  document.getElementById('banner').innerHTML=S.flight?
    '<b>Flight mode. Propellers on, clear space, nothing above you.</b> The '+
    'link is WiFi to a browser: if it drops for 1.5 s the firmware cuts the '+
    'motors and the drone falls. Fly low over something soft. The tilt cutoff '+
    'disarms past '+S.tilt+'&deg;.'
    :
    '<b>Propellers off. Gimbal mounted.</b> This closes a feedback loop from '+
    'the IMU onto the motors. A wrong gain oscillates the frame hard and an '+
    'unrestrained airframe will flip itself off the bench. Mount it in a '+
    'two-axis gimbal that lets it rotate but not translate, and keep a hand '+
    'near STOP.';
}
function setThr(v){v=Math.max(0,Math.min(90,+v));thrR.value=v;
  thrBig.textContent=v+'%';thrCmd.textContent=v+'%';j('/api/pidset?base='+v)}
function thrStep(d){setThr((+thrR.value)+d)}
function yawGain(v){v=parseFloat(v)||0;S.yawkp=v;ykp.value=v;ykpr.value=v;
  j('/api/pidset?yawkp='+v)}
function setMode(m){
  S.mode=m;S.ax=(m===2)?1:0;paintMode();
  j('/api/pidset?mode='+m);
}
function paintMode(){
  for(let i=0;i<4;i++)document.getElementById('md'+i).className=(S.mode===i)?'sel':'';
  gr.style.opacity=(S.mode===1||S.mode===3)?1:.35;
  gp.style.opacity=(S.mode===2||S.mode===3)?1:.35;
  spAx.textContent=(S.ax?'pitch':'roll')+' target';
}
function setSp(v){
  S.sp[S.ax]=+v;spV.textContent=(+v).toFixed(1)+String.fromCharCode(176);
  j('/api/pidset?sp'+S.ax+'='+v);
}
function level(){spR.value=0;setSp(0);beginStep(0)}
function step(d){const v=+d;spR.value=v;setSp(v);beginStep(v)}
function copyAxis(){for(const k of['kp','ki','kd'])gain(1,k,S[k][0])}
function zeroGains(){for(let a=0;a<2;a++)for(const k of['kp','ki','kd'])gain(a,k,0)}
function resetI(){j('/api/pidset?reseti=1')}
function invert(){S.inv[S.ax]=S.inv[S.ax]?0:1;j('/api/pidset?inv'+S.ax+'='+S.inv[S.ax]);
  saved.textContent='axis '+(S.ax?'pitch':'roll')+' inverted ('+(S.inv[S.ax]?'on':'off')+')'}
async function saveGains(){await j('/api/pidsave');
  saved.textContent='gains saved to NVS at '+new Date().toLocaleTimeString()}
function toggleRun(){S.run=!S.run;j('/api/pidset?run='+(S.run?1:0))}
function estop(){S.run=false;thrR.value=0;thrBig.textContent='0%';
  thrCmd.textContent='0%';j('/api/stop')}
function setWin(w){S.win=w}

/* ------------------------------------------------------ step measurement -- */
function beginStep(target){
  if(LOG.cur)finishStep();
  LOG.steps.push(now());
  stepSt.textContent='measuring';
  const a=S.ax,last=LOG.s.length?LOG.s[LOG.s.length-1]:null;
  LOG.cur={t0:now(),ax:a,from:last?last.ang[a]:0,to:target,peak:null,
           rise:null,settle:null,r0:null,r1:null};
}
function finishStep(){
  const c=LOG.cur;if(!c)return;
  metrics(true);                   // final pass while cur is still populated
  LOG.cur=null;
  stepSt.textContent='final';
  const rows=LOG.s.filter(r=>r.t>=c.t0);
  if(rows.length<8)return;
  LOG.hist.unshift({t:c.t0,ax:c.ax,to:c.to,rise:c.rise,over:c.over,settle:c.settle});
  if(LOG.hist.length>12)LOG.hist.pop();
  paintHist();
}
function paintHist(){
  const b=document.getElementById('hist');
  if(!LOG.hist.length){b.innerHTML='<tr><td colspan=6 style=color:var(--dim)>no steps yet</td></tr>';return}
  let h='';
  for(const s of LOG.hist)h+='<tr><td>'+mmss(s.t)+'</td><td>'+(s.ax?'pitch':'roll')+'</td>'+
    '<td>'+s.to.toFixed(0)+String.fromCharCode(176)+'</td>'+
    '<td>'+(s.rise!=null?s.rise.toFixed(2)+'s':'--')+'</td>'+
    '<td>'+(s.over!=null?s.over.toFixed(0)+'%':'--')+'</td>'+
    '<td>'+(s.settle!=null?s.settle.toFixed(2)+'s':'--')+'</td></tr>';
  b.innerHTML=h;
}
/* live metrics for the step in progress */
function metrics(fin){
  const c=LOG.cur;
  if(!c){return}
  const a=c.ax,span=c.to-c.from;
  const rows=LOG.s.filter(r=>r.t>=c.t0);
  if(rows.length<4||Math.abs(span)<0.5){return}

  let peak=c.from,pt=c.t0;
  for(const r of rows){const v=r.ang[a];
    if(Math.abs(v-c.from)>Math.abs(peak-c.from)){peak=v;pt=r.t}}
  c.peak=peak;
  c.over=Math.max(0,((peak-c.to)/span)*100);

  const lo=c.from+span*0.1, hi=c.from+span*0.9;
  let t10=null,t90=null;
  for(const r of rows){const v=r.ang[a];
    if(t10==null&&(span>0?v>=lo:v<=lo))t10=r.t;
    if(t10!=null&&t90==null&&(span>0?v>=hi:v<=hi))t90=r.t}
  c.rise=(t10!=null&&t90!=null)?(t90-t10):null;

  const band=Math.max(0.4,Math.abs(span)*0.02);
  let settle=null;
  for(let i=rows.length-1;i>=0;i--){
    if(Math.abs(rows[i].ang[a]-c.to)>band){settle=(i+1<rows.length)?rows[i+1].t-c.t0:null;break}
    if(i===0)settle=0;
  }
  c.settle=settle;

  const tail=rows.filter(r=>r.t>rows[rows.length-1].t-1.0);
  c.sse=tail.length?tail.reduce((s,r)=>s+(c.to-r.ang[a]),0)/tail.length:null;
  let sq=0;for(const r of rows)sq+=Math.pow(c.to-r.ang[a],2);
  c.rms=Math.sqrt(sq/rows.length);

  const D=String.fromCharCode(176);
  mRise.textContent=c.rise!=null?c.rise.toFixed(2)+' s':
    (fin?'never reached 90%':'rising');
  mOvr.textContent=c.over!=null?c.over.toFixed(0)+' %':'--';
  mSet.textContent=c.settle!=null?c.settle.toFixed(2)+' s':
    (fin?'never settled':'settling');
  mSse.textContent=c.sse!=null?c.sse.toFixed(2)+D:'--';
  mPk.textContent=peak.toFixed(1)+D;
  mRms.textContent=c.rms.toFixed(2)+D;
}

/* ---------------------------------------------------------------- plots -- */
let hoverT=null;
function view(){
  if(!LOG.s.length)return[];
  if(!S.win)return LOG.s;
  const cut=LOG.s[LOG.s.length-1].t-S.win;
  let i=LOG.s.length-1;while(i>0&&LOG.s[i-1].t>=cut)i--;
  return LOG.s.slice(i);
}
function plot(id,rows,lines,opts){
  const cv=document.getElementById(id),dpr=devicePixelRatio||1;
  const w=cv.clientWidth,h=cv.clientHeight;
  if(cv.width!==Math.round(w*dpr)){cv.width=Math.round(w*dpr);cv.height=Math.round(h*dpr)}
  const x=cv.getContext('2d');x.setTransform(dpr,0,0,dpr,0,0);
  x.clearRect(0,0,w,h);x.fillStyle='#12161c';x.fillRect(0,0,w,h);
  if(rows.length<2){x.fillStyle='#7d8998';x.font='11px ui-monospace,monospace';
    x.fillText('start the loop to see a trace',8,h/2);return}

  const PL=42,PR=4,PT=6,PB=13,gw=w-PL-PR,gh=h-PT-PB;
  const t0=rows[0].t,t1=rows[rows.length-1].t,dt=(t1-t0)||1;
  const sk=Math.max(1,Math.ceil(rows.length/900));
  let lo=opts.lo,hi=opts.hi;
  if(lo==null){lo=Infinity;hi=-Infinity;
    for(let i=0;i<rows.length;i+=sk)for(const L of lines){
      const v=L.f(rows[i]);if(v<lo)lo=v;if(v>hi)hi=v}}
  if(!isFinite(lo)){lo=0;hi=1}
  if(hi-lo<opts.min){const c=(hi+lo)/2;lo=c-opts.min/2;hi=c+opts.min/2}
  if(opts.lo==null){const pad=(hi-lo)*0.12;lo-=pad;hi+=pad}
  const X=t=>PL+((t-t0)/dt)*gw, Y=v=>PT+gh-((v-lo)/(hi-lo))*gh;

  x.strokeStyle='#232a34';x.lineWidth=1;x.fillStyle='#7d8998';
  x.font='9px ui-monospace,monospace';x.textAlign='right';
  for(let k=0;k<=3;k++){const v=lo+(hi-lo)*k/3,y=Math.round(Y(v))+.5;
    x.beginPath();x.moveTo(PL,y);x.lineTo(w-PR,y);x.stroke();
    x.fillText(v.toFixed(opts.dp),PL-5,y+3)}
  if(lo<0&&hi>0){const y=Math.round(Y(0))+.5;x.strokeStyle='#36404d';
    x.beginPath();x.moveTo(PL,y);x.lineTo(w-PR,y);x.stroke()}

  for(const s of LOG.steps){if(s<t0||s>t1)continue;
    const px=Math.round(X(s))+.5;x.strokeStyle='rgba(224,179,65,.5)';
    x.beginPath();x.moveTo(px,PT);x.lineTo(px,PT+gh);x.stroke()}

  for(const L of lines){
    x.strokeStyle=L.c;x.lineWidth=L.w||1.5;x.lineJoin='round';
    if(L.dash)x.setLineDash([4,3]);
    x.beginPath();let st=false;
    for(let i=0;i<rows.length;i+=sk){const v=L.f(rows[i]);if(v==null)continue;
      const px=X(rows[i].t),py=Y(v);
      if(!st){x.moveTo(px,py);st=true}else x.lineTo(px,py)}
    const last=rows[rows.length-1],lv=L.f(last);
    if(st&&lv!=null)x.lineTo(X(last.t),Y(lv));
    x.stroke();x.setLineDash([]);
  }
  x.textAlign='left';x.fillStyle='#7d8998';x.fillText(mmss(t0),PL,h-3);
  x.textAlign='right';x.fillText(mmss(t1),w-PR,h-3);
  if(hoverT!=null&&hoverT>=t0&&hoverT<=t1){
    const px=Math.round(X(hoverT))+.5;x.strokeStyle='#7d8998';x.setLineDash([2,3]);
    x.beginPath();x.moveTo(px,PT);x.lineTo(px,PT+gh);x.stroke();x.setLineDash([])}
}
for(const id of ['cA','cT','cM']){
  const cv=document.getElementById(id);
  cv.addEventListener('mousemove',e=>{const rows=view();if(rows.length<2)return;
    const b=cv.getBoundingClientRect(),PL=42,PR=4;
    const f=(e.clientX-b.left-PL)/(b.width-PL-PR);
    hoverT=rows[0].t+f*(rows[rows.length-1].t-rows[0].t)});
  cv.addEventListener('mouseleave',()=>{hoverT=null});
}
function nearest(t){const r=view();if(!r.length)return null;
  let best=r[0],bd=1e9;for(const s of r){const d=Math.abs(s.t-t);if(d<bd){bd=d;best=s}}return best}

function redraw(){
  const rows=view(),a=S.ax,D=String.fromCharCode(176);
  plot('cA',rows,[{c:'#8b96a5',f:r=>r.sp[a],dash:1,w:1.2},
                  {c:'#4fd08a',f:r=>r.ang[a]}],{min:6,dp:1});
  plot('cT',rows,[{c:'#4fd08a',f:r=>r.p[a]},{c:'#e0b341',f:r=>r.i[a]},
                  {c:'#5aa9e0',f:r=>r.d[a]},{c:'#d7dee8',f:r=>r.o[a],w:1.2}],{min:4,dp:0});
  plot('cM',rows,[{c:'#4fd08a',f:r=>r.duty[0]},{c:'#5aa9e0',f:r=>r.duty[1]},
                  {c:'#e0b341',f:r=>r.duty[2]},{c:'#c77ae0',f:r=>r.duty[3]}],
                  {lo:0,hi:100,min:1,dp:0});
  const h=hoverT!=null?nearest(hoverT):(LOG.s.length?LOG.s[LOG.s.length-1]:null);
  if(h){
    cvA.textContent=h.sp[a].toFixed(1)+' / '+h.ang[a].toFixed(1)+D;
    cvT.textContent='P '+h.p[a].toFixed(1)+'  I '+h.i[a].toFixed(1)+'  D '+h.d[a].toFixed(1);
    cvM.textContent=h.duty.map(v=>v+'%').join(' ');
  }
  metrics();
}
setInterval(redraw,120);

/* ----------------------------------------------------------------- csv --- */
function exportCsv(){
  if(!LOG.s.length){alert('no trace recorded yet');return}
  let c='t_s,axis,setpoint_deg,angle_deg,error_deg,rate_dps,p_pct,i_pct,d_pct,out_pct,'+
        'm1,m2,m3,m4,vbat_v\n';
  const a=S.ax;
  for(const r of LOG.s)c+=[r.t.toFixed(3),a?'pitch':'roll',r.sp[a].toFixed(2),
    r.ang[a].toFixed(3),(r.sp[a]-r.ang[a]).toFixed(3),r.rate[a].toFixed(2),
    r.p[a].toFixed(3),r.i[a].toFixed(3),r.d[a].toFixed(3),r.o[a].toFixed(3),
    r.duty[0],r.duty[1],r.duty[2],r.duty[3],r.vb.toFixed(3)].join(',')+'\n';
  const el=document.createElement('a');
  el.href=URL.createObjectURL(new Blob([c],{type:'text/csv'}));
  el.download='litewing_pid_'+(a?'pitch':'roll')+'.csv';el.click();
  setTimeout(()=>URL.revokeObjectURL(el.href),4000);
}
function clearLog(){LOG.s=[];LOG.steps=[];LOG.cur=null;LOG.t0=Date.now()}

/* ------------------------------------------------------- shareable log --- */
function stats(f,rows){
  let mn=1e9,mx=-1e9,sum=0,sq=0;
  for(const r of rows){const v=f(r);if(v<mn)mn=v;if(v>mx)mx=v;sum+=v;sq+=v*v}
  const n=rows.length||1;
  return{min:mn,max:mx,mean:sum/n,rms:Math.sqrt(sq/n)};
}
/* dominant oscillation frequency from mean-crossings of the error signal */
function oscHz(f,rows){
  if(rows.length<8)return 0;
  const m=stats(f,rows).mean;
  let cross=0,prev=f(rows[0])-m;
  for(let i=1;i<rows.length;i++){const v=f(rows[i])-m;
    if((prev<0&&v>=0)||(prev>0&&v<=0))cross++;prev=v}
  const dur=rows[rows.length-1].t-rows[0].t;
  return dur>0?(cross/2)/dur:0;
}
function makeLog(){
  const rows=LOG.s;
  if(rows.length<5){logOut.value='';logMsg.textContent='not enough data - fly first, then Generate';return}
  const D=String.fromCharCode(176),n=rows.length;
  const dur=rows[n-1].t-rows[0].t;
  const eR=r=>r.sp[0]-r.ang[0], eP=r=>r.sp[1]-r.ang[1];
  const sR=stats(r=>r.ang[0],rows), sP=stats(r=>r.ang[1],rows);
  const xR=stats(eR,rows), xP=stats(eP,rows);
  const vb=stats(r=>r.vb,rows), th=stats(r=>r.thr,rows);
  const MODE=['off','roll','pitch','both'][S.mode];
  const f=v=>v.toFixed(2);

  let o='=== LiteWing PID log ===\n';
  o+='mode '+(S.flight?'FLIGHT':'bench')+'   axis '+MODE+'   samples '+n+
     '   duration '+dur.toFixed(1)+'s\n';
  o+='throttle cmd '+S.base+'%  applied mean '+f(th.mean)+'%  max '+f(th.max)+'%\n';
  o+='vbat '+f(vb.max)+' -> '+f(vb.min)+' V\n';
  o+='roll  Kp '+S.kp[0]+'  Ki '+S.ki[0]+'  Kd '+S.kd[0]+'  inv '+S.inv[0]+'\n';
  o+='pitch Kp '+S.kp[1]+'  Ki '+S.ki[1]+'  Kd '+S.kd[1]+'  inv '+S.inv[1]+'\n';
  o+='yaw damper Kp '+S.yawkp+'   tilt cutoff '+S.tilt+D+'\n';
  o+='--- angle stats (deg) ---\n';
  o+='roll  mean '+f(sR.mean)+'  min '+f(sR.min)+'  max '+f(sR.max)+
     '  err rms '+f(xR.rms)+'  osc '+oscHz(eR,rows).toFixed(2)+' Hz\n';
  o+='pitch mean '+f(sP.mean)+'  min '+f(sP.min)+'  max '+f(sP.max)+
     '  err rms '+f(xP.rms)+'  osc '+oscHz(eP,rows).toFixed(2)+' Hz\n';
  const last=rows[n-1];
  o+='motors last '+last.duty.join('/')+'   yaw out '+f(last.yaw)+'%\n';
  o+='--- trace (downsampled) ---\n';
  o+='t,roll,pitch,errR,errP,Pr,Ir,Dr,Pp,Ip,Dp,m1,m2,m3,m4,thr,vbat\n';
  const step=Math.max(1,Math.ceil(n/60));
  for(let i=0;i<n;i+=step){const r=rows[i];
    o+=[r.t.toFixed(2),r.ang[0].toFixed(1),r.ang[1].toFixed(1),
        eR(r).toFixed(1),eP(r).toFixed(1),
        r.p[0].toFixed(1),r.i[0].toFixed(1),r.d[0].toFixed(1),
        r.p[1].toFixed(1),r.i[1].toFixed(1),r.d[1].toFixed(1),
        r.duty[0],r.duty[1],r.duty[2],r.duty[3],
        r.thr.toFixed(0),r.vb.toFixed(2)].join(',')+'\n';
  }
  logOut.value=o;
  logMsg.textContent=o.length+' characters, '+Math.ceil(n/step)+' rows - safe to paste';
}
function copyLog(){
  if(!logOut.value){makeLog();if(!logOut.value)return}
  logOut.select();logOut.setSelectionRange(0,999999);
  let ok=false;
  try{ok=document.execCommand('copy')}catch(e){}
  if(navigator.clipboard&&navigator.clipboard.writeText){
    navigator.clipboard.writeText(logOut.value).then(
      ()=>{logMsg.textContent='copied to clipboard'},
      ()=>{if(!ok)logMsg.textContent='could not copy - select the text and copy manually'});
  } else logMsg.textContent=ok?'copied to clipboard':
    'could not copy - select the text and copy manually';
}

/* ---------------------------------------------------------------- poll --- */
async function tick(){
  const s=await j('/api/pidstate');
  if(!s){stRun.textContent='no link';return}
  S.run=s.run;
  runb.textContent=s.run?'STOP LOOP':'START LOOP';
  runb.className=s.run?'runb on':'runb';
  stRun.textContent=s.run?'running':'stopped';
  stHz.textContent=s.run?s.hz+' Hz':'--';
  stImu.textContent=s.imu?'online':'no response';
  stImu.style.color=s.imu?'var(--ok)':'var(--hot)';
  stV.textContent=s.vbat.toFixed(2)+' V';
  stYaw.textContent=s.yrate.toFixed(1)+String.fromCharCode(176)+'/s';
  const tilt=Math.max(Math.abs(s.ang[0]),Math.abs(s.ang[1]));
  stTilt.textContent=tilt.toFixed(0)+String.fromCharCode(176);
  stTilt.style.color=tilt>S.tilt*0.7?'var(--hot)':'var(--fg)';
  thrNow.textContent=s.thr.toFixed(1)+'%';
  eY.textContent='rate '+s.yrate.toFixed(1)+String.fromCharCode(176)+'/s';
  if(s.fault){fault.className='fault on';
    fault.textContent=s.fault===1?
      'CUT OUT: tilt exceeded '+S.tilt+String.fromCharCode(176)+
      ' - the loop stopped and disarmed. Level it, then START LOOP again.':
      'CUT OUT: IMU stopped responding. The loop stopped and disarmed.';
  } else fault.className='fault';
  const D=String.fromCharCode(176);
  eR.textContent='err '+(s.sp[0]-s.ang[0]).toFixed(1)+D;
  eP.textContent='err '+(s.sp[1]-s.ang[1]).toFixed(1)+D;

  if(LOG.s.length<CAP)
    LOG.s.push({t:now(),sp:s.sp,ang:s.ang,p:s.p,i:s.i,d:s.d,o:s.o,
                rate:s.rate,duty:s.duty,vb:s.vbat,yaw:s.yaw,
                yrate:s.yrate,thr:s.thr});
}
async function poll(){await tick();setTimeout(poll,90)}
loadCfg();paintHist();paintFlight();poll();
/* close out a step 4 s after it was issued so the history row lands */
const STEP_WIN=6;               // measurement window, seconds
setInterval(()=>{if(LOG.cur&&now()-LOG.cur.t0>STEP_WIN)finishStep()},250);
</script></body></html>)PIDHTML";

// ---------------------------------------------------------------- routes ---
static void hState() {
  lastCmd = millis();
  float axg=ax/4096.0f, ayg=ay/4096.0f, azg=az/4096.0f;
  String j = "{";
  j += "\"vbat\":" + String(vbat(),3);
  j += ",\"imu\":" + String(imuOk?"true":"false");
  j += ",\"armed\":" + String(armed?"true":"false");
  j += ",\"a\":[" + String(axg,3) + "," + String(ayg,3) + "," + String(azg,3) + "]";
  j += ",\"g\":[" + String(gx/16.4f-gBias[0],2) + "," + String(gy/16.4f-gBias[1],2) + "," + String(gz/16.4f-gBias[2],2) + "]";
  j += ",\"ar\":[" + String(ax) + "," + String(ay) + "," + String(az) + "]";
  j += ",\"gr\":[" + String(gx) + "," + String(gy) + "," + String(gz) + "]";
  j += ",\"bias\":[" + String(gBias[0],2) + "," + String(gBias[1],2) + "," + String(gBias[2],2) + "]";
  j += ",\"roll\":" + String(roll,2) + ",\"pitch\":" + String(pitch,2);
  j += ",\"aroll\":" + String(aRoll,2) + ",\"apitch\":" + String(aPitch,2);
  j += ",\"temp\":" + String(traw/340.0f+36.53f,1);
  j += ",\"duty\":[" + String(duty[0]) + "," + String(duty[1]) + "," +
                       String(duty[2]) + "," + String(duty[3]) + "]";
  j += ",\"sweep\":" + String(sweepRunning?"true":"false");
  j += ",\"swm\":" + String(sweepRunning ? swMotor+1 : 0);
  j += ",\"t\":" + String(millis());
  j += "}";
  server.send(200, "application/json", j);
}
static void hMotor() {
  lastCmd = millis();
  if (pidRun) pidStop();          // manual control wins; drop the loop first
  int i = server.arg("i").toInt(), v = server.arg("v").toInt();
  if (i>=0 && i<4) { duty[i] = constrain(v,0,100); applyMotors(); }
  server.send(200, "application/json", "{\"ok\":true}");
}
static void hArm() {
  lastCmd = millis();
  armed = server.arg("v").toInt() == 1;
  applyMotors();
  server.send(200, "application/json", "{\"ok\":true}");
}
static void hStop() { pidStop(); server.send(200,"application/json","{\"ok\":true}"); }
static void hCalib(){ allStop(); calibGyro(); server.send(200,"application/json","{\"ok\":true}"); }
static void hSweep(){
  lastCmd = millis();
  sweepStart();
  server.send(200,"application/json","{\"ok\":true}");
}
static void hSweepLog(){
  server.send(200,"application/json",
    "{\"running\":" + String(sweepRunning?"true":"false") +
    ",\"log\":\"" + sweepLog + "\"}");
}
static void hPidCfg() {
  String j = "{";
  j += "\"kp\":[" + String(pid.kp[0],4) + "," + String(pid.kp[1],4) + "]";
  j += ",\"ki\":[" + String(pid.ki[0],4) + "," + String(pid.ki[1],4) + "]";
  j += ",\"kd\":[" + String(pid.kd[0],4) + "," + String(pid.kd[1],4) + "]";
  j += ",\"base\":" + String(pid.base) + ",\"mode\":" + String(pid.mode);
  j += ",\"ilim\":" + String(pid.iLim,2) + ",\"olim\":" + String(pid.oLim,2);
  j += ",\"inv\":[" + String(pid.inv[0]) + "," + String(pid.inv[1]) + "]";
  j += ",\"flight\":" + String(pid.flight);
  j += ",\"yawkp\":" + String(pid.yawKp,3) + ",\"yawlim\":" + String(pid.yawLim,1);
  j += ",\"tilt\":" + String(pid.tiltCut,0) + ",\"thrmin\":" + String(pid.thrMin);
  j += ",\"run\":" + String(pidRun?"true":"false") + "}";
  server.send(200, "application/json", j);
}
static void hPidSet() {
  lastCmd = millis();
  if (server.hasArg("kp0")) pid.kp[0] = server.arg("kp0").toFloat();
  if (server.hasArg("kp1")) pid.kp[1] = server.arg("kp1").toFloat();
  if (server.hasArg("ki0")) pid.ki[0] = server.arg("ki0").toFloat();
  if (server.hasArg("ki1")) pid.ki[1] = server.arg("ki1").toFloat();
  if (server.hasArg("kd0")) pid.kd[0] = server.arg("kd0").toFloat();
  if (server.hasArg("kd1")) pid.kd[1] = server.arg("kd1").toFloat();
  if (server.hasArg("base")) {
    pid.base = constrain(server.arg("base").toInt(), 0, pid.flight ? 90 : 60);
    baseTgt  = pid.base;
    if (!pidRun) baseNow = 0;
  }
  if (server.hasArg("flight")) {
    int f = server.arg("flight").toInt() ? 1 : 0;
    if (f != pid.flight) { pidStop(); pid.flight = f; }   // never switch live
  }
  if (server.hasArg("yawkp"))  pid.yawKp  = constrain(server.arg("yawkp").toFloat(), 0.0f, 2.0f);
  if (server.hasArg("yawlim")) pid.yawLim = constrain(server.arg("yawlim").toFloat(), 0.0f, 30.0f);
  if (server.hasArg("tilt"))   pid.tiltCut= constrain(server.arg("tilt").toFloat(), 20.0f, 89.0f);
  if (server.hasArg("clearf")) pidFault = 0;
  if (server.hasArg("mode")) pid.mode = constrain(server.arg("mode").toInt(), 0, 3);
  if (server.hasArg("ilim")) pid.iLim = constrain(server.arg("ilim").toFloat(), 0.0f, 50.0f);
  if (server.hasArg("olim")) pid.oLim = constrain(server.arg("olim").toFloat(), 0.0f, 50.0f);
  if (server.hasArg("inv0")) pid.inv[0] = server.arg("inv0").toInt() ? 1 : 0;
  if (server.hasArg("inv1")) pid.inv[1] = server.arg("inv1").toInt() ? 1 : 0;
  if (server.hasArg("sp0"))  pidSp[0] = constrain(server.arg("sp0").toFloat(), -30.0f, 30.0f);
  if (server.hasArg("sp1"))  pidSp[1] = constrain(server.arg("sp1").toFloat(), -30.0f, 30.0f);
  if (server.hasArg("reseti")) { iTerm[0] = iTerm[1] = 0; }
  if (server.hasArg("run"))  { server.arg("run").toInt() ? pidStart() : pidStop(); }
  server.send(200, "application/json", "{\"ok\":true}");
}
static void hPidState() {
  lastCmd = millis();
  String j = "{";
  j += "\"run\":" + String(pidRun?"true":"false");
  j += ",\"armed\":" + String(armed?"true":"false");
  j += ",\"ang\":[" + String(roll,3) + "," + String(pitch,3) + "]";
  j += ",\"sp\":[" + String(pidSp[0],2) + "," + String(pidSp[1],2) + "]";
  j += ",\"p\":[" + String(pTerm[0],3) + "," + String(pTerm[1],3) + "]";
  j += ",\"i\":[" + String(iTerm[0],3) + "," + String(iTerm[1],3) + "]";
  j += ",\"d\":[" + String(dTerm[0],3) + "," + String(dTerm[1],3) + "]";
  j += ",\"o\":[" + String(pidOut[0],3) + "," + String(pidOut[1],3) + "]";
  j += ",\"rate\":[" + String(gx/16.4f-gBias[0],2) + "," + String(gy/16.4f-gBias[1],2) + "]";
  j += ",\"duty\":[" + String(duty[0]) + "," + String(duty[1]) + "," +
                       String(duty[2]) + "," + String(duty[3]) + "]";
  j += ",\"vbat\":" + String(vbat(),3);
  j += ",\"imu\":" + String(imuOk?"true":"false");
  j += ",\"yaw\":" + String(yawOut,3);
  j += ",\"yrate\":" + String(gz/16.4f-gBias[2],2);
  j += ",\"thr\":" + String(baseNow,1) + ",\"thrt\":" + String(baseTgt);
  j += ",\"flight\":" + String(pid.flight);
  j += ",\"fault\":" + String(pidFault);
  j += ",\"hz\":" + String(pidDt > 0 ? 1.0f/pidDt : 0.0f, 0);
  j += ",\"t\":" + String(millis()) + "}";
  server.send(200, "application/json", j);
}
static void hPidSave() { pidSave(); server.send(200,"application/json","{\"ok\":true}"); }

static void hScan() {
  String j = "{" + scanBus(Wire,"i2c0") + "," + scanBus(Wire1,"i2c1") + ",\"id\":{";
  Wire.beginTransmission(0x68); bool mpu = (Wire.endTransmission()==0);
  Wire1.beginTransmission(0x29); bool tof = (Wire1.endTransmission()==0);
  Wire.beginTransmission(0x77); bool baro = (Wire.endTransmission()==0);
  Wire.beginTransmission(0x1E); bool mag = (Wire.endTransmission()==0);
  bool flow = (pmwRead(0x00) == 0x49);
  j += "\"MPU6050 imu (0x68)\":" + String(mpu?"true":"false");
  j += ",\"VL53L1X tof (0x29 aux)\":" + String(tof?"true":"false");
  j += ",\"MS5611 baro (0x77)\":" + String(baro?"true":"false");
  j += ",\"HMC5883 mag (0x1e)\":" + String(mag?"true":"false");
  j += ",\"PMW3901 flow (spi)\":" + String(flow?"true":"false");
  j += "}}";
  server.send(200, "application/json", j);
}
static void hPinsGet() {
  String j = "{\"m\":[" + String(cfg.m[0]) + "," + String(cfg.m[1]) + "," +
             String(cfg.m[2]) + "," + String(cfg.m[3]) + "]," +
             "\"vbatPin\":" + String(cfg.vbatPin) + ",\"vbatDiv\":" + String(cfg.vbatDiv,2) + "}";
  server.send(200, "application/json", j);
}
static void hPinsSet() {
  if (server.hasArg("m0")) {
    prefs.begin("bench", false);
    for (int i=0;i<4;i++) prefs.putInt(("m"+String(i)).c_str(), server.arg("m"+String(i)).toInt());
    prefs.putInt("vbat", server.arg("vbat").toInt());
    prefs.putFloat("div", server.arg("div").toFloat());
    prefs.end();
    server.send(200,"application/json","{\"ok\":true}");
    delay(300); ESP.restart();
  }
  hPinsGet();
}

// ---------------------------------------------------------------- setup ----
void setup() {
  Serial.begin(115200);

  prefs.begin("bench", true);
  for (int i=0;i<4;i++) cfg.m[i] = prefs.getInt(("m"+String(i)).c_str(), cfg.m[i]);
  cfg.vbatPin = prefs.getInt("vbat", cfg.vbatPin);
  cfg.vbatDiv = prefs.getFloat("div", cfg.vbatDiv);
  prefs.end();

  for (int i=0;i<4;i++) { pinMode(cfg.m[i], OUTPUT); digitalWrite(cfg.m[i], LOW); pwmAttach(cfg.m[i]); }
  applyMotors();

  analogReadResolution(12);
  analogSetPinAttenuation(cfg.vbatPin, ADC_11db);

  Wire.begin(I2C0_SDA, I2C0_SCL, 400000);
  Wire1.begin(I2C1_SDA, I2C1_SCL, 400000);
  pinMode(SPI_CS, OUTPUT); digitalWrite(SPI_CS, HIGH);
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS);
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

  pidLoad();

  imuOk = imuInit();
  lastImu = micros();
  if (imuOk) calibGyro();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("AP %s  ->  http://%s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/", [](){ server.send_P(200, "text/html", PAGE); });
  server.on("/api/state", hState);
  server.on("/api/motor", hMotor);
  server.on("/api/arm",   hArm);
  server.on("/api/stop",  hStop);
  server.on("/api/calib", hCalib);
  server.on("/api/sweep", hSweep);
  server.on("/api/sweeplog", hSweepLog);
  server.on("/pid", [](){ server.send_P(200, "text/html", PAGE_PID); });
  server.on("/api/pid",      hPidCfg);
  server.on("/api/pidset",   hPidSet);
  server.on("/api/pidstate", hPidState);
  server.on("/api/pidsave",  hPidSave);
  server.on("/api/scan",  hScan);
  server.on("/api/pins",  hPinsSet);
  server.begin();
  lastCmd = millis();
}

void loop() {
  server.handleClient();
  sweepTick();
  imuRead();
  pidTick();
  if (armed && !sweepRunning && millis() - lastCmd > WD_MS) {
    pidStop();                       // UI went quiet -> cut motors and the loop
    Serial.println("watchdog: disarmed");
  }
  delay(2);
}
