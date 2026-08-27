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
static void healthSweep() {
  sweepRunning = true;
  sweepLog = "";
  float rest = vbat();
  sweepLog += "rest " + String(rest,2) + "V\\n";
  int levels[3] = {30,50,70};
  for (int i=0;i<4;i++) {
    String line = "M" + String(i+1) + " ";
    for (int L=0; L<3; L++) {
      for (int k=0;k<4;k++) duty[k]=0;
      duty[i] = levels[L]; armed = true; applyMotors();
      float lo = 9.9f;
      uint32_t t0 = millis();
      while (millis()-t0 < 700) { float v=vbat(); if (v<lo) lo=v; delay(20); }
      line += String(levels[L]) + "%:" + String(rest-lo,3) + "V  ";
    }
    allStop(); delay(400);
    sweepLog += line + "\\n";
  }
  sweepRunning = false;
}

// ---------------------------------------------------------------- html -----
static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head>
<meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>LiteWing Bench</title><style>
:root{--bg:#0e1116;--pnl:#161b22;--ln:#2a323d;--fg:#d7dee8;--dim:#7d8998;--ok:#4fd08a;--warn:#e0b341;--hot:#e05a4f}
*{box-sizing:border-box}body{margin:0 auto;max-width:560px;background:var(--bg);
color:var(--fg);font:13px/1.45 ui-monospace,SFMono-Regular,Menlo,monospace;padding:10px}
h1{font-size:14px;letter-spacing:.14em;text-transform:uppercase;margin:2px 0 12px;color:var(--dim)}
.p{background:var(--pnl);border:1px solid var(--ln);padding:10px;margin-bottom:10px}
.p>h2{font-size:11px;letter-spacing:.14em;text-transform:uppercase;color:var(--dim);margin:0 0 8px;font-weight:500}
.row{display:flex;justify-content:space-between;padding:2px 0}
.row b{font-weight:600}
button{font:inherit;background:#222a35;color:var(--fg);border:1px solid var(--ln);
padding:7px 12px;cursor:pointer}button:active{background:#2e3846}
button:focus-visible{outline:2px solid var(--ok);outline-offset:1px}
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

/* --- 3d attitude view ------------------------------------------------ */
.scene{height:186px;display:flex;align-items:center;justify-content:center;
perspective:600px;margin:2px 0 8px;overflow:hidden}
.rig{width:150px;height:150px;position:relative;transform-style:preserve-3d;
transform:rotateX(62deg)}
.grid{position:absolute;inset:-30px;border:1px solid var(--ln);transform:translateZ(-40px);
background:
repeating-linear-gradient(0deg,transparent 0 23px,rgba(125,137,152,.15) 23px 24px),
repeating-linear-gradient(90deg,transparent 0 23px,rgba(125,137,152,.15) 23px 24px)}
.drone{position:absolute;inset:0;transform-style:preserve-3d;
transition:transform .08s linear}
.boom{position:absolute;left:50%;top:50%;width:130px;height:6px;
margin:-3px 0 0 -65px;background:#2f3a48;border:1px solid var(--ln);border-radius:3px}
.boom.a{transform:rotate(45deg)}.boom.b{transform:rotate(-45deg)}
.rot{position:absolute;width:42px;height:42px;margin:-21px 0 0 -21px;border-radius:50%;
border:1.5px solid #55606f;background:rgba(85,96,111,.10)}
.rot.f{border-color:var(--ok);background:rgba(79,208,138,.12)}
.r1{left:calc(50% + 46px);top:calc(50% - 46px)}
.r2{left:calc(50% - 46px);top:calc(50% - 46px)}
.r3{left:calc(50% - 46px);top:calc(50% + 46px)}
.r4{left:calc(50% + 46px);top:calc(50% + 46px)}
.hub{position:absolute;left:50%;top:50%;width:36px;height:28px;margin:-14px 0 0 -18px;
background:#394556;border:1px solid var(--ln);border-radius:5px}
.nose{position:absolute;left:50%;top:calc(50% - 46px);margin-left:-6px;width:0;height:0;
border-left:6px solid transparent;border-right:6px solid transparent;
border-bottom:12px solid var(--ok)}
.att{display:flex;gap:18px;justify-content:center;margin-bottom:10px;color:var(--dim)}
.att b{color:var(--fg);font-variant-numeric:tabular-nums}

/* --- raw / filtered table -------------------------------------------- */
.t{width:100%;border-collapse:collapse;font-size:12px}
.t th{color:var(--dim);font-weight:500;text-align:right;padding:0 0 5px;
font-size:10px;letter-spacing:.12em;text-transform:uppercase}
.t th:first-child{text-align:left}
.t td{padding:2px 0;text-align:right;font-variant-numeric:tabular-nums}
.t td:first-child{text-align:left;color:var(--dim)}
.t td.r{color:var(--dim)}
.t tr.sep td{border-top:1px solid var(--ln);padding-top:6px}
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
<button onclick=sweep()>Health sweep</button>
</div>
<pre id=sw style=margin-top:8px></pre>
</div>

<div class=p><h2>Attitude</h2>
<div class=scene><div class=rig>
  <div class=grid></div>
  <div class=drone id=drn>
    <div class="boom a"></div><div class="boom b"></div>
    <div class="rot f r1"></div><div class="rot f r2"></div>
    <div class="rot r3"></div><div class="rot r4"></div>
    <div class=hub></div><div class=nose></div>
  </div>
</div></div>
<div class=att><span>roll <b id=abr>--</b></span><span>pitch <b id=abp>--</b></span></div>
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
<div class=cap>raw accel/gyro are LSB counts; raw
roll/pitch are accelerometer-only. filtered = scaled, bias-corrected,
complementary.</div>
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
let armed=false;
const SGN={r:1,p:-1};   // attitude sign convention, see docs/UI.md
const mots=document.getElementById('mots');
for(let i=0;i<4;i++){mots.insertAdjacentHTML('beforeend',
`<div class=m><div class=row><span>M${i+1}</span><b id=d${i}>0%</b></div>
<input type=range min=0 max=100 value=0 id=s${i} oninput=setM(${i},this.value)>
<div class=g><button onclick=pulse(${i},30)>30% 1s</button><button onclick=pulse(${i},60)>60% 1s</button></div></div>`);}

async function j(u,o){try{const r=await fetch(u,o);return await r.json()}catch(e){return null}}
function setM(i,v){document.getElementById('d'+i).textContent=v+'%';j('/api/motor?i='+i+'&v='+v)}
async function pulse(i,v){document.getElementById('s'+i).value=v;setM(i,v);
setTimeout(()=>{document.getElementById('s'+i).value=0;setM(i,0)},1000)}
function toggleArm(){armed=!armed;j('/api/arm?v='+(armed?1:0))}
function stop(){armed=false;for(let i=0;i<4;i++){document.getElementById('s'+i).value=0;
document.getElementById('d'+i).textContent='0%'}j('/api/stop')}
function calib(){document.getElementById('gbs').textContent='calibrating...';j('/api/calib')}
async function sweep(){document.getElementById('sw').textContent='running...';
const r=await j('/api/sweep');document.getElementById('sw').textContent=r?r.log:'failed'}
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

async function tick(){const s=await j('/api/state');if(!s)return;
vb.textContent=s.vbat.toFixed(2)+' V';
io.innerHTML=s.imu?'<span class=tag>online</span>':'<span class=bad>no response</span>';
ar.innerHTML=s.armed?'<span class=bad>ARMED</span>':'<span class=tag>safe</span>';
armed=s.armed;armb.textContent=s.armed?'DISARM':'ARM';armb.className=s.armed?'arm on':'arm';
const R=['rax','ray','raz'],F=['fax','fay','faz'];
for(let i=0;i<3;i++){document.getElementById(R[i]).textContent=s.ar[i];
document.getElementById(F[i]).textContent=s.a[i].toFixed(3)+' g'}
const RG=['rgx','rgy','rgz'],FG=['fgx','fgy','fgz'];
for(let i=0;i<3;i++){document.getElementById(RG[i]).textContent=s.gr[i];
document.getElementById(FG[i]).textContent=s.g[i].toFixed(2)+String.fromCharCode(176)+'/s'}
const D=String.fromCharCode(176);
rrl.textContent=s.aroll.toFixed(1)+D; frl.textContent=s.roll.toFixed(1)+D;
rpt.textContent=s.apitch.toFixed(1)+D; fpt.textContent=s.pitch.toFixed(1)+D;
gbs.textContent=s.bias.map(v=>v.toFixed(2)).join('  ')+' '+D+'/s';
tp.textContent=s.temp.toFixed(1)+' C';
abr.textContent=s.roll.toFixed(1)+D; abp.textContent=s.pitch.toFixed(1)+D;
// SGN: flip a sign here if the model tilts the wrong way for your board
drn.style.transform='rotateY('+(SGN.r*s.roll)+'deg) rotateX('+(SGN.p*s.pitch)+'deg)'}
loadPins();setInterval(tick,120);tick();
</script></body></html>)HTML";

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
  j += "}";
  server.send(200, "application/json", j);
}
static void hMotor() {
  lastCmd = millis();
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
static void hStop() { allStop(); server.send(200,"application/json","{\"ok\":true}"); }
static void hCalib(){ allStop(); calibGyro(); server.send(200,"application/json","{\"ok\":true}"); }
static void hSweep(){
  healthSweep();
  String l = sweepLog; l.replace("\\n","\\n");
  server.send(200,"application/json","{\"log\":\"" + l + "\"}");
}
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
  server.on("/api/scan",  hScan);
  server.on("/api/pins",  hPinsSet);
  server.begin();
  lastCmd = millis();
}

void loop() {
  server.handleClient();
  imuRead();
  if (armed && !sweepRunning && millis() - lastCmd > WD_MS) {
    allStop();                       // UI went quiet -> cut motors
    Serial.println("watchdog: disarmed");
  }
  delay(2);
}
