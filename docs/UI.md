# The dashboard

Served from the drone at **<http://192.168.4.1>** once you join the
`LiteWing-Bench` access point (password `litewing`).

The whole interface — markup, CSS and JavaScript — is embedded in the sketch as
a `PROGMEM` string and served from a single route. No filesystem image, no CDN,
no build step. It works with no internet connection, which is the point: the
drone is the only thing on the network.

Design is a dark monospace console, laid out in a single column so it reads on
a phone held one-handed next to the bench. Panels are ordered by how often you
need them.

```
+--------------------------------------+
|  LITEWING BENCH                      |
|  Remove propellers before arming.    |
+--------------------------------------+
|  STATUS                              |
|    Battery                    3.87 V |
|    IMU                        online |
|    Armed                        safe |
+--------------------------------------+
|  MOTORS                              |
|    [        ARM        ]             |
|    M1                             0% |
|    [======|--------------]           |
|    [ 30% 1s ]  [ 60% 1s ]            |
|    ... M2, M3, M4 ...                |
|    [ Stop all ]  [ Health sweep ]    |
+--------------------------------------+
|  IMU / SENSORS / PINS                |
+--------------------------------------+
```

The page polls `/api/state` every **120 ms** and repaints Status and IMU from
the response. That poll is also what feeds the comms watchdog — see
[Safety](#safety-behaviour).

---

## Status

Three lines, the ones you glance at before touching anything else.

- **Battery** — pack voltage in volts. Only as accurate as the divider ratio
  configured in Pins.
- **IMU** — `online` in green, or `no response` in red. Reflects whether the
  last I2C read of the MPU6050 succeeded, so it updates live if the sensor
  drops off the bus mid-session.
- **Armed** — `safe` in green, or `ARMED` in red.

---

## Motors

**ARM** is a latching master switch. Nothing spins while disarmed — sliders
still move and still send their values, but duty is forced to zero until armed.
The button turns solid red and reads `DISARM` when live, so the arm state is
readable across the room.

Per motor:

- **Slider**, 0–100% duty, applied continuously as you drag.
- **`30% 1s` / `60% 1s`** — timed pulse. Sets the duty, then returns to zero
  after one second. This is the control to use for a quick "is it alive" check;
  it avoids leaving a motor running because you got distracted.

**Stop all** zeroes every slider and disarms, both in the browser and on the
device.

**Health sweep** is the automated test. It arms motors itself and runs the
sequence described below.

### Reading a health sweep

Output looks like:

```
rest 3.94V
M1 30%:0.041V  50%:0.088V  70%:0.143V
M2 30%:0.039V  50%:0.085V  70%:0.139V
M3 30%:0.002V  50%:0.003V  70%:0.004V
M4 30%:0.044V  50%:0.091V  70%:0.148V
```

Each figure is **battery sag** — how far the pack dropped below the resting
voltage while that motor ran alone at that level.

- **Sag rising with duty**, roughly in line across all four motors: healthy.
  The absolute numbers depend on your pack, its charge state and its internal
  resistance, so compare motors against *each other*, not against a fixed
  threshold.
- **Near-zero sag at every level** — `M3` above: the motor is drawing no
  meaningful current. Either a dead winding, a blown IRLML6344, a broken solder
  joint, or that motor's GPIO is mapped wrong.
- **One motor sagging far more than its peers**: a partially shorted winding or
  a seized bearing forcing stall current. Check whether it is hot to the touch.
- **All four near zero**: almost always a wrong pin map rather than four dead
  motors. Fix the Pins panel first, then re-sweep.
- **Falling sag as duty rises**: not physical. Suspect a loose battery
  connector browning out under load, or thermal cut-back.

Two caveats. The sweep **blocks the web server for about 10 seconds** and the
UI freezes while it runs — if `fetch` times out in the browser, the sweep still
completed on the device. And the resting voltage is captured once at the start,
so on a pack that is already sagging from a long session, later motors will
read slightly higher sag than earlier ones purely from cumulative drain.
Re-sweep on a fresh pack before condemning a motor.

---

## IMU

Live telemetry from the MPU6050, repainting at the 120 ms poll rate.

- **accel g** — X, Y, Z in g. At rest on a flat surface, expect roughly
  `0.00  0.00  1.00`. A missing 1 g on Z means the board is not flat or the
  sensor is not oriented as assumed.
- **gyro dps** — X, Y, Z in degrees per second, bias-corrected. At rest these
  should sit near zero; persistent non-zero values mean the bias calibration is
  stale.
- **roll / pitch** — complementary-filter attitude in degrees. Tilt the drone
  and confirm the numbers track the physical motion in the right sense. There
  is no yaw estimate.
- **die temp** — MPU6050 die temperature in Celsius.

**Calibrate gyro bias** disarms, then averages 400 samples to establish zero
rates and resets the attitude accumulators. Keep the drone completely still and
flat for it — about 1.2 seconds. Calibrating while it moves bakes the motion in
as permanent bias. This also runs automatically at boot.

---

## Sensors

**Scan buses** walks both I2C buses across addresses `0x01`–`0x7E`, lists every
address that ACKs, and then reports the specific parts it expects to find:

```
I2C0 (IMU bus): 0x68 0x77
I2C1 (aux bus): nothing found

[ok]  MPU6050 imu (0x68)
[--]  VL53L1X tof (0x29 aux)
[ok]  MS5611 baro (0x77)
[--]  HMC5883 mag (0x1e)
[ok]  PMW3901 flow (spi)
```

`[ok]` means the part answered. For the I2C sensors that is an address ACK
only, so a part that responds but is internally faulty still reports `[ok]`.
The PMW3901 check is a real product-ID read, so its `[ok]` means more.

`[--]` on a sensor your board should have means a bus, wiring or seating
problem. `nothing found` on an entire bus points at the pin map or a deck that
is not seated.

---

## Pins

Every uncertain GPIO assignment, editable in the browser: the four motor pins,
the battery ADC pin, and the battery divider ratio.

**Save pins and reboot** writes to NVS and restarts the ESP32 — the reboot is
required so the PWM channels re-attach to the new pins. The AP drops for a few
seconds and the browser will need a refresh.

This panel exists because the motor and battery pins in the firmware are
**guesses**. Expect to use it. See
[HARDWARE.md](HARDWARE.md#unverified-pin-map).

---

## Safety behaviour

- **Comms watchdog, 1500 ms.** The 120 ms `/api/state` poll doubles as a
  keepalive. Close the tab, lock the phone, or walk out of WiFi range, and the
  firmware disarms and cuts all four motors within 1.5 seconds. The health
  sweep is exempt while it runs, since it holds motors on deliberately.
- **Arm does not survive a reboot.** Saving pins, or any reset, comes back
  disarmed with all duties at zero.
- **Nothing announces that motors are live.** The buzzer is mapped in the
  firmware but never driven. The red `DISARM` button is the only indication.
