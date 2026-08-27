# LiteWing Bench

A hardware test rig for the **LiteWing V1.2** (ESP32-S3) drone. It replaces the
stock flight firmware with a self-hosted diagnostic dashboard, served over the
drone's own WiFi access point, so you can find dead motors, blown MOSFETs and
unresponsive sensors without a flight controller, a transmitter, or a PC in the
loop.

Point a phone at it, and the drone tells you what's broken.

> **Remove the propellers before arming anything.** The motor panel spins
> motors on command and the health sweep spins each one automatically.

---

## What it does

| Panel | Purpose |
|---|---|
| **Status** | Live battery voltage, IMU reachability, arm state |
| **Motors** | Per-motor PWM slider, timed pulses, master arm, health sweep |
| **Attitude** | Live 3D airframe on a level grid, props spinning at real duty — pure CSS |
| **Session log** | Records every sample, plots 4 live charts, per-test battery sag, CSV export |
| **IMU** | Raw *and* filtered accel/gyro/angles side by side, gyro bias calibration |
| **Sensors** | I2C bus scan on both buses + SPI flow sensor identification |
| **Pins** | Edit every GPIO assignment from the browser, persisted to NVS |

The **health sweep** is the reason this exists. It runs each motor alone at 30%,
50% and 70% for 700 ms and records how far the battery voltage sags under each
load. A healthy coreless motor pulls current and drags the pack down a
measurable amount. A motor with a dead winding or a blown IRLML6344 gate driver
draws nothing and produces no sag — it shows up immediately as a near-zero
column in the log.

---

## Quick start

1. **Wire it up.** USB-C to the drone. It enumerates as a CH340K
   (`VID_1A86 & PID_7522`); on Windows it appears as `USB-SERIAL CH340K (COMx)`.
   If no port appears at all, see [Troubleshooting](#troubleshooting).
2. **Install** the Arduino ESP32 core, **version 3.x** ([why](#known-issues)).
3. **Configure the board** exactly as below.
4. **Flash**, then power the drone from its LiPo.
5. Join WiFi **`LiteWing-Bench`**, password **`litewing`**.
6. Open **<http://192.168.4.1>**.

### Arduino IDE settings

| Setting | Value |
|---|---|
| Board | **ESP32S3 Dev Module** |
| USB CDC On Boot | **Disabled** |
| Upload Speed | 921600 (drop to 115200 if uploads fail) |
| Port | the CH340K COM port |

`USB CDC On Boot` **must be Disabled.** LiteWing V1.2 bridges USB to UART0
through a CH340K rather than using the ESP32-S3's native USB peripheral.
Enabling CDC routes `Serial` to the native USB pins, which are not connected to
the USB-C jack — the serial monitor would stay silent.

If an upload fails with `Failed to connect to ESP32-S3`, hold **BOOT**, tap
**RESET**, release **BOOT**, and upload again. Not every CH340 board has the
auto-reset transistors populated.

---

## Documentation

- **[docs/HARDWARE.md](docs/HARDWARE.md)** — pin map, sensor addresses, motor
  drive topology, battery sensing, verified against the vendor schematic
- **[docs/UI.md](docs/UI.md)** — every panel, control and readout, and how to
  interpret a health sweep
- **[docs/API.md](docs/API.md)** — the HTTP endpoints, for scripting the rig
  from `curl` or Python instead of the browser

---

## Safety

This firmware will spin motors on a web request. Treat it accordingly.

- **Props off.** Every time. The sweep arms motors without further confirmation.
- **Arming is a real state.** The `ARM` button latches. Motors respond to slider
  changes only while armed, and the button turns solid red when live.
- **Comms watchdog.** If the browser stops polling for **1500 ms** — tab closed,
  phone locked, WiFi dropped — the firmware disarms and cuts all four motors.
  The sweep is exempt while it runs, because it deliberately holds motors on.
- **The AP is not a security boundary.** WPA2 with a hardcoded password that is
  published in this README. Anyone in range who joins can arm the motors. Don't
  leave it powered and unattended with props on.
- **Flashing this erases the stock firmware.** The Crazyflie / ESP-Drone image
  is gone until you restore it with the CircuitDigest web flasher.

---

## Known issues

**Stale pin values in NVS survive a reflash.** The motor and battery pins are
now verified against the vendor firmware and schematic (see
[docs/HARDWARE.md](docs/HARDWARE.md#motor-and-battery-pin-map)), but if you
saved wrong values from the Pins panel at any point, those persist in NVS and
override the firmware defaults — flashing does not erase the NVS partition.
Correct them in the Pins panel, or reflash with **Erase All Flash Before Sketch
Upload** enabled.

**Requires Arduino-ESP32 core 3.x.** The 2.x compatibility branch in
`pwmWrite()` is broken: core 2.x's `ledcWrite()` takes an LEDC *channel*, but
`applyMotors()` passes a *pin number*. On 3.x the pin-based API is correct and
this is a non-issue. On 2.x the writes land on unattached channels and no motor
will ever spin.

**The session log lives in the browser, not the drone.** Closing the tab
discards it. Export to CSV before you close. It caps at 36,000 samples (over an
hour) and then stops recording rather than growing without bound.

**The buzzer is mapped but unused.** `BUZZER` is defined as GPIO 39 and never
driven. There is no audible arm warning — a deliberate omission worth knowing
about, since nothing announces that motors are live.

**ADC2 pins will not work for battery sensing.** If you remap `vbatPin` from
the Pins panel, keep it on ADC1. ADC2 is unavailable while WiFi is active, and
the AP is always up.

---

## Troubleshooting

**No COM port appears when you plug in USB-C.**
In order of likelihood:

1. **Charge-only cable.** The most common cause by a wide margin. Many USB-C
   cables shipped with power banks and accessories have no data lines. Swap for
   a cable you have confirmed moves data.
2. **Board not powered.** USB-C may only feed the charger IC. Connect a charged
   LiPo and switch the drone on, then plug in.
3. **Hub or dock.** Go straight into the machine.
4. **Driver.** The CH340K needs the WCH driver. Windows 11 usually has it; if
   the device shows a yellow bang in Device Manager, install it from WCH.

On Windows you can confirm what actually enumerated with:

```powershell
Get-PnpDevice -PresentOnly -Class Ports | Select-Object Status,FriendlyName,InstanceId
```

A working board shows `USB-SERIAL CH340K (COMx)` with status `OK`. If nothing
appears at all — no port, no unknown device, no failed-descriptor entry — the
data lines are not reaching the chip, which points at causes 1 through 3 rather
than at software.

**Sensors panel shows `nothing found` on a bus.** Check the I2C pin map in
[docs/HARDWARE.md](docs/HARDWARE.md). Both buses run at 400 kHz; a badly seated
sensor deck will scan empty.

**IMU reads `no response`.** The MPU6050 is on I2C0 at `0x68`. If the bus scan
finds `0x68` but the panel still says no response, the init sequence failed —
power-cycle rather than resetting, so the sensor gets a clean reset too.

---

## Repository layout

```
LiteWing_Bench/
  LiteWing_Bench.ino     the firmware - single translation unit, no libraries
                         beyond the ESP32 core
docs/
  HARDWARE.md            pin map, sensors, drive topology
  UI.md                  dashboard walkthrough
  API.md                 HTTP endpoint reference
```

The dashboard's HTML, CSS and JavaScript are embedded in the sketch as a
`PROGMEM` string literal. There is no build step and no filesystem image to
upload — flashing the sketch ships the UI with it.
