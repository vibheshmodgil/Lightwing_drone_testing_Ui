# Hardware

Target board: **LiteWing V1.2**, ESP32-S3, from CircuitDigest.

---

## Confirmed pin map

These come from the published LiteWing hardware documentation and are used as
compile-time constants in the sketch. They are not editable from the UI.

| Function | GPIO | Notes |
|---|---|---|
| I2C0 SDA | 11 | primary sensor bus |
| I2C0 SCL | 10 | |
| I2C1 SDA | 40 | auxiliary / expansion deck bus |
| I2C1 SCL | 41 | |
| SPI SCK | 36 | |
| SPI MISO | 37 | |
| SPI MOSI | 35 | |
| SPI CS | 42 | PMW3901 chip select |
| Buzzer | 39 | mapped, never driven by this firmware |

Both I2C buses are initialised at **400 kHz**. The SPI bus runs at **2 MHz,
MSB-first, mode 3**, which is what the PMW3901 expects.

---

## Motor and battery pin map

Verified. Motor pins are confirmed against the vendor's own firmware,
[`LiteWing-Arduino/Arduino-LiteWing-DMP/motors.h`](https://github.com/Circuit-Digest/LiteWing/blob/main/LiteWing-Arduino/Arduino-LiteWing-DMP/motors.h).
The battery ADC pin and divider are traced from the `ADC_BAT` net in
[`hardware/LiteWingV2.5C/LiteWingV2.5C.kicad_sch`](https://github.com/Circuit-Digest/LiteWing/tree/main/hardware/LiteWingV2.5C).

| Function | GPIO | Schematic net | Source |
|---|---|---|---|
| Motor 1 (front right) | 5 | `MOT_1` | `motors.h` |
| Motor 2 (back left) | 6 | `MOT_2` | `motors.h` |
| Motor 3 (back right) | 3 | `MOT_3` | `motors.h` |
| Motor 4 (front left) | 4 | `MOT_4` | `motors.h` |
| Battery ADC | 2 | `ADC_BAT` | schematic trace |
| Battery divider | 2.0 | R26/R27, 100K/100K | schematic BOM |

GPIO 2 is **ADC1_CH1**, which matters: ADC2 is unusable while WiFi is active,
and this firmware runs an access point continuously.

GPIO 3 is an ESP32-S3 strapping pin (JTAG source select). It is safe to drive
as a motor output at runtime — the gate pulldown keeps it defined at boot — but
do not add an external pull-up to it.

### Pins these are *not*

An earlier revision of this firmware guessed motors on GPIO `7, 8, 9, 12` and
battery sense on GPIO `4`. Those are wrong, and wrong in a way that fails
silently:

| GPIO | Actually connected to |
|---|---|
| 7 | `LED_BLUE` |
| 8 | `LED_RED` |
| 9 | `LED_GREEN` |
| 12 | `MPU_INT` |

Driving the motor sliders under that map dimmed the RGB status LED instead of
spinning anything, and GPIO 4 — read as the battery ADC — is really `MOT_4`, so
voltage readings were meaningless too.

### Overriding from the UI

The Pins panel still overrides all six values at runtime, persisted to NVS
under the Preferences namespace **`bench`**, keys `m0`–`m3`, `vbat`, `div`.
Saving triggers a reboot so the PWM channels re-attach to the new pins.

**NVS wins over the firmware defaults.** If stale values were saved from an
earlier session, reflashing will *not* clear them — flashing does not erase the
NVS partition. Either correct them in the Pins panel, or reflash with
**Erase All Flash Before Sketch Upload** enabled.

There is no validation on write: an invalid GPIO is accepted, persisted, and
fails at the next boot.

---

## Sensors

The Sensors panel scans both buses and then probes for specific parts.

| Part | Bus | Address | What it is |
|---|---|---|---|
| MPU6050 | I2C0 | `0x68` | 6-axis IMU — the only sensor this firmware reads continuously |
| MS5611 | I2C0 | `0x77` | barometric altimeter |
| HMC5883 | I2C0 | `0x1E` | 3-axis magnetometer |
| VL53L1X | I2C1 | `0x29` | time-of-flight rangefinder, on the aux bus |
| PMW3901 | SPI | CS 42 | optical flow — identified by product ID `0x49` at register `0x00` |

Presence detection for the I2C parts is an address ACK, not a
who-am-I read. A part that ACKs but is internally faulty will report `[ok]`.
The PMW3901 check is stronger: it reads the product ID register and compares
against the expected `0x49`.

### IMU configuration

The MPU6050 is the only sensor driven beyond identification. It is configured
at startup as:

| Register | Value | Meaning |
|---|---|---|
| `0x6B` | `0x80` then `0x01` | reset, then wake with gyro-X PLL clock |
| `0x19` | `0x00` | 1 kHz sample rate |
| `0x1A` | `0x03` | DLPF, 44 Hz bandwidth |
| `0x1B` | `0x18` | gyro full scale +/-2000 dps -> **16.4 LSB/dps** |
| `0x1C` | `0x10` | accel full scale +/-8 g -> **4096 LSB/g** |

Attitude comes from a complementary filter with **k = 0.98** — 98% gyro
integration, 2% accelerometer correction per update — fusing integrated rate
against the gravity vector. It is adequate for confirming the IMU is alive and
correctly oriented. It is not a flight-grade attitude estimate: there is no
magnetometer fusion, so yaw is not estimated at all, and roll/pitch will drift
under sustained lateral acceleration.

Gyro bias calibration averages **400 samples** at ~3 ms intervals, about 1.2
seconds, and zeroes the attitude accumulators. Leave the drone still on a flat
surface for it. It runs automatically at boot when the IMU is detected.

Die temperature is derived as `raw / 340.0 + 36.53`, per the MPU6050 datasheet.
It measures the sensor die, not ambient and not the motors.

---

## Motor drive

Four brushed coreless motors, low-side switched through **IRLML6344** N-channel
MOSFETs.

| Parameter | Value |
|---|---|
| PWM frequency | 8 kHz |
| PWM resolution | 10-bit (0–1023) |
| UI duty range | 0–100%, linearly mapped to 0–1023 |

8 kHz is above the audible whine range for coreless motors and matches typical
Betaflight brushed configurations. Duty is applied only while armed; disarming
writes 0 to all four channels immediately.

This firmware uses the pin-based `ledcAttach` / `ledcWrite` API from
Arduino-ESP32 **core 3.x**. The 2.x fallback path compiles but does not work —
see the known issues in the [README](../README.md#known-issues).

---

## Battery sensing

Read from the configured ADC pin at 12-bit resolution with 11 dB attenuation,
using `analogReadMilliVolts()` so the ESP32-S3's factory ADC calibration is
applied. Each reading is the mean of **16 samples**, then multiplied by the
configured divider ratio.

Accuracy depends entirely on the divider ratio being right. The default of 2.0
assumes a matched-pair resistor divider. If reported voltage is consistently
off by a constant factor, correct the ratio in the Pins panel rather than
trusting the number.

### How the health sweep uses it

The sweep records a resting voltage, then runs each motor **alone** at 30%, 50%
and 70% for 700 ms, tracking the minimum voltage seen during each burst. What
it reports per level is **sag**: `rest - minimum`.

Sag is a proxy for current draw. A working coreless motor loads the pack enough
to produce clearly increasing sag across the three levels. Interpretation is
covered in [UI.md](UI.md#reading-a-health-sweep).
