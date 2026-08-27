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

## Unverified pin map

**These are guesses.** LiteWing publishes the sensor pinout but not the motor
mapping or the battery-sense channel. The values below are the sketch defaults
and the starting point, not ground truth.

| Function | Default GPIO | Status |
|---|---|---|
| Motor 1 | 7 | guess |
| Motor 2 | 8 | guess |
| Motor 3 | 9 | guess |
| Motor 4 | 12 | guess |
| Battery ADC | 4 | guess |
| Battery divider ratio | 2.0 | guess |

Correct them from the **Pins** panel in the dashboard — no reflash needed. The
authoritative source is `DroneV2.5C_Schematics.pdf` in the
[`Circuit-Digest/LiteWing`](https://github.com/Circuit-Digest/LiteWing) repo.

Values are persisted to NVS under the Preferences namespace **`bench`**, keys
`m0`–`m3`, `vbat`, `div`. Saving triggers a reboot so the PWM channels
re-attach to the new pins.

If you remap the battery pin, **keep it on ADC1**. ADC2 is unusable while WiFi
is active, and this firmware runs an access point continuously.

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
