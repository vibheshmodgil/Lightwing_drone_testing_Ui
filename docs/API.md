# HTTP API

Base URL **`http://192.168.4.1`** once you have joined the `LiteWing-Bench`
access point.

The stabiliser endpoints (`/api/pid`, `/api/pidset`, `/api/pidstate`,
`/api/pidsave`) are documented separately in [PID.md](PID.md).

Every endpoint is a plain `GET` with query-string parameters and a JSON
response — deliberately, so the whole rig is drivable from `curl` without a
browser. There is no authentication: anyone on the AP can arm the motors.

The server is single-threaded (`WebServer` on core 0, serviced from `loop()`).
Requests are handled strictly one at a time, so a long-running call blocks
every other endpoint until it returns.

---

## `GET /`

The dashboard. Serves the embedded HTML page from PROGMEM.

---

## `GET /api/state`

Full telemetry snapshot. The dashboard polls this every 120 ms.

**This call feeds the comms watchdog.** Polling it is what keeps the firmware
armed; go quiet for 1500 ms and it disarms. If you are scripting motor control,
you must poll `/api/state` (or any other watchdog-touching endpoint) on an
interval, or your motors will cut out mid-test.

```json
{
  "vbat":   3.874,
  "imu":    true,
  "armed":  false,
  "a":      [0.001, -0.004, 0.998],
  "g":      [0.12, -0.05, 0.01],
  "ar":     [4, -16, 4088],
  "gr":     [2, -1, 0],
  "bias":   [1.42, -0.87, 0.33],
  "roll":   0.21,
  "pitch":  -0.44,
  "aroll":  0.35,
  "apitch": -0.51,
  "temp":   31.2
}
```

| Field | Type | Meaning |
|---|---|---|
| `vbat` | float | pack voltage, V, 16-sample mean x divider ratio |
| `imu` | bool | last MPU6050 read succeeded |
| `armed` | bool | master arm state |
| `a` | float[3] | accelerometer X/Y/Z in **g** (scaled) |
| `g` | float[3] | gyro X/Y/Z in **dps**, bias-corrected |
| `ar` | int[3] | accelerometer X/Y/Z as **raw LSB counts** (4096 LSB/g) |
| `gr` | int[3] | gyro X/Y/Z as **raw LSB counts** (16.4 LSB/dps), no bias removed |
| `bias` | float[3] | gyro bias currently being subtracted, dps |
| `roll` | float | complementary-filtered roll, degrees |
| `pitch` | float | complementary-filtered pitch, degrees |
| `aroll` | float | **accelerometer-only** roll, unfiltered, degrees |
| `apitch` | float | **accelerometer-only** pitch, unfiltered, degrees |
| `temp` | float | MPU6050 die temperature, Celsius |
| `duty` | int[4] | **live** duty per motor, 0-100, as the device has it |
| `sweep` | bool | a health sweep is currently running |
| `swm` | int | motor the sweep is on (1-4), 0 when idle |
| `t` | int | device uptime in ms, from `millis()` |

Note that `/api/motor` and `/api/stop` both stop the PID stabiliser if it is running — manual motor control always takes precedence over the loop.

Comparing the raw and filtered pairs is the point: `ar`/`gr` against `a`/`g`
shows whether scaling is sane, and `aroll`/`apitch` against `roll`/`pitch`
shows what the complementary filter is actually contributing. On a still,
level board all four angles should sit near zero and near each other; if
`aroll` is noisy but `roll` is smooth, the filter is doing its job.

---

## `GET /api/motor?i={0-3}&v={0-100}`

Set one motor's duty cycle.

| Param | Range | Meaning |
|---|---|---|
| `i` | 0–3 | motor index — M1 in the UI is `i=0` |
| `v` | 0–100 | duty percent, clamped, mapped to 10-bit PWM |

Stores the duty regardless of arm state, but **output stays at zero until
armed**. Out-of-range `i` is ignored silently. Touches the watchdog.

```
{"ok":true}
```

---

## `GET /api/arm?v={0|1}`

Master arm. `v=1` arms, anything else disarms. On arming, previously-set duties
take effect **immediately** — set duties to zero first unless you intend motors
to spin the instant you arm. Touches the watchdog.

```
{"ok":true}
```

---

## `GET /api/stop`

Emergency stop. Zeroes all four duties and disarms in one call. Always safe to
call, whatever the current state.

```
{"ok":true}
```

---

## `GET /api/calib`

Calibrate gyro bias. Disarms first, then averages 400 samples at ~3 ms
intervals and zeroes the roll/pitch accumulators.

**Blocks for roughly 1.2 seconds.** Keep the drone still and flat for the
duration, or the motion is baked in as permanent bias.

```
{"ok":true}
```

---

## `GET /api/sweep`

Start the motor health sweep: each motor alone at 30%, 50% and 70% for 700 ms,
recording battery sag against a resting voltage captured at the start.

**Returns immediately.** The sweep runs as a state machine driven from
`loop()`, so the server keeps answering while it works — that is what lets the
browser log battery voltage across the whole sweep. Poll `/api/state` and watch
`sweep` and `swm` for progress.

**Arms motors without further confirmation.** Props off.

The comms watchdog is suspended while the sweep runs, since it holds motors on
deliberately. All motors are stopped and disarmed when it finishes.

```
{"ok":true}
```

Calling it while a sweep is already running is a no-op.

---

## `GET /api/sweeplog`

Read the sweep result. Safe to call at any time; while a sweep is running it
returns the partial log built so far.

```json
{
  "running": false,
  "log": "rest 3.94V
M1 30%:0.041V  50%:0.088V  70%:0.143V
..."
}
```

Interpretation is covered in [UI.md](UI.md#reading-a-health-sweep).

---

## `GET /api/scan`

Scan both I2C buses and identify the expected sensors. Takes a few hundred
milliseconds — it walks 126 addresses on each bus.

```json
{
  "i2c0": ["0x68", "0x77"],
  "i2c1": [],
  "id": {
    "MPU6050 imu (0x68)":        true,
    "VL53L1X tof (0x29 aux)":    false,
    "MS5611 baro (0x77)":        true,
    "HMC5883 mag (0x1e)":        false,
    "PMW3901 flow (spi)":        true
  }
}
```

The `id` booleans for I2C parts are address ACKs, not who-am-I reads. The
PMW3901 entry is a genuine product-ID check against `0x49`.

---

## `GET /api/pins`

**Without parameters** — read the current configuration:

```json
{"m":[7,8,9,12],"vbatPin":4,"vbatDiv":2.00}
```

**With parameters** — write it. All six are required; the handler keys off the
presence of `m0` and falls through to a plain read if it is absent.

| Param | Meaning |
|---|---|
| `m0`–`m3` | motor GPIO numbers |
| `vbat` | battery ADC GPIO |
| `div` | divider ratio, float |

```
/api/pins?m0=7&m1=8&m2=9&m3=12&vbat=4&div=2.0
```

Persists to NVS namespace `bench`, responds `{"ok":true}`, then **reboots after
300 ms** so the PWM channels re-attach. The connection drops and the AP takes a
few seconds to return. There is no validation — an invalid GPIO will be
accepted, written, and will fail at the next boot.

---

## Scripting example

Pulse each motor for a second in turn, keeping the watchdog fed:

```python
import requests, time

BASE = "http://192.168.4.1"

def poll():                      # keeps the watchdog happy
    return requests.get(f"{BASE}/api/state", timeout=2).json()

requests.get(f"{BASE}/api/stop")               # known state
requests.get(f"{BASE}/api/arm?v=1")

for motor in range(4):
    requests.get(f"{BASE}/api/motor?i={motor}&v=40")
    end = time.time() + 1.0
    while time.time() < end:
        poll()                                 # < 1500 ms between calls
        time.sleep(0.2)
    requests.get(f"{BASE}/api/motor?i={motor}&v=0")

requests.get(f"{BASE}/api/stop")
```

For the sweep, raise the client timeout past the blocking window:

```python
log = requests.get(f"{BASE}/api/sweep", timeout=20).json()["log"]
print(log)
```
