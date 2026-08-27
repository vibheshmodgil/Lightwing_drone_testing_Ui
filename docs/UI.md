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
|  ATTITUDE                            |
|            ___   /\   ___            |
|           (   )--\/--(   )           |
|              \   ||   /              |
|            ___\  ||  /___            |
|           (   )--/\--(   )           |
|         roll 2.10  pitch -0.4o       |
+--------------------------------------+
|  IMU            RAW      FILTERED    |
|    accel x      -132     -0.03 g     |
|    gyro x        -18     -0.02 o/s   |
|    roll        -1.8o      -1.7o      |
+--------------------------------------+
|  SENSORS / PINS                      |
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
  motors — check the Pins panel against
  [HARDWARE.md](HARDWARE.md#motor-and-battery-pin-map) before suspecting
  hardware. A telltale: if moving a motor slider changes the brightness of the
  onboard RGB LED, the pins are set to `7, 8, 9, 12`, which are the LED
  channels, not the motors.
- **Falling sag as duty rises**: not physical. Suspect a loose battery
  connector browning out under load, or thermal cut-back.

The sweep runs as a state machine on the device, so the UI stays live
throughout — the motor sliders track its progress and the session log captures
the whole thing. For per-level sag rather than this summary, read the **load
segments** table, which breaks the sweep into one row per motor per level.

One caveat: the resting voltage is captured once at the start, so on a pack
already sagging from a long session, later motors read slightly higher sag than
earlier ones purely from cumulative drain. Re-sweep on a fresh pack before
condemning a motor.

---

## Attitude

A 3D model of the airframe on a fixed ground grid, driven by the
complementary-filter roll and pitch. The grid and its shadow stay level and the
drone banks against them, so tilt reads instantly without parsing numbers.

The model is a real X-quad: a shaded body box, booms with visible sides, motor
pods, and **propellers that actually spin at a rate set by each motor's live
duty cycle** — read from the device, not from the sliders, so a sweep or a
watchdog cut shows up here too. A translucent disc fades in over a spinning
rotor as duty rises. The status LED on the body turns red when armed.

Motion is interpolated with `requestAnimationFrame` between polls rather than
snapping at the poll rate, which is what makes it read as smooth at ~8 Hz of
actual data.

All of it is CSS 3D transforms — no WebGL, no library, no external request.
That is a requirement, not a preference: the page is served from the drone's own
access point with no route to the internet.

**Yaw is not shown, because it is not measured.** The magnetometer is never
fused, so the model's heading is fixed. Only roll and pitch move.

### If the model tilts the wrong way

Whether positive roll means "right side down" depends on how the MPU6050 sits
on your board, which the firmware cannot know. If the model leans opposite to
the real drone, flip the relevant sign in the sketch — one line, near the top
of the `<script>` block:

```js
const SGN={r:1,p:-1};   // r = roll, p = pitch
```

Set `r:-1` to invert roll, `p:1` to invert pitch. The numeric readouts are
unaffected; this only changes the visual.

---

## Session log

Records everything the bench sees while you work, plots it live, and exports it
as CSV. This is where the engineering happens — the panels above tell you what
is true *now*, the session log tells you what happened *over time*.

**Recording happens in the browser, not on the drone.** The page already polls
`/api/state` several times a second; the log just keeps every sample instead of
discarding it. That costs the ESP32 nothing and gives you effectively unlimited
history. The trade is that **closing the tab loses the log** — export before
you close. The buffer caps at 36,000 samples (over an hour) and stops recording
rather than growing without bound.

### Controls

- **RECORD** — starts and stops. Turns red while recording. Timestamps are
  relative to the first sample.
- **Clear** — discards everything and resets the clock.
- **Export CSV** — writes three files (see below).
- **30 s / 2 min / All** — the time window the charts show. Recording is
  unaffected; this is only the view.

### Charts

Four stacked plots on a shared time axis, drawn on `<canvas>`:

| Chart | Series |
|---|---|
| **battery volts** | pack voltage, autoscaled |
| **motor duty %** | all four motors, fixed 0–100 |
| **attitude deg** | roll and pitch |
| **gyro dps** | gyro X/Y/Z |

Vertical amber lines are **event markers** — arm, disarm, stop, each pulse,
sweep start and end, calibration. They are what let you line a voltage drop up
against the thing that caused it.

Hovering any chart drops a crosshair and the value readouts at the right of
each title switch to the values at that instant, across all four charts at
once. Move the mouse away and they return to live.

Long sessions are decimated to roughly 900 points per line for drawing, so
redraw cost stays flat whether you have 500 samples or 36,000. The CSV export
is never decimated.

### Load segments

The table under the charts is the direct answer to "how much does the battery
drop for each test".

A **segment** is any period of constant motor output. Every time the duty
vector changes — you move a slider, a pulse fires, the sweep steps to its next
level — the current segment closes and a new one opens. Each row records:

| Column | Meaning |
|---|---|
| `t` | when the segment started, mm:ss.s |
| `output` | which motors at what duty, e.g. `M2 50%` |
| `dur` | how long it held |
| `V rest` | resting voltage from the idle period immediately before |
| `V min` | lowest voltage seen during the segment |
| `sag` | `V rest - V min` |

Sag is coloured green when meaningful and **red when under 5 mV**, which is the
signature of a motor drawing no current — a dead winding, a blown IRLML6344, a
broken joint, or a wrong pin. Compare motors against each other rather than
against a fixed threshold; the absolute numbers depend on your pack and its
state of charge.

Because a health sweep steps through 30/50/70% per motor, it produces twelve
segments — one per level per motor — so you get per-level sag rather than the
single summary line the sweep log prints.

### CSV export

Three files, so each is a clean table:

- **`litewing_samples.csv`** — every sample: `t_s, vbat_v, m1..m4_pct, armed,
  roll_deg, pitch_deg, gx/gy/gz_dps, ax/ay/az_g, temp_c`
- **`litewing_segments.csv`** — the segment table, including `v_rest`, `v_min`
  and `sag_v`
- **`litewing_events.csv`** — timestamped event markers

All three share the same `t_s` time base, so they join cleanly in a spreadsheet,
pandas, or whatever you analyse in.

---

## IMU

Every value in two columns, **raw** and **filtered**, repainting at the 120 ms
poll rate.

| Row | Raw column | Filtered column |
|---|---|---|
| accel x/y/z | LSB counts, 4096 LSB/g | scaled to **g** |
| gyro x/y/z | LSB counts, 16.4 LSB/dps, bias still in | **dps**, bias subtracted |
| roll | **accelerometer-only**, unfiltered | complementary-filtered |
| pitch | **accelerometer-only**, unfiltered | complementary-filtered |

Below those: the gyro bias currently being subtracted, and MPU6050 die
temperature.

Two columns rather than one because the comparison is the diagnostic:

- **Raw counts near zero on all three accel axes** — the sensor is answering
  but returning nothing. Bus problem, not a scaling problem.
- **Raw accel z near 4096 at rest, filtered z near 1.00 g** — scaling is
  correct. If raw looks right but filtered does not, the full-scale constant is
  wrong.
- **Raw gyro far from zero at rest, filtered near zero** — normal. That gap is
  exactly the bias being removed, and it should match the gyro bias row.
- **Raw gyro near zero but filtered far off** — stale calibration. Recalibrate.
- **Raw roll/pitch jittery, filtered smooth** — the complementary filter
  working as intended.
- **Both roll columns drifting apart steadily** — gyro bias has drifted since
  calibration, usually thermal. Recalibrate.

At rest on a flat surface, expect accel roughly `0.00 / 0.00 / 1.00` g, gyro
near zero, and all four angle figures near zero.

**Calibrate gyro bias** disarms, then averages 400 samples to establish zero
rates and resets the attitude accumulators. Keep the drone completely still and
flat for it — about 1.2 seconds. Calibrating while it moves bakes the motion in
as permanent bias. This also runs automatically at boot.

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

Firmware defaults are now the verified values, so this panel is for
overrides and board revisions rather than routine setup. See
[HARDWARE.md](HARDWARE.md#motor-and-battery-pin-map). Note that values saved
here **override the firmware defaults and survive a reflash**.

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
