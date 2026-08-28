# PID stabiliser

A second page, at **<http://192.168.4.1/pid>**, that closes a feedback loop from
the IMU onto the motors and gives you the instrumentation to tune it. The bench
page at `/` is unchanged and independent.

For a hands-on walkthrough of the controls and the axis mapping, start with
[TUNING.md](TUNING.md). This document is the reference behind it.

This is a **gimbal tuning rig**, not a flight controller. It holds roll and
pitch angles against a setpoint so you can find gains on a stand before
trusting them in the air.

---

## Safety

Read this before the first `START LOOP`.

- **Propellers off. Always.** Nothing here needs thrust to tune — the loop
  works on differential duty, and you can read the whole response with bare
  motors.
- **Mount it in a two-axis gimbal.** The frame must be free to rotate in roll
  and pitch but unable to translate. An unrestrained airframe with a wrong gain
  will flip itself off the bench.
- **Keep a hand near STOP.** The red STOP button cuts the loop, zeroes all four
  motors and disarms in one call. It is safe to hit at any time.
- **The comms watchdog applies.** If the browser stops polling for 1500 ms —
  tab closed, phone locked, WiFi dropped — the firmware kills the loop and the
  motors. Closing the tab is a valid emergency stop.
- **The stabiliser and the health sweep are mutually exclusive.** Starting one
  stops the other, and touching a motor slider on the bench page drops the loop
  immediately. Manual control always wins.

---

## The control law

Standard parallel PID per axis, running from `loop()` at whatever rate the main
loop sustains — typically 200–400 Hz, shown live as `rate` on the page.

```
error   = setpoint - angle
P       = Kp * error
I      += Ki * error * dt        (clamped to +/- iLim)
D       = -Kd * gyro_rate
output  = clamp(P + I + D, -oLim, +oLim)
```

Two details that matter:

**D acts on the gyro rate, not on the differentiated angle.** Differentiating a
noisy angle amplifies the noise, and it kicks hard the instant the setpoint
moves — a step command would produce an impulse in D that has nothing to do
with the plant. Rate-on-measurement has neither problem, and the MPU6050 gives
the rate directly.

**The integral is clamped and reset on start.** `iLim` bounds windup, and
`pidStart()` zeroes the accumulator so a loop never inherits stale integral
from a previous run.

### Mixer

X-quad, matching the `MOT_1..MOT_4` order in the [pin map](HARDWARE.md):

```
M1 front-right = base - roll - pitch
M2 back-left   = base + roll + pitch
M3 back-right  = base - roll + pitch
M4 front-left  = base + roll - pitch
```

All four clamp to 0–100%.

**Sign conventions depend on how the MPU6050 is mounted**, which the firmware
cannot know. If an axis diverges instead of correcting — the frame runs to the
stop rather than returning to level — hit STOP and press **Invert axis**. That
flips the correction sign for the currently selected axis and persists with the
gains.

---

## Tuning procedure

**One axis at a time.** Select **Roll**, lock pitch in the gimbal, tune, then
repeat for pitch. Only select **Both** once each axis is settled on its own.

1. **Zero all gains.** Start from nothing so you can see each term's effect.
2. **Raise base throttle** until the motors just spin up — typically 12–20%. At
   0% the loop can only ever subtract, so it has no authority to correct. This
   is the single most common reason a first attempt does nothing.
3. **START LOOP.**
4. **Raise Kp** until the frame holds level and a **Step +10°** produces a
   crisp move to target. Keep going until you see sustained oscillation, then
   back off to roughly 60% of that value.
5. **Add Kd** to damp the overshoot Kp introduced. Increase until the response
   stops ringing. If the trace turns fuzzy and the motors buzz, Kd is
   amplifying gyro noise — back off.
6. **Add Ki last, sparingly**, and only if a steady offset remains — the frame
   settles consistently a degree or two off target. Ki removes that. Too much
   causes a slow wallowing oscillation and overshoot on every step.
7. **Save to NVS** once you are happy. Gains persist across reboots.

### Reading the response chart

The **setpoint vs angle** plot is the tune. Dashed grey is the command, solid
green is what the IMU measured.

| What you see | What it means |
|---|---|
| Slow crawl toward target, never arrives | Kp too low, or base throttle too low to give authority |
| Reaches target then rings before settling | Kp high — add Kd, or reduce Kp |
| Sustained oscillation that never decays | Kp far too high; back off hard |
| Settles consistently off target | Needs a little Ki |
| Slow wallow, overshoot on every step | Ki too high |
| Fuzzy, hairy trace and audible motor buzz | Kd too high, amplifying gyro noise |
| Runs away to the mechanical stop | Wrong sign — STOP, then **Invert axis** |

The **term contributions** plot shows P, I and D separately in output percent,
plus the clamped total. This is what tells you *which* gain is responsible.
A D trace that is visibly noisier than P is the classic sign of too much Kd.
An I trace that ramps and never comes back is windup.

The **motor duty** plot shows all four outputs. In roll-only mode you should
see two motors rise as the opposite pair falls, symmetrically about base
throttle. If all four move together, the mixer signs are wrong for your frame.

---

## Step response metrics

Every **Step**, **Level** or setpoint change starts a 6-second measurement
window. The panel reads `measuring` while it is open and `final` once it
closes, so a frozen number is never mistaken for a live one.

| Metric | Definition |
|---|---|
| **rise 10–90%** | time from 10% to 90% of the commanded change |
| **overshoot** | peak excursion past target, as a percent of the step size |
| **settling ±2%** | time until the response stays inside a ±2% band (floor 0.4°) |
| **steady error** | mean error over the last second of the window |
| **peak** | largest excursion reached |
| **RMS error** | root-mean-square error across the whole window |

A good tune on a gimbal: **overshoot under ~15%, settling under a second,
steady error near zero.** `never reached 90%` and `never settled` are honest
readouts, not failures of the tool — they mean the response was still moving
when the window closed, which itself says the gains are too low.

The last twelve steps are kept in the history table so you can compare gain
sets directly rather than from memory.

---

## Export

**Export CSV** writes `litewing_pid_<axis>.csv` for the current axis:

```
t_s, axis, setpoint_deg, angle_deg, error_deg, rate_dps,
p_pct, i_pct, d_pct, out_pct, m1, m2, m3, m4, vbat_v
```

Every term is separated, so you can fit a model, compare gain sets offline, or
plot the response properly. As with the bench session log, this lives in the
browser — closing the tab discards it.

---

## API

All endpoints are plain `GET`, same as the bench page.

### `GET /api/pid`

Current configuration.

```json
{"kp":[0.6,0.6],"ki":[0,0],"kd":[0.03,0.03],
 "base":0,"mode":0,"ilim":15,"olim":30,"inv":[0,0],"run":false}
```

Index 0 is roll, index 1 is pitch throughout.

### `GET /api/pidset?...`

Set any subset. Only the parameters present are changed.

| Param | Range | Meaning |
|---|---|---|
| `kp0` `ki0` `kd0` | float | roll gains |
| `kp1` `ki1` `kd1` | float | pitch gains |
| `base` | 0–60 | base throttle % |
| `mode` | 0–3 | 0 off, 1 roll, 2 pitch, 3 both |
| `ilim` | 0–50 | integral clamp, output % |
| `olim` | 0–50 | per-axis authority clamp, output % |
| `inv0` `inv1` | 0/1 | invert correction sign per axis |
| `sp0` `sp1` | ±30 | setpoint, degrees |
| `reseti` | any | zero both integrators |
| `run` | 0/1 | stop or start the loop |

Values are clamped server-side; the page cannot command an unsafe range.

### `GET /api/pidstate`

Fast telemetry for the tuning plots. Touches the comms watchdog.

```json
{"run":true,"armed":true,"ang":[0.4,-0.2],"sp":[0,0],
 "p":[0.24,0],"i":[0,0],"d":[-0.03,0],"o":[0.21,0],
 "rate":[1.1,-0.4],"duty":[15,19,15,19],
 "vbat":3.86,"imu":true,"hz":312,"t":91240}
```

`hz` is the measured PID loop rate, from the last `dt`. If it drops well below
~100 Hz the derivative term gets coarse — check what else is loading the ESP32.

### `GET /api/pidsave`

Persist gains, limits and invert flags to NVS under the `bench` namespace.
Setpoint, mode and base throttle are deliberately **not** saved, so a reboot
always comes back stopped and level.
