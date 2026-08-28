# Tuning walkthrough

A practical guide to the `/pid` page: what each control does, how the two axes
work, and the order to turn the knobs in.

For the control law, the maths and the API, see [PID.md](PID.md). This page is
the hands-on version.

---

## Before you start

- **Propellers off.** Nothing here needs thrust. The loop works on differential
  duty and the full response is readable with bare motors.
- **Mount it in a gimbal** that lets the frame rotate in roll and pitch but not
  translate.
- **Keep a hand near STOP.** It cuts the loop, zeroes all four motors and
  disarms in one call.

Open **<http://192.168.4.1/pid>** after joining the `LiteWing-Bench` network.

---

## How the axes work

The four motors sit on an X. Looking down from above, nose away from you:

```
              front
       M4 ○           ○ M1
            \       /
             \     /
              [   ]          body
             /     \
            /       \
       M2 ○           ○ M3
              back
```

Each axis drives one **pair against the opposite pair**, around the base
throttle:

| Axis | Raises | Lowers | Pairs |
|---|---|---|---|
| **Roll** | M2, M4 (left) | M1, M3 (right) | left vs right |
| **Pitch** | M2, M3 (back) | M1, M4 (front) | back vs front |

In firmware that is:

```
M1 front-right = base − roll − pitch
M2 back-left   = base + roll + pitch
M3 back-right  = base − roll + pitch
M4 front-left  = base + roll − pitch
```

So a positive roll output lifts the **left** side, and a positive pitch output
lifts the **back**. Whether that *corrects* your error or makes it worse
depends on which way your MPU6050 is mounted, which the firmware cannot know.

**If an axis runs away to the mechanical stop instead of returning to level:
hit STOP, then press Invert axis.** That flips the correction sign for the
selected axis and saves with the gains. It is a one-press fix, not a bug.

**Tune one axis at a time.** Lock the other one in the gimbal. Roll and pitch
share a mixer, so a badly tuned pitch will pollute your roll results and you
will chase your own tail.

---

## What each control does

### Top block

| Control | What it does |
|---|---|
| **STOP** | Kills the loop, zeroes all motors, disarms. Safe any time. |
| **START LOOP** | Arms and closes the feedback loop. Turns green while running. |
| `loop` / `rate` | Running state, and the measured PID rate in Hz |
| `IMU` / `battery` | Sensor health and pack voltage |

If `rate` drops well below ~100 Hz the derivative term gets coarse — something
else is loading the ESP32.

### Axis

**Off / Roll / Pitch / Both.** Selects which axis the loop controls. The
inactive axis is dimmed in the Gains panel and its output is forced to zero.
The setpoint slider and all step buttons follow whichever axis is selected.

Use **Both** only after each axis is settled individually.

### Base throttle

The level all four motors sit at before corrections are added. **This is not
optional.** At 0% the loop can only ever subtract, so it has no authority to
correct and nothing will appear to happen. Raise it until the motors just spin
up — typically **12–20%**.

### Gains

`Kp`, `Ki`, `Kd` per axis, as a slider and a number box — drag for a coarse
sweep, type for a precise value. Changes apply immediately, live, while the
loop is running.

| Button | What it does |
|---|---|
| **Roll → pitch** | Copies roll's gains onto pitch as a starting point |
| **Zero all** | Sets all six gains to zero |
| **Save to NVS** | Persists gains, limits and invert flags across reboots |
| **Reset I** | Zeroes both integrators — use after a windup |
| **Invert axis** | Flips the correction sign for the selected axis |

Setpoint, axis mode and base throttle are deliberately **not** saved, so a
reboot always comes back stopped and level.

### Setpoint

The angle the loop is trying to hold, for the selected axis.

- **Level (0°)** — the normal working target
- **Step +10° / −10°** — command a jump and measure the response

A step is how you actually read a tune. Each one opens a 6-second measurement
window and fills in the Step response panel.

### Response charts

Three plots on a shared time axis:

- **setpoint vs angle** — dashed grey is the command, solid green is the
  measurement. This is the tune.
- **term contributions** — P, I and D separately, plus the clamped total. This
  tells you *which* gain is responsible for what you are seeing.
- **motor duty** — all four outputs. In roll-only mode you should see two rise
  as the other two fall, symmetrically about base throttle.

Hover any chart for a crosshair that reads all three at that instant. The
10 s / 30 s / All buttons only change the view, never the recording.

### Step response

Measured from the most recent step. Reads `measuring` while the window is open
and `final` once it closes, so a frozen number is never mistaken for a live
one.

`never reached 90%` and `never settled` are honest results, not tool failures —
they mean the response was still moving when the window closed, which itself
tells you the gains are too low.

The history table keeps the last twelve steps so you can compare gain sets
directly instead of from memory.

---

## The tuning order

1. **Zero all** gains.
2. **Base throttle to ~15%.**
3. **Axis → Roll**, pitch locked in the gimbal.
4. **START LOOP.**
5. **Raise Kp** until `Step +10°` moves crisply to target. Keep raising until
   it oscillates and does not stop, then back off to about 60% of that value.
6. **Add Kd** until the ringing damps out. Stop when the trace turns fuzzy and
   the motors buzz — that is gyro noise being amplified.
7. **Add Ki last, sparingly**, and only if the frame settles a degree or two
   off target and stays there. That residual offset is the only thing Ki fixes.
8. **Save to NVS.**
9. **Repeat for pitch**, then try **Both**.

Aim for **overshoot under ~15%, settling under a second, steady error near
zero**. Compare candidate tunes on the RMS error column of the history table.

---

## Diagnosis

| What you see | Turn this |
|---|---|
| Slow crawl, never arrives | Kp up — or base throttle up, if all terms are small |
| Reaches target then rings | Kd up, or Kp down |
| Oscillates forever, growing | Kp down, hard |
| Settles a degree or two off | Ki up, a little |
| Slow wallow, overshoots every step | Ki down |
| Fuzzy trace, motors buzzing | Kd down |
| Runs away to the stop | STOP → **Invert axis** |
| All four motors move together | Mixer sign wrong for your frame |
| Total term clipped flat | Saturating `oLim` — see below |

---

## Two traps

**Authority is only symmetric while `|output| ≤ base`.** With base 15% and
`oLim` 30%, any correction above 15% drives the low pair to 0 and clamps: full
push up, nothing down. The loop goes one-sided exactly when it is working
hardest, which reads as unexplained asymmetric overshoot. Keep **`oLim` ≈
`base`** for symmetric response, or know where you saturate.

**Manual control always wins.** Touching a motor slider on the bench page, or
starting a health sweep, stops the loop immediately. So does the comms
watchdog — if the browser stops polling for 1.5 s, the firmware cuts the loop
and the motors, so closing the tab is a valid emergency stop.

---

## Quick reference

| Gain | Units | Typical start |
|---|---|---|
| Kp | % duty per degree | 0.6 |
| Ki | % duty per degree-second | 0.0 |
| Kd | % duty per deg/s | 0.03 |
| base | % duty | 15 |
| `oLim` | % duty, per axis | ≈ base |

With Kp = 0.6 and base = 15%, a 10° error gives a 6% correction: the low pair
runs at 9%, the high pair at 21%.
