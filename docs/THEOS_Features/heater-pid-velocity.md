# Velocity-Form PID

## What this is

Stock Klipper drives heaters with a textbook positional PID controller:
each cycle computes a fresh proportional, integral, and derivative term
from the absolute temperature error and feeds the sum to the heater PWM.
The integral term in particular is a running accumulator, which can wind
up after a long disturbance and overshoot when the disturbance ends.

This fork adds a second control algorithm — **velocity-form PID** —
selectable per heater via `control: pid_v`. The velocity form computes
the *change* in PWM each cycle from the *change* in error, derived
backwards from the same Kp/Ki/Kd gains. The integral never accumulates
explicitly, so a sudden setpoint change or a long thermal disturbance
cannot wind it up.

The two algorithms share calibration: a `PID_CALIBRATE` run on either
form produces gains that work in both. You switch between them by
changing the `control:` line.

## When to use this

- Your heater overshoots after a long preheat hold or a setpoint step,
  even with a clean PID tune. Integral windup is the usual cause.
- You're driving a high-mass heater (chamber, thick aluminium bed) where
  the response time makes the positional integral fragile.
- You want to use [`SET_HEATER_PID`](#set_heater_pid) to retune live
  during a print without restarting Klipper, and the heater is currently
  drifting because of accumulated integral error.

You don't need this if your existing positional `control: pid` heater is
quiet, settles cleanly, and doesn't overshoot. Velocity-form is an
alternative, not a replacement.

## Configuration

Set the algorithm on the heater section. Calibrated `pid_Kp`/`pid_Ki`/
`pid_Kd` values from a positional tune are reused as-is.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `control` | `pid` | Set to `pid_v` to use velocity-form PID. Other allowed values: `watermark`, `pid`, `dual_loop_pid`, `mpc`. |

### Minimal example

```ini
[extruder]
# ... heater_pin, sensor_pin, etc ...
control: pid_v
pid_Kp: 18.4
pid_Ki: 0.92
pid_Kd: 92.0
```

## G-code commands

### `SET_HEATER_PID`

Update Kp/Ki/Kd at runtime, without restarting Klipper. Works for both
positional (`control: pid`) and velocity-form (`control: pid_v`)
heaters; rejected for `watermark`, `dual_loop_pid`, and `mpc` heaters.

| Parameter | Meaning |
|-----------|---------|
| `HEATER` | Section name of the heater. |
| `KP` | New proportional gain (in the same units as `pid_Kp`). Optional. |
| `KI` | New integral gain. Optional. |
| `KD` | New derivative gain. Optional. |

Any combination of KP/KI/KD may be set in a single call; omitted gains
keep their current value.

```
SET_HEATER_PID HEATER=heater_bed KP=42.0 KI=1.5 KD=300
SET_HEATER_PID HEATER=extruder KD=110
```

The command updates the live controller; it does **not** write the
gains back to disk. Use [`PID_PROFILE SAVE`](heater-pid-profiles.md)
afterwards if you want the change to survive a restart.

## Things to know

- **Same calibration, different algorithm.** A `PID_CALIBRATE` run
  produces a Kp/Ki/Kd triple that works for both algorithms. Switching
  `control: pid` ↔ `control: pid_v` does not require recalibration.
- **No explicit integral state means no startup spike.** A cold heater
  with `control: pid_v` won't accumulate a large integral term during
  the heat-up; the integral derivative naturally tapers as the error
  shrinks.
- **Velocity-form needs a meaningful first sample to compute the
  initial derivative.** Klipper handles this internally by holding the
  PWM at zero on the first temperature callback after startup, so the
  first non-zero PWM cycle has a valid `last_temp` baseline.
- **`SET_HEATER_PID` rejects non-PID controllers.** It's keyed on the
  current `control` value being one of `pid` or `pid_v`. To retune a
  `dual_loop_pid` heater live, use `PID_PROFILE LOAD` against a
  pre-tuned profile — see [PID profiles](heater-pid-profiles.md).

## Credits

Based on the velocity-form PID implementation from the Kalico community
(Dans98), imported and re-derived against the current Klipper heater
control class layout. The `SET_HEATER_PID` runtime-tuning command was
extended to accept the velocity controller as part of the same port.
