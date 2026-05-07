# PID Autotune (Adaptive Relay)

## What this is

Stock Klipper's `PID_CALIBRATE` runs a single Åström-Hägglund relay cycle
on the heater, fits the response with the Ziegler-Nichols formula, and
saves the result. This works on a low-thermal-mass hotend, but on a thick
aluminium bed or a high-mass chamber heater the single-cycle estimate
often picks gains that overshoot for hours or fail to settle at all.

This fork replaces the `PID_CALIBRATE` core with an **adaptive relay
autotune** based on Taysom & Sorensen's iterative method. The relay
oscillation is run multiple times with the duty cycle re-tuned between
iterations until the temperature-amplitude ratio converges, then a
classical PID gain set is fitted. A second short heat test verifies the
result against the requested target before the gains are written back.

The G-code surface is unchanged: you still call `PID_CALIBRATE
HEATER=<name> TARGET=<temperature>`. The improvements are entirely
inside the calibration logic.

## When to use this

- You have a heated bed that overshoots or oscillates after a stock-Klipper
  PID tune.
- You're tuning a chamber heater, an enclosure heater, or any heater with
  significant thermal mass where the single-cycle method is known to be
  fragile.
- You want a tune you can re-run without restarting Klipper between
  attempts.

A small all-metal hotend usually tunes fine with the stock single-cycle
method too. The adaptive autotune costs no extra time on those, so there
is no reason to fall back, but you don't gain a lot either.

## G-code commands

### `PID_CALIBRATE`

Run the autotune on the named heater.

| Parameter | Meaning |
|-----------|---------|
| `HEATER` | Section name of the heater to calibrate (e.g. `extruder`, `heater_bed`, `heater_generic chamber`). |
| `TARGET` | Calibration setpoint in °C. The autotune oscillates around this temperature. |
| `TOLERANCE` | Optional convergence tolerance on the amplitude ratio between consecutive iterations. Default `0.02` (2 %). Tighten to `0.01` for high-precision beds; loosen to `0.05` if a noisy thermistor prevents convergence. |
| `WRITE_FILE` | Optional. When `1`, dumps the per-sample temperature trace to `/tmp/heattest.csv` for offline plotting. Default `0`. |
| `PROFILE` | Optional profile name to write the result into when used with the per-heater profile system. Defaults to the active profile (`default` if none has been loaded). |

```
PID_CALIBRATE HEATER=extruder TARGET=240
PID_CALIBRATE HEATER=heater_bed TARGET=80 TOLERANCE=0.015
PID_CALIBRATE HEATER=heater_generic chamber TARGET=55 WRITE_FILE=1
```

When the autotune completes, Klipper writes `pid_kp`, `pid_ki`, `pid_kd`
into the active profile and prompts you to run `SAVE_CONFIG` to persist
them.

## Calibration

1. Bring the heater near room temperature. Cold start gives the cleanest
   relay swing.
2. Run `PID_CALIBRATE HEATER=<name> TARGET=<setpoint>` with the same
   surrounding conditions you'll print under (chamber doors closed, part
   fan duty as during the print — see [heater-pid-profiles](heater-pid-profiles.md)
   if you need separate profiles for fan-on and fan-off).
3. The autotune runs three or more relay cycles, then a verification
   heat-up. The whole sequence takes 5–15 minutes for a hotend, 30–60
   minutes for a heated bed.
4. When the prompt to `SAVE_CONFIG` appears, run it to persist the
   gains. Klipper restarts and the new gains are active.

If the autotune fails to converge within 12 iterations it raises an
error and leaves the existing gains untouched. Common causes: a
thermistor that's loose, a heater wire with a bad crimp introducing
high-frequency noise, or a `TARGET` too close to the heater's `max_temp`
to leave headroom for the relay swing.

## Things to know

- **Targets must leave room for the relay swing.** Pick `TARGET` at least
  20 °C below `max_temp` so the upper relay edge can overshoot without
  tripping the safety interlock. Targets within 5 °C of `max_temp` will
  fail.
- **Convergence is amplitude-based, not time-based.** A heater that
  reaches steady oscillation in two cycles converges in two cycles. A
  noisy heater may need six or seven. There is no fixed iteration count.
- **The verification heat test costs ~2 minutes.** After the relay
  converges, the autotune drives a brief setpoint step and checks that
  the fitted gains track the requested target before writing them back.
  If the step misses by more than the configured tolerance, the autotune
  reports the discrepancy in the log and aborts.
- **`SAVE_CONFIG` writes the active PID profile.** When the per-heater
  profile system is in use, the gains land in whichever profile was last
  loaded (or `default` if none was). To calibrate into a named profile
  in one go, pass `PROFILE=<name>` to `PID_CALIBRATE` — see the
  [PID profiles](heater-pid-profiles.md) doc.
- **The `pid_v` velocity-form controller calibrates with the same
  command.** The fitted gains are valid for both positional (`control:
  pid`) and velocity (`control: pid_v`) controllers; you can switch
  between them after calibration without re-tuning. See
  [heater-pid-velocity](heater-pid-velocity.md).

## Credits

Based on the adaptive-relay autotune work from the Kalico community
(Dan Sorensen, drawing on the Taysom & Sorensen iterative method),
imported and re-derived against the current Klipper `pid_calibrate`
shape. The verification heat test, the `TOLERANCE` parameter, and the
`WRITE_FILE` trace dump are part of the same work.
