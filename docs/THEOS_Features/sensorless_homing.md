# Sensorless Homing

## What this is

StallGuard-based sensorless homing lets a Trinamic stepper driver (TMC2130, TMC2209, TMC2240, TMC2660, TMC5160) detect the end of travel from the motor's own back-EMF signature when it stalls against a physical limit — no dedicated endstop switch required. Stock Klipper already ships the virtual-endstop plumbing for this. What this fork adds is the operational polish the virtual-endstop path needs to be reliable on real printers: a separate motor current while homing, a minimum-travel check that rehomes the axis when the first trigger fires too close to the starting position, and a handful of safety rails around the current swap.

Concretely, this fork adds:

1. **`home_current` per `[tmc_*]`** — a distinct run current programmed only for the duration of a homing move. Lower current produces a cleaner StallGuard event on most motors; you keep your full printing current for everything else.
2. **`use_sensorless_homing` per `[stepper_*]`** — an explicit switch for the extra homing behaviour. Defaults to true when the rail's `endstop_pin` names a `tmc*:virtual_endstop`.
3. **`min_home_dist` per `[stepper_*]`** — a lower bound on the distance the first homing pass must travel before the StallGuard trigger is considered reliable. If the axis trips short of this distance, the homing loop retracts against `min_home_dist` and performs a second pass.
4. **`homing_accel` per `[stepper_*]`** — an optional override for the toolhead's `max_accel` that is active only during homing moves on this rail. The printing acceleration is restored automatically when homing finishes, including when homing errors out.

Stock Klipper's homing behaviour is unchanged when `use_sensorless_homing` is off. None of the additions affect rails that home against a physical switch.

## When to use this

- You run TMC2130/2209/2240/2660/5160 drivers and want to drop physical endstops from one or more axes (common on CoreXY/Voron-style gantries, dual-Z beds, IDEX carriages where mechanical endstops are awkward).
- You already use `virtual_endstop` on an axis and occasionally see it "home" about a millimetre in from the true limit — that is StallGuard triggering on a chassis resonance partway through the approach. `min_home_dist` fixes it by insisting on a minimum first-pass travel.
- You want a gentler homing motion on a heavy axis without lowering your printing current. `home_current` lets you drop the amps just for homing.
- You home a Z axis against a strain-sensitive bed and want a different acceleration profile only during the homing move. `homing_accel` lets you do that without touching `max_accel` globally.

You do not need any of this if all your axes home against physical switches. Stock Klipper handles that case just fine.

## Configuration

The new parameters split between the `[tmc_*]` section (current handling) and the `[stepper_*]` section (homing mechanics).

### TMC driver current

Parameters on `[tmc2130 stepper_*]`, `[tmc2209 stepper_*]`, `[tmc2240 stepper_*]`, `[tmc2660 stepper_*]`, and `[tmc5160 stepper_*]`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `home_current` | same as `run_current` | Motor current (amps RMS) programmed on the driver for the duration of a homing move. Must be above 0 and no greater than the driver's maximum. If equal to `run_current`, no current swap is performed. |
| `current_change_dwell_time` | `0.5` | Dwell (seconds) inserted between programming the new current and starting the homing move. Gives the driver time to settle. |

### Rail homing mechanics

Parameters on `[stepper_x]`, `[stepper_y]`, `[stepper_z]`, `[stepper_x1]`, etc.:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `use_sensorless_homing` | auto-`True` when `endstop_pin` contains `:virtual_endstop` (except `:z_virtual_endstop` from a probe); otherwise `False` | Enables the extra SL-homing behaviour on this rail. When False, the rail homes with stock Klipper semantics. |
| `min_home_dist` | `homing_retract_dist` | Minimum distance (mm) the first homing pass must travel before the trigger is considered reliable. If the pass trips short, the rail retracts against `min_home_dist` and rehomes. |
| `homing_accel` | unset | When set, programs `max_accel` on the toolhead for the duration of homing moves on this rail. Restored to the configured `max_accel` when homing finishes. Must be above 0. |
| `second_homing_speed` | `homing_speed / 2` normally; `homing_speed` when `use_sensorless_homing` is True | When SL-homing, the second pass runs at the same speed as the first. StallGuard sensitivity depends on speed, so a half-speed retest produces unreliable triggers. |

### Minimal example

```ini
[stepper_x]
step_pin: PF0
dir_pin: !PF1
enable_pin: !PD7
rotation_distance: 40
microsteps: 16
endstop_pin: tmc2209_stepper_x:virtual_endstop
position_endstop: 0
position_max: 250
homing_speed: 30
homing_retract_dist: 0
min_home_dist: 10

[tmc2209 stepper_x]
uart_pin: PK0
run_current: 0.800
home_current: 0.400
diag_pin: ^PG6
driver_SGTHRS: 90
```

`homing_retract_dist: 0` is the typical pairing — SL-homing reliability comes from `min_home_dist` rather than from a second pass at reduced retract, so once the first trigger satisfies `min_home_dist` there's no benefit to a second approach.

## G-code commands

### `SET_TMC_CURRENT`

Stock Klipper's `SET_TMC_CURRENT` gains one parameter:

| Parameter | Meaning |
|-----------|---------|
| `HOMECURRENT` | New home current (amps RMS) for this stepper. Takes effect on the next homing move; does not change the current right now. |

```
SET_TMC_CURRENT STEPPER=stepper_x HOMECURRENT=0.35
```

The existing `CURRENT` and `HOLDCURRENT` parameters behave as upstream. All three can be set independently in a single command.

## Things to know

- **StallGuard sensitivity depends on motor speed.** Hence the `second_homing_speed == homing_speed` default for SL-homing rails. Overriding `second_homing_speed` manually on an SL-homing rail is allowed but rarely advisable.
- **The 0.5 mm rehome tolerance is hardcoded.** A first-pass travel within 0.5 mm of `min_home_dist` is accepted; shorter triggers force a rehome. The tolerance absorbs sub-millimetre StallGuard jitter from motor temperature, belt backlash, and current ripple that would otherwise cause unnecessary rehome cycles without improving positional accuracy.
- **Current restoration is guaranteed on errors.** If the homing move raises (timeout, endstop lost, shutdown), the driver's `run_current` is restored in a `finally` block before the error propagates. A homing failure does not leave the motor running at `home_current`.
- **Dual-stepper rails swap current per driver.** A `[stepper_x]` with a `[stepper_x1]` twin, each on its own TMC driver, will have both drivers' `home_current` programmed for the homing pass. The post-home dwell fires once across the whole batch, not once per driver.
- **`thigh` save/restore is already in stock Klipper.** SL-homing works on drivers whose spreadcycle/stealthchop transition is tuned via `thigh`; stock Klipper saves and restores the field around the homing window. No extra configuration is needed.
- **`:z_virtual_endstop` is not sensorless homing.** That pin name is Klipper's probe virtual endstop; it does not turn on SL-homing defaults. Set `use_sensorless_homing: True` manually if you have a genuinely sensorless Z axis.

## Credits

This fork's sensorless homing polish is a re-derivation on the current Klipper base of the work the Kalico project assembled in [PR #65](https://github.com/KalicoCrew/kalico/pull/65) (Brandon Nance, Bea Nance), [PR #90](https://github.com/KalicoCrew/kalico/pull/90) (with acknowledgement to fbeauKmi and nielsvz), [PR #109](https://github.com/KalicoCrew/kalico/pull/109) and [PR #110](https://github.com/KalicoCrew/kalico/pull/110) (Brandon Nance), [PR #117](https://github.com/KalicoCrew/kalico/pull/117) and [PR #120](https://github.com/KalicoCrew/kalico/pull/120) (dwell-defer fix), [PR #213](https://github.com/KalicoCrew/kalico/pull/213) and [PR #739](https://github.com/KalicoCrew/kalico/pull/739) (Zeanon — home-current tracking and the `needs_hold_current_change` fix), [PR #236](https://github.com/KalicoCrew/kalico/pull/236) (Rogerio Goncalves — consolidated set_current refactor), [PR #474](https://github.com/KalicoCrew/kalico/pull/474) (Rogerio Goncalves — homing_accel), and [PR #549](https://github.com/KalicoCrew/kalico/pull/549) (Rogerio Goncalves — second_homing_speed alignment), plus Ruiqi Mao's endstop-state-reset idea from an earlier Kalico exchange. The `thigh` save/restore it builds on is upstream Klipper work by Kevin O'Connor. On this fork, the Kalico ideas were re-implemented against Klipper's current `TMCCommandHelper` layout and the renamed `GenericPrinterRail` endstop model rather than cherry-picked, so the behaviour matches Kalico's without carrying its base-state assumptions.
