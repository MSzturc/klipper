# PID Profiles

## What this is

Stock Klipper stores one PID gain set per heater, hardcoded in the
heater's config section. To run different gains for different conditions
— part-fan on vs. off, ABS chamber temperature vs. PETG, a thinner
print-bed sheet vs. the stock one — you'd edit the config and restart
Klipper, or live-tune with `SET_HEATER_PID` and lose the gains on
restart.

This fork adds a **per-heater profile system**. Each heater section can
hold any number of named profiles in dedicated `[pid_profile <heater>
<name>]` config sections; one profile is active at a time and can be
swapped at runtime. Calibrating, saving, loading, and removing profiles
all work via G-code, with `SAVE_CONFIG` persisting the result.

The profile system also exposes per-profile control of the derivative
smoothing window (`smooth_time`) so a high-mass heater that needs heavy
smoothing can be tuned independently of a hotend that does not.

## When to use this

- You print materials with very different heater requirements (PLA vs.
  ABS, with vs. without an enclosure) and want a one-line gcode switch
  between tuned gain sets.
- You added or replaced a heated bed surface (PEI vs. flexible, magnetic
  swap, glass) and want a profile per surface.
- You're running with and without a part-cooling fan, and the heater
  needs different gains in each regime.
- You want a calibration record. Each profile carries its own `pid_Kp`/
  `Ki`/`Kd`, `smooth_time`, and `pid_target`/`pid_tolerance`, so you can
  trace which conditions a profile was tuned for.

You don't need profiles if a single PID tune is good enough for every
condition you care about. The default profile is created automatically
from the heater's existing `pid_Kp`/`Ki`/`Kd` config keys, so existing
configs keep working.

## Configuration

The heater's main config section continues to declare its gains as
before. Extra profiles are added as sibling `[pid_profile <heater_short>
<profile_name>]` sections.

### Heater section (existing)

Treat the heater section as the implicit `default` profile. The
existing keys stay valid:

| Parameter | Meaning |
|-----------|---------|
| `pid_Kp`, `pid_Ki`, `pid_Kd` | Default-profile gains. |
| `pid_target` | Optional target to record against the default profile. |
| `pid_tolerance` | Optional convergence tolerance recorded with the default profile. |
| `smooth_time` | Optional derivative smoothing window in seconds. |

### `[pid_profile <heater> <profile_name>]`

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `pid_version` | `1` | Profile schema version. Bumped when a future profile change is incompatible. |
| `control` | `pid` | Algorithm for this profile: `pid`, `pid_v`, or `dual_loop_pid`. |
| `pid_Kp`, `pid_Ki`, `pid_Kd` | required | Profile gains. |
| `pid_target` | unset | Calibration target temperature recorded with the profile. |
| `pid_tolerance` | unset | Convergence tolerance recorded with the profile. |
| `smooth_time` | inherits heater | Per-profile smoothing window. |
| `inner_pid_kp`, `inner_pid_ki`, `inner_pid_kd` | required for `dual_loop_pid` | Inner-loop gains for dual-loop profiles only. |

### Minimal example

A bed with two profiles — quiet (slow ramp, gentler gains) and
aggressive (fast ramp, accepts overshoot):

```ini
[heater_bed]
heater_pin: PA1
sensor_type: ATC Semitec 104GT-2
sensor_pin: PF3
control: pid
pid_Kp: 54.0
pid_Ki: 1.20
pid_Kd: 600
min_temp: 0
max_temp: 110

[pid_profile heater_bed aggressive]
pid_Kp: 72.0
pid_Ki: 2.10
pid_Kd: 480
smooth_time: 0.8
```

The default profile is the heater section itself; the `aggressive`
profile lives in its own section and is loaded via
`PID_PROFILE LOAD=aggressive HEATER=heater_bed`.

## G-code commands

### `PID_PROFILE`

Multiplexed by `HEATER`; the action is selected by which optional
parameter is set. Exactly one of `LOAD`, `SAVE`, `REMOVE` is required.

| Parameter | Meaning |
|-----------|---------|
| `HEATER` | Section short name of the heater (e.g. `extruder`, `heater_bed`). |
| `LOAD` | Profile name to switch the heater to. Activates the named profile's gains immediately. |
| `SAVE` | Profile name to write the heater's *current* gains into. If the profile didn't exist, it is created; if it did, it is overwritten. Use `SAVE_CONFIG` afterwards to persist. |
| `REMOVE` | Profile name to delete. Refuses to delete the active profile. |
| `KEEP_TARGET` | When loading, set to `1` to keep the heater's current target temperature (default `0` resets target to 0). |
| `LOAD_CLEAN` | When loading, set to `1` to reset the controller's internal state (integral term, last derivative). Default `0` carries state across the swap. |

```
PID_PROFILE LOAD=aggressive HEATER=heater_bed
PID_PROFILE SAVE=fan_full HEATER=extruder
PID_PROFILE REMOVE=old_petg HEATER=heater_bed
```

A typical sequence: tune with `PID_CALIBRATE`, save the result into a
named profile, switch back to default for normal printing, load the
named profile only when the relevant condition holds.

### `PID_CALIBRATE` with profiles

`PID_CALIBRATE` accepts an optional `PROFILE=<name>` parameter. When
set, the calibrated gains are written into that named profile instead of
the active one. Useful for tuning a profile from scratch:

```
PID_CALIBRATE HEATER=heater_bed TARGET=80 PROFILE=PETG
SAVE_CONFIG
```

The above runs the autotune and writes the result straight into a new
`[pid_profile heater_bed PETG]` section; no need to load the profile
first.

### `SET_SMOOTH_TIME`

Adjust the derivative smoothing window of the active profile, optionally
saving back to the profile.

| Parameter | Meaning |
|-----------|---------|
| `HEATER` | Section short name of the heater. |
| `SMOOTH_TIME` | New smoothing window in seconds. Omit to reset to the heater's configured `smooth_time`. |
| `SAVE_TO_PROFILE` | Set to `1` to write the new value into the active profile (still requires `SAVE_CONFIG` to persist). Default `0` updates only the live controller. |

```
SET_SMOOTH_TIME HEATER=heater_bed SMOOTH_TIME=1.2
SET_SMOOTH_TIME HEATER=extruder SMOOTH_TIME=0.4 SAVE_TO_PROFILE=1
```

## Things to know

- **The default profile is the heater section.** No separate
  `[pid_profile heater_bed default]` section is created — that profile
  *is* the heater section. `PID_PROFILE LOAD=default HEATER=<heater>` switches
  back to whatever gains are written in the heater section itself.
- **`SAVE_CONFIG` is required after `PID_PROFILE SAVE` and after
  `PID_CALIBRATE PROFILE=`.** Without it, the new profile lives only
  until restart.
- **Profile keys are case-insensitive on read.** Older configs that
  saved gains as `pid_Kp` (mixed case) keep working; the profile system
  normalises to lowercase (`pid_kp`) when writing back.
- **Loading a profile preserves the integral state by default.**
  `LOAD_CLEAN=1` is useful when a stale integral from the previous
  profile would push the new tune off-target on the first cycle —
  typical when the heater has been idle for a while.
- **`PID_PROFILE REMOVE` cannot delete the active profile.** Switch to
  another profile first (or `PID_PROFILE LOAD=default HEATER=<heater>`), then remove.
- **`smooth_time` per profile changes derivative behaviour, not
  scheduling.** Klipper still samples and recomputes PWM at the
  heater's native rate; `smooth_time` only widens the moving-average
  window the derivative term reads from.

## Credits

Based on the per-heater PID-profile work from the Kalico community
(profile system in PR #162 by Vladimir Vukicevic, with Tomas Petrlik's
follow-up fixes), imported and re-derived against the current Klipper
heater control class shape. The `PID_CALIBRATE PROFILE=` integration is
part of the same port.
