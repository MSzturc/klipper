# TMC Drivers

## What this is

Trinamic's TMC stepper drivers (TMC2130, TMC2208/2209, TMC2240, TMC2660, TMC5160) expose dozens of tunable register fields that determine how the chopper switches the motor coils, how StealthChop crosses over to SpreadCycle, how StallGuard reads back-EMF, how CoolStep modulates current under load, and how the regulator quantises run/hold/home currents. Stock Klipper exposes those fields for manual tuning. This fork adds four things on top of the stock driver layer:

1. **A motor-data-driven autotuning subsystem** that derives the chopper, hysteresis, StallGuard, CoolStep, overvoltage, high-velocity, dcStep, and PWM register fields from the motor's electrical specification (resistance, inductance, holding torque, rated current) plus the supply voltage. You point a TMC stepper at a `[motor_constants <name>]` section and a `voltage:` value; everything else is computed at every (re-)apply of the run current.

2. **Three named tuning goals — `performance`, `balanced`, `silent`** — selectable per stepper via `tuning_goal:`. The goal is the macro-level lever above the register fields: it expresses *what the user wants* (max acceleration / daily-driver / minimum noise) and the autotune translates that intent into ~30 coordinated register choices. Default is `balanced`.

3. **A per-stepper homing profile** that swaps the chopper, CoolStep, and IRUN/IHOLD bits to a low-noise / robust-StallGuard configuration for the duration of every homing move, then restores the autotuned run profile afterwards. Independent of the existing `home_current` swap — the chopper change happens even when `home_current == run_current`, because StallGuard noise is a separate concern.

4. **TMC5160 and TMC2240 current-calculation fixes**: the IRUN/IHOLD/GLOBALSCALER math for the two big stepper drivers now ceiling-rounds toward the requested current (so RMS sits at or above target, never silently below), uses a `cs+1` denominator that maximises GLOBALSCALER for fewer quantisation steps per IRUN bit, and exposes a `driver_cs` config option for users who need to override the auto-pick.

The autotune is **opt-in**: drivers that don't specify both `motor:` and `voltage:` keep stock Klipper behaviour. The homing profile is automatic when set up via the workflow above.

The tuning subsystem follows a **convention-over-configuration paradigm with explicit user pins**: the `tuning_goal:` selects a coherent set of defaults, and any individual register field can be pinned via `driver_X:` to override the goal-derived value. The autotune then computes the remaining (unpinned) values around the user's pin. You never have to choose between "all-default" and "set everything by hand" — pin the fields you have an opinion on, let the autotune solve for the rest.

## When to use this

- You are configuring a printer with new motors and don't want to hand-pick TMC chopper / hysteresis / StallGuard / PWM / CoolStep parameters from the datasheet. Set `motor:` and `voltage:`, optionally `tuning_goal:`, and let the driver tune itself for the operating point.
- You are torn between "max torque" and "quiet" and want to flip between them per axis without rewriting your TMC config — set `tuning_goal: performance` on X/Y, `tuning_goal: silent` on Z and E.
- You see noisy or jittery StallGuard triggers and want a cleaner homing signal without giving up your full printing current.
- You run TMC5160 or TMC2240 with a sense_resistor / Rref combination where the previous current math saturated IRUN at 31 with a small GLOBALSCALER, producing audible quantisation noise at low currents (a common complaint on lower-`Rref` TMC2240 boards).
- You run TMC5160 at a high supply voltage (above 52 V) and want the driver to enforce the datasheet's short-detection requirements rather than trust the chip's OTP defaults.
- You have a stepstick (BTT EZ5160, FYSETC 5161, Mellow Fly, …) where you'd rather reference the carrier-board model than look up `sense_resistor` and `max_current` from the silkscreen.

You do not need any of this if your existing stock-Klipper TMC config is working and quiet. Autotune doesn't activate until both `motor:` and `voltage:` are set; the per-driver fixes only change behaviour at very low currents or with unusual `Rref` choices.

## Configuration

### Mandatory `sense_resistor` / `rref`

Every TMC stepper section must specify the carrier-board's sense resistance, either explicitly or via a stepstick lookup. There is no default. A wrong sense resistance silently miscalibrates every IRUN/IHOLD computation and can damage the driver, motor, or board.

| Parameter | Where | Meaning |
|-----------|-------|---------|
| `sense_resistor` | `[tmc2130 stepper_*]`, `[tmc2660 stepper_*]`, `[tmc5160 stepper_*]` | Sense resistor value in ohms. Mandatory. |
| `rref` | `[tmc2240 stepper_*]` | TMC2240 reference resistor (12000–60000 Ω). Mandatory. |
| `stepstick_type` | any TMC stepper section | Carrier-board name; resolves to `(sense_resistor, max_current)` via the stepstick database. Use this *or* `sense_resistor`, not both. On TMC2240 only the `max_current` portion is consumed (the chip uses `rref` instead of a sense resistor); the `sense_resistor` entry is ignored. |

Supported `stepstick_type` values: `REFERENCE_WOTT`, `REFERENCE_2209`, `REFERENCE_5160`, `KRAKEN_2160_8A`, `KRAKEN_2160_3A`, `BTT_2240`, `BTT_EZ_5160_PRO`, `BTT_EZ_5160_RGB`, `BTT_EZ_6609`, `BTT_5160T`, `WOTT_2209`, `COREVUS_2209`, `COREVUS_2160_OLD`, `COREVUS_2160_5A`, `COREVUS_2160`, `FYSETC_2225`, `FYSETC_5161`, `MKS_2226`, `MELLOW_FLY_5160`, `MELLOW_FLY_HV_5160_Pro`. New carriers can be added as `[stepstick <name>]` sections in `config/steppers/database/stepsticks.cfg`.

### Autotuning

Activated when both `motor:` and `voltage:` are present on a TMC stepper section. Without those, every other option below still works as a plain manual override; only the derivation-from-motor-and-current is skipped.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `motor` | unset | Name of a `[motor_constants <name>]` section that describes the motor (resistance, inductance, holding_torque, max_current, steps_per_revolution). `config/steppers/database/motors.cfg` ships specs for ~120 common stepper models and is pulled in automatically by the THEOS base layer. |
| `voltage` | unset | Stepper supply voltage, 0–60 V. Required for back-EMF / PWM-grad math. |
| `tuning_goal` | `balanced` | `performance` / `balanced` / `silent`. See *Tuning goals* below. |
| `pwm_freq_target` | 55 kHz (20 kHz on TMC2240) | StealthChop PWM frequency target, 10–100 kHz. TMC2240 runs hot at the higher target so it defaults lower. |
| `chopper_freq_target` | goal-aware (see *Tuning goals*) | SpreadCycle target chopper frequency, 10–100 kHz. The autotune picks `TOFF` to land at or below this so the chopper sits just above the audible band. User pin always wins over the goal default. |
| `extra_hysteresis` | 0 | Add a fixed amount to the computed `HSTRT/HEND` sum (0–15). Useful when the auto-derived hysteresis sounds too aggressive. |
| `overvoltage_vth` | unset | Programmed overvoltage trip in volts (0–60). When set, `OVERVOLTAGE_VTH` register is written. |
| `coolstep_threshold` | unset | Velocity (in mm/s) above which CoolStep activates. Translated to TCOOLTHRS in TSTEP units at config-time. Mutually exclusive with `driver_TCOOLTHRS:`. |
| `high_velocity_threshold` | unset | Velocity (in mm/s) above which the high-velocity register set engages (THIGH). Mutually exclusive with `driver_THIGH:`. |
| `stealthchop_threshold` | unset | Velocity (in mm/s) below which StealthChop is active. Translated to TPWMTHRS at config-time. Three-tier precedence: `driver_TPWMTHRS` > `stealthchop_threshold` > goal-default. |

The `[motor_constants <name>]` sections (in `config/steppers/database/motors.cfg`) cover the major stepper-motor manufacturers: LDO, Moons, OMC, Stepperonline, Wantai, MOTECH, Trinamic, Phidgets, Soyo, BTT, BIQU, etc. Each section provides `resistance`, `inductance`, `holding_torque`, `max_current`, and `steps_per_revolution` from the manufacturer's datasheet.

### Tuning goals

The three tuning goals encapsulate three different operating points. Pick one per stepper based on what the axis is doing, not based on the motor type — the same motor wants different settings at the X/Y carriage versus the Z lead-screw.

- **`performance`** — max acceleration / torque headroom. SpreadCycle dominant, dcStep + VHIGH-Fullstep active above THIGH so the driver keeps torque at top speed by switching to step-pulse-counting. Audible SpreadCycle hissing as background noise. Pick this for X/Y on a CoreXY when you push high accelerations and care about not skipping steps.
- **`balanced`** (default) — daily-driver. StealthChop low-speed, SpreadCycle at print speed, no dcStep, smoothed CoolStep (`sfilt=1`, `iholddelay=10`) so coil current does not breathe under varying load. Pick this when you don't have a strong preference; it's a good compromise for hours-long prints. This is the default for any autotune-active stepper section without `tuning_goal:`.
- **`silent`** — minimum audible noise. StealthChop until the physical PWM-tracking limit (1.2 × vmaxpwm), CoolStep disabled (`semin=0`) so there is no current-modulation noise, `small_hysteresis=1` for smoother microstep transitions, `TPFD=0` to remove passive-fast-decay artefacts, ultrasonic chopper target (45 kHz) above human hearing. Significantly reduced peak torque, no dcStep step-loss protection. Pick this for Z and extruder where audible noise dominates and the load is light.

The goal is a **starting point**, not a constraint: any field listed in *Pin-respecting field overrides* below can be pinned via `driver_X:` to override the goal default. The autotune solves for the unpinned fields around your pin.

#### Goal comparison reference

The following table is the canonical reference for what each goal sets at the register level. **User pins via `driver_X` always override the goal default.** Empty cells (`—`) are inactive in that goal because of an upstream gate (e.g. CoolStep fields when `semin=0`).

| Aspect | `performance` | `balanced` (default) | `silent` |
|---|---|---|---|
| **StealthChop / SpreadCycle crossover** | | | |
| TPWMTHRS | `0xfffff` (StealthChop never) | `0.3 × vmaxpwm` | `1.2 × vmaxpwm` |
| THIGH | `1.2 × vmaxpwm × rotation_distance` | `0` (window open) | `0` (window open) |
| vhighfs / vhighchm | True / True (dcStep active) | False / False | False / False |
| dc_time / dc_sg | auto from motor (AN-003 §4) | inactive (vhighchm=0) | inactive |
| **PWM tuning** | | | |
| chopper_freq_target | 20 kHz | 35 kHz | 45 kHz (ultrasonic) |
| pwm_reg | 15 (fast PI) | 8 (moderate) | 4 (smooth) |
| pwm_lim | 4 | 8 | 12 |
| pwm_autoscale / pwm_autograd | True / True | True / True | True / True |
| **Hysteresis / resonance** | | | |
| TPFD | computed | computed | 0 |
| small_hysteresis | False | False | True |
| **CoolStep** | | | |
| semin | 2 | 2 | 0 (CoolStep off) |
| semax | 4 | 4 | — |
| seup | 3 | 3 | — |
| sedn | 2 | 2 | — |
| seimin | 1 (¼ IRUN) | 1 | — |
| sfilt | 0 (fast SG response) | 1 (smoothed) | — |
| **Standstill** | | | |
| iholddelay | 12 | 10 | 12 |
| tpowerdown | 10 | 10 | 10 |
| **GCONF flags** | | | |
| faststandstill | True | True | True |
| multistep_filt | True | True | True |

**THIGH and the CoolStep activation window:** THIGH=0 keeps the CoolStep window open at all velocities. Per the TMC5160 datasheet, CoolStep is active when `TCOOLTHRS ≥ TSTEP > THIGH`; with THIGH=0 the condition `TSTEP > 0` is always satisfied, so CoolStep remains active up to the physical limit set by TCOOLTHRS. Using `THIGH=0xfffff` would close the window (TSTEP > 0xfffff is essentially never true), suppressing CoolStep regardless of `semin`. This matches the Klipper upstream `TMCVhighHelper` convention (default THIGH=0). For `silent`, CoolStep is disabled via `semin=0` regardless; THIGH=0 is used for consistency with the upstream convention.

### Pin-respecting field overrides

Any of the following register fields can be pinned in the config via `driver_X:`. The autotune treats your pin as a hard constraint and computes the remaining (unpinned) values around it. This is the same mechanism that already existed for `driver_TBL/TOFF/TPFD/HSTRT/HEND/SGTHRS/SGT/cs`; what changed is that the driver now respects pins consistently across PWM, GCONF, CoolStep, and high-velocity registers as well.

| Group | Fields |
|---|---|
| Chopper | `driver_TBL`, `driver_TOFF`, `driver_TPFD`, `driver_HSTRT`, `driver_HEND` |
| StallGuard | `driver_SGTHRS`, `driver_SGT`, `driver_cs` |
| PWM / StealthChop | `driver_PWM_FREQ`, `driver_PWM_AUTOSCALE`, `driver_PWM_AUTOGRAD`, `driver_PWM_GRAD`, `driver_PWM_OFS`, `driver_PWM_REG`, `driver_PWM_LIM`, `driver_TPWMTHRS` |
| GCONF flags | `driver_FASTSTANDSTILL`, `driver_SMALL_HYSTERESIS`, `driver_MULTISTEP_FILT` |
| CoolStep + IHOLDDELAY | `driver_SEMIN`, `driver_SEMAX`, `driver_SEUP`, `driver_SEDN`, `driver_SEIMIN`, `driver_SFILT`, `driver_IHOLDDELAY` |
| Velocity thresholds + high-speed | `driver_TCOOLTHRS`, `driver_THIGH`, `driver_VHIGHFS`, `driver_VHIGHCHM` |
| TMC5160 DRV_CONF | `driver_FILT_ISENSE`, `driver_OTSELECT`, `driver_DRVSTRENGTH`, `driver_BBMTIME`, `driver_BBMCLKS` |
| TMC5160 DCCTRL (dcStep) | `driver_DC_TIME`, `driver_DC_SG` |
| TMC5160 SHORT_CONF | `driver_S2VS_LEVEL`, `driver_S2G_LEVEL`, `driver_SHORT_FILTER`, `driver_SHORTDELAY` |

**Three-tier precedence for velocity-form thresholds.** The autotune resolves TCOOLTHRS, THIGH, and TPWMTHRS in this order:

1. Raw register pin (`driver_TCOOLTHRS:`, `driver_THIGH:`, `driver_TPWMTHRS:`) — wins unconditionally.
2. Velocity-form pin (`coolstep_threshold:`, `high_velocity_threshold:`, `stealthchop_threshold:`) — translated from mm/s to TSTEP at config-time.
3. Goal-default — derived from motor and `tuning_goal:`.

Setting both the raw pin and the velocity-form pin for the same threshold is mutually exclusive and raises a `config error` at boot — the driver tells the user which knob to pick rather than silently letting one win.

### TMC5160 / TMC2240 driver current

Both drivers gained two new options as part of the current-calculation rework.

| Parameter | Default | Where | Meaning |
|-----------|---------|-------|---------|
| `driver_cs` | unset = auto | `[tmc5160 …]`, `[tmc2240 …]` | Override the auto-picked CS (current-scale) bits, 0–31. With `driver_cs` unset, the driver picks the smallest CS that delivers the requested run current at the active GLOBALSCALER, maximising the regulator's resolution. |
| `current_range` | smallest range that covers `run_current` | `[tmc2240 …]` | TMC2240 only. Locks a higher CURRENT_RANGE for headroom; cannot be set below the auto-picked floor. |

### TMC5160 high-VS sense filter (`filt_isense`)

Per TMC5160 datasheet §6, at supply voltages above 52 V the sense-line ringing during chopper switching is large enough that the chip's 1 µs `filt_isense` filter measurably cleans up hysteresis regulation. The driver now picks a VS-aware default:

- `voltage > 52 V` → `filt_isense = 1` (filter on)
- `voltage ≤ 52 V` → `filt_isense = 0` (filter off, matches chip reset)
- `driver_FILT_ISENSE:` overrides either default.

This ships even when the autotune is off (no `motor:`); the driver re-reads `voltage:` at config-load time so a pure manual config still gets the high-VS-aware default.

### TMC5160 short-detection thresholds

TMC5160 only. Setting both fields programs the chip's `SHORT_CONF` register; setting one without the other is a config error because the chip's OTP defaults are not readable.

| Parameter | Range | Meaning |
|-----------|-------|---------|
| `driver_s2vs_level` | 4–15 | Short-to-supply trip level. |
| `driver_s2g_level` | 2–15 | Short-to-ground trip level. |

When set, also fills `short_filter` and `shortdelay` from defaults; can be overridden via `driver_short_filter` / `driver_shortdelay`.

#### High-VS safety enforcement

Per TMC5160 datasheet §6.3, when the supply voltage exceeds 52 V the `s2g_level` field must be ≥ 12 to avoid false short-to-GND faults during normal high-voltage operation. The driver enforces this at boot:

- `driver_S2G_LEVEL: <value>` with `<value> < 12` and `voltage > 52` raises a `config error` with a clear remediation message (raise the level, or lower the supply voltage).
- `voltage > 52` without explicit SHORT_CONF programming emits a startup warning. The chip's OTP defaults are usually workable but the datasheet recommends explicit programming. Silence the warning by setting both `driver_S2VS_LEVEL` (≥ 4) and `driver_S2G_LEVEL` (≥ 12).

Pure footgun-prevention; no performance impact for properly-configured printers.

### TMC5160 dcStep tuning (DCCTRL)

dcStep is the TMC5160's step-loss-protection mode: above THIGH, with `vhighfs` and `vhighchm` both set, the chip uses load-angle feedback to count step pulses rather than rely on the chopper to deliver each microstep. The two tuning fields live in the DCCTRL register (`0x6E`):

| Field | Auto-derived as | Source |
|---|---|---|
| `dc_time` | motor commutation time, `t = L × Ipeak / V` (single-pole RL approximation) | TMC AN-003 §4.1 |
| `dc_sg` | `dc_time / 16` | TMC AN-003 §4.2 default |

`driver_DC_TIME:` and `driver_DC_SG:` override the derivation. Active under `tuning_goal: performance` where vhighfs and vhighchm are both enabled; the values are written for deterministic register state under `balanced` / `silent` but inactive there.

### CHOPCONF boundary validation

Two TMC5160 datasheet constraints are enforced at config-load time, regardless of whether the autotune is active:

- **TOFF=1 with TBL=0 is rejected** (datasheet §5.2 CHOPCONF). When the user explicitly pins both `driver_TOFF: 1` and `driver_TBL: 0`, the driver raises a `config error` rather than silently bumping TBL to 1. When only one of the two is pinned, the autotune is allowed to correct the other.
- **dcStep with `TOFF < 3` is rejected** (datasheet §13.2). When `driver_VHIGHFS: 1`, `driver_VHIGHCHM: 1`, and `driver_TOFF: <3>` are all pinned, the driver raises a `config error` — dcStep needs at least three chopper cycles to establish load-angle feedback.

### Per-stepper homing profile

Applied around every homing move on this stepper section, regardless of whether sensorless homing is in use. The defaults are conservative low-noise SpreadCycle settings tuned for clean StallGuard reads. Each option overrides the corresponding default; unset means "use the bundled default".

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `homing_toff` | 3 | Chopper TOFF during homing, 1–15. |
| `homing_tbl` | 1 | Chopper TBL (blanking time) during homing, 0–3. |
| `homing_hstrt` | 0 | Chopper HSTRT during homing, 0–7. |
| `homing_hend` | 4 | Chopper HEND during homing, 0–15. |
| `homing_tpfd` | 4 | Passive fast-decay during homing, 0–15. |
| `homing_sfilt` | 0 | StallGuard filter during homing, 0 or 1. Set to 1 if SG signal looks noisy on a heavy or high-resonance axis. |
| `homing_cs` | 8 | Low-noise CS bits used by `_calc_homing_current` on TMC5160 / TMC2240. Lower CS = larger GLOBALSCALER = finer regulator step, at the cost of a smaller dynamic range; the driver auto-bumps CS if the requested homing current cannot fit at the configured value. |

The homing profile additionally turns CoolStep off (`semin/semax/seup/sedn/seimin = 0`), enables fast-standstill, and slows IHOLDDELAY to 8 — these aren't user-configurable but follow the same low-noise rationale. The homing profile is orthogonal to `tuning_goal:` — both performance and silent steppers home with the same low-noise chopper config; only the run profile changes between goals.

## Minimal example

A single TMC2240-driven X axis on a BTT 2240 stepstick at 24 V, with autotuning, sensorless homing, and the per-stepper homing profile:

```ini
# The motor and stepstick databases load automatically via the THEOS base
# layer, so `motor:` and `stepstick_type:` resolve without any extra include.

[tmc2240 stepper_x]
uart_pin: PB1
diag1_pin: ^!PE5
stepstick_type: BTT_2240
rref: 12000

motor: ldo-42sth48-2504ah
voltage: 24
# tuning_goal: balanced is the default

# Optional manual overrides — leave commented to autotune.
# driver_cs: 22
# driver_TOFF: 4
# extra_hysteresis: 2

# Per-stepper homing profile overrides (defaults shown).
# homing_sfilt: 1     # enable SG filter if signal is noisy

run_current: 1.4
home_current: 0.6

[stepper_x]
endstop_pin: tmc2240_stepper_x:virtual_endstop
homing_speed: 50
```

A TMC5160 axis configured for max acceleration with a custom dcStep sensitivity:

```ini
[tmc5160 stepper_x]
spi_bus: spi1
cs_pin: PA15
diag1_pin: ^!PE0
stepstick_type: BTT_EZ_5160_PRO
sense_resistor: 0.075

motor: ldo-42sth48-2504ac
voltage: 48
tuning_goal: performance

driver_DC_SG: 16     # tighten dcStep load-angle threshold
driver_S2VS_LEVEL: 6
driver_S2G_LEVEL: 12

run_current: 2.0
home_current: 0.8
```

A TMC5160 Z lead-screw configured for absolute quiet, with CoolStep forced on despite the silent default:

```ini
[tmc5160 stepper_z]
# ... usual SPI / pins / sense_resistor ...

motor: ldo-42sth48-2504ac
voltage: 48
tuning_goal: silent
driver_SEMIN: 4      # enable CoolStep; autotune picks SEMAX/SEUP/SEDN to fit

run_current: 0.8
home_current: 0.4
```

## Reading the autotune log

When autotune is active, `klippy.log` contains one summary line per phase per re-apply. The `goal=` annotation on the PWM line shows which `tuning_goal` is in effect.

```
tmc stepper_x autotune: tuning for 1.400A at 24V (clock 12.500 MHz)
tmc stepper_x autotune (goal=balanced): pwm_freq=2 (~12.2 kHz) pwmgrad=43 pwmofs=27 pwm_reg=8 pwm_lim=8 tpwmthrs=812
tmc stepper_x autotune: tbl=1 toff=4 tpfd=4
tmc stepper_x autotune: Ipeak=1.980A tblank=2.88us tsd=11.68us cs=22 hysteresis=2 hstrt=2 hend=3
```

When a homing move starts:

```
tmc stepper_x homing: cs=8 gs=243 (raw 243) irun=8 ihold=8 (target 0.600A)
```

Use these to confirm the autotune picked sensible values for your operating point, and to verify that switching `tuning_goal:` produced the register changes you expected.

## Migration

Configs without `motor:` / `voltage:` (autotune off) are bit-identical across the upgrade — the addition of static `set_config_field` defaults for fields like `tpwmthrs`, `faststandstill`, `filt_isense` only writes a value when the user pinned the corresponding `driver_X:`, so plain manual configs see no change.

Configs with autotune ON (both `motor:` and `voltage:` set) but no explicit `tuning_goal:` now get `balanced` as the default. This differs from the previous max-performance behaviour at the following fields:

| What was | What is now (`balanced` default) | Effect |
|---|---|---|
| TPWMTHRS = 0xfffff (SpreadCycle always) | TPWMTHRS = 0.3 × vmaxpwm | Noticeably quieter at low print speeds |
| pwm_reg=15, pwm_lim=4 | pwm_reg=8, pwm_lim=8 | Smoother current behaviour, less PI overshoot |
| sfilt=0 | sfilt=1 | More stable CoolStep, no "current breathing" |
| iholddelay=12 | iholddelay=10 | Marginally faster hold-drop |
| vhighfs / vhighchm active | both off | No dcStep / VHIGH-fullstep above THIGH |
| chopper_freq_target = 20 kHz | 35 kHz | Less SpreadCycle hissing, slightly more switching loss |

**If you were happy with the prior behaviour and want maximum performance, set `tuning_goal: performance` explicitly in your X/Y `[tmc5160 …]` sections.** That gives you bit-identical behaviour to before.

For Z and extruder, `tuning_goal: silent` is usually a strict upgrade unless you specifically rely on Z-axis CoolStep load modulation.

## Compatibility

- Existing TMC sections continue to work without changes as long as they specify `sense_resistor` (or `rref` for TMC2240) — the only mandatory tightening. Configs that previously relied on the implicit defaults need an explicit value or a `stepstick_type:` line added.
- All previous `driver_*` field overrides remain available and take precedence over autotune-derived values. The expanded set in *Pin-respecting field overrides* extends, rather than replaces, the prior `driver_TBL/TOFF/TPFD/HSTRT/HEND/SGTHRS/SGT/cs` list.
- You can adopt the new tuning goals incrementally: leave existing axes untouched (they get `balanced`), set `tuning_goal: performance` on axes where the prior behaviour was specifically tuned for max torque.
- The per-stepper homing profile is always active. To disable it on a specific axis, set every `homing_*` option to the value the autotune would have written for the run profile.
- The CHOPCONF boundary validation (TOFF=1+TBL=0, dcStep+TOFF<3) is unconditional and fires for autotune-OFF configs too. It only triggers when the user pins both fields of a violating combination explicitly; existing configs that rely on autotune to choose one of the two are unaffected.
