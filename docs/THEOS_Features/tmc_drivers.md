# TMC Drivers

## What this is

Trinamic's TMC stepper drivers (TMC2130, TMC2208/2209, TMC2240, TMC2660, TMC5160) expose dozens of tunable register fields that determine how the chopper switches the motor coils, how StallGuard reads back-EMF, and how the regulator quantises the run/hold/home currents. Stock Klipper exposes those fields for manual tuning. This fork adds three things on top of the stock driver layer:

1. **A motor-data-driven autotuning subsystem** that derives the chopper, hysteresis, StallGuard, CoolStep, overvoltage, and high-velocity register fields from the motor's electrical specification (resistance, inductance, holding torque, rated current) plus the supply voltage. You point a TMC stepper at a `[motor_constants <name>]` section and a `voltage:` value; everything else is computed at every (re-)apply of the run current.

2. **A per-stepper homing profile** that swaps the chopper, CoolStep, and IRUN/IHOLD bits to a low-noise / robust-StallGuard configuration for the duration of every homing move, then restores the autotuned run profile afterwards. Independent of the existing `home_current` swap — the chopper change happens even when `home_current == run_current`, because StallGuard noise is a separate concern.

3. **TMC5160 and TMC2240 current-calculation fixes**: the IRUN/IHOLD/GLOBALSCALER math for the two big stepper drivers now ceiling-rounds toward the requested current (so RMS sits at or above target, never silently below), uses a `cs+1` denominator that maximises GLOBALSCALER for fewer quantisation steps per IRUN bit, and exposes a `driver_cs` config option for users who need to override the auto-pick.

The autotune is **opt-in**: drivers that don't specify both `motor:` and `voltage:` keep stock Klipper behaviour. The homing profile is automatic when set up via the workflow above.

## When to use this

- You are configuring a printer with new motors and don't want to hand-pick TMC chopper / hysteresis / StallGuard parameters from the datasheet. Set `motor:` and `voltage:` and let the driver tune itself for the operating point.
- You see noisy or jittery StallGuard triggers and want a cleaner homing signal without giving up your full printing current.
- You run TMC5160 or TMC2240 with a sense_resistor / Rref combination where the previous current math saturated IRUN at 31 with a small GLOBALSCALER, producing audible quantisation noise at low currents (a common complaint on lower-`Rref` TMC2240 boards).
- You have a stepstick (BTT EZ5160, FYSETC 5161, Mellow Fly, …) where you'd rather reference the carrier-board model than look up `sense_resistor` and `max_current` from the silkscreen.

You do not need any of this if your existing stock-Klipper TMC config is working and quiet. Autotune doesn't activate until both `motor:` and `voltage:` are set; the per-driver fixes only change behaviour at very low currents or with unusual `Rref` choices.

## Configuration

### Mandatory `sense_resistor` / `rref`

Every TMC stepper section must specify the carrier-board's sense resistance, either explicitly or via a stepstick lookup. There is no default. A wrong sense resistance silently miscalibrates every IRUN/IHOLD computation and can damage the driver, motor, or board.

| Parameter | Where | Meaning |
|-----------|-------|---------|
| `sense_resistor` | `[tmc2130 stepper_*]`, `[tmc2660 stepper_*]`, `[tmc5160 stepper_*]` | Sense resistor value in ohms. Mandatory. |
| `rref` | `[tmc2240 stepper_*]` | TMC2240 reference resistor (12000–60000 Ω). Mandatory. |
| `stepstick_type` | any TMC stepper section | Carrier-board name; resolves to `(sense_resistor, max_current)` via the bundled lookup table. Use this *or* `sense_resistor`, not both. On TMC2240 only the `max_current` portion is consumed (the chip uses `rref` instead of a sense resistor); the `sense_resistor` entry is ignored. |

Supported `stepstick_type` values: `REFERENCE_WOTT`, `REFERENCE_2209`, `REFERENCE_5160`, `KRAKEN_2160_8A`, `KRAKEN_2160_3A`, `BTT_2240`, `BTT_EZ_5160_PRO`, `BTT_EZ_5160_RGB`, `BTT_EZ_6609`, `BTT_5160T`, `WOTT_2209`, `COREVUS_2209`, `COREVUS_2160_OLD`, `COREVUS_2160_5A`, `COREVUS_2160`, `FYSETC_2225`, `FYSETC_5161`, `MKS_2226`, `MELLOW_FLY_5160`, `MELLOW_FLY_HV_5160_Pro`. New entries can be added to `klippy/extras/stepstick_defs.py`.

### Autotuning

Activated when both `motor:` and `voltage:` are present on a TMC stepper section. Without those, every other option below still works as a plain manual override; only the derivation-from-motor-and-current is skipped.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `motor` | unset | Name of a `[motor_constants <name>]` section that describes the motor (resistance, inductance, holding_torque, max_current, steps_per_revolution). The bundled `motor_database.cfg` contains specs for ~100 common stepper models; `[include motor_database.cfg]` from your printer.cfg to pull them all in. |
| `voltage` | unset | Stepper supply voltage, 0–60 V. Required for back-EMF / PWM-grad math. |
| `pwm_freq_target` | 55 kHz (20 kHz on TMC2240) | StealthChop PWM frequency target, 10–100 kHz. TMC2240 runs hot at the higher target so it defaults lower. |
| `chopper_freq_target` | 20 kHz | SpreadCycle target chopper frequency, 10–100 kHz. The autotune picks `TOFF` to land at or below this so the chopper sits just above the audible band. |
| `extra_hysteresis` | 0 | Add a fixed amount to the computed `HSTRT/HEND` sum (0–15). Useful when the auto-derived hysteresis sounds too aggressive. |
| `overvoltage_vth` | unset | Programmed overvoltage trip in volts (0–60). When set, `OVERVOLTAGE_VTH` register is written. |
| `driver_TBL`, `driver_TOFF`, `driver_TPFD`, `driver_HSTRT`, `driver_HEND`, `driver_SGTHRS`, `driver_SGT`, `driver_cs` | unset | Per-field manual override. When set, the autotune skips deriving that specific field and uses the value as-is. Mix and match — set the ones you want pinned and let the rest tune. |

The bundled `[motor_constants <name>]` sections (in `motor_database.cfg`) cover the major stepper-motor manufacturers: LDO, Moons, OMC, Stepperonline, Wantai, MOTECH, Trinamic, Phidgets, Soyo, BTT, BIQU, etc. Each section provides `resistance`, `inductance`, `holding_torque`, `max_current`, and `steps_per_revolution` from the manufacturer's datasheet.

### TMC5160 / TMC2240 driver current

Both drivers gained two new options as part of the current-calculation rework.

| Parameter | Default | Where | Meaning |
|-----------|---------|-------|---------|
| `driver_cs` | unset = auto | `[tmc5160 …]`, `[tmc2240 …]` | Override the auto-picked CS (current-scale) bits, 0–31. With `driver_cs` unset, the driver picks the smallest CS that delivers the requested run current at the active GLOBALSCALER, maximising the regulator's resolution. |
| `current_range` | smallest range that covers `run_current` | `[tmc2240 …]` | TMC2240 only. Locks a higher CURRENT_RANGE for headroom; cannot be set below the auto-picked floor. |

### TMC5160 short-detection thresholds

TMC5160 only. Setting both fields programs the chip's `SHORT_CONF` register; setting one without the other is a config error because the chip's OTP defaults are not readable.

| Parameter | Range | Meaning |
|-----------|-------|---------|
| `driver_s2vs_level` | 4–15 | Short-to-supply trip level. |
| `driver_s2g_level` | 2–15 | Short-to-ground trip level. |

When set, also fills `short_filter` and `shortdelay` from defaults; can be overridden via `driver_short_filter` / `driver_shortdelay`.

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

The homing profile additionally turns CoolStep off (`semin/semax/seup/sedn/seimin = 0`), enables fast-standstill, and slows IHOLDDELAY to 8 — these aren't user-configurable but follow the same low-noise rationale.

## Minimal example

A single TMC2240-driven X axis on a BTT 2240 stepstick at 24 V, with autotuning, sensorless homing, and the per-stepper homing profile:

```ini
[include motor_database.cfg]

[tmc2240 stepper_x]
uart_pin: PB1
diag1_pin: ^!PE5
stepstick_type: BTT_2240
rref: 12000

motor: ldo-42sth48-2504ah
voltage: 24

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

## Reading the autotune log

When autotune is active, `klippy.log` contains one summary line per phase per re-apply:

```
tmc stepper_x autotune: tuning for 1.400A at 24V (clock 12.500 MHz)
tmc stepper_x autotune: pwm_freq=2 (~12.2 kHz) pwmgrad=43 pwmofs=27
tmc stepper_x autotune: tbl=1 toff=4 tpfd=4
tmc stepper_x autotune: Ipeak=1.980A tblank=2.88us tsd=11.68us cs=22 hysteresis=2 hstrt=2 hend=3
```

When a homing move starts:

```
tmc stepper_x homing: cs=8 gs=243 (raw 243) irun=8 ihold=8 (target 0.600A)
```

Use these to confirm the autotune picked sensible values for your operating point.

## Compatibility

- Existing TMC sections continue to work without changes as long as they specify `sense_resistor` (or `rref` for TMC2240) — the only mandatory tightening. Configs that previously relied on the implicit defaults need an explicit value or a `stepstick_type:` line added.
- All previous `driver_*` field overrides remain available and take precedence over autotune-derived values. You can adopt autotune incrementally by setting `motor:`/`voltage:` while keeping any per-field pins you've already validated.
- The per-stepper homing profile is always active. To disable it on a specific axis, set every `homing_*` option to the value the autotune would have written for the run profile.
