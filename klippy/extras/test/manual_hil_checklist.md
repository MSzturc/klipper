# TMC5160 Driver Extensions — Manual HIL Verification

> Manual on-printer test checklist for the TMC5160 driver extensions
> work.  Run after merging Phase 1 + Phase 2 to a real T100 or T250.

## Pre-Test Setup

1. Flash the THEOS image with the new driver code.
2. Verify autotune is ON: `klippy.log` should show
   `tmc <name> autotune (goal=...): pwm_freq=...` lines on boot.
3. Capture the baseline `DUMP_TMC` output for one X stepper:
   `DUMP_TMC STEPPER=stepper_x` and save to `baseline_X.txt`.

## Per-Goal Verification

### Goal: `performance`

Set `tuning_goal: performance` for X/Y/Z/E in printer.cfg, restart Klipper.

- [ ] `DUMP_TMC STEPPER=stepper_x` shows:
  - `TPWMTHRS = 0xfffff`
  - `vhighfs = 1, vhighchm = 1`
  - `THIGH > 0` and `< 0xfffff` (active high-speed mode)
  - `sfilt = 0`
  - `dc_time, dc_sg` populated to non-default values
- [ ] Print a 3DBenchy at 1000 mm/s travel, 800 mm/s outer wall.
  Expected: completes without layer shift; some audible SpreadCycle
  hissing during prints.
- [ ] Heat check: motor casing temperature after 30 min print should
  not exceed 70 °C.

### Goal: `balanced`

Set `tuning_goal: balanced` (or remove the line — that's the default).

- [ ] `DUMP_TMC STEPPER=stepper_x` shows:
  - `TPWMTHRS` is a moderate value (not 0xfffff and not 0)
  - `vhighfs = 0, vhighchm = 0`
  - `THIGH = 0`
  - `sfilt = 1`
  - `pwm_reg = 8, pwm_lim = 8`
- [ ] Print a 1-hour calibration cube. Expected: no surface artefacts,
  no current-breathing visible on cube walls; noticeably quieter than
  performance during slow moves.
- [ ] After 1-hour print, motor temperature ≤ 65 °C.

### Goal: `silent`

Set `tuning_goal: silent` for X/Y/Z/E.

- [ ] `DUMP_TMC STEPPER=stepper_x` shows:
  - `TPWMTHRS` is high (close to vmaxpwm threshold)
  - `vhighfs = 0, vhighchm = 0`
  - `semin = 0` (CoolStep disabled)
  - `small_hysteresis = 1`
  - `tpfd = 0`
- [ ] Audible test in a quiet room (background ≤ 30 dB):
  - Move axes at 30 mm/s — should be effectively silent at standstill
    and very quiet during motion.
  - No coil-whining at standstill.
- [ ] Print a slow-speed (50 mm/s) test object. Expected: smooth
  surface, no missed steps at this velocity.

## Pin-Override Verification

- [ ] Set `tuning_goal: silent` + `driver_SEMIN: 4` for stepper_x. Boot.
  `DUMP_TMC STEPPER=stepper_x` should show `semin = 4` (pin respected
  over silent's default of 0).

- [ ] Set `tuning_goal: performance` + `driver_TPWMTHRS: 0` for
  stepper_y. Boot. `DUMP_TMC STEPPER=stepper_y` should show
  `TPWMTHRS = 0` (pin respected over performance's default of
  0xfffff). StealthChop will be active at all speeds — expect to
  see `stealth = 1` in DRV_STATUS during slow moves.

## High-Voltage Safety

- [ ] Set `voltage: 56` and `driver_S2G_LEVEL: 8` in the config.
  Klipper should refuse to start with a config error mentioning
  "s2g_level must be >=12" and the datasheet section reference.

- [ ] Set `voltage: 56` without any SHORT_CONF pins. Boot. `klippy.log`
  should contain a warning about SHORT_CONF programming at high VS.

## Homing Profile Orthogonality

- [ ] With `tuning_goal: performance` (vhighfs/vhighchm active),
  trigger sensorless homing on X. During the homing window the
  `klippy.log` should record `vhighfs=0, vhighchm=0, semin=0` (homing
  profile applied). After homing completes, those fields should be
  back at their performance values.

- [ ] Repeat with `tuning_goal: silent` and `tuning_goal: balanced`.
  Homing should produce identical behaviour regardless of run goal.

## Sign-Off

- [ ] Tester name: ______________________
- [ ] Date: __________________
- [ ] Printer model: T100 / T250 (circle one)
- [ ] All boxes checked → ready to merge to develop
