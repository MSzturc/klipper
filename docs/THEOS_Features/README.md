# THEOS Klipper Fork

A fork of [Klipper](https://www.klipper3d.org/) maintained by Matt The Printing Nerd, bundling additional features used by the [THEOS](https://github.com/MSzturc/THEOS) printer operating system on top of stock Klipper.

The core Klipper experience is unchanged: if you already run Klipper, everything you know still applies. Stock documentation at [klipper3d.org](https://www.klipper3d.org/) remains authoritative for installation, kinematics, tuning, and everything not listed below.

## Fork-specific features

- [**Sensorless Homing**](sensorless_homing.md) — StallGuard-based homing with per-rail `home_current`, `min_home_dist`, and `homing_accel`; guaranteed post-home current restore; multi-stepper-per-rail current swap. Based on work by Brandon Nance, Bea Nance, Rogerio Goncalves, and Zeanon on Kalico, re-derived on the current Klipper base.
- [**TMC Drivers**](tmc_drivers.md) — motor-data-driven autotuning of chopper/PWM/hysteresis/StallGuard/CoolStep, per-stepper low-noise homing profile, TMC5160 driver_cs and short_conf, TMC2240 driver_cs / current_range / mandatory rref, mandatory sense_resistor, stepstick lookup table. Combines Klipper3d PR #6644 (Honest Brothers), Kalico PRs #382/#444/#525/#556/#613, and original autotuning work.
- [**ConfigParser Extensions**](configparser_extensions.md) — variable interpolation (`${section.option}`), arithmetic evaluation with `min`/`max`/`abs`/`round`, default values (`:default`, `:None`), printer.cfg-overrides-sub-configs semantics, conditional includes, variable references in `[include]` paths, recursive subfolder globs, escape syntax (`\${literal}`), and the `RELOAD_GCODE_MACROS` G-code. Based on Kalico PRs #448/#482/#753 by Frank Tackitt and original work by Matt Szturc.
