# `RESET_VELOCITY_LIMIT` G-code

## What this is

A G-code command that restores the printer's velocity-related limits to whatever was declared in `printer.cfg`. When `SET_VELOCITY_LIMIT` or `M204` has been used during a print to temporarily override `max_velocity`, `max_accel`, `square_corner_velocity`, or `minimum_cruise_ratio`, this command rolls all four back to the configured values in one call.

The command does not require parameters. It always restores all four limits together.

## When to use this

- A macro temporarily lowered acceleration for a delicate move (`SET_VELOCITY_LIMIT ACCEL=...`) and you want a clean way to undo it without remembering the original values.
- A print profile tuned `max_velocity` per layer or per region and you want the next print to start from the configured baseline regardless of how the previous one ended.
- You're scripting a calibration routine that mutates velocity limits and want a single reset call at the end of the routine, instead of four `SET_VELOCITY_LIMIT` invocations.
- During development, when a quick `RESET_VELOCITY_LIMIT` after experimentation lets you test from the configured baseline without restarting Klipper.

If you never override velocity limits at runtime, you don't need this command — the limits stay at the configured values for the lifetime of the Klipper session anyway.

## G-code commands

### `RESET_VELOCITY_LIMIT`

Restores `max_velocity`, `max_accel`, `square_corner_velocity`, and `minimum_cruise_ratio` to the values declared in the `[printer]` config section, then prints the active limits to the console.

```
RESET_VELOCITY_LIMIT
```

The command takes no parameters. The four-value snapshot is taken once at toolhead init; later edits to `printer.cfg` are not reflected until a Klipper restart.

After the reset, the same diagnostic message that `SET_VELOCITY_LIMIT` (without arguments) would emit is printed:

```
max_velocity: 300.000000
max_accel: 3000.000000
minimum_cruise_ratio: 0.500000
square_corner_velocity: 5.000000
```

## Things to know

- **The reset reads from the snapshot, not from the live config file.** If you `SAVE_CONFIG`, the snapshot still holds the values that were live at boot. Restart Klipper to refresh.
- **`SET_VELOCITY_LIMIT` without arguments only prints the current values.** It does not reset them. `RESET_VELOCITY_LIMIT` is the explicit reset.
- **All four limits reset together.** There is no syntax to reset only `max_accel` while keeping a `max_velocity` override active. Re-issue `SET_VELOCITY_LIMIT VELOCITY=…` after the reset if you need that.
- **Acceleration overrides during homing are independent.** The sensorless-homing layer uses `set_accel` / `reset_accel` internally and snapshots `max_accel` separately from this feature; a homing run does not interact with `RESET_VELOCITY_LIMIT`.

## Credits

Based on Kalico PR [#472](https://github.com/KalicoCrew/kalico/pull/472) by Matt Szturc, with the snapshot mechanism re-derived against the current Klipper toolhead structure (the snapshot lives in `ToolHead.__init__`, the command lives in `ToolHeadCommandHelper`, and both go through the canonical `set_max_velocities` setter).
