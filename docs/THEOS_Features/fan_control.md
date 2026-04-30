# Fan Control

## What this is

Stock Klipper's fan plumbing accepts a requested speed between 0.0 and 1.0, scales it by `max_power`, and drives the PWM pin. That is enough for many printers, but it has rough edges: a slow fan request can land below the duty needed for the rotor to actually start spinning, the only thing exposed in status is the requested speed, `[temperature_fan]` only knows watermark and PID control, and `SET_FAN_SPEED` is keyed on the section name with no shorter alias.

This fork polishes the fan path:

1. **`min_power` and `power` status** — define a duty floor so any non-zero request lifts the fan to at least that level, and read back the actual PWM duty separately from the user-requested speed.
2. **`initial_speed`** — start a fan at a configured duty as soon as Klipper finishes loading, with no explicit gcode.
3. **Curve control for `[temperature_fan]`** — interpolate fan speed along a list of temperature/speed points, with optional heating and cooling hysteresis around each point.
4. **Named `[fan_generic]`** — register an alternate `FAN=<id>` alias so `SET_FAN_SPEED` can target a fan by a stable short name.

Stock fan behaviour is unchanged when none of these new options are set.

## When to use this

- A fan that stalls at low PWM duty. `min_power` keeps the duty above the rotor's minimum spin threshold whenever the fan is on.
- A part-cooling or filter fan that should run continuously from boot. `initial_speed` removes the need for a `START_PRINT` macro to set it up.
- A controller-board, MCU, or chamber fan whose RPM should track temperature smoothly rather than snap between two values. `control: curve` gives you N points instead of one threshold.
- A long or section-name-suffixed `[fan_generic]` whose default `FAN=<section>` argument is awkward to type in macros. `id:` gives you a short stable handle.
- A monitoring tool that needs the actual PWM duty, not the requested speed (for instance to log a fan's power draw). Read `printer["fan"].power`.

You do not need any of this if you set fan speed manually each print, your fans always spin at any duty you ask for, and your `[temperature_fan]` thresholds are already adequate.

## Configuration

All four additions are exposed as new optional parameters on existing fan sections. No new section type is introduced.

### `[fan]`, `[heater_fan]`, `[controller_fan]`, `[fan_generic]`, `[temperature_fan]`

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `min_power` | `0.0` | Lower bound on the fan's PWM duty when it is on. Any non-zero requested speed is mapped linearly into the range `[min_power, max_power]`. A request of zero still turns the fan fully off. Range 0.0–1.0. |
| `off_below` | unset | Deprecated alias for `min_power`. Old configs continue to work and emit a deprecation warning at startup. Setting both `min_power` and `off_below` in the same section is rejected. The migration changes semantics: `off_below` historically *cut off* below the threshold, while `min_power` *lifts up* to it. |
| `initial_speed` | unset | Speed (0.0–1.0) the fan is commanded to once at startup, after Klipper finishes loading. |

### `[fan_generic]` only

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `id` | unset | Alternate short name. When set, `SET_FAN_SPEED FAN=<id>` is registered alongside the default `SET_FAN_SPEED FAN=<section_suffix>`. |

### `[temperature_fan]` curve control

When `control: curve`, the watermark/PID parameters do not apply. Instead:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `points` | required | A list of `<temperature>,<speed>` pairs (one per line, or comma-separated). At least two pairs. Must be monotonically increasing in both columns. All temperatures must lie in `[min_temp, target_temp]`; all speeds must lie in `[min_speed, max_speed]`. |
| `heating_hysteresis` | `0.0` | Temperature offset (°C) added to each curve point when the fan steps up. The fan only increases speed once the temperature rises past `<curve_temp> + heating_hysteresis`. |
| `cooling_hysteresis` | `0.0` | Temperature offset (°C) subtracted from each curve point when the fan steps down. The fan only decreases speed once the temperature falls below `<curve_temp> - cooling_hysteresis`. |

`SET_TEMPERATURE_FAN_TARGET TARGET=...` is rejected for curve-controlled fans — the curve, not a single setpoint, governs the fan. `MIN_SPEED` and `MAX_SPEED` adjustments are still accepted.

### Minimal example

```ini
[fan]
pin: PA8
min_power: 0.18
initial_speed: 0.0

[temperature_fan chamber]
pin: PE5
sensor_type: Generic 3950
sensor_pin: PF4
min_temp: 0
max_temp: 75
target_temp: 60
min_speed: 0.0
control: curve
points:
    25, 0.00
    40, 0.50
    60, 1.00
heating_hysteresis: 1.5
cooling_hysteresis: 2.0

[fan_generic filter]
pin: PD12
id: nevermore
```

With this config: the part-cooling fan never sits below 18% duty when on; the chamber fan is fully off below 25 °C, ramps to 50% by 40 °C and full speed by 60 °C with hysteresis preventing chatter; and the filter fan can be driven via `SET_FAN_SPEED FAN=nevermore SPEED=0.6` as well as the default `FAN=filter` form.

## G-code commands

The fan command surface is unchanged. `M106` / `M107`, `SET_FAN_SPEED`, and `SET_TEMPERATURE_FAN_TARGET` accept the same parameters as upstream Klipper. The behavioural change for `SET_TEMPERATURE_FAN_TARGET` is that `TARGET=<temp>` raises an error if the fan is configured with `control: curve` — there is no single target temperature in that mode.

## Things to know

- **`min_power` is a duty floor, not a threshold.** Old configs that relied on `off_below` to *cut off* a low requested speed will see the fan run at the threshold instead. If you used `off_below=0.2` to silence the fan below 20%, recalibrate: a request of 0.05 will now run the fan at roughly 24% duty, not 0%.
- **Kick-start always holds for `kick_start_time`.** Even a request for full speed waits the configured kick window before subsequent fan commands flush. This matches stock Klipper's behaviour and ensures the rotor has spun up before any later duty change is applied.
- **`speed` and `power` differ when `min_power` or `max_power` are set.** `printer["fan"].speed` is what the user requested (0.0–1.0). `printer["fan"].power` is the actual duty being driven, after mapping into `[min_power, max_power]`. UIs that read `speed` keep working; tools that need the real duty should read `power`.
- **Curve control needs both endpoints inside the configured temperature range.** Klipper extends the curve flat at the ends to `min_temp` and `max_temp` if your `points:` don't reach. Setting a `target_temp` lower than the highest `points:` temperature is rejected at config-load time.
- **`hardware_pwm: True` shutdown clamps to `max_power`.** When `hardware_pwm` is False, the configured `shutdown_speed` is passed through to the MCU directly (it is validated by the PWM pin setup itself). When True, `shutdown_speed` is clamped to `max_power` to stay within the hardware's electrical envelope.
- **`initial_speed` runs through the normal command queue.** It fires after the `klippy:ready` event, which means it will appear in the gcode queue alongside any startup macros. If your `START_PRINT` macro also touches the same fan, the macro wins.

## Credits

Based on Kalico community work: the PWM-scaling rework by Kubaracek (Kalico PR #44), the curve-control implementation by Zeanon as later refined to a numpy-based form by NokkOnEffect (Kalico PRs #193 and #466), the displayed-speed clamp by Frédéric Beaucamp (Kalico PR #433), the `initial_speed` parameter (Kalico PR #436), the kick-start last-value reset by Zeanon (Kalico PR #819), and the hardware-PWM shutdown handling by Rogerio Goncalves. The `id:` alternative for `SET_FAN_SPEED` is a fork-local addition. Adapted to the current Klipper fan API with a few semantic changes — most notably `speed` in fan status keeps its 0.0–1.0 meaning rather than being repurposed.
