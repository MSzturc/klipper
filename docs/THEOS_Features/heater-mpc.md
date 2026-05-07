# Model Predictive Temperature Control (MPC)

## What this is

PID heater control reacts to the temperature error: the controller has
no idea what the heater *should* do at any given moment, only what the
sensor *says*. That works well for steady-state holding but reacts late
to predictable disturbances — filament cooling the hotend during fast
extrusion, the part fan kicking up, a cold load on the bed.

Model Predictive Control (MPC) flips the model around. Instead of
reacting to error, MPC carries an internal thermal model of the heater
(its heat capacity, its loss to ambient, the energy carried away by
filament, the additional cooling from a part fan) and computes the PWM
duty needed to maintain the requested temperature *plus* the energy
needed to balance any disturbance the model knows about. The sensor is
used to correct model drift, not to drive the actuator directly.

This fork imports MPC as a third heater control algorithm
(`control: mpc`) alongside `pid` and `pid_v`. Calibration runs through
`MPC_CALIBRATE`, live tuning through `MPC_SET`, and the model's
internal block-temperature and ambient-temperature estimates are
exposed as virtual `temperature_sensor` objects so they can be charted,
logged, or referenced by other macros.

## When to use this

- A hotend that drops temperature noticeably during fast volumetric
  extrusion. MPC sees the requested filament feed rate, predicts the
  cooling load, and adds PWM ahead of the sensor reading the dip.
- A hotend whose target temperature lags by 5–10 °C when the part fan
  ramps from 0 to 100 %. MPC includes a fan-driven ambient-transfer term
  that compensates without you having to tune for it.
- A heatbed that overshoots when the chamber is closed and undershoots
  when it's open. MPC's ambient-temperature estimate updates with the
  measured loss rate; PID would need separate tunes for each case.
- A user who wants the heater target to be a setpoint rather than a
  controller-tuning exercise. MPC is set-it-and-forget-it once the
  model parameters are calibrated.

You don't need MPC if your existing PID-tuned heater is already quiet
and tracks setpoints to within 1 °C under your normal operating range.
PID is simpler to reason about and has fewer config parameters; reach
for MPC when the predictive aspect actually buys you something.

## Configuration

A heater opts into MPC by setting `control: mpc`. The heater section
then takes a number of additional keys describing the thermal model.
Most have sensible defaults; the only mandatory MPC-specific key is
`heater_power` (the wattage of the heater element).

### Required

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `control` | `pid` | Set to `mpc`. |
| `heater_power` | required | Heater element power in watts. Drives the model's energy-input term. For a hotend cartridge, the rated wattage on the manufacturer's spec sheet (40 W, 60 W, etc.). For a heated bed, the measured power draw at 24 V × the configured `max_power`. |

### Block thermal properties

Calibrated automatically by `MPC_CALIBRATE`. You can pre-seed them from
a previous calibration to skip the first calibration pass.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `block_heat_capacity` | calibrated | Effective thermal capacity of the heated mass in J/°C. Higher = slower response. |
| `ambient_transfer` | calibrated | Conductive/convective heat loss to ambient in W/°C. |
| `sensor_responsiveness` | calibrated | Sensor lag coefficient. Higher = sensor reads slower than the block changes. |
| `target_reach_time` | `2.0` | Time in seconds the controller is allowed to ramp into a step setpoint change. |
| `smoothing` | `0.83` | Output smoothing factor, 0–1. Higher = smoother PWM, lower = more responsive. |
| `min_ambient_change` | `1.0` | Minimum ambient-temperature drift in °C before the model updates its ambient estimate. |
| `steady_state_rate` | `0.5` | Block-temperature change per second below which the model considers itself in steady state. |

### Filament cooling load (hotends only)

When extrusion is active, MPC subtracts the energy the filament carries
away from the block. This needs the filament's heat capacity and how
much filament is currently being fed.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `filament_diameter` | `1.75` | Filament diameter in mm. |
| `filament_density` | `1.2` | Filament density in g/cm³. PLA ≈ 1.24, PETG ≈ 1.27, ABS ≈ 1.04. |
| `filament_heat_capacity` | `0.0` | Filament-specific heat capacity in J/(g·°C). `0.0` disables the extrusion-cooling feedforward; set to a filament-appropriate value (PLA ≈ 1.8, PETG ≈ 1.7, ABS ≈ 1.4) to enable it. |
| `maximum_retract` | `2.0` | Maximum retract distance in mm; clamps a single retract event so a runaway extruder cannot poison the cooling estimate. |
| `filament_temperature_source` | `ambient` | One of: `ambient` (use the model's ambient estimate), `sensor` (use a configured ambient sensor — see below), or a fixed °C value (e.g. `25`) for a constant assumed filament temperature. |
| `ambient_temp_sensor` | unset | Section name of a `[temperature_sensor]` to read filament-feed temperature from when `filament_temperature_source: sensor`. |

### Cooling fan tracking

When the heater is influenced by a part-cooling fan, MPC can fold that
into the model so changes to fan speed don't introduce setpoint error.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `cooling_fan` | unset | Section name of the fan whose speed influences this heater. Typically the part-cooling fan for a hotend. |
| `fan_ambient_transfer` | empty | Comma-separated list of additional `ambient_transfer` values, one per discrete fan-speed step (0 %, 25 %, 50 %, 75 %, 100 %). The model interpolates between them by current fan duty. Leave empty to ignore fan effects. |

### Minimal example

A 40 W hotend on a Voron 2.4-style printer with a part-cooling fan:

```ini
[extruder]
heater_pin: PA2
sensor_type: ATC Semitec 104GT-2
sensor_pin: PF4
min_temp: 0
max_temp: 320
control: mpc
heater_power: 40
filament_diameter: 1.75
filament_density: 1.24
filament_heat_capacity: 1.8
cooling_fan: fan
fan_ambient_transfer: 0.07, 0.085, 0.1, 0.115, 0.13
```

`block_heat_capacity`, `ambient_transfer`, and `sensor_responsiveness`
are calibrated automatically — leave them out for the first start, run
`MPC_CALIBRATE`, then `SAVE_CONFIG`.

## G-code commands

### `MPC_CALIBRATE`

Run a thermal calibration on the named heater. Drives the heater to the
target, observes the warm-up curve, and fits `block_heat_capacity`,
`ambient_transfer`, and `sensor_responsiveness`. Takes 10–30 minutes
depending on heater mass.

| Parameter | Meaning |
|-----------|---------|
| `HEATER` | Short name of the heater (e.g. `extruder`, `heater_bed`, `chamber`). |
| `TARGET` | Calibration target temperature in °C. Default: 200 for a hotend, 90 for a heated bed. Minimum allowed value 60 °C. |
| `FAN_BREAKPOINTS` | Optional integer 2–10 specifying how many discrete fan speeds to characterise (only meaningful when the heater has a `cooling_fan`). Default `5`. |

```
MPC_CALIBRATE HEATER=extruder
MPC_CALIBRATE HEATER=heater_bed TARGET=80
MPC_CALIBRATE HEATER=chamber TARGET=55
```

After calibration completes successfully, `SAVE_CONFIG` writes the
fitted parameters into the heater section.

### `MPC_SET`

Update calibrated MPC parameters at runtime, no restart required.
Useful for live experimentation and for tuning into multiple profiles
without re-running the full calibration.

| Parameter | Meaning |
|-----------|---------|
| `HEATER` | Short name of the heater. |
| `BLOCK_HEAT_CAPACITY` | Override `block_heat_capacity`. |
| `SENSOR_RESPONSIVENESS` | Override `sensor_responsiveness`. |
| `AMBIENT_TRANSFER` | Override `ambient_transfer`. |
| `FAN_AMBIENT_TRANSFER` | Comma-separated list overriding `fan_ambient_transfer`. |
| `FILAMENT_DIAMETER` | Override `filament_diameter`. |
| `FILAMENT_DENSITY` | Override `filament_density`. |
| `FILAMENT_HEAT_CAPACITY` | Override `filament_heat_capacity`. |

Any combination may be set in one call; omitted keys keep their current
value. `MPC_SET` does **not** persist; use `SAVE_CONFIG` after a
permanent change.

```
MPC_SET HEATER=extruder BLOCK_HEAT_CAPACITY=14.5 SENSOR_RESPONSIVENESS=0.65
MPC_SET HEATER=heater_bed AMBIENT_TRANSFER=0.085
```

## Virtual sensors

MPC's internal block-temperature estimate (the model's prediction of
the actual heater-block temperature, distinct from the thermistor
reading) and ambient-temperature estimate (the inferred environmental
temperature the model uses for loss calculations) are available as
ordinary `[temperature_sensor]` objects. Add a section per estimate:

```ini
[mpc_block_temperature extruder_block]
heater_name: extruder
ignore_limits: True

[mpc_ambient_temperature extruder_ambient]
heater_name: extruder
ignore_limits: True
```

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `heater_name` | required | Section name of the MPC-controlled heater whose internal estimate to expose. |
| `ignore_limits` | `False` | When `True`, the sensor's reading is not range-checked against `min_temp`/`max_temp`; useful for ambient-temperature sensors whose range covers normal room conditions but might not include cold-start values. |

The exposed sensors show up in printer status as
`temperature_sensor extruder_block` etc., chartable by every Klipper
front-end and consumable as a sensor reference by other features
(e.g. as a `cooling_fan` source for a chamber MPC heater).

## Calibration

1. Configure `control: mpc` and the mandatory `heater_power` on the
   heater. Leave the calibrated parameters out for first run.
2. Cold-start the printer.
3. Run `MPC_CALIBRATE HEATER=<name>`. Stand by — the autotune will heat
   to the target, hold, observe cooldown, and run a fan-step pass if a
   `cooling_fan` is configured.
4. When the prompt to `SAVE_CONFIG` appears, run it. Klipper restarts
   with the calibrated parameters baked into the section.
5. Optionally seed `filament_temperature_source: sensor` and a chamber
   ambient sensor for tighter behaviour on enclosed printers.

For a heater whose calibration drifts seasonally (a chamber heater
that's tuned for summer ambient versus winter), use `MPC_SET` to nudge
`ambient_transfer` between the two regimes; or save two profiles and
swap between them with the [PID profile system](heater-pid-profiles.md)
since profiles support `control: mpc` too.

## Things to know

- **MPC needs an honest `heater_power`.** Wrong wattage produces a model
  that systematically over- or under-drives the PWM. Measure with a
  bench supply if the rating sticker is questionable.
- **Calibration cannot start cold and dry.** The model needs a stable
  ambient before it can run the warm-up fit. Don't trigger
  `MPC_CALIBRATE` immediately after powering on — give the printer 10
  minutes for the bed to settle near room temperature.
- **Calibration minimum is 60 °C.** Below that, the response curve is
  too short to fit reliably. The default targets (200 °C hotend, 90 °C
  bed) are tuned for typical printer geometries.
- **Fan steps are interpolated linearly between the breakpoints.** With
  a `fan_ambient_transfer` list of five values, the model uses the
  literal value at 0 / 25 / 50 / 75 / 100 % fan and interpolates
  linearly in between. There is no physical assumption about the
  underlying curve being linear; the breakpoints define the curve.
- **Switching `control:` between MPC and PID requires a restart.** The
  controller class is bound at heater setup. To swap algorithms live,
  define both as profiles (one `control: pid`, one `control: mpc`) and
  use `PID_PROFILE LOAD`.
- **Virtual MPC sensors are read-only and update with the heater.**
  They follow the heater's normal sample rate (the `temperature_callback`
  cadence). They cannot be used as the sensor of another heater
  directly — they're observation points, not control inputs.
- **`filament_heat_capacity` defaults to `0.0`.** A value of `0.0`
  disables the extrusion-cooling feedforward term entirely; set it to
  a filament-appropriate value (PLA ≈ 1.8 J/(g·°C)) to enable
  feedforward compensation during extrusion.

## Credits

The MPC controller is based on the Klipper-MPC implementation by Lasse
Dalegaard. The runtime calibration parameters and `MPC_SET` live tuning
were added by the Kalico community (PR #447), with follow-up fixes for
the filament-heat-capacity defaults, the `MPC_CALIBRATE` minimum
target, and the `filament_density` / `filament_heat_capacity` status
exposure (KalicoCrew PRs #468/#517/#555/#708/#732/#819). The virtual
block- and ambient-temperature sensors are based on PR #775 by the
KalicoCrew community. All pieces have been re-derived against the
current Klipper heater module shape.
