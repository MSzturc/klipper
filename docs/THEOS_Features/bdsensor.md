# BDSensor

## What this is

The BDSensor (Bed Distance Sensor) is a contactless inductive probe that measures the distance from the toolhead to the bed in real time. Unlike a single-point trigger probe (BLTouch, optical, etc.), it returns a continuous distance value while the toolhead is in motion, which lets Klipper run high-speed bed meshes, perform real-time first-layer Z compensation, and home Z without ever touching the bed surface.

This page covers the in-tree integration of [Mark Yue's BDSensor](https://github.com/markniu/Bed_Distance_sensor). The fork ships:

1. **`klippy/extras/BDsensor.py`** — the Klipper extra that registers the `[BDsensor]` config section, exposes probe commands, and drives the I²C-style bit-banged protocol.
2. **`src/BD_sensor.c`** — the firmware-side companion compiled into every MCU image (`src-y += BD_sensor.c`), so any board can host a BDSensor without rebuild flags.
3. **Pin alias resolution** — `sda_pin`, `scl_pin`, and `endstop_pin` accept `[board_pins]` aliases and resolve them to the hardware pin before talking to the MCU.

The integration follows the in-tree pattern: drop the config block in, point `sda_pin` and `scl_pin` at your wiring, and the sensor is available as a regular probe.

## When to use this

- You want bed-mesh probing in the 5–15 minutes/mesh range rather than 60+ seconds — BDSensor's continuous-distance readout lets the bed-mesh routine sample while moving.
- You print on a surface that doesn't tolerate physical contact (PEI, soft glass, fragile beds) and a deflection-style probe is undesirable.
- You want first-layer Z compensation that responds to live bed deflection without re-probing.
- You have an existing BLTouch / inductive probe and the print quality is fine — keep stock Klipper probing; BDSensor adds complexity that isn't worth taking on without a concrete reason.

## Configuration

The `[BDsensor]` section replaces the stock `[probe]` section. Do not declare both. Wire the sensor's data and clock lines to two GPIOs on the same MCU and reference those pins via `sda_pin` / `scl_pin`.

### Required keys

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `sda_pin` | — | GPIO connected to the sensor's data line. Aliases declared via `[board_pins]` are resolved automatically. |
| `scl_pin` | — | GPIO connected to the sensor's clock line. Aliases resolved as above. |
| `delay` | — | Per-bit timing constant for the bit-banged I²C transport, in MCU clock ticks. Sensor-specific; consult the BDSensor wiring guide for your board. |
| `z_offset` | — | Probe trigger Z offset relative to the nozzle. Calibrated via `PROBE_CALIBRATE`. |

### Optional keys

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `x_offset` | `0.0` | Probe X position relative to the nozzle. |
| `y_offset` | `0.0` | Probe Y position relative to the nozzle. |
| `z_adjust` | `0.0` | Live offset added to every BDSensor reading. Range `-0.3..0.3`. Use for fine-tuning without recalibrating. |
| `position_endstop` | `0.7` | Distance (mm) at which the sensor reports the endstop as triggered during homing. Recommended `1.0–2.8`. |
| `speed` | `3.0` | Probing speed (mm/s) for `PROBE`/`PROBE_CALIBRATE`. |
| `no_stop_probe` | (unset) | When set to `true`, the toolhead does not pause at each probe point during `BED_MESH_CALIBRATE`; the sensor reads continuously. Required for fast meshing. |
| `collision_homing` | `0` | When `1`, homes Z by detecting nozzle collision instead of using BDSensor distance. Useful for unusual bed materials. |
| `collision_calibrate` | `0` | When `1`, the auto-calibration routine uses nozzle-collision rather than the sensor signal. |
| `endstop_pin` | (defaults to sda) | Optional separate endstop pin if the sensor exposes one, otherwise the sda pin doubles as the endstop. Aliases resolved. |
| `homing_cmd` | `G28` | The command issued by `BDSENSOR_CALIBRATE` to home the printer before measurement. Override if you use a custom homing macro. |

### Minimal example

```ini
[board_pins kraken]
aliases:
    PROBE_SENSOR=PG1, PROBE_CONTROL=PE9

[BDsensor]
sda_pin: PROBE_SENSOR
scl_pin: PROBE_CONTROL
delay: 10
x_offset: -25
y_offset: 0
z_offset: 1.0
position_endstop: 0.6
no_stop_probe: true
speed: 15

[stepper_z]
endstop_pin: probe:z_virtual_endstop
# ... rest of stepper_z config

[bed_mesh]
mesh_min: 10,10
mesh_max: 280,280
algorithm: bicubic
```

## G-code commands

Standard probe commands work as on any other probe — `PROBE`, `QUERY_PROBE`, `PROBE_CALIBRATE`, `PROBE_ACCURACY`, `Z_OFFSET_APPLY_PROBE`, `BED_MESH_CALIBRATE`. The sensor adds five BDSensor-specific commands:

### `BDSENSOR_VERSION`

Reports the firmware version reported by the sensor. Useful when troubleshooting or reporting issues to the upstream project.

### `BDSENSOR_DISTANCE`

Reads the current sensor distance and prints it to the console. The toolhead doesn't move; this is a single one-shot read at the current Z position.

### `BDSENSOR_CALIBRATE`

Drives the bed through the full per-board distance calibration routine. The sensor measures known reference distances and writes a calibration table back to its internal flash. Run this once after install, then re-run only if the sensor or hot-end is repositioned. The command issues `homing_cmd` (default `G28`) at the start, so the printer must be ready to home.

### `BDSENSOR_READ_CALIBRATION`

Prints the calibration table currently stored in the sensor's flash. Use this to verify that a calibration was saved and to spot anomalies (non-monotonic distances, large gaps).

### `BDSENSOR_SET`

Sets BDSensor parameters and persists them to `printer.cfg` via `SAVE_CONFIG`.

| Parameter | Meaning |
|-----------|---------|
| `Z_ADJUST=<float>` | Set `z_adjust` to the given value (range `-0.3..0.3`). |
| `NO_STOP_PROBE=<int>` | `1` enables rapid-scan mode for `BED_MESH_CALIBRATE`, `0` disables it. |

```
BDSENSOR_SET Z_ADJUST=-0.05
BDSENSOR_SET NO_STOP_PROBE=1
```

`NO_STOP_PROBE` takes effect immediately for the current session and is persisted via `SAVE_CONFIG`. `Z_ADJUST` is applied to the Python-side state and persisted via `SAVE_CONFIG`, but the value baked into the MCU's `z_offset` field is refreshed only on the next Klipper restart — for a runtime-effective offset change, prefer `Z_OFFSET_APPLY_PROBE` followed by `SAVE_CONFIG` and a restart.

## Calibration

1. Wire the sensor and verify the MCU recognises it: `BDSENSOR_VERSION` should return a non-empty version string.
2. Run `BDSENSOR_CALIBRATE` once. The toolhead homes, lowers to the bed, and the sensor records its reference distances. Don't power off mid-calibration.
3. Run `BDSENSOR_READ_CALIBRATION` and verify the table is populated and the values are monotonically increasing. If they aren't, redo the calibration on a clean, flat reference.
4. Run `PROBE_CALIBRATE` to set the nozzle-to-trigger Z offset (`z_offset`). Standard Klipper procedure — drop a paper, jog the nozzle to it, accept.
5. Print a first layer at the calibrated offset and verify squish. Use `BDSENSOR_SET Z_ADJUST=...` to tune at runtime, or save a finer `z_offset` via `Z_OFFSET_APPLY_PROBE`.

## Things to know

- **The sensor needs the firmware-side companion.** `BD_sensor.c` is built into every MCU image automatically (`src-y += BD_sensor.c` in `src/Makefile`). If you're cross-compiling against an older fork checkout, make sure the file is present and the build picks it up.
- **`sda_pin` and `scl_pin` accept aliases.** If your `printer.cfg` references board-pin aliases declared via `[board_pins <name>]`, the BDSensor module unwraps them before forwarding to the MCU. Both raw pin names (`PG1`) and aliases (`PROBE_SENSOR`) are equivalent at the config layer.
- **`stepper_z`'s `endstop_pin` must be `probe:z_virtual_endstop`.** BDSensor implements the endstop as a virtual probe trigger, not as a real GPIO endstop. This is the same pattern as BLTouch and other distance probes.
- **Speed and acceleration during probing are sensor-bound.** With `no_stop_probe: true` you can run `BED_MESH_CALIBRATE` at the configured `speed`, but exceeding ~25 mm/s in either axis tends to introduce reading noise. Tune `speed` per setup.
- **`collision_homing` and `collision_calibrate` are escape hatches** for setups where the inductive read fails (extremely thin sheets, mirrored bed, exotic materials). Leave them at `0` unless you've confirmed the optical/inductive path is the limiting factor.
- **The sensor talks bit-banged I²C, not real I²C.** `delay` controls the per-bit timing and is highly sensor-dependent. Start from the upstream BDSensor recommendation for your board and only tune it if reads come back as `1024` (the protocol error sentinel) at the otherwise-correct distance.

## Credits

Based on Mark Yue's [Bed Distance Sensor](https://github.com/markniu/Bed_Distance_sensor) project, integrated in-tree from the upstream `new` branch (HEAD `8d0f833`). The pin-alias resolution helper at the three pin-lookup sites is a fork-local addition that extends the side-load behaviour to handle `[board_pins]` aliases consistently. Original integration pattern from Kalico (`a5357c00c`, `17567b1d1`, `43edf01e7`) by Matt Szturc.
