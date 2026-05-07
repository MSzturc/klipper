# Dual-Loop PID

## What this is

A standard PID heater controls one sensor and one heater element. That
works for nozzles, beds, and chambers where a single thermistor sees
the temperature you actually care about. It breaks down for hardware
where the heater you're driving and the temperature you're regulating
are separated by significant thermal lag — for instance, a heatbreak
heater (cartridge near the heat-block) where the **load** sensor sits
on the heat-block surface and the **element** sensor sits inside the
cartridge body.

Driving such a system with a single PID loop forces a compromise: gains
fast enough to respond to load disturbances will swing the cartridge
through dangerous temperatures; gains slow enough to protect the
cartridge can't reject load steps in time.

This fork adds a **dual-loop PID controller** (`control: dual_loop_pid`)
that runs two cascaded PID loops. The outer loop reads the primary
(load) sensor and produces a temperature setpoint for the inner loop.
The inner loop reads the secondary (element) sensor and runs a fast PID
that drives the heater PWM, clamped at a configurable hard ceiling
(`inner_max_temp`) so the element can never exceed its rated
temperature.

## When to use this

- A hotend or heatbreak heater where the heater element sits separated
  from the load surface and you want firm protection against the
  element overheating.
- An industrial-style heated chamber where the heating element is
  ducted away from the chamber air and direct PID on the chamber
  thermistor would either lag the load or overheat the element.
- A heated build plate with a separate heating mat below an aluminium
  build surface, where the mat itself can run hotter than the surface
  and you want both temperatures measured.

You don't need this if a single sensor gives you a faithful read of the
temperature you care about *and* the heater can't exceed its rated
temperature in normal operation. Dual-loop adds tuning complexity; only
reach for it when the single-loop approach is genuinely the wrong tool.

## Configuration

A dual-loop heater needs two sensors. Declare the second one as a
plain `[temperature_sensor <name>]` and reference it from the heater
section via `inner_sensor_name`.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `control` | `pid` | Set to `dual_loop_pid` to enable cascaded control. |
| `inner_sensor_name` | unset | Short name of the `[temperature_sensor]` section to use as the inner-loop (element) sensor. |
| `inner_max_temp` | required when `dual_loop_pid` | Hard ceiling on the element temperature in °C. The inner-loop PID treats this as its maximum setpoint regardless of what the outer loop requests. |
| `pid_Kp`, `pid_Ki`, `pid_Kd` | required | Outer-loop (load) PID gains. |
| `inner_pid_kp`, `inner_pid_ki`, `inner_pid_kd` | required | Inner-loop (element) PID gains. |
| `smooth_time` | `1.0` | Derivative smoothing window for both loops. |

### Minimal example

A hotend with a heatbreak cartridge driving a separate hotblock thermistor:

```ini
[temperature_sensor hotblock]
sensor_type: ATC Semitec 104GT-2
sensor_pin: PF5
min_temp: 0
max_temp: 350

[extruder]
heater_pin: PA2
sensor_type: PT1000
sensor_pin: PF4
min_temp: 0
max_temp: 320
control: dual_loop_pid
inner_sensor_name: hotblock
inner_max_temp: 380
pid_Kp: 24.0
pid_Ki: 1.05
pid_Kd: 138.0
inner_pid_kp: 14.0
inner_pid_ki: 0.6
inner_pid_kd: 60.0
```

In the example, the load thermistor is the in-nozzle PT1000; the
heatbreak cartridge has a separate thermistor wired to a generic ADC
pin. The inner loop holds the cartridge below 380 °C even if the outer
loop ever asks for more.

## Calibration

Dual-loop tuning is a two-pass version of standard PID tuning. The
`PID_CALIBRATE` workflow handles both passes in one command.

1. Cold-start the heater. Both sensors should read room temperature.
2. Run `PID_CALIBRATE HEATER=<name> TARGET=<load_setpoint>`. The
   autotune first tunes the inner (element) loop with the outer loop
   pinned, then tunes the outer (load) loop with the inner loop active.
   The whole sequence takes 15–40 minutes depending on thermal mass.
3. The autotune writes both `pid_Kp/Ki/Kd` (outer) and
   `inner_pid_kp/ki/kd` (inner) into the active profile, plus a
   `/tmp/heattest.csv` (outer pass) and `/tmp/heattest_secondary.csv`
   (inner pass) trace.
4. Run `SAVE_CONFIG` to persist.

If the inner loop fails to converge, check that `inner_max_temp` is at
least 30 °C above your typical load setpoint and that the inner sensor
is electrically clean. The autotune raises an explicit error if the
inner loop's relay swing would clip against `inner_max_temp`.

Per-condition profiles (e.g. one tune for a closed enclosure, another
for an open machine) work the same way as for single-loop heaters —
see [PID profiles](heater-pid-profiles.md). Dual-loop profiles must
include both inner and outer gain triples.

## G-code commands

The runtime command surface is shared with the regular PID controllers:

- [`PID_CALIBRATE`](heater-pid-autotune.md) extended to handle the
  two-pass dual-loop calibration when the heater's `control` is
  `dual_loop_pid`.
- [`PID_PROFILE`](heater-pid-profiles.md) handles dual-loop profiles
  the same as positional or velocity-form profiles; profiles that store
  `dual_loop_pid` gains carry both inner and outer triples.

`SET_HEATER_PID` is **not** available for dual-loop heaters — the
controller has six independent gains that don't map cleanly to the
single-triple `KP/KI/KD` parameter set. Use `PID_PROFILE LOAD` against a
pre-tuned profile to retune live, or `SAVE_CONFIG` after a fresh
`PID_CALIBRATE` run.

## Things to know

- **`inner_max_temp` is a safety wall, not a setpoint hint.** Pick it
  with margin above your highest expected load setpoint (typically
  `target_temp + 50–80 °C`). The inner loop will saturate at the wall
  during a cold-start ramp; that's intended.
- **Both sensors must be configured before the heater section.** Klipper
  loads `[temperature_sensor]` sections during heater setup and binds
  them by short name — a typo in `inner_sensor_name` produces a config
  error at startup.
- **Calibrate inner first, then outer.** The autotune does this
  automatically. If you tune the gains by hand, follow the same order:
  the outer loop's response depends on the inner loop already being
  stable.
- **Cold-extrude / `min_extrude_temp` is enforced on the load sensor.**
  The dual-loop heater honours `min_extrude_temp` against the smoothed
  primary (load) temperature, exactly like a single-sensor heater. The
  inner-sensor reading is irrelevant to the extrusion interlock.
- **Profile-stored dual-loop gains include `inner_pid_*` keys.** When
  running `PID_PROFILE SAVE` against a dual-loop heater, the saved
  profile section includes both gain triples; profiles tuned for
  positional or velocity controllers cannot be loaded into a dual-loop
  heater.

## Credits

Based on the dual-loop PID controller from the Kalico community
(Zeanon, KalicoCrew PR #735), imported and re-derived against the
current Klipper heater control class layout. Companion bugfixes for
the secondary heat-test handling and output filename consistency
(KalicoCrew PRs #758 and #809) are folded into the same change.
