# `use_probe_xy_offsets` Config Key

## What this is

A config key that lets sections using Klipper's `ProbePointsHelper` (such as `[bed_mesh]`, `[bed_tilt]`, `[z_tilt]`, `[quad_gantry_level]`, `[delta_calibrate]`, and `[screws_tilt_adjust]`) override whether the probe's X/Y offsets are applied to each probe point during probing.

Stock Klipper's `bed_mesh` always applies the probe's `x_offset` / `y_offset` (it calls `use_xy_offsets(True)` unconditionally). Other sections leave offsets off. This is hard-coded and the user has no way to change it from `printer.cfg`. With `use_probe_xy_offsets`, every section consuming `ProbePointsHelper` exposes the choice.

## When to use this

- You probe with two different probes and the secondary one shouldn't have its offsets applied (e.g., a touch probe with measured offsets and an inductive sensor whose offsets are baked into the kinematic).
- You're debugging mesh accuracy issues and want to confirm whether offset compensation is active.
- You're building a non-standard kinematic where probe-XY-offset application would double-shift the toolhead.

If your printer has a single probe and the stock behaviour is correct, you can ignore this key — defaults match upstream.

## Configuration

The key reads as a boolean from the section that owns the `ProbePointsHelper`. Default behaviour matches stock Klipper for every section.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `use_probe_xy_offsets` | `True` for `[bed_mesh]`; `False` elsewhere | When `True`, each probe point is shifted by the probe's `x_offset` / `y_offset` so the nozzle ends at the requested mesh coordinate after probing. When `False`, the toolhead moves to the literal probe-point coordinates. |

### Minimal example

```ini
[bed_mesh]
mesh_min: 10,10
mesh_max: 280,280
# The default for [bed_mesh] is True. Set False to make the toolhead
# move to the literal mesh coordinates without offset compensation.
use_probe_xy_offsets: False
```

```ini
[z_tilt]
z_positions:
    -50, 100
    250, 100
    100, 250
points:
    50, 50
    200, 50
    125, 200
# z_tilt's default is False. Set True if your probe's xy offsets should
# be compensated during the screw-adjust probing.
use_probe_xy_offsets: True
```

## Things to know

- **The key is scoped to the section that owns the helper.** Setting `use_probe_xy_offsets` in `[bed_mesh]` only affects bed mesh probing. To change the behaviour for `[z_tilt]`, set it in the `[z_tilt]` section.
- **The default per section matches upstream Klipper behaviour.** No printer.cfg that worked before this key changes behaviour after upgrade.
- **The override is a single boolean.** There's no per-axis toggle; X and Y offsets are applied (or not) together.
- **The `ProbePointsHelper.use_xy_offsets()` setter still exists** for code that constructs the helper imperatively, but it is no longer needed: passing `use_offsets=` to the constructor and overriding via the config key are the canonical paths.

## Credits

Based on Kalico PR [#500](https://github.com/KalicoCrew/kalico/pull/500) by Rogerio Goncalves. Re-derived on the current Klipper base; the helper signature still matches develop's, so the patch carries over directly without further adaptation.
