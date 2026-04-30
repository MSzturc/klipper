# Autoload Modules

## What this is

`[force_move]`, `[respond]`, and `[exclude_object]` are loaded by default — you do not need to list them in `printer.cfg` to get their commands. Each module still accepts an `enable_<name>` config option (default `True`) that lets you disable it explicitly when you don't want the surface area.

In stock Klipper these three modules are inert until their config sections appear in `printer.cfg`. Klipper-Slicer macro libraries, calibration scripts, and slicer profiles routinely assume the corresponding gcode commands are available; missing one of these sections is a frequent and noisy footgun.

| Module | Default-enabled commands |
|--------|--------------------------|
| `force_move` | `FORCE_MOVE`, `SET_KINEMATIC_POSITION`, `STEPPER_BUZZ` |
| `respond` | `M118`, `RESPOND` |
| `exclude_object` | `EXCLUDE_OBJECT`, `EXCLUDE_OBJECT_START`, `EXCLUDE_OBJECT_END`, `EXCLUDE_OBJECT_DEFINE` |

## When to use this

You don't actively reach for this — it's a default. The opt-out exists for a small set of cases:

- A printer where `FORCE_MOVE` is dangerous (e.g. a kinematic the operator should not bypass) and you want the command unavailable to tighten the surface.
- A pipeline where `RESPOND` / `M118` collides with another responder mechanism.
- A printer that doesn't run sliced jobs and never benefits from `[exclude_object]`.

## Configuration

The modules autoload without any section. To opt out, add a section with `enable_<name>: False`:

```ini
[force_move]
enable_force_move: False

[respond]
enable_respond: False

[exclude_object]
enable_exclude_object: False
```

When a section is present without the `enable_*` key, the module behaves as if `enable_*: True` was set (the default).

## Things to know

- **`STEPPER_BUZZ` always registers.** That command is registered per stepper in `register_stepper`, which runs before the `enable_force_move` gate. Disabling `force_move` only suppresses `FORCE_MOVE` and `SET_KINEMATIC_POSITION`.
- **`exclude_object` opt-out skips event handlers too.** When `enable_exclude_object: False`, the module returns early from its `__init__` — no `klippy:connect` or `virtual_sdcard:reset_file` handlers are registered. This is stronger than just dropping the gcode commands.
- **`respond` opt-out keeps `default_prefix` parsing.** The config keys `default_type` and `default_prefix` are read before the gate, so they don't error out when the module is disabled. They're simply unused.
- **No behavioural change for users with explicit sections.** If your `printer.cfg` already has `[force_move]`, `[respond]`, or `[exclude_object]` sections, autoload is a no-op for those — the explicit section is loaded first, autoload sees the module is already loaded and skips it.

## Credits

Based on Kalico PRs #135 (force_move autoload), #306 (exclude_object autoload), and the respond-by-default change by Rogerio Goncalves. Re-derived on the current Klipper base; the `enable_<name>` opt-out semantics are preserved.
