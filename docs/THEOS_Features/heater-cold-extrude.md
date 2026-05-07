# Cold Extrude

## What this is

Klipper refuses to extrude when the hotend is below `min_extrude_temp`
(default 170 °C). That interlock prevents jammed feeders and cold
filament damaging the gears, but there are situations where you
genuinely want a cold extrude move: loading filament from cold,
clearing a heatbreak with a hot pull, debugging the extruder kinematics
without wasting a heat-up cycle.

This fork adds two ways to override the interlock at runtime — without
restarting Klipper, without editing your config, and without losing the
safety net the next time you actually want it:

1. **`M302`** — the Marlin-compatible cold-extrude command, so existing
   slicer macros and host scripts that emit M302 do the right thing on
   Klipper too.
2. **`COLD_EXTRUDE`** — a Klipper-native multiplex command that targets
   any heater (extruder, generic) by short name and reports the current
   state when called with no parameters.

Both commands can also adjust the heater's `min_extrude_temp` for the
session and write the new value to disk via `SAVE_CONFIG`.

## When to use this

- Loading filament for the first time on a brand-new printer where you
  haven't run a hotend heat yet.
- Clearing a clogged heatbreak with a cold pull (heat to the right
  point, then pull while still warm but below normal extrude temp).
- Debugging a stepper or drive-train issue where you want extruder
  steps without warming up the hotend.
- Switching an active job temporarily into a cold-extrude mode (some
  filament-load macros do this in their last step) and switching back
  cleanly afterwards.

You don't need this for normal printing. The default
`min_extrude_temp: 170` interlock is on for a reason.

## G-code commands

### `M302` (Marlin-compatible)

| Parameter | Meaning |
|-----------|---------|
| `T<index>` | Optional extruder index (`T0` is the primary `[extruder]`, `T1` is `[extruder1]`, etc.). Without `T`, the active extruder is used. |
| `P<0\|1>` | Enable cold extrusion: `P1` allows extrusion at any temperature, `P0` re-enables the `min_extrude_temp` interlock. |
| `S<temp>` | Set the heater's `min_extrude_temp` to `<temp>` °C for the current session. Persisted on the next `SAVE_CONFIG`. |

```
M302 P1            ; allow cold extrusion on the active extruder
M302 P0            ; re-enable the safety interlock
M302 S170          ; set the threshold to 170 °C, leave enable state as set
M302 P1 S0         ; allow cold extrusion and lower the threshold to 0 °C
M302 T1 P1         ; allow cold extrusion on the second extruder only
```

With no parameters, `M302` is a no-op (the parameter parsing requires
at least one of `T`/`P`/`S` to do anything). Use `COLD_EXTRUDE` (below)
if you want to query the current state.

### `COLD_EXTRUDE`

Klipper-native form. Targets any heater — not just extruders — and
prints the current state when called with no parameters.

| Parameter | Meaning |
|-----------|---------|
| `HEATER` | Short name of the heater (e.g. `extruder`, `extruder1`, `heater_generic chamber`). |
| `ENABLE` | `1` to allow extrusion at any temperature, `0` to re-enable the interlock. Optional. |
| `MIN_EXTRUDE_TEMP` | Set the heater's `min_extrude_temp` to this value in °C, for the session. Persisted on the next `SAVE_CONFIG`. Optional. |

```
COLD_EXTRUDE HEATER=extruder ENABLE=1
COLD_EXTRUDE HEATER=extruder MIN_EXTRUDE_TEMP=180
COLD_EXTRUDE HEATER=extruder            ; report current state
```

## Things to know

- **Setting only `MIN_EXTRUDE_TEMP` clears the cold-extrude flag.** Both
  `M302 S<temp>` (without `P`) and `COLD_EXTRUDE MIN_EXTRUDE_TEMP=…`
  (without `ENABLE`) reset `cold_extrude` to `0` as a side effect. This
  matches Marlin's M302 semantics; if you want to keep cold-extrude
  enabled while also adjusting the threshold, pass both `P1` (or
  `ENABLE=1`) and `S` (or `MIN_EXTRUDE_TEMP`) in the same call.
- **`SAVE_CONFIG` only persists the `MIN_EXTRUDE_TEMP` change.** The
  `cold_extrude` flag itself is intentionally a runtime-only setting —
  there is no `cold_extrude` config key and a printer restart always
  re-enables the interlock. Don't expect `M302 P1` to survive a
  `FIRMWARE_RESTART`.
- **Threshold parameters are bounded by the heater's `min_temp`/
  `max_temp`.** `M302 S<temp>` and `COLD_EXTRUDE MIN_EXTRUDE_TEMP=…`
  reject values outside the heater's configured temperature range. The
  bounds are enforced by gcmd parameter validation.
- **Dual-loop heaters honour cold-extrude too.** A hotend running
  `control: dual_loop_pid` reads `cold_extrude` against the smoothed
  primary (load) sensor temperature in exactly the same way as a
  single-sensor heater.
- **There is no per-extruder default cold-extrude config key.** If you
  routinely run a printer without the interlock (e.g. a paste
  extruder), set `min_extrude_temp: 0` in the heater section. The
  runtime override is for occasional use, not for permanent disabling.

## Credits

Based on the M302 / COLD_EXTRUDE implementation from the Kalico
community (Zeanon, KalicoCrew PR #750), with the dual-sensor heater
follow-up (KalicoCrew PR #766) folded into the same change.
Re-derived against the current Klipper extruder and heater layout.
