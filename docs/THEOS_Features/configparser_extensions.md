# ConfigParser Extensions

## What this is

`printer.cfg` (and every file pulled in via `[include …]`) is parsed by an
extended config reader that supports variable interpolation, simple
arithmetic, default values, declarative overrides between sub-configs and the
main file, and a few include conveniences. None of these features are
on/off switches — they're always available and only kick in when you write a
config that uses them.

The extensions are:

1. **Variable interpolation** — write `${section.option}` (or `${option}` to
   reference the current section) anywhere in a value. The reader
   substitutes the referenced value before any consumer sees it.
2. **Default values** — append `:default` to a placeholder
   (`${section.option:default}`) and the literal `default` is used when the
   referenced key is missing.
3. **Arithmetic evaluation** — if a value, after interpolation, is a pure
   numeric expression or one of `min(...)`, `max(...)`, `abs(...)`,
   `round(...)`, the reader evaluates it and stores the result.
4. **Default value `None`** — when an interpolation falls through to the
   literal `None`, the entry is dropped from the section entirely. Lets you
   author `${constants.opt:None}` patterns that gate optional config blocks.
5. **`printer.cfg` always wins over sub-configs** — values declared in the
   main file are not overwritten by any later include. Sub-configs can
   declare defaults that the user overrides at the top level.
6. **Conditional includes** — `[include if:${expr} path/to/file.cfg]` only
   pulls in the file when the expression evaluates truthy. Expressions can
   reference values from any already-parsed section.
7. **Variable references inside `[include]` paths** — `[include
   ${constants.profile}.cfg]` resolves the placeholder before searching for
   the file.
8. **Recursive subfolder globs in includes** — `[include sub/**/*.cfg]`
   pulls in every `.cfg` under `sub/`, regardless of nesting depth.
9. **Escape syntax** — write `\${literal}` to keep a literal `${literal}`
   in the output (useful for macros that emit text containing
   placeholders).
10. **`RELOAD_GCODE_MACROS`** — a G-code command that re-reads the config
    and replaces the body of every `[gcode_macro …]` section without
    restarting Klipper.

## When to use this

- You maintain a multi-printer or multi-toolhead setup and want to keep
  shared values like motor current, build area, or include profile in one
  place — define them once in a `[constants]` section and reference them
  everywhere.
- You modularise your config into per-feature files and want the main
  `printer.cfg` to be able to override any sub-config value without editing
  the sub-config.
- You're iterating on a `[gcode_macro]` body and don't want to wait for a
  full Klipper restart between attempts.
- You want a single config tree that adapts to a runtime flag (e.g.
  enable a sensor section only when `${features.has_probe}` is `true`).

You don't need any of this if your printer has a single `printer.cfg` with
no includes and no shared values — the extensions sit out of the way.

## Configuration

There is no enabling flag — the extensions are always active. They only
take effect on lines that use the new syntax.

### The `[constants]` section

A reserved section name for declaring values to be referenced from
elsewhere. The reader doesn't error on `[constants]` containing options
that no module reads, so it's the natural home for `${constants.X}`
sources.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| (any) | n/a | Any key/value you want to reference via `${constants.<key>}`. |

### Variable interpolation syntax

| Syntax | Meaning |
|--------|---------|
| `${option}` | Look up `option` in the same section as the value being read. |
| `${section.option}` | Look up `option` in `section`, regardless of where the value being read lives. |
| `${section.option:default}` | Same as above, but if `section.option` does not exist, use the literal `default`. |
| `${section.option:None}` | Drop the entry entirely when the lookup fails. |
| `\${literal}` | Output a literal `${literal}` — no interpolation. |

### Arithmetic evaluation

After all `${...}` placeholders in a value are resolved, the reader
checks whether the result is a pure expression. If yes, it evaluates and
stores the numeric result. Supported forms:

| Form | Example | Result |
|------|---------|--------|
| Pure numeric | `1 + 2 * 3` | `7` |
| `min(a, b, ...)` | `min(150, ${constants.size_x})` | the smaller of the two |
| `max(a, b, ...)` | `max(0, ${constants.offset})` | the larger of the two |
| `abs(x)` | `abs(${constants.delta})` | absolute value |
| `round(x)` | `round(${constants.size_x})` | nearest integer |

Mixing these is fine: `max(0, ${constants.size_x} - ${constants.margin})`.

### Conditional includes

```ini
[constants]
has_probe: true
profile: high

[include shared/base.cfg]
[include shared/${constants.profile}.cfg]
[include if:${constants.has_probe} probe/bltouch.cfg]
[include shared/macros/**/*.cfg]
```

The expression in `if:${...}` is evaluated as Python with no built-ins,
in a context where every already-parsed section is available as a
namespace (so `${stepper_x.endstop_pin}` is a string, etc.). Truthy
result → include applied; falsy or evaluation error → include skipped
(with a warning to the log).

### Override semantics

When the main `printer.cfg` and an included file both declare the same
`[section] option`, the value from `printer.cfg` always wins, regardless
of where the `[include]` line sits. This makes it safe to keep
sub-configs full of "good defaults" and override individual values from
the top level.

### Minimal example

```ini
[constants]
run_current: 1.5
size_x: 200
size_y: 200

[stepper_x]
position_max: ${constants.size_x}
position_min: 0

[stepper_y]
position_max: ${constants.size_y}

[tmc5160 stepper_x]
run_current: ${constants.run_current}

[tmc5160 stepper_y]
run_current: ${constants.run_current}

[gcode_macro PARK_CENTER]
gcode:
  G1 X{ ${constants.size_x} / 2 } Y{ ${constants.size_y} / 2 } F6000
```

## G-code commands

### `RELOAD_GCODE_MACROS`

Re-reads `printer.cfg` (with all includes) and replaces the body of
every existing `[gcode_macro …]` section with the new gcode template.
Macro sections that did not exist before the reload are NOT created;
sections that have been deleted are NOT removed. To pick up structural
changes you still need a full Klipper restart.

```
RELOAD_GCODE_MACROS
```

## Things to know

- **Interpolation depth is capped.** A reference chain that would require
  more than 10 substitutions is treated as a runtime error. Avoid
  circular references — `${a.x}` referencing `${b.y}` referencing
  `${a.x}` will hit the cap and raise.
- **Arithmetic is restricted by design.** The reader will not call
  arbitrary Python functions. If a value isn't a pure numeric expression
  or one of `min`/`max`/`abs`/`round`, it stays as-is — no error, no
  silent attempt to evaluate.
- **`abs()` and `round()` take exactly one argument.** Passing more raises
  a parse error.
- **`round(x)` returns an integer**, not a float — `round(3.7)` is `4`,
  not `4.0`. `min` and `max` return floats when fed any float.
- **The `[constants]` section is allowed to contain unused keys.** Other
  sections still error on undefined options; only `[constants]` is
  exempt from the unused-options check.
- **`RELOAD_GCODE_MACROS` only swaps macro bodies.** It does not
  re-instantiate gcode_macro objects, does not pick up newly added
  `[gcode_macro …]` sections, and does not undo deletions. Treat it as a
  fast path for iterating on existing macros, not as a config reload
  command.
- **Conditional include expressions run with `__builtins__: None`.** You
  cannot use `len`, `print`, `import`, etc. — only attribute access on
  the section namespaces and basic operators (`==`, `>`, `and`, `or`,
  `not`).
- **Escaping with `\${}` is for the value text only.** It's not a config
  reader directive; the backslash stays in the source file. If a macro
  emits `${something}` from Jinja, write `{% raw %}${something}{% endraw %}`
  in the macro body — the configfile escape is for situations where the
  config author wants `${...}` to land literally in the parsed value.

## Credits

Variable interpolation is based on the Kalico fork's PR #448 by Frank
Tackitt, with the non-string-value safety guard from PR #482 and the
`\$` escape syntax from PR #753. The remaining additions
(arithmetic evaluation with `min`/`max`/`abs`/`round`, default values,
default `None`, override-from-printer.cfg semantics, conditional
includes, variable-reference includes, recursive subfolder globs,
`RELOAD_GCODE_MACROS`, `configfile.warn()` API) are original work by
Matt Szturc.
