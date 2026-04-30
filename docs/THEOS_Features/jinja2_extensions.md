# Jinja2 Extensions

## What this is

Klipper's gcode_macro Jinja2 environment now enables two standard Jinja2 extensions:

1. **`jinja2.ext.do`** — adds the `{% do %}` tag, which evaluates an expression and discards the result. Useful for calling methods that mutate state (`list.append`, `dict.update`) without printing the return value.
2. **`jinja2.ext.loopcontrols`** — adds `{% break %}` and `{% continue %}`, the two flow-control constructs missing from a stock Jinja2 `{% for %}` loop.

Stock macros work unchanged. The new tags become available everywhere the macro template is parsed.

## When to use this

- A macro that builds a list while iterating over `printer.objects` and needs `{% do mylist.append(x) %}`.
- A loop that should bail out early once a match is found — `{% break %}`.
- A loop that should skip an iteration based on a condition — `{% continue %}`.

You don't need this if your macros only use simple variable interpolation and conditionals; stock Jinja2 covers those.

## Configuration

There is nothing to configure. Both extensions are loaded into the gcode_macro Jinja2 environment unconditionally.

## Examples

### Building a list with `{% do %}`

```ini
[gcode_macro LIST_HEATERS]
gcode:
    {% set names = [] %}
    {% for name, heater in printer.heaters.available_heaters | groupby(0) %}
        {% do names.append(name) %}
    {% endfor %}
    RESPOND MSG="Heaters: {names | join(', ')}"
```

### Bailing out with `{% break %}`

```ini
[gcode_macro FIND_FIRST_HOMED]
gcode:
    {% set found = namespace(axis=none) %}
    {% for axis in 'xyz' %}
        {% if printer.toolhead.homed_axes is iterable and axis in printer.toolhead.homed_axes %}
            {% set found.axis = axis %}
            {% break %}
        {% endif %}
    {% endfor %}
    RESPOND MSG="First homed axis: {found.axis}"
```

### Skipping with `{% continue %}`

```ini
[gcode_macro WIPE_ALL_NOZZLES]
gcode:
    {% for ext in printer.extruders %}
        {% if printer[ext].temperature < 150 %}
            {% continue %}
        {% endif %}
        ACTIVATE_EXTRUDER EXTRUDER={ext}
        WIPE
    {% endfor %}
```

## Things to know

- **Both extensions are part of the standard Jinja2 distribution.** No additional dependency is introduced. They have always existed; Klipper's gcode_macro env simply did not enable them.
- **Stock Jinja2 documentation applies.** The behaviour matches [the official Jinja2 docs](https://jinja.palletsprojects.com/en/stable/templates/#extensions); nothing fork-specific is layered on top.
- **`{% do %}` does not suppress side effects.** It only suppresses the *output*. If the expression has side effects (which is the whole point of `{% do %}`), they still happen.
- **No interaction with `RELOAD_GCODE_MACROS`.** Macro reload re-parses the template body with the same env, so newly added `{% do %}` and `{% break %}` constructs are picked up by the next reload.

## Credits

Based on Kalico PR #26 (`jinja2.ext.do`) by Morten Lindhardt and PR #442 (`jinja2.ext.loopcontrols`) by Frank Tackitt. Re-derived on the current Klipper base.
