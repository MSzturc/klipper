# Plugin System

## What this is

Stock Klipper's module loader looks for `[section]` handlers under `klippy/extras/`. Touching that directory marks the install as `-dirty` in the version banner because git treats anything under tracked source as a modification. Third-party plugins (Shake&Tune, custom macros, vendor extensions) therefore either ship as patches against `klippy/extras/` or live outside the repo and are awkward to install.

This fork adds a parallel directory `klippy/plugins/` that the loader consults alongside `klippy/extras/`. Anything dropped into `klippy/plugins/` is loaded as if it were an extras module, but lives outside the tracked source tree — so adding a plugin does not flip your install to `-dirty`.

1. **`klippy/plugins/<name>.py`** — single-file plugins. Same shape as an extras module: define `load_config(config)` (or `load_config_prefix(config)`).
2. **`klippy/plugins/<name>/__init__.py`** — package-style plugins. Useful when a plugin ships its own helpers or assets in subdirectories.

Stock extras still take precedence: if a name exists in both `klippy/extras/` and `klippy/plugins/`, the plugin version wins. This lets you override an upstream extra by dropping a same-named file into `klippy/plugins/`.

## When to use this

- You want to install a third-party plugin (Shake&Tune, custom calibration, vendor SDKs) without modifying the Klipper source tree.
- You ship your own private extras module for a single printer and want git status to stay clean.
- You want to override the behaviour of a stock extras module without forking the whole repo.

You don't need this if you write all your customisations as gcode macros — those live in `printer.cfg`, not in Python.

## Configuration

The plugin directory is created on demand:

```bash
mkdir -p ~/klipper/klippy/plugins
```

Drop your plugin files there. No `printer.cfg` change is required to enable the directory itself; configuration is per-plugin and follows whatever `[section]` name the plugin's `load_config` registers.

### Single-file plugin layout

```
klippy/plugins/
  my_plugin.py
```

`my_plugin.py` exposes `load_config(config)` (or `load_config_prefix(config)` for sectioned plugins like `[my_plugin foo]`).

### Package plugin layout

```
klippy/plugins/
  shaketune/
    __init__.py
    helpers/
      ...
```

`__init__.py` exposes `load_config(config)`. Submodules can be imported normally from within the package (`from .helpers import X`).

## G-code commands

This feature adds no gcode commands of its own. Each loaded plugin contributes whatever commands it registers.

## Things to know

- **Plugins shadow extras.** A `klippy/plugins/foo.py` file silently overrides `klippy/extras/foo.py`. This is intentional but easy to forget — when stock behaviour seems wrong, check `klippy/plugins/` first.
- **Plugin imports are namespaced as `extras.<name>`.** Plugins are loaded under the `extras.<name>` module identity so existing `from extras.<other> import X` patterns continue to work for any module that has already been loaded. To pull in another plugin or extras module that has not been loaded yet, call `printer.load_object(config, "<name>")` from your plugin's `__init__` — that is the standard Klipper dependency-resolution pattern, and it works for both stock extras and plugins. A bare top-level `import extras.<other>` only resolves if the target was loaded earlier in the config's load order.
- **No automatic discovery.** Plugins are loaded only when their `[section]` appears in `printer.cfg`. Dropping a file into `klippy/plugins/` without a corresponding config section does nothing.
- **`-dirty` only avoids untracked plugins.** If you locally modify a plugin file (git tracks it because it lives in your fork), the install banner still reports the modification. The dirty exemption is for *added* plugin files, not *edited* tracked files.
- **Klippy must be restarted to load a new plugin.** The loader resolves modules at config-read time. New plugin files are not picked up until the next `RESTART` or `FIRMWARE_RESTART`.

## Credits

Original implementation by Brandon Nance on Kalico (PRs #82 and #100), with package-directory support contributed by Uku. Re-derived on the current Klipper base.
