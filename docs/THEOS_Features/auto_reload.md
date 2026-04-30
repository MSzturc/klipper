# Auto Reload

## What this is

A new `[auto_reload]` config section starts a background watcher on `~/printer_data/config/printer.cfg`. When the file's modification time changes, the watcher schedules `RELOAD_GCODE_MACROS` on the gcode reactor — the same command the [ConfigParser Extensions](configparser_extensions.md) feature already provides for in-place macro reloading.

The result: edit `printer.cfg` in your editor, hit save, and your `[gcode_macro ...]` bodies refresh automatically without typing `RELOAD_GCODE_MACROS` or restarting Klipper.

## When to use this

- You iterate on macro logic frequently and want each save to take effect immediately.
- You edit `printer.cfg` over SSH or via the Mainsail/Fluidd file editor and would rather not switch to the gcode console after every save.
- You're tuning a macro from the print floor (touchscreen, tablet) where typing gcode is awkward.

You don't need this if you only edit `printer.cfg` between prints and `RESTART` is your normal workflow anyway.

## Configuration

A single empty section enables the watcher:

```ini
[auto_reload]
```

The watched path is fixed at `~/printer_data/config/printer.cfg` — the canonical Mainsail/Fluidd/Moonraker convention. There is no config knob today; if you keep `printer.cfg` outside that directory, you cannot use this feature.

## Things to know

- **Reload semantics match `RELOAD_GCODE_MACROS`.** Only the *bodies* of existing `[gcode_macro ...]` sections are refreshed. Adding a new macro section to `printer.cfg` does not register a new macro; deleting a section does not unregister the macro. For those changes, run `RESTART`.
- **Polling, not inotify.** The watcher polls the file's mtime once per second. There is a sub-second delay between save and reload. CPU cost is negligible; it is one stat call per second.
- **The watcher stops on `klippy:disconnect`.** Each klippy restart cleanly shuts down the polling thread before the new instance starts a fresh one. Long-running klippy sessions therefore do not accumulate watcher threads across `RESTART` / `FIRMWARE_RESTART` cycles.
- **Reloads are scheduled, not synchronous.** When a file change is detected, the watcher hands the reload to the reactor as an async callback. The reactor runs it on its own thread with the gcode mutex held — so reloads never race with active gcode processing.
- **Errors in the watched file are reported through the normal reload path.** A syntax error in `printer.cfg` causes `RELOAD_GCODE_MACROS` to fail with the same message you'd see if you had typed the gcode by hand. The watcher itself does not catch or hide config errors.

## Credits

Original work in this fork. The reload mechanism it triggers is the [ConfigParser Extensions](configparser_extensions.md) `RELOAD_GCODE_MACROS` command, which is in turn based on Kalico PR #305 by Rogerio Goncalves.
