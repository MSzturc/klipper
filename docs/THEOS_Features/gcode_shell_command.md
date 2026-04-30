# gcode_shell_command

## What this is

A new config section `[gcode_shell_command <name>]` registers a `RUN_SHELL_COMMAND CMD=<name>` G-code that executes a configured shell command. Output is streamed back into the gcode console line by line. The command times out after a configurable interval and is terminated if it exceeds it.

This is useful for triggering OS-level actions from gcode macros without writing a custom Klipper extension: shell scripts, system calls, file operations, hardware controls outside Klipper's MCU surface.

## When to use this

- A `START_PRINT` or `END_PRINT` macro that needs to call out to the OS — sync a file, toggle a relay via `gpio` tools, run a maintenance script.
- A button macro that flips a Linux-side setting (light, fan, USB power) without going through Moonraker.
- A test harness that spawns helper processes during a print.

You don't need this if all your automation runs through Klipper extras and gcode macros, or if you already trigger shell scripts via Moonraker webhooks.

## Configuration

Each shell command is a separate `[gcode_shell_command <name>]` section. The `<name>` is the value passed as `CMD=` to `RUN_SHELL_COMMAND`.

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `command` | required | The command line to run. `~` is expanded to the user's home directory; environment variables (`$HOME`, `$VAR`) are expanded. The command is split with shell-style quoting, so multi-word arguments must be quoted in `printer.cfg`. |
| `timeout` | `2.0` | Maximum runtime in seconds. The process is terminated if it exceeds this. |
| `verbose` | `True` | When true, stdout (merged with stderr) is streamed back to the gcode console. When false, the command runs silently. |

### Minimal example

```ini
[gcode_shell_command beep]
command: aplay /home/pi/printer_data/sounds/beep.wav
timeout: 3.0
verbose: False

[gcode_macro PRINT_DONE_BEEP]
gcode:
    RUN_SHELL_COMMAND CMD=beep
```

## G-code commands

### `RUN_SHELL_COMMAND`

Runs the shell command registered under `<name>`.

| Parameter | Meaning |
|-----------|---------|
| `CMD` | The `<name>` matching a `[gcode_shell_command <name>]` section. |
| `PARAMS` | Optional. Extra arguments appended to the configured command, parsed with shell-style quoting. |

```
RUN_SHELL_COMMAND CMD=beep
RUN_SHELL_COMMAND CMD=relay PARAMS="on outlet1"
```

## Things to know

- **Output is streamed but not interactive.** The command's stdout is read line by line and forwarded to the gcode console. There is no way to feed input into the running process.
- **`stderr` is merged into `stdout`.** Errors from the script appear in the same output stream and are forwarded to the gcode console (when `verbose: True`).
- **Timeouts terminate, not kill.** When the timeout expires, the process receives `SIGTERM`. Long-running commands that ignore SIGTERM survive; tune `timeout` accordingly.
- **`PARAMS` is split with shell quoting.** Quote multi-word parameters: `PARAMS="word1 word2"` is one argument; `PARAMS=word1 word2` is two.
- **`~` and `$VAR` are expanded once at config-read time.** The expansion happens when klippy loads `printer.cfg`, not on each invocation. Changing the user's home directory or environment after klippy has started has no effect.

## Credits

Based on Eric Callahan's [gcode_shell_command](https://github.com/Arksine/klipper_gcode_shell_command) extension, brought into Kalico via PR #71 by Rogerio Goncalves and refined in PR #777 by Clifford. Re-derived on the current Klipper base.
