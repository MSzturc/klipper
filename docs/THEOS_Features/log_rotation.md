# Log Rotation

## What this is

Stock Klipper's `klippy.log` rotates at midnight via `TimedRotatingFileHandler`. That works for printers running 24/7, but is awkward for shop printers that run a few hours per day, restart often, and would benefit from a fresh log per session for diagnostics.

This fork extends the logger with two ways to trigger rotation explicitly:

1. **`--rotate-log-at-restart` startup flag** — passed to `klippy.py` on the command line. When set, `klippy.log` rotates at every klippy startup and at every restart in the same process (e.g. after `RESTART` or `FIRMWARE_RESTART`).
2. **`LOG_ROLLOVER` G-code command** — triggers a rotation on demand from any gcode console.

Both share the same backup count as upstream (`backupCount=5`).

## When to use this

- You diagnose a startup or boot issue and want each restart in its own log file.
- You hit an issue mid-print and want to capture only that print's log without sifting through midnight rollovers.
- You run a printer that's powered off most of the day and prefer one log per session over one log per day.

You don't need this if midnight rotation is fine for your use.

## Configuration

There is nothing to add to `printer.cfg`. The startup flag is set wherever klippy is launched — typically in the systemd unit or the `klippy.service` start command:

```
ExecStart=/home/pi/klippy-env/bin/python /home/pi/klipper/klippy/klippy.py /home/pi/printer_data/config/printer.cfg --rotate-log-at-restart -l /home/pi/printer_data/logs/klippy.log -I /home/pi/printer_data/comms/klippy.serial -a /home/pi/printer_data/comms/klippy.sock
```

When `--rotate-log-at-restart` is set, the underlying `TimedRotatingFileHandler` is configured with a 24-hour interval rather than a midnight trigger — so passive rotation still happens once per day even if you never restart.

## G-code commands

### `LOG_ROLLOVER`

Triggers a log rotation immediately. Works whether or not `--rotate-log-at-restart` was set; the only requirement is that klippy was started with `-l <logfile>` (otherwise there is no rotating handler to call).

```
LOG_ROLLOVER
```

The active log line `=============== Log rollover at <timestamp> ===============` separates pre- and post-rollover content in the new file.

## Things to know

- **`LOG_ROLLOVER` works without `--rotate-log-at-restart`.** The flag controls automatic rotation behaviour; the gcode is independent and always available when a logfile is configured.
- **`LOG_ROLLOVER` errors out cleanly when no logfile is configured.** Klippy launched without `-l` has no `bglogger` to call; the gcode reports a clear error rather than crashing.
- **Rollover info survives.** Versions, git status, and CPU/device info are re-emitted into the new file via the same rollover-info hook upstream uses.
- **Backup count is 5.** The 6th rotation deletes the oldest archive. This matches upstream and is not currently configurable.

## Credits

Based on Kalico PR #181 by Rogerio Goncalves (rotate-at-restart) and PR #498 (LOG_ROLLOVER gcode). Re-derived on the current Klipper base.
