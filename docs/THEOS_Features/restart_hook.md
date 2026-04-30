# Restart Hook

## What this is

When Klippy restarts (via `RESTART`, `FIRMWARE_RESTART`, or any internal restart trigger), it looks for an executable shell script at:

```
~/printer_data/config/scripts/before-restart-klipper.sh
```

If the file exists and is executable, Klippy runs it once between the old reactor finalising and the new `Printer()` being constructed. The script's stdout is logged at debug level; stderr is logged at warning level. The script is allowed up to 60 seconds to complete.

The hook is **best-effort**. Whatever the script does — succeeds, fails, times out, raises Python exceptions — the restart loop always proceeds. A broken hook does not leave the printer offline.

## When to use this

- You want to clear a transient state file that should not survive a klippy restart.
- You sync logs, snapshot state, or notify an external system at every restart.
- You toggle a hardware line (LED, relay) tied to klippy lifecycle rather than print state.

You don't need this if you have no per-restart side effects to perform.

## Configuration

There is no `printer.cfg` configuration. Drop a script at the path above and make it executable:

```bash
mkdir -p ~/printer_data/config/scripts
cat > ~/printer_data/config/scripts/before-restart-klipper.sh <<'EOF'
#!/bin/bash
echo "Klippy restarting at $(date)"
EOF
chmod +x ~/printer_data/config/scripts/before-restart-klipper.sh
```

## Things to know

- **Errors never abort the restart.** The script's exit code is logged but not acted on. Timeouts (60 seconds) and exceptions are logged and the restart proceeds anyway. This is intentional: the MCU has already been reset by the time the hook runs, so failing to start the next klippy would leave the printer dangling.
- **No environment guarantees.** The script is launched as a child of the klippy process. It inherits klippy's environment, working directory, and user — typically the `pi` user with the standard shell environment. Don't rely on subshell-specific PATH or cwd.
- **Output is captured, not streamed.** stdout and stderr are buffered until the script exits and only then logged. A long-running script with chatty output appears silent in the klippy log until it finishes.
- **No script means no hook.** If `before-restart-klipper.sh` does not exist, the restart proceeds without comment. There's no error or warning logged in that case — the absence of a hook is the normal state.
- **Hook runs after the MCU is already reset.** The `klippy:firmware_restart` event fires inside `printer.run()`, before main()'s restart loop reaches the hook. So when your script runs, the MCU is already mid-reset. Don't try to query MCU state from within the script.

## Credits

Original work in this fork.
