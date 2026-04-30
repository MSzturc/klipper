# File-watcher that runs RELOAD_GCODE_MACROS on printer.cfg changes
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os
import threading


class AutoReload:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.stop_event = threading.Event()
        self.monitor_thread = None
        filepath_to_monitor = os.path.expanduser(
            "~/printer_data/config/printer.cfg")
        self.printer.register_event_handler(
            "klippy:disconnect", self._handle_disconnect)
        self.start_monitoring(filepath_to_monitor)

    def _handle_disconnect(self):
        self.stop_monitoring()

    def _trigger_reload(self):
        def _do_reload(eventtime):
            # Call the reload handler directly under the gcode mutex
            # instead of routing through gcode.run_script.  run_script
            # dispatches through GCodeDispatch._process_commands, whose
            # catch-all branch invokes printer.invoke_shutdown() on any
            # non-CommandError exception (configfile.error included).
            # A syntax error in printer.cfg would therefore shut Klipper
            # down rather than just surfacing the parse error.
            try:
                gcode_macro = self.printer.lookup_object('gcode_macro')
                with self.gcode.get_mutex():
                    try:
                        gcode_macro.cmd_RELOAD_GCODE_MACROS(None)
                    except Exception as e:
                        # Surface the failure to connected clients via
                        # the same "!!" channel run_script's normal
                        # error path would have used, then continue --
                        # never propagate out of the async callback
                        # (would hit the reactor's error-exit path).
                        logging.exception(
                            "auto_reload: RELOAD_GCODE_MACROS failed")
                        self.gcode.respond_raw(
                            "!! auto_reload: RELOAD_GCODE_MACROS failed:"
                            " %s" % (e,))
            except Exception:
                logging.exception("auto_reload: reload dispatch failed")
        self.reactor.register_async_callback(_do_reload)

    def monitor_file(self, filepath):
        if not os.path.exists(filepath):
            logging.warning("auto_reload: file %s does not exist; "
                            "monitoring disabled", filepath)
            return
        try:
            last_modified_time = os.path.getmtime(filepath)
        except OSError as e:
            logging.warning("auto_reload: cannot stat %s: %s", filepath, e)
            return
        logging.info("auto_reload: monitoring %s", filepath)
        try:
            while not self.stop_event.is_set():
                if self.stop_event.wait(timeout=1.0):
                    break
                try:
                    current_modified_time = os.path.getmtime(filepath)
                except OSError:
                    continue
                if current_modified_time != last_modified_time:
                    last_modified_time = current_modified_time
                    logging.debug(
                        "auto_reload: change detected in %s; "
                        "scheduling RELOAD_GCODE_MACROS", filepath)
                    self._trigger_reload()
        except Exception as e:
            logging.error(
                "auto_reload: error while monitoring %s: %s", filepath, e)
        finally:
            logging.debug("auto_reload: stopped monitoring %s", filepath)

    def start_monitoring(self, filepath):
        self.stop_event.clear()
        self.monitor_thread = threading.Thread(
            target=self.monitor_file, args=(filepath,))
        self.monitor_thread.daemon = True
        self.monitor_thread.start()
        logging.debug("auto_reload: monitoring thread started for %s",
                      filepath)

    def stop_monitoring(self):
        self.stop_event.set()
        logging.debug("auto_reload: monitoring thread stop requested")


def load_config(config):
    return AutoReload(config)
