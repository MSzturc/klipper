import os
import time
import logging
from pathlib import Path
import threading

class AutoReload:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.stop_event = threading.Event()
        filepath_to_monitor = Path("~/printer_data/config/printer.cfg").expanduser()
        self.start_monitoring(filepath_to_monitor)

    def monitor_file(self, filepath):
        filepath = Path(filepath)
        if not filepath.exists():
            logging.warning(f"The file {filepath} does not exist.")
            return
        
        last_modified_time = filepath.stat().st_mtime
        logging.info(f"Started monitoring the file {filepath}.")

        try:
            while not self.stop_event.is_set():
                time.sleep(1)  # Check every second
                current_modified_time = filepath.stat().st_mtime
                if current_modified_time != last_modified_time:
                    last_modified_time = current_modified_time
                    logging.debug(f"Change detected in {filepath}. Executing RELOAD_GCODE_MACROS.")
                    self.gcode.run_script_from_command("RELOAD_GCODE_MACROS")
        except Exception as e:
            logging.error(f"Error while monitoring {filepath}: {e}")
        finally:
            logging.debug(f"Stopped monitoring the file {filepath}.")

    def start_monitoring(self, filepath):
        """Starts file monitoring in a separate thread."""
        self.stop_event.clear()
        monitor_thread = threading.Thread(target=self.monitor_file, args=(filepath,), daemon=True)
        monitor_thread.start()
        logging.debug(f"File monitoring started for {filepath}.")

    def stop_monitoring(self):
        """Stops the file monitoring."""
        self.stop_event.set()
        logging.debug("File monitoring is being stopped.")

def load_config(config):
    return AutoReload(config)
