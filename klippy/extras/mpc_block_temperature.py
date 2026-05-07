# Virtual sensor that exposes an MPC heater's internal block-state temperature
#
# Copyright (C) 2025  Klipper Developers
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class MpcBlockTemperature:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.heater = None
        self.heater_name = config.get('heater_name')
        self.temperature_callback = None
        self.temp = self.min_temp = self.max_temp = 0.
        self.ignore = config.getboolean('ignore_limits', False)
        self.echo_limits_to_console = config.getboolean(
            'echo_limits_to_console', False)
        self.printer.register_event_handler('klippy:ready',
                                            self._handle_ready)
    def _handle_ready(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)
        self.heater.add_mpc_sensor(self)
    def setup_callback(self, temperature_callback):
        self.temperature_callback = temperature_callback
    def setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
    def get_report_time_delta(self):
        return self.heater.sensor.get_report_time_delta()
    def process_temp_update(self, control, read_time):
        ctype = control.get_type()
        if ctype == 'mpc':
            self.temp = control.state_block_temp
        else:
            # During MPC_CALIBRATE the active control is TuningControl which
            # has no state_block_temp; fall back to the heater's smoothed
            # sensor reading so the virtual sensor keeps reporting a
            # plausible value rather than raising AttributeError.
            self.temp = self.heater.smoothed_temp
        if self.temp is None:
            self.temp = 0.
        else:
            mcu = self.heater.mcu_pwm.get_mcu()
            if (not mcu.non_critical_disconnected
                and (self.temp < self.min_temp or self.temp > self.max_temp)):
                msg = ("MPC Heater Block %s\n"
                       "Temperature %0.1f outside range of %0.1f-%0.1f"
                       % (self.name, self.temp,
                          self.min_temp, self.max_temp))
                if not self.ignore:
                    self.printer.invoke_shutdown(msg)
                elif self.echo_limits_to_console:
                    gcode = self.printer.lookup_object('gcode')
                    gcode.respond_error(msg)
        self.temperature_callback(read_time, self.temp)
    def set_report_time(self, report_time):
        pass


def load_config(config):
    pheaters = config.get_printer().load_object(config, 'heaters')
    pheaters.add_sensor_factory('mpc_block_temperature',
                                MpcBlockTemperature)
