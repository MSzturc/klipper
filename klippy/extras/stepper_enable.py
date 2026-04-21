# Support for enable pins on stepper motor drivers
#
# Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

DISABLE_STALL_TIME = 0.100

# Tracking of shared stepper enable pins
class StepperEnablePin:
    def __init__(self, mcu_enable, enable_count):
        self.mcu_enable = mcu_enable
        self.enable_count = enable_count
        self.is_dedicated = True
    def set_enable(self, print_time):
        did_emit = not self.enable_count
        if did_emit:
            self.mcu_enable.set_digital(print_time, 1)
        self.enable_count += 1
        mcu = self.mcu_enable.get_mcu() if self.mcu_enable is not None else None
        if mcu is not None and getattr(mcu, "is_non_critical", False):
            logging.info("Stepper enable pin set_enable emit=%s count=%d",
                         did_emit, self.enable_count)
    def set_disable(self, print_time):
        self.enable_count -= 1
        did_emit = not self.enable_count
        if did_emit:
            self.mcu_enable.set_digital(print_time, 0)
        mcu = self.mcu_enable.get_mcu() if self.mcu_enable is not None else None
        if mcu is not None and getattr(mcu, "is_non_critical", False):
            logging.info("Stepper enable pin set_disable emit=%s count=%d",
                         did_emit, self.enable_count)
    def reset_after_mcu_disconnect(self):
        # A hot-unplug or reboot clears the MCU GPIO state; the host-side
        # reference count must be reset so the next enable emits a real
        # pin update against the freshly-booted driver.
        # Preserve the no-enable-pin sentinel (mcu_enable is None,
        # count=9999) used by steppers that are always-enabled with no
        # dedicated line - resetting it to zero would make the next
        # set_enable attempt to drive a None pin.
        if self.mcu_enable is None:
            return
        self.enable_count = 0

def setup_enable_pin(printer, pin):
    if pin is None:
        # No enable line (stepper always enabled)
        enable = StepperEnablePin(None, 9999)
        enable.is_dedicated = False
        return enable
    ppins = printer.lookup_object('pins')
    pin_params = ppins.lookup_pin(pin, can_invert=True,
                                  share_type='stepper_enable')
    enable = pin_params.get('class')
    if enable is not None:
        # Shared enable line
        enable.is_dedicated = False
        return enable
    mcu_enable = pin_params['chip'].setup_pin('digital_out', pin_params)
    mcu_enable.setup_max_duration(0.)
    enable = pin_params['class'] = StepperEnablePin(mcu_enable, 0)
    return enable

# Enable line tracking for each stepper motor
class EnableTracking:
    def __init__(self, stepper, enable):
        self.stepper = stepper
        self.enable = enable
        self.callbacks = []
        self.is_enabled = False
        self._resync_needed = False
        self.stepper.add_active_callback(self.motor_enable)
        mcu = self.stepper.get_mcu()
        if getattr(mcu, "is_non_critical", False):
            printer = mcu.get_printer()
            printer.register_event_handler(
                mcu.get_non_critical_disconnect_event_name(),
                self._handle_noncritical_disconnect)
            printer.register_event_handler(
                mcu.get_non_critical_reconnect_event_name(),
                self._handle_noncritical_reconnect)
    def register_state_callback(self, callback):
        self.callbacks.append(callback)
    def motor_enable(self, print_time):
        mcu = self.stepper.get_mcu()
        # Defer enable transitions while a non-critical MCU is offline or
        # currently reconnecting - this callback can fire from the flush
        # path while the serial queue is down, where an enable write would
        # raise and force a full printer shutdown.
        if (getattr(mcu, "is_non_critical", False)
                and (getattr(mcu, "non_critical_disconnected", False)
                     or getattr(mcu, "_connecting", False))):
            self.is_enabled = False
            self.stepper.add_active_callback(self.motor_enable)
            return
        if not self.is_enabled:
            for cb in self.callbacks:
                cb(print_time, True)
            self.enable.set_enable(print_time)
            self.is_enabled = True
        self._resync_needed = False
    def motor_disable(self, print_time):
        if self.is_enabled:
            # Enable stepper on future stepper movement
            for cb in self.callbacks:
                cb(print_time, False)
            self.enable.set_disable(print_time)
            self.is_enabled = False
            self.stepper.add_active_callback(self.motor_enable)
        self._resync_needed = False
    def is_motor_enabled(self):
        return self.is_enabled
    def has_dedicated_enable(self):
        return self.enable.is_dedicated
    def _handle_noncritical_disconnect(self):
        # MCU reboot drops driver enable state - reset the shared count so
        # the next move emits a real enable update via the callback path.
        self.enable.reset_after_mcu_disconnect()
        logging.info(
            "Reset stepper enable count for '%s' on non-critical disconnect",
            self.stepper.get_name())
        if not self.is_enabled:
            self._resync_needed = False
            return
        self._resync_needed = True
        self.is_enabled = False
        self.stepper.add_active_callback(self.motor_enable)
        logging.info(
            "Marked stepper '%s' disabled due to non-critical disconnect",
            self.stepper.get_name())
    def _handle_noncritical_reconnect(self):
        if not self._resync_needed:
            return
        mcu = self.stepper.get_mcu()
        if getattr(mcu, "non_critical_disconnected", False):
            return
        self._resync_needed = False
        logging.info(
            "Stepper '%s' will re-enable on next movement after"
            " non-critical reconnect", self.stepper.get_name())

# Global stepper enable line tracking
class PrinterStepperEnable:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.enable_lines = {}
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)
        # Register M18/M84 commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("M18", self.cmd_M18)
        gcode.register_command("M84", self.cmd_M18)
    def register_stepper(self, config, mcu_stepper):
        name = mcu_stepper.get_name()
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('SET_STEPPER_ENABLE', "STEPPER", name,
                                   self.cmd_SET_STEPPER_ENABLE,
                                   desc=self.cmd_SET_STEPPER_ENABLE_help)
        enable = setup_enable_pin(self.printer, config.get('enable_pin', None))
        self.enable_lines[name] = EnableTracking(mcu_stepper, enable)
    def set_motors_enable(self, stepper_names, enable):
        toolhead = self.printer.lookup_object('toolhead')
        # Flush steps to ensure all auto enable callbacks invoked
        toolhead.flush_step_generation()
        print_time = None
        did_change = False
        for stepper_name in stepper_names:
            el = self.enable_lines[stepper_name]
            if el.is_motor_enabled() == enable:
                continue
            if print_time is None:
                # Dwell for sufficient delay from any previous auto enable
                if not enable:
                    toolhead.dwell(DISABLE_STALL_TIME)
                print_time = toolhead.get_last_move_time()
            if enable:
                el.motor_enable(print_time)
            else:
                el.motor_disable(print_time)
            did_change = True
        # Dwell to ensure sufficient delay prior to a future auto enable
        if did_change and not enable:
            toolhead.dwell(DISABLE_STALL_TIME)
        return did_change
    def motor_off(self):
        self.set_motors_enable(self.get_steppers(), False)
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.get_kinematics().clear_homing_state("xyz")
        self.printer.send_event("stepper_enable:motor_off")
    def get_status(self, eventtime):
        steppers = { name: et.is_motor_enabled()
                     for (name, et) in self.enable_lines.items() }
        return {'steppers': steppers}
    def _handle_request_restart(self, print_time):
        self.motor_off()
    def cmd_M18(self, gcmd):
        # Turn off motors
        self.motor_off()
    cmd_SET_STEPPER_ENABLE_help = "Enable/disable individual stepper by name"
    def cmd_SET_STEPPER_ENABLE(self, gcmd):
        stepper_name = gcmd.get('STEPPER')
        stepper_enable = gcmd.get_int('ENABLE', 1)
        self.set_motors_enable([stepper_name], stepper_enable)
        if stepper_enable:
            logging.info("%s has been manually enabled", stepper_name)
        else:
            logging.info("%s has been manually disabled", stepper_name)
    def lookup_enable(self, name):
        if name not in self.enable_lines:
            raise self.printer.config_error("Unknown stepper '%s'" % (name,))
        return self.enable_lines[name]
    def get_steppers(self):
        return list(self.enable_lines.keys())

def load_config(config):
    return PrinterStepperEnable(config)
