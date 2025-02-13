# Mechanical bed tilt calibration with multiple Z steppers
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import mathutil
from . import probe

class ZAdjustHelper:
    def __init__(self, config, z_count):
        # Initialize ZAdjustHelper with printer configuration and expected Z steppers.
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.z_count = z_count
        self.z_steppers = []
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        logging.info(f"ZAdjustHelper initialized for {self.name} with {z_count} Z steppers.")

    def handle_connect(self):
        # Handle connection and validate Z steppers.
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        z_steppers = [s for s in kin.get_steppers() if s.is_active_axis('z')]
        if len(z_steppers) != self.z_count:
            raise self.printer.config_error(
                f"{self.name} z_positions needs exactly {len(z_steppers)} items")
        if len(z_steppers) < 2:
            raise self.printer.config_error(
                f"{self.name} requires multiple Z steppers")
        self.z_steppers = z_steppers
        logging.info(f"Z steppers connected: {[s.get_name() for s in z_steppers]}.")

    def adjust_steppers(self, adjustments, speed):
        # Adjust Z steppers to achieve leveling.
        toolhead = self.printer.lookup_object('toolhead')
        gcode = self.printer.lookup_object('gcode')
        curpos = toolhead.get_position()

        # Report adjustments.
        stepstrs = [f"{s.get_name()} = {a:.6f}" for s, a in zip(self.z_steppers, adjustments)]
        msg = "Making the following Z adjustments:\n%s" % ("\n".join(stepstrs),)
        gcode.respond_info(msg)
        logging.info(msg)

        # Disable Z stepper movements.
        toolhead.flush_step_generation()
        for s in self.z_steppers:
            s.set_trapq(None)

        # Sort and move each Z stepper.
        positions = [(-a, s) for a, s in zip(adjustments, self.z_steppers)]
        positions.sort(key=lambda k: k[0])
        first_stepper_offset, first_stepper = positions[0]
        z_low = curpos[2] - first_stepper_offset

        for i in range(len(positions) - 1):
            stepper_offset, stepper = positions[i]
            next_stepper_offset, next_stepper = positions[i + 1]
            toolhead.flush_step_generation()
            stepper.set_trapq(toolhead.get_trapq())
            curpos[2] = z_low + next_stepper_offset
            try:
                toolhead.move(curpos, speed)
                toolhead.set_position(curpos)
                logging.debug(f"Stepper {stepper.get_name()} moved to position {curpos[2]:.6f}.")
            except:
                logging.exception("Error during Z adjustment.")
                toolhead.flush_step_generation()
                for s in self.z_steppers:
                    s.set_trapq(toolhead.get_trapq())
                raise

        # Final cleanup to ensure leveling.
        last_stepper_offset, last_stepper = positions[-1]
        toolhead.flush_step_generation()
        last_stepper.set_trapq(toolhead.get_trapq())
        curpos[2] += first_stepper_offset
        toolhead.set_position(curpos)
        logging.info("Z leveling adjustments complete.")

class ZAdjustStatus:
    def __init__(self, printer):
        # Track the status of Z adjustments.
        self.applied = False
        printer.register_event_handler("stepper_enable:motor_off", self._motor_off)
        logging.info("ZAdjustStatus initialized.")

    def check_retry_result(self, retry_result):
        # Update status based on retry result.
        if retry_result == "done":
            self.applied = True
        return retry_result

    def reset(self):
        # Reset the status.
        self.applied = False
        logging.debug("ZAdjustStatus reset.")

    def get_status(self, eventtime):
        # Provide the current status.
        return {'applied': self.applied}

    def _motor_off(self, print_time):
        # Reset status when motors are disabled.
        self.reset()

class RetryHelper:
    def __init__(self, config, error_msg_extra=""):
        # Initialize retry logic with configuration parameters.
        self.gcode = config.get_printer().lookup_object('gcode')
        self.default_max_retries = config.getint("retries", 0, minval=0)
        self.default_retry_tolerance = config.getfloat("retry_tolerance", 0., above=0.)
        self.value_label = "Probed points range"
        self.error_msg_extra = error_msg_extra
        logging.info("RetryHelper initialized.")

    def start(self, gcmd):
        # Start retry logic with command parameters.
        self.max_retries = gcmd.get_int('RETRIES', self.default_max_retries, minval=0, maxval=30)
        self.retry_tolerance = gcmd.get_float('RETRY_TOLERANCE', self.default_retry_tolerance, minval=0.0, maxval=1.0)
        self.current_retry = 0
        self.previous = None
        self.increasing = 0
        logging.debug(f"Retry started: max_retries={self.max_retries}, retry_tolerance={self.retry_tolerance}")

    def check_increase(self, error):
        # Check if error has consistently increased.
        if self.previous and error > self.previous + 0.0000001:
            self.increasing += 1
        elif self.increasing > 0:
            self.increasing -= 1
        self.previous = error
        return self.increasing > 1

    def check_retry(self, z_positions):
        # Evaluate retry conditions based on Z positions.
        if self.max_retries == 0:
            return "done"
        error = round(max(z_positions) - min(z_positions), 6)
        self.gcode.respond_info(f"Retries: {self.current_retry}/{self.max_retries} {self.value_label}: {error:.6f} tolerance: {self.retry_tolerance:.6f}")
        logging.debug(f"Retry check: current_retry={self.current_retry}, error={error}, tolerance={self.retry_tolerance}")

        if self.check_increase(error):
            raise self.gcode.error(f"Retries aborting: {self.value_label} is increasing. {self.error_msg_extra}")
        if error <= self.retry_tolerance:
            return "done"
        self.current_retry += 1
        if self.current_retry > self.max_retries:
            raise self.gcode.error("Too many retries")
        return "retry"


class ZTilt:
    def __init__(self, config):
        # Initialize Z tilt adjustment process.
        self.printer = config.get_printer()
        self.z_positions = config.getlists('z_positions', seps=(',', '\n'), parser=float, count=2)
        self.retry_helper = RetryHelper(config)
        self.probe_helper = probe.ProbePointsHelper(config, self.probe_finalize)
        self.probe_helper.minimum_points(2)
        self.z_status = ZAdjustStatus(self.printer)
        self.z_helper = ZAdjustHelper(config, len(self.z_positions))
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('Z_TILT_ADJUST', self.cmd_Z_TILT_ADJUST, desc=self.cmd_Z_TILT_ADJUST_help)
        logging.info("ZTilt initialized.")

    cmd_Z_TILT_ADJUST_help = "Adjust the Z tilt"

    def cmd_Z_TILT_ADJUST(self, gcmd):
        # Execute Z tilt adjustment via G-code.
        logging.info("Executing Z_TILT_ADJUST command.")
        self.z_status.reset()
        self.retry_helper.start(gcmd)
        self.probe_helper.start_probe(gcmd)

    def probe_finalize(self, offsets, positions):
        # Finalize the probe and calculate adjustments.
        z_offset = offsets[2]
        logging.info(f"Calculating bed tilt with positions: {positions}")
        params = {'x_adjust': 0., 'y_adjust': 0., 'z_adjust': z_offset}

        # Perform coordinate descent optimization.
        def adjusted_height(pos, params):
            x, y, z = pos
            return z - x * params['x_adjust'] - y * params['y_adjust'] - params['z_adjust']

        def errorfunc(params):
            total_error = sum((adjusted_height(pos, params) ** 2 for pos in positions))
            return total_error

        new_params = mathutil.coordinate_descent(params.keys(), params, errorfunc)
        logging.info(f"Calculated bed tilt parameters: {new_params}")

        # Apply calculated adjustments.
        speed = self.probe_helper.get_lift_speed()
        x_adjust = new_params['x_adjust']
        y_adjust = new_params['y_adjust']
        z_adjust = (new_params['z_adjust'] - z_offset - x_adjust * offsets[0] - y_adjust * offsets[1])
        adjustments = [x * x_adjust + y * y_adjust + z_adjust for x, y in self.z_positions]
        self.z_helper.adjust_steppers(adjustments, speed)

        return self.z_status.check_retry_result(self.retry_helper.check_retry([p[2] for p in positions]))

    def get_status(self, eventtime):
        # Get the current status of Z tilt adjustments.
        return self.z_status.get_status(eventtime)

def load_config(config):
    return ZTilt(config)
