# Helper code for implementing homing operations
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4

# Return a completion that completes when all completions in a list complete
def multi_complete(printer, completions):
    if len(completions) == 1:
        return completions[0]
    # Build completion that waits for all completions
    reactor = printer.get_reactor()
    cp = reactor.register_callback(lambda e: [c.wait() for c in completions])
    # If any completion indicates an error, then exit main completion early
    for c in completions:
        reactor.register_callback(
            lambda e, c=c: cp.complete(1) if c.wait() else 0)
    return cp

# Tracking of stepper positions during a homing/probing move
class StepperPosition:
    def __init__(self, stepper, endstop_name):
        self.stepper = stepper
        self.endstop_name = endstop_name
        self.stepper_name = stepper.get_name()
        self.start_pos = stepper.get_mcu_position()
        self.start_cmd_pos = stepper.mcu_to_commanded_position(self.start_pos)
        self.halt_pos = self.trig_pos = None
    def note_home_end(self, trigger_time):
        self.halt_pos = self.stepper.get_mcu_position()
        self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)
    def verify_no_probe_skew(self, haltpos):
        new_start_pos = self.stepper.get_mcu_position(self.start_cmd_pos)
        if new_start_pos != self.start_pos:
            logging.warning(
                "Stepper '%s' position skew after probe: pos %d now %d",
                self.stepper.get_name(), self.start_pos, new_start_pos)

# Implementation of homing/probing moves
class HomingMove:
    def __init__(self, printer, endstops, toolhead=None):
        # Initialize the HomingMove object with printer, endstops, and optionally a toolhead.
        self.printer = printer
        self.endstops = endstops
        if toolhead is None:
            toolhead = printer.lookup_object('toolhead')
        self.toolhead = toolhead
        self.stepper_positions = []
        self.distance_elapsed = []
        logging.info("HomingMove initialized with printer and endstops.")

    def get_mcu_endstops(self):
        # Return a list of MCU endstops.
        logging.debug("Fetching MCU endstops.")
        return [es for es, name in self.endstops]

    def _calc_endstop_rate(self, mcu_endstop, movepos, speed):
        # Calculate the rate for an endstop move based on distance and speed.
        startpos = self.toolhead.get_position()
        axes_d = [mp - sp for mp, sp in zip(movepos, startpos)]
        move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        move_t = move_d / speed
        max_steps = max([(abs(s.calc_position_from_coord(startpos)
                              - s.calc_position_from_coord(movepos))
                          / s.get_step_dist())
                         for s in mcu_endstop.get_steppers()])
        if max_steps <= 0.:
            logging.info("Maximum steps calculated as zero or negative. Returning minimal rate.")
            return .001
        logging.debug(f"Calculated endstop rate: {move_t / max_steps}")
        return move_t / max_steps

    def calc_toolhead_pos(self, kin_spos, offsets):
        # Calculate the toolhead position based on kinematic positions and offsets.
        kin_spos = dict(kin_spos)
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()
        thpos = self.toolhead.get_position()
        logging.debug(f"Calculated toolhead position: {thpos}")
        return list(kin.calc_position(kin_spos))[:3] + thpos[3:]

    def homing_move(self, movepos, speed, probe_pos=False,
                    triggered=True, check_triggered=True):
        # Perform a homing move to a specified position at a given speed.
        logging.info("Starting homing move.")
        self.printer.send_event("homing:homing_move_begin", self)

        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        self.stepper_positions = [ StepperPosition(s, name)
                                   for es, name in self.endstops
                                   for s in es.get_steppers() ]

        print_time = self.toolhead.get_last_move_time()
        endstop_triggers = []
        for mcu_endstop, name in self.endstops:
            rest_time = self._calc_endstop_rate(mcu_endstop, movepos, speed)
            wait = mcu_endstop.home_start(print_time, ENDSTOP_SAMPLE_TIME,
                                          ENDSTOP_SAMPLE_COUNT, rest_time,
                                          triggered=triggered)
            endstop_triggers.append(wait)
        logging.info("Endstop triggers initialized.")
        all_endstop_trigger = multi_complete(self.printer, endstop_triggers)
        self.toolhead.dwell(HOMING_START_DELAY)

        error = None
        try:
            self.toolhead.drip_move(movepos, speed, all_endstop_trigger)
            logging.debug("Issued homing move command.")
        except self.printer.command_error as e:
            error = f"Error during homing move: {str(e)}"
            logging.error(error)

        trigger_times = {}
        move_end_print_time = self.toolhead.get_last_move_time()
        for mcu_endstop, name in self.endstops:
            try:
                trigger_time = mcu_endstop.home_wait(move_end_print_time)
                logging.debug(f"Endstop {name} triggered at time: {trigger_time}")
            except self.printer.command_error as e:
                if error is None:
                    error = f"Error during homing {name}: {str(e)}"
                logging.error(error)
                continue
            if trigger_time > 0.:
                trigger_times[name] = trigger_time
            elif check_triggered and error is None:
                error = f"No trigger on {name} after full movement"
                logging.warning(error)

        self.toolhead.flush_step_generation()
        for sp in self.stepper_positions:
            tt = trigger_times.get(sp.endstop_name, move_end_print_time)
            sp.note_home_end(tt)

        if probe_pos:
            halt_steps = {sp.stepper_name: sp.halt_pos - sp.start_pos
                          for sp in self.stepper_positions}
            trig_steps = {sp.stepper_name: sp.trig_pos - sp.start_pos
                          for sp in self.stepper_positions}
            haltpos = trigpos = self.calc_toolhead_pos(kin_spos, trig_steps)
            if trig_steps != halt_steps:
                haltpos = self.calc_toolhead_pos(kin_spos, halt_steps)
            self.toolhead.set_position(haltpos)
            logging.info("Probe position determined.")
            for sp in self.stepper_positions:
                sp.verify_no_probe_skew(haltpos)
        else:
            haltpos = trigpos = movepos
            over_steps = {sp.stepper_name: sp.halt_pos - sp.trig_pos
                          for sp in self.stepper_positions}
            steps_moved = {
                sp.stepper_name: (sp.halt_pos - sp.start_pos)
                * sp.stepper.get_step_dist()
                for sp in self.stepper_positions
            }
            filled_steps_moved = {
                sname: steps_moved.get(sname, 0)
                for sname in [s.get_name() for s in kin.get_steppers()]
            }
            self.distance_elapsed = kin.calc_position(filled_steps_moved)
            logging.debug(f"Calculated distance elapsed: {self.distance_elapsed}")
            if any(over_steps.values()):
                self.toolhead.set_position(movepos)
                halt_kin_spos = {s.get_name(): s.get_commanded_position()
                                 for s in kin.get_steppers()}
                haltpos = self.calc_toolhead_pos(halt_kin_spos, over_steps)
            self.toolhead.set_position(haltpos)

        try:
            self.printer.send_event("homing:homing_move_end", self)
            logging.info("Homing move completed successfully.")
        except self.printer.command_error as e:
            if error is None:
                error = str(e)
                logging.error(error)
        if error is not None:
            raise self.printer.command_error(error)
        return trigpos

    def check_no_movement(self):
        # Check if there has been no movement during the homing process.
        logging.debug("Checking for no movement.")
        if self.printer.get_start_args().get('debuginput') is not None:
            return None
        for sp in self.stepper_positions:
            if sp.start_pos == sp.trig_pos:
                logging.info(f"No movement detected for endstop: {sp.endstop_name}")
                return sp.endstop_name
        return None

    def moved_less_than_dist(self, min_dist, homing_axes):
        # Check if the movement is less than a specified minimum distance on the homing axes.
        homing_axis_distances = [
            dist
            for i, dist in enumerate(self.distance_elapsed)
            if i in homing_axes
        ]
        distance_tolerance = .75
        if any(
            [
                abs(dist) < min_dist
                and min_dist - abs(dist) >= distance_tolerance
                for dist in homing_axis_distances
            ]
        ):
            logging.warning("Movement less than the minimum distance detected.")
            return True
        return False
class Homing:
    def __init__(self, printer):
        # Initialize the Homing object with the printer instance.
        self.printer = printer
        self.toolhead = printer.lookup_object('toolhead')
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}
        logging.info("Homing initialized.")

    def set_axes(self, axes):
        # Set the axes affected by the homing process.
        self.changed_axes = axes
        logging.debug(f"Homing axes set to: {axes}")

    def get_axes(self):
        # Get the axes affected by the homing process.
        logging.debug(f"Retrieving homing axes: {self.changed_axes}")
        return self.changed_axes

    def get_trigger_position(self, stepper_name):
        # Get the trigger position for a given stepper.
        logging.debug(f"Retrieving trigger position for stepper: {stepper_name}")
        return self.trigger_mcu_pos.get(stepper_name, None)

    def set_stepper_adjustment(self, stepper_name, adjustment):
        # Set the adjustment for a specific stepper.
        self.adjust_pos[stepper_name] = adjustment
        logging.debug(f"Set adjustment for stepper {stepper_name}: {adjustment}")

    def _fill_coord(self, coord):
        # Fill in any None entries in 'coord' with current toolhead position.
        thcoord = list(self.toolhead.get_position())
        for i in range(len(coord)):
            if coord[i] is not None:
                thcoord[i] = coord[i]
        logging.debug(f"Filled coordinates: {thcoord}")
        return thcoord

    def set_homed_position(self, pos):
        # Set the current homed position of the toolhead.
        filled_pos = self._fill_coord(pos)
        self.toolhead.set_position(filled_pos)
        logging.info(f"Homed position set to: {filled_pos}")

    def _set_homing_accel(self, accel, pre_homing):
        if accel is None:
            return
        if pre_homing:
            self.toolhead.set_accel(accel)
        else:
            self.toolhead.reset_accel()

    def _set_current_homing(self, homing_axes, pre_homing):
        logging.info(f"Adjusting Current for homing axes: {homing_axes}")
        # Adjust current settings for homing on the specified axes.
        print_time = self.toolhead.get_last_move_time()
        affected_rails = set()
        for axis in homing_axes:
            axis_name = "xyz"[axis]  # Only valid for Cartesian systems
            partial_rails = self.toolhead.get_active_rails_for_axis(axis_name)
            affected_rails.update(partial_rails)

        dwell_time = 0.0
        for rail in affected_rails:
            chs = rail.get_tmc_current_helpers()
            for ch in chs:
                if ch is not None:
                    current_dwell_time = ch.set_current_for_homing(print_time, pre_homing)
                    dwell_time = max(dwell_time, current_dwell_time)
        if dwell_time:
            self.toolhead.dwell(dwell_time)

    def _reset_endstop_states(self, endstops):
        # Reset the states of all specified endstops.
        print_time = self.toolhead.get_last_move_time()
        for endstop in endstops:
            endstop[0].query_endstop(print_time)
        logging.debug("Endstop states reset.")

    def home_rails(self, rails, forcepos, movepos):
        # Perform the homing process on the specified rails.
        logging.info("Starting homing process.")
        self.printer.send_event("homing:home_rails_begin", self, rails)

        # Alter kinematics to consider the printer at the forced position.
        homing_axes = [axis for axis in range(3) if forcepos[axis] is not None]
        startpos = self._fill_coord(forcepos)
        homepos = self._fill_coord(movepos)
        self.toolhead.set_position(startpos, homing_axes=homing_axes)
        logging.debug(f"Initial positions set. Start position: {startpos}, Home position: {homepos}")

        # Initialize endstops and perform the first homing move.
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = rails[0].get_homing_info()
        logging.debug(f"Homing info retrieved: {hi}")

        hmove = HomingMove(self.printer, endstops)
        self._set_homing_accel(hi.accel, pre_homing=True)
        self._set_current_homing(homing_axes, pre_homing=True)
        self._reset_endstop_states(endstops)
        logging.debug(f"Endstops reset. Performing homing move to {homepos} at speed {hi.speed}")

        hmove.homing_move(homepos, hi.speed)

        needs_rehome = False
        retract_dist = hi.retract_dist
        if hmove.moved_less_than_dist(hi.min_home_dist, homing_axes):
            needs_rehome = True
            retract_dist = hi.min_home_dist
            logging.debug(f"Homing move detected insufficient movement. Needs rehome: {needs_rehome}, Retract distance: {retract_dist}")

        # Perform the second homing move if necessary.
        if retract_dist:
            logging.info("Performing second homing move.")
            startpos = self._fill_coord(forcepos)
            homepos = self._fill_coord(movepos)
            axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
            move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
            retract_r = min(1.0, retract_dist / move_d)
            logging.debug(f"Calculated movement vector: {axes_d}, Distance: {move_d}, Retract ratio: {retract_r}")

            retractpos = [hp - ad * retract_r for hp, ad in zip(homepos, axes_d)]
            logging.debug(f"Calculated retract position: {retractpos}")

            self.toolhead.move(retractpos, hi.retract_speed)
            logging.debug(f"Retracted to {retractpos} at speed {hi.retract_speed}")

            if not hi.use_sensorless_homing or needs_rehome:
                self.toolhead.dwell(0.5)
                logging.debug("Dwell for sensorless homing.")

                startpos = [rp - ad * retract_r for rp, ad in zip(retractpos, axes_d)]
                logging.debug(f"New start position after retract: {startpos}")

                self.toolhead.set_position(startpos)
                self._reset_endstop_states(endstops)
                hmove = HomingMove(self.printer, endstops)

                logging.debug(f"Performing second homing move to {homepos} at speed {hi.second_homing_speed}")
                hmove.homing_move(homepos, hi.second_homing_speed)

                if hmove.check_no_movement() is not None:
                    error_message = "Endstop still triggered after retract"
                    logging.error(error_message)
                    raise self.printer.command_error(error_message)

                if hi.use_sensorless_homing and needs_rehome and hmove.moved_less_than_dist(hi.min_home_dist, homing_axes):
                    error_message = "Early homing trigger on second home!"
                    logging.error(error_message)
                    raise self.printer.command_error(error_message)

        self._set_current_homing(homing_axes, pre_homing=False)
        self._set_homing_accel(hi.accel, pre_homing=False)
        self.toolhead.flush_step_generation()
        logging.debug("Homing settings reset after final movement.")

        # Save final homing positions.
        self.trigger_mcu_pos = {sp.stepper_name: sp.trig_pos for sp in hmove.stepper_positions}
        self.adjust_pos = {}
        logging.debug(f"Final trigger positions: {self.trigger_mcu_pos}")

        self.printer.send_event("homing:home_rails_end", self, rails)

        if any(self.adjust_pos.values()):
            kin = self.toolhead.get_kinematics()
            homepos = self.toolhead.get_position()
            kin_spos = {s.get_name(): (s.get_commanded_position() + self.adjust_pos.get(s.get_name(), 0.0)) for s in kin.get_steppers()}
            newpos = kin.calc_position(kin_spos)
            logging.debug(f"Calculated kinematic positions: {kin_spos}, New position: {newpos}")
            for axis in homing_axes:
                homepos[axis] = newpos[axis]
            self.toolhead.set_position(homepos)

        logging.info("Homing process completed.")

class PrinterHoming:
    def __init__(self, config):
        # Initialize PrinterHoming with the given configuration.
        self.printer = config.get_printer()
        logging.info("PrinterHoming initialized.")

        # Register G-code commands.
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('G28', self.cmd_G28)
        logging.debug("G-code command 'G28' registered.")

    def manual_home(self, toolhead, endstops, pos, speed, triggered, check_triggered):
        # Perform a manual homing move using the specified parameters.
        logging.info(f"Starting manual home: pos={pos}, speed={speed}, triggered={triggered}, check_triggered={check_triggered}")
        hmove = HomingMove(self.printer, endstops, toolhead)
        try:
            hmove.homing_move(pos, speed, triggered=triggered, check_triggered=check_triggered)
            logging.debug("Manual homing move completed successfully.")
        except self.printer.command_error as e:
            if self.printer.is_shutdown():
                error_message = "Homing failed due to printer shutdown"
                logging.error(error_message)
                raise self.printer.command_error(error_message)
            logging.error("Manual homing move failed.")
            raise

    def probing_move(self, mcu_probe, pos, speed):
        # Perform a probing move using the specified parameters.
        logging.info(f"Starting probing move: pos={pos}, speed={speed}")
        endstops = [(mcu_probe, "probe")]
        hmove = HomingMove(self.printer, endstops)
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True)
            logging.debug(f"Probing move completed. Endpoint position: {epos}")
        except self.printer.command_error as e:
            if self.printer.is_shutdown():
                error_message = "Probing failed due to printer shutdown"
                logging.error(error_message)
                raise self.printer.command_error(error_message)
            logging.error("Probing move failed.")
            raise
        if hmove.check_no_movement() is not None:
            error_message = "Probe triggered prior to movement"
            logging.error(error_message)
            raise self.printer.command_error(error_message)
        return epos

    def cmd_G28(self, gcmd):
        # Handle the G28 G-code command for homing.
        logging.info("Executing G28 command for homing.")
        axes = []
        for pos, axis in enumerate('XYZ'):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
                logging.debug(f"Axis {axis} added to homing list.")
        if not axes:
            axes = [0, 1, 2]  # Default to homing all axes.
            logging.debug("No specific axis specified. Defaulting to all axes.")

        homing_state = Homing(self.printer)
        homing_state.set_axes(axes)
        logging.debug(f"Homing axes set: {axes}")

        kin = self.printer.lookup_object('toolhead').get_kinematics()
        try:
            kin.home(homing_state)
            logging.info("G28 homing operation completed successfully.")
        except self.printer.command_error as e:
            if self.printer.is_shutdown():
                error_message = "Homing failed due to printer shutdown"
                logging.error(error_message)
                raise self.printer.command_error(error_message)
            logging.error("Homing operation failed. Turning off motors.")
            self.printer.lookup_object('stepper_enable').motor_off()
            raise


def load_config(config):
    return PrinterHoming(config)
