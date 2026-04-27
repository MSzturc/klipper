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
        self.printer = printer
        self.endstops = [es for es in endstops if es[0].get_steppers()]
        if toolhead is None:
            toolhead = printer.lookup_object('toolhead')
        self.toolhead = toolhead
        self.stepper_positions = []
        # Populated at the end of homing_move() with the cartesian
        # distance each kinematic axis actually travelled during the
        # homing move. The sensorless-homing rehome check compares this
        # against min_home_dist to decide whether a second pass is
        # warranted.
        self.distance_elapsed = []
    def get_mcu_endstops(self):
        return [es for es, name in self.endstops]
    def _calc_endstop_rate(self, mcu_endstop, movepos, speed):
        startpos = self.toolhead.get_position()
        axes_d = [mp - sp for mp, sp in zip(movepos, startpos)]
        move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        move_t = move_d / speed
        max_steps = max([(abs(s.calc_position_from_coord(startpos)
                              - s.calc_position_from_coord(movepos))
                          / s.get_step_dist())
                         for s in mcu_endstop.get_steppers()])
        if max_steps <= 0.:
            return .001
        return move_t / max_steps
    def calc_toolhead_pos(self, kin_spos, offsets):
        kin_spos = dict(kin_spos)
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()
        thpos = self.toolhead.get_position()
        cpos = kin.calc_position(kin_spos)
        return [cp if cp is not None else tp
                for cp, tp in zip(cpos, thpos[:3])] + thpos[3:]
    def homing_move(self, movepos, speed, probe_pos=False,
                    triggered=True, check_triggered=True):
        # Notify start of homing/probing move
        self.printer.send_event("homing:homing_move_begin", self)
        # Note start location
        self.toolhead.flush_step_generation()
        kin = self.toolhead.get_kinematics()
        kin_spos = {s.get_name(): s.get_commanded_position()
                    for s in kin.get_steppers()}
        self.stepper_positions = [ StepperPosition(s, name)
                                   for es, name in self.endstops
                                   for s in es.get_steppers() ]
        # Start endstop checking
        print_time = self.toolhead.get_last_move_time()
        endstop_triggers = []
        for mcu_endstop, name in self.endstops:
            rest_time = self._calc_endstop_rate(mcu_endstop, movepos, speed)
            wait = mcu_endstop.home_start(print_time, ENDSTOP_SAMPLE_TIME,
                                          ENDSTOP_SAMPLE_COUNT, rest_time,
                                          triggered=triggered)
            endstop_triggers.append(wait)
        all_endstop_trigger = multi_complete(self.printer, endstop_triggers)
        self.toolhead.dwell(HOMING_START_DELAY)
        # Issue move
        error = None
        try:
            self.toolhead.drip_move(movepos, speed, all_endstop_trigger)
        except self.printer.command_error as e:
            error = "Error during homing move: %s" % (str(e),)
        # Wait for endstops to trigger
        trigger_times = {}
        move_end_print_time = self.toolhead.get_last_move_time()
        for mcu_endstop, name in self.endstops:
            try:
                trigger_time = mcu_endstop.home_wait(move_end_print_time)
            except self.printer.command_error as e:
                if error is None:
                    error = "Error during homing %s: %s" % (name, str(e))
                continue
            if trigger_time > 0.:
                trigger_times[name] = trigger_time
            elif check_triggered and error is None:
                error = "No trigger on %s after full movement" % (name,)
        # Determine stepper halt positions
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
            for sp in self.stepper_positions:
                sp.verify_no_probe_skew(haltpos)
        else:
            haltpos = trigpos = movepos
            over_steps = {sp.stepper_name: sp.halt_pos - sp.trig_pos
                          for sp in self.stepper_positions}
            # Record how far each kinematic axis travelled from move
            # start to StallGuard trigger (used by the SL-homing rehome
            # check). Compute it by subtracting two absolute
            # calc_position() evaluations rather than feeding step
            # deltas through calc_position directly: kinematics like
            # delta (trilateration on absolute actuator positions) and
            # generic_cartesian (carriage offsets are subtracted from
            # absolute positions) are not linear in step deltas, but
            # their difference between two absolute solutions cancels
            # out the offsets and non-linearities for any affine
            # kinematic.  Use trig_pos (when StallGuard fired), not
            # halt_pos (where the decel ramp came to rest), so
            # deceleration overshoot can't mask an early trigger.
            trig_kin_spos = {
                sp.stepper_name: (
                    sp.start_cmd_pos
                    + (sp.trig_pos - sp.start_pos)
                    * sp.stepper.get_step_dist())
                for sp in self.stepper_positions}
            filled_trig_spos = {
                s.get_name(): trig_kin_spos.get(s.get_name(),
                                                kin_spos[s.get_name()])
                for s in kin.get_steppers()}
            trig_cart = kin.calc_position(filled_trig_spos)
            start_cart = kin.calc_position(kin_spos)
            self.distance_elapsed = [
                (t - s) if (t is not None and s is not None) else 0.
                for t, s in zip(trig_cart, start_cart)]
            if any(over_steps.values()):
                self.toolhead.set_position(movepos)
                halt_kin_spos = {s.get_name(): s.get_commanded_position()
                                 for s in kin.get_steppers()}
                haltpos = self.calc_toolhead_pos(halt_kin_spos, over_steps)
            self.toolhead.set_position(haltpos)
        # Signal homing/probing move complete
        try:
            self.printer.send_event("homing:homing_move_end", self)
        except self.printer.command_error as e:
            if error is None:
                error = str(e)
        if error is not None:
            raise self.printer.command_error(error)
        return trigpos
    def check_no_movement(self):
        if self.printer.get_start_args().get('debuginput') is not None:
            return None
        for sp in self.stepper_positions:
            if sp.start_pos == sp.trig_pos:
                return sp.endstop_name
        return None
    # Return True when every homing axis fell short of min_dist by at
    # least the hardcoded tolerance — i.e. nothing in the homing batch
    # showed a reliably long first-pass travel. Used by the sensorless-
    # homing state machine to decide whether to rehome against
    # min_home_dist. `homing_axes` is a string of axis letters
    # (e.g. "x", "xy", "xyz"). `all` (not `any`) is required for
    # kinematics like Delta/Deltesian where home_rails passes
    # homing_axes="xyz" but cartesian motion is along a single axis —
    # `any` would force a rehome on every Delta home because the two
    # non-moving cartesian axes always read 0. The 0.5 mm tolerance
    # absorbs sub-mm StallGuard jitter (motor current/temperature/
    # belt-backlash) that would otherwise force a rehome cycle without
    # improving accuracy.
    def moved_less_than_dist(self, min_dist, homing_axes, tolerance=0.5):
        axis_indices = [i for i, ch in enumerate("xyz") if ch in homing_axes]
        moved = [
            dist for i, dist in enumerate(self.distance_elapsed)
            if i in axis_indices]
        return bool(moved) and all(
            abs(d) < min_dist and (min_dist - abs(d)) >= tolerance
            for d in moved)

# State tracking of homing requests
class Homing:
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object('toolhead')
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}
    def set_axes(self, axes):
        self.changed_axes = axes
    def get_axes(self):
        return self.changed_axes
    def get_trigger_position(self, stepper_name):
        return self.trigger_mcu_pos[stepper_name]
    def set_stepper_adjustment(self, stepper_name, adjustment):
        self.adjust_pos[stepper_name] = adjustment
    def _fill_coord(self, coord):
        # Fill in any None entries in 'coord' with current toolhead position
        thcoord = list(self.toolhead.get_position())
        for i in range(len(coord)):
            if coord[i] is not None:
                thcoord[i] = coord[i]
        return thcoord
    def set_homed_position(self, pos):
        self.toolhead.set_position(self._fill_coord(pos))
    # Pre/post-homing TMC current swap. Collect every rail whose
    # steppers are active on the axes being homed, then invoke each
    # rail's current helpers in one pass so the dwell required after
    # the current swap fires exactly once (the maximum across all
    # affected helpers), regardless of how many rails participate.
    def _set_current_homing(self, homing_axes, pre_homing):
        logging.info("SL-homing: adjusting current for homing axes: %s",
                     homing_axes)
        print_time = self.toolhead.get_last_move_time()
        affected_rails = set()
        for axis_name in homing_axes:
            affected_rails.update(
                self.toolhead.get_active_rails_for_axis(axis_name))
        dwell_time = 0.
        for rail in affected_rails:
            get_chs = getattr(rail, 'get_tmc_current_helpers', None)
            if get_chs is None:
                continue
            for ch in get_chs():
                if ch is None:
                    continue
                dwell_time = max(
                    dwell_time,
                    ch.set_current_for_homing(print_time, pre_homing))
        if dwell_time:
            self.toolhead.dwell(dwell_time)
    # Apply any per-rail homing_accel override for the duration of a
    # homing move. Called with pre_homing=True before the move, and
    # pre_homing=False after, so max_accel unwinds to the configured
    # value.
    def _set_homing_accel(self, accel, pre_homing):
        if accel is None:
            return
        if pre_homing:
            self.toolhead.set_accel(accel)
        else:
            self.toolhead.reset_accel()
    # Explicit endstop-state reset before homing. Stale StallGuard
    # triggers from a prior home can otherwise cause the next home to
    # trigger immediately on move start.
    def _reset_endstop_states(self, endstops):
        print_time = self.toolhead.get_last_move_time()
        for es, _ in endstops:
            es.query_endstop(print_time)
    def home_rails(self, rails, forcepos, movepos):
        # Notify of upcoming homing operation
        self.printer.send_event("homing:home_rails_begin", self, rails)
        # Alter kinematics class to think printer is at forcepos
        force_axes = [axis for axis in range(3) if forcepos[axis] is not None]
        homing_axes = "".join(["xyz"[i] for i in force_axes])
        startpos = self._fill_coord(forcepos)
        homepos = self._fill_coord(movepos)
        self.toolhead.set_position(startpos, homing_axes=homing_axes)
        # Collect endstops across all participating rails. All rails in
        # a single home_rails() call share their homing parameters via
        # the first rail's homing_info.
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = rails[0].get_homing_info()
        try:
            # Pre-homing setup runs inside the try so that a partial
            # failure (e.g. a TMC driver write error mid-current-swap
            # on a multi-rail batch) still hits the finally and
            # attempts to roll the affected drivers back to
            # run_current.  The rollback is idempotent — drivers that
            # never moved are no-ops in needs_*_current_change().
            #
            # Order: optional homing-accel override, then TMC current
            # swap with a single batched dwell, then endstop state
            # reset so stale StallGuard triggers don't fire on the
            # first move.
            self._set_homing_accel(hi.accel, pre_homing=True)
            self._set_current_homing(homing_axes, pre_homing=True)
            self._reset_endstop_states(endstops)
            # Perform first home
            logging.debug("SL-homing: first home at speed %s to %s",
                          hi.speed, homepos)
            hmove = HomingMove(self.printer, endstops)
            hmove.homing_move(homepos, hi.speed)
            # Decide whether the first pass moved far enough for a
            # StallGuard-based endstop trigger to be reliable. If not,
            # rehome against min_home_dist instead of retract_dist.
            needs_rehome = False
            retract_dist = hi.retract_dist
            if (hi.use_sensorless_homing
                and hmove.moved_less_than_dist(hi.min_home_dist,
                                                homing_axes)):
                needs_rehome = True
                retract_dist = hi.min_home_dist
                logging.info(
                    "SL-homing: rehome triggered (min_home_dist=%s)",
                    hi.min_home_dist)
            # Perform second home
            if retract_dist:
                # Retract — computed kinematic-agnostically via
                # axes_d/retract_r so Delta/CoreXZ/etc. are handled
                # uniformly (no axis-specific arithmetic).
                startpos = self._fill_coord(forcepos)
                homepos = self._fill_coord(movepos)
                axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
                move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
                retract_r = min(1., retract_dist / move_d)
                retractpos = [hp - ad * retract_r
                              for hp, ad in zip(homepos, axes_d)]
                self.toolhead.move(retractpos, hi.retract_speed)
                # Run the second pass only when a non-sensorless home
                # is happening, or when the first sensorless pass came
                # up short and a rehome is needed.
                if (not hi.use_sensorless_homing) or needs_rehome:
                    # Home again
                    startpos = [rp - ad * retract_r
                                for rp, ad in zip(retractpos, axes_d)]
                    self.toolhead.set_position(startpos)
                    self._reset_endstop_states(endstops)
                    hmove = HomingMove(self.printer, endstops)
                    hmove.homing_move(homepos, hi.second_homing_speed)
                    if hmove.check_no_movement() is not None:
                        raise self.printer.command_error(
                            "Endstop %s still triggered after retract"
                            % (hmove.check_no_movement(),))
                    if (hi.use_sensorless_homing and needs_rehome
                            and hmove.moved_less_than_dist(
                                hi.min_home_dist, homing_axes)):
                        raise self.printer.command_error(
                            "Early homing trigger on second home!")
                    # After a rehome second pass, retract once more to
                    # leave the toolhead at the conventional post-home
                    # standoff distance.
                    if needs_rehome and hi.retract_dist:
                        startpos = self._fill_coord(forcepos)
                        homepos = self._fill_coord(movepos)
                        axes_d = [hp - sp
                                  for hp, sp in zip(homepos, startpos)]
                        move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
                        retract_r = min(1., hi.retract_dist / move_d)
                        retractpos = [hp - ad * retract_r
                                      for hp, ad in zip(homepos, axes_d)]
                        self.toolhead.move(retractpos, hi.retract_speed)
        finally:
            # Guarantee the motors are back to run_current (and any
            # homing-accel override is rolled back) even when the
            # homing move raised an error. Without this, an SL-homing
            # timeout would leave the driver stuck on home_current
            # until the user caught it in HMI.
            self._set_current_homing(homing_axes, pre_homing=False)
            self._set_homing_accel(hi.accel, pre_homing=False)
        # Signal home operation complete
        self.toolhead.flush_step_generation()
        self.trigger_mcu_pos = {sp.stepper_name: sp.trig_pos
                                for sp in hmove.stepper_positions}
        self.adjust_pos = {}
        self.printer.send_event("homing:home_rails_end", self, rails)
        if any(self.adjust_pos.values()):
            # Apply any homing offsets
            kin = self.toolhead.get_kinematics()
            homepos = self.toolhead.get_position()
            kin_spos = {s.get_name(): (s.get_commanded_position()
                                       + self.adjust_pos.get(s.get_name(), 0.))
                        for s in kin.get_steppers()}
            newpos = kin.calc_position(kin_spos)
            for axis in force_axes:
                if newpos[axis] is None:
                    raise self.printer.command_error(
                            "Cannot determine position of toolhead on "
                            "axis %s after homing" % "xyz"[axis])
                homepos[axis] = newpos[axis]
            self.toolhead.set_position(homepos)

class PrinterHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        # Register g-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('G28', self.cmd_G28)
    def manual_home(self, toolhead, endstops, pos, speed,
                    probe_pos, triggered, check_triggered):
        hmove = HomingMove(self.printer, endstops, toolhead)
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=probe_pos,
                                     triggered=triggered,
                                     check_triggered=check_triggered)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            raise
        return epos
    def probing_move(self, mcu_probe, pos, speed):
        endstops = [(mcu_probe, "probe")]
        hmove = HomingMove(self.printer, endstops)
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Probing failed due to printer shutdown")
            raise
        if hmove.check_no_movement() is not None:
            raise self.printer.command_error(
                "Probe triggered prior to movement")
        return epos
    def cmd_G28(self, gcmd):
        # Move to origin
        axes = []
        for pos, axis in enumerate('XYZ'):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            axes = [0, 1, 2]
        homing_state = Homing(self.printer)
        homing_state.set_axes(axes)
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        try:
            kin.home(homing_state)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            self.printer.lookup_object('stepper_enable').motor_off()
            raise

def load_config(config):
    return PrinterHoming(config)
