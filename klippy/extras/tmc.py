# Common helper code for TMC stepper drivers
#
# Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, collections, os, math
import stepper
from . import stepstick_defs

######################################################################
# Field helpers
######################################################################

# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1

class FieldHelper:
    def __init__(self, all_fields, signed_fields=[], field_formatters={},
                 registers=None):
        self.all_fields = all_fields
        self.signed_fields = {sf: 1 for sf in signed_fields}
        self.field_formatters = field_formatters
        self.registers = registers
        if self.registers is None:
            self.registers = collections.OrderedDict()
        self.field_to_register = { f: r for r, fields in self.all_fields.items()
                                   for f in fields }
    def lookup_register(self, field_name, default=None):
        return self.field_to_register.get(field_name, default)
    def get_field(self, field_name, reg_value=None, reg_name=None):
        # Returns value of the register field
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        field_value = (reg_value & mask) >> ffs(mask)
        if field_name in self.signed_fields and ((reg_value & mask)<<1) > mask:
            field_value -= (1 << field_value.bit_length())
        return field_value
    def set_field(self, field_name, field_value, reg_value=None, reg_name=None):
        # Returns register value with field bits filled with supplied value
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        new_value = (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
        self.registers[reg_name] = new_value
        return new_value
    def set_config_field(self, config, field_name, default):
        # Allow a field to be set from the config file
        config_name = "driver_" + field_name.upper()
        reg_name = self.field_to_register[field_name]
        mask = self.all_fields[reg_name][field_name]
        maxval = mask >> ffs(mask)
        if maxval == 1:
            val = config.getboolean(config_name, default)
        elif field_name in self.signed_fields:
            val = config.getint(config_name, default,
                                minval=-(maxval//2 + 1), maxval=maxval//2)
        else:
            val = config.getint(config_name, default, minval=0, maxval=maxval)
        if default is None and val is None:
            return
        return self.set_field(field_name, val)
    def pretty_format(self, reg_name, reg_value):
        # Provide a string description of a register
        reg_fields = self.all_fields.get(reg_name, {})
        reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
        fields = []
        for mask, field_name in reg_fields:
            field_value = self.get_field(field_name, reg_value, reg_name)
            sval = self.field_formatters.get(field_name, str)(field_value)
            if sval and sval != "0":
                fields.append(" %s=%s" % (field_name, sval))
        return "%-11s %08x%s" % (reg_name + ":", reg_value, "".join(fields))
    def get_reg_fields(self, reg_name, reg_value):
        # Provide fields found in a register
        reg_fields = self.all_fields.get(reg_name, {})
        return {field_name: self.get_field(field_name, reg_value, reg_name)
                for field_name, mask in reg_fields.items()}


######################################################################
# Periodic error checking
######################################################################

class TMCErrorCheck:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        name_parts = config.get_name().split()
        self.stepper_name = ' '.join(name_parts[1:])
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.check_timer = None
        self.last_drv_status = self.last_drv_fields = None
        # Setup for GSTAT query
        reg_name = self.fields.lookup_register("drv_err")
        if reg_name is not None:
            self.gstat_reg_info = [0, reg_name, 0xffffffff, 0xffffffff, 0]
        else:
            self.gstat_reg_info = None
        self.clear_gstat = True
        # Setup for DRV_STATUS query
        self.irun_field = "irun"
        reg_name = "DRV_STATUS"
        mask = err_mask = cs_actual_mask = 0
        if name_parts[0] == 'tmc2130':
            # TMC2130 driver quirks
            self.clear_gstat = False
            cs_actual_mask = self.fields.all_fields[reg_name]["cs_actual"]
        elif name_parts[0] == 'tmc2660':
            # TMC2660 driver quirks
            self.irun_field = "cs"
            reg_name = "READRSP@RDSEL2"
            cs_actual_mask = self.fields.all_fields[reg_name]["se"]
        err_fields = ["ot", "s2ga", "s2gb", "s2vsa", "s2vsb"]
        warn_fields = ["otpw", "t120", "t143", "t150", "t157"]
        for f in err_fields + warn_fields:
            if f in self.fields.all_fields[reg_name]:
                mask |= self.fields.all_fields[reg_name][f]
                if f in err_fields:
                    err_mask |= self.fields.all_fields[reg_name][f]
        self.drv_status_reg_info = [0, reg_name, mask, err_mask, cs_actual_mask]
        # Setup for temperature query
        self.adc_temp = None
        self.adc_temp_reg = self.fields.lookup_register("adc_temp")
        if self.adc_temp_reg is not None:
            pheaters = self.printer.load_object(config, 'heaters')
            pheaters.register_monitor(config)
    def _query_register(self, reg_info, try_clear=False):
        last_value, reg_name, mask, err_mask, cs_actual_mask = reg_info
        cleared_flags = 0
        count = 0
        while 1:
            try:
                val = self.mcu_tmc.get_register(reg_name)
            except self.printer.command_error as e:
                count += 1
                if count < 3 and str(e).startswith("Unable to read tmc uart"):
                    # Allow more retries on a TMC UART read error
                    reactor = self.printer.get_reactor()
                    reactor.pause(reactor.monotonic() + 0.050)
                    continue
                raise
            if val & mask != last_value & mask:
                fmt = self.fields.pretty_format(reg_name, val)
                logging.info("TMC '%s' reports %s", self.stepper_name, fmt)
            reg_info[0] = last_value = val
            if not val & err_mask:
                if not cs_actual_mask or val & cs_actual_mask:
                    break
                irun = self.fields.get_field(self.irun_field)
                if self.check_timer is None or irun < 4:
                    break
                if (self.irun_field == "irun"
                    and not self.fields.get_field("ihold")):
                    break
                # CS_ACTUAL field of zero - indicates a driver reset
            count += 1
            if count >= 3:
                fmt = self.fields.pretty_format(reg_name, val)
                raise self.printer.command_error("TMC '%s' reports error: %s"
                                                 % (self.stepper_name, fmt))
            if try_clear and val & err_mask:
                try_clear = False
                cleared_flags |= val & err_mask
                self.mcu_tmc.set_register(reg_name, val & err_mask)
        return cleared_flags
    def _query_temperature(self):
        try:
            self.adc_temp = self.mcu_tmc.get_register(self.adc_temp_reg)
        except self.printer.command_error as e:
            # Ignore comms error for temperature
            self.adc_temp = None
            return
    def _do_periodic_check(self, eventtime):
        try:
            self._query_register(self.drv_status_reg_info)
            if self.gstat_reg_info is not None:
                self._query_register(self.gstat_reg_info)
            if self.adc_temp_reg is not None:
                self._query_temperature()
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
            return self.printer.get_reactor().NEVER
        return eventtime + 1.
    def stop_checks(self):
        if self.check_timer is None:
            return
        self.printer.get_reactor().unregister_timer(self.check_timer)
        self.check_timer = None
    def start_checks(self):
        if self.check_timer is not None:
            self.stop_checks()
        cleared_flags = 0
        self._query_register(self.drv_status_reg_info)
        if self.gstat_reg_info is not None:
            cleared_flags = self._query_register(self.gstat_reg_info,
                                                 try_clear=self.clear_gstat)
        reactor = self.printer.get_reactor()
        curtime = reactor.monotonic()
        self.check_timer = reactor.register_timer(self._do_periodic_check,
                                                  curtime + 1.)
        if cleared_flags:
            reset_mask = self.fields.all_fields["GSTAT"]["reset"]
            if cleared_flags & reset_mask:
                return True
        return False
    def get_status(self, eventtime=None):
        if self.check_timer is None:
            return {'drv_status': None, 'temperature': None}
        temp = None
        if self.adc_temp is not None:
            temp = round((self.adc_temp - 2038) / 7.7, 2)
        last_value, reg_name = self.drv_status_reg_info[:2]
        if last_value != self.last_drv_status:
            self.last_drv_status = last_value
            fields = self.fields.get_reg_fields(reg_name, last_value)
            self.last_drv_fields = {n: v for n, v in fields.items() if v}
        return {'drv_status': self.last_drv_fields, 'temperature': temp}


######################################################################
# G-Code command helpers
######################################################################

class TMCCommandHelper:
    def __init__(self, config, mcu_tmc, current_helper):
        self.printer = config.get_printer()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.current_helper = current_helper
        self.echeck_helper = TMCErrorCheck(config, mcu_tmc)
        self.fields = mcu_tmc.get_fields()
        self.read_registers = self.read_translate = None
        self.toff = None
        self.mcu_phase_offset = None
        self.stepper = None
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.printer.register_event_handler("stepper:sync_mcu_position",
                                            self._handle_sync_mcu_pos)
        self.printer.register_event_handler("stepper:set_sdir_inverted",
                                            self._handle_sync_mcu_pos)
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        # Set microstep config options
        TMCMicrostepHelper(config, mcu_tmc)
        # Register commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("SET_TMC_FIELD", "STEPPER", self.name,
                                   self.cmd_SET_TMC_FIELD,
                                   desc=self.cmd_SET_TMC_FIELD_help)
        gcode.register_mux_command("INIT_TMC", "STEPPER", self.name,
                                   self.cmd_INIT_TMC,
                                   desc=self.cmd_INIT_TMC_help)
        gcode.register_mux_command("SET_TMC_CURRENT", "STEPPER", self.name,
                                   self.cmd_SET_TMC_CURRENT,
                                   desc=self.cmd_SET_TMC_CURRENT_help)
    def _init_registers(self, print_time=None):
        # Send registers
        for reg_name in list(self.fields.registers.keys()):
            val = self.fields.registers[reg_name] # Val may change during loop
            self.mcu_tmc.set_register(reg_name, val, print_time)
    cmd_INIT_TMC_help = "Initialize TMC stepper driver registers"
    def cmd_INIT_TMC(self, gcmd):
        logging.info("INIT_TMC %s", self.name)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self._init_registers(print_time)
    cmd_SET_TMC_FIELD_help = "Set a register field of a TMC driver"
    def cmd_SET_TMC_FIELD(self, gcmd):
        field_name = gcmd.get('FIELD').lower()
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise gcmd.error("Unknown field name '%s'" % (field_name,))
        value = gcmd.get_int('VALUE', None)
        velocity = gcmd.get_float('VELOCITY', None, minval=0.)
        if (value is None) == (velocity is None):
            raise gcmd.error("Specify either VALUE or VELOCITY")
        if velocity is not None:
            if self.mcu_tmc.get_tmc_frequency() is None:
                raise gcmd.error(
                    "VELOCITY parameter not supported by this driver")
            value = TMCtstepHelper(self.mcu_tmc, velocity,
                                   pstepper=self.stepper)
        reg_val = self.fields.set_field(field_name, value)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self.mcu_tmc.set_register(reg_name, reg_val, print_time)
    cmd_SET_TMC_CURRENT_help = "Set the current of a TMC driver"
    def cmd_SET_TMC_CURRENT(self, gcmd):
        ch = self.current_helper
        (
            run_current,
            hold_current,
            req_hold_current,
            home_current,
        ) = ch.get_current()
        max_current = ch.get_max_current()
        cmd_run_current = gcmd.get_float(
            "CURRENT", None, minval=0.0, maxval=max_current
        )
        cmd_hold_current = gcmd.get_float(
            "HOLDCURRENT", None, above=0.0, maxval=max_current
        )
        cmd_home_current = gcmd.get_float(
            "HOMECURRENT", None, above=0.0, maxval=max_current
        )
        if cmd_run_current:
            run_current = cmd_run_current
            ch.set_run_current(run_current)
        if cmd_hold_current:
            hold_current = cmd_hold_current
            ch.set_hold_current(hold_current)
        if cmd_home_current:
            home_current = cmd_home_current
            ch.set_home_current(home_current)

        toolhead = self.printer.lookup_object("toolhead")
        print_time = toolhead.get_last_move_time()
        # Re-apply whichever profile is currently active with force=True
        # so SET_TMC_CURRENT always takes effect immediately.
        if ch._homing_active:
            ch.actual_current = ch.req_home_current
            ch.apply_current(print_time)
            ch.tune_driver(ch.req_home_current, force=True)
        else:
            ch.actual_current = ch.req_run_current
            ch.apply_current(print_time)
            ch.tune_driver(ch.req_run_current, force=True)
        # Report values
        if hold_current is None:
            gcmd.respond_info(
                "Run Current: %0.2fA Home Current: %0.2fA"
                % (run_current, home_current)
            )
        else:
            gcmd.respond_info(
                "Run Current: %0.2fA Hold Current: %0.2fA Home Current: %0.2fA"
                % (run_current, hold_current, home_current)
            )
    # Stepper phase tracking
    def _get_phases(self):
        return (256 >> self.fields.get_field("mres")) * 4
    def get_phase_offset(self):
        return self.mcu_phase_offset, self._get_phases()
    def _query_phase(self):
        field_name = "mscnt"
        if self.fields.lookup_register(field_name, None) is None:
            # TMC2660 uses MSTEP
            field_name = "mstep"
        reg = self.mcu_tmc.get_register(self.fields.lookup_register(field_name))
        return self.fields.get_field(field_name, reg)
    def _handle_sync_mcu_pos(self, stepper):
        if stepper.get_name() != self.stepper_name:
            return
        try:
            driver_phase = self._query_phase()
        except self.printer.command_error as e:
            logging.info("Unable to obtain tmc %s phase", self.stepper_name)
            self.mcu_phase_offset = None
            enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
            if enable_line.is_motor_enabled():
                raise
            return
        if not stepper.get_dir_inverted()[0]:
            driver_phase = 1023 - driver_phase
        phases = self._get_phases()
        phase = int(float(driver_phase) / 1024 * phases + .5) % phases
        moff = (phase - stepper.get_mcu_position()) % phases
        if self.mcu_phase_offset is not None and self.mcu_phase_offset != moff:
            logging.warning("Stepper %s phase change (was %d now %d)",
                            self.stepper_name, self.mcu_phase_offset, moff)
        self.mcu_phase_offset = moff
    # Stepper enable/disable tracking
    def _do_enable(self, print_time):
        try:
            if self.toff is not None:
                # Shared enable via comms handling
                self.fields.set_field("toff", self.toff)
            self._init_registers()
            did_reset = self.echeck_helper.start_checks()
            if did_reset:
                self.mcu_phase_offset = None
            # Calculate phase offset
            if self.mcu_phase_offset is not None:
                return
            gcode = self.printer.lookup_object("gcode")
            with gcode.get_mutex():
                if self.mcu_phase_offset is not None:
                    return
                logging.info("Pausing toolhead to calculate %s phase offset",
                             self.stepper_name)
                self.printer.lookup_object('toolhead').wait_moves()
                self._handle_sync_mcu_pos(self.stepper)
            logging.info(f"Tuning TMC Driver for stepper: {self.stepper}")
            self.current_helper.tune_driver(force=True)
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
    def _do_disable(self, print_time):
        try:
            if self.toff is not None:
                val = self.fields.set_field("toff", 0)
                reg_name = self.fields.lookup_register("toff")
                self.mcu_tmc.set_register(reg_name, val, print_time)
            self.echeck_helper.stop_checks()
        except self.printer.command_error as e:
            self.printer.invoke_shutdown(str(e))
    def _handle_mcu_identify(self):
        # Lookup stepper object
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        self.stepper.set_tmc_current_helper(self.current_helper)
        # Note pulse duration and step_both_edge optimizations available
        self.stepper.setup_default_pulse_duration(.000000100, True)
    def _handle_stepper_enable(self, print_time, is_enable):
        if is_enable:
            cb = (lambda ev: self._do_enable(print_time))
        else:
            cb = (lambda ev: self._do_disable(print_time))
        self.printer.get_reactor().register_callback(cb)
    def _handle_connect(self):
        # Check if using step on both edges optimization
        pulse_duration, step_both_edge = self.stepper.get_pulse_duration()
        if step_both_edge:
            self.fields.set_field("dedge", 1)
        # Check for soft stepper enable/disable
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        enable_line.register_state_callback(self._handle_stepper_enable)
        if not enable_line.has_dedicated_enable():
            self.toff = self.fields.get_field("toff")
            self.fields.set_field("toff", 0)
            logging.info("Enabling TMC virtual enable for '%s'",
                         self.stepper_name)
        # Send init
        try:
            if self.mcu_tmc.mcu.non_critical_disconnected:
                logging.info(
                    "TMC %s failed to init - non_critical_mcu: %s is disconnected!",
                    self.name,
                    self.mcu_tmc.mcu.get_name(),
                )
            else:
                self._init_registers()
        except self.printer.command_error as e:
            logging.info("TMC %s failed to init: %s", self.name, str(e))
    # get_status information export
    def get_status(self, eventtime=None):
        cpos = None
        if self.stepper is not None and self.mcu_phase_offset is not None:
            cpos = self.stepper.mcu_to_commanded_position(self.mcu_phase_offset)
        current = self.current_helper.get_current()
        res = {'mcu_phase_offset': self.mcu_phase_offset,
               'phase_offset_position': cpos,
               'run_current': current[0],
               'hold_current': current[1]}
        res.update(self.echeck_helper.get_status(eventtime))
        return res
    # DUMP_TMC support
    def setup_register_dump(self, read_registers, read_translate=None):
        self.read_registers = read_registers
        self.read_translate = read_translate
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command("DUMP_TMC", "STEPPER", self.name,
                                   self.cmd_DUMP_TMC,
                                   desc=self.cmd_DUMP_TMC_help)
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, gcmd):
        logging.info("DUMP_TMC %s", self.name)
        reg_name = gcmd.get('REGISTER', None)
        if reg_name is not None:
            reg_name = reg_name.upper()
            val = self.fields.registers.get(reg_name)
            if (val is not None) and (reg_name not in self.read_registers):
                # write-only register
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            elif reg_name in self.read_registers:
                # readable register
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            else:
                raise gcmd.error("Unknown register name '%s'" % (reg_name))
        else:
            gcmd.respond_info("========== Write-only registers ==========")
            for reg_name, val in self.fields.registers.items():
                if reg_name not in self.read_registers:
                    gcmd.respond_info(self.fields.pretty_format(reg_name, val))
            gcmd.respond_info("========== Queried registers ==========")
            for reg_name in self.read_registers:
                val = self.mcu_tmc.get_register(reg_name)
                if self.read_translate is not None:
                    reg_name, val = self.read_translate(reg_name, val)
                gcmd.respond_info(self.fields.pretty_format(reg_name, val))


######################################################################
# TMC virtual pins
######################################################################

# Helper class for "sensorless homing"
class TMCVirtualPinHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        if self.fields.lookup_register('diag0_stall') is not None:
            if config.get('diag0_pin', None) is not None:
                self.diag_pin = config.get('diag0_pin')
                self.diag_pin_field = 'diag0_stall'
            else:
                self.diag_pin = config.get('diag1_pin', None)
                self.diag_pin_field = 'diag1_stall'
        else:
            self.diag_pin = config.get('diag_pin', None)
            self.diag_pin_field = None
        self.mcu_endstop = None
        self.en_pwm = False
        self.pwmthrs = self.coolthrs = self.thigh = 0
        # Reference to current_helper – set via set_current_helper()
        # ASSUMPTION: set_current_helper() is called from TMC5160.__init__
        self._current_helper = None
        # Register virtual_endstop pin
        name_parts = config.get_name().split()
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)

    def set_current_helper(self, current_helper):
        """Kept for API compatibility; snapshot/restore is now handled via
        homing_start/end events registered in BaseTMCCurrentHelper."""
        self._current_helper = current_helper

    def setup_pin(self, pin_type, pin_params):
        # Validate pin
        ppins = self.printer.lookup_object('pins')
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise ppins.error("tmc virtual endstop only useful as endstop")
        if pin_params['invert'] or pin_params['pullup']:
            raise ppins.error("Can not pullup/invert tmc virtual pin")
        if self.diag_pin is None:
            raise ppins.error("tmc virtual endstop requires diag pin config")
        # Setup for sensorless homing
        self.printer.register_event_handler("homing:homing_move_begin",
                                            self.handle_homing_move_begin)
        self.printer.register_event_handler("homing:homing_move_end",
                                            self.handle_homing_move_end)
        self.mcu_endstop = ppins.setup_pin('endstop', self.diag_pin)
        return self.mcu_endstop

    def handle_homing_move_begin(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # Here we only manage the StallGuard-specific GCONF/TCOOLTHRS/THIGH
        # registers that must be set for each individual homing move.
        logging.info("tmc VirtualPinHelper: homing_move_begin")
        # Enable/disable stealthchop
        self.pwmthrs = self.fields.get_field("tpwmthrs")
        reg = self.fields.lookup_register("en_pwm_mode", None)
        if reg is None:
            # On "stallguard4" drivers, "stealthchop" must be enabled
            self.en_pwm = not self.fields.get_field("en_spreadcycle")
            tp_val = self.fields.set_field("tpwmthrs", 0)
            self.mcu_tmc.set_register("TPWMTHRS", tp_val)
            val = self.fields.set_field("en_spreadcycle", 0)
        else:
            # On earlier drivers, "stealthchop" must be disabled
            self.en_pwm = self.fields.get_field("en_pwm_mode")
            self.fields.set_field("en_pwm_mode", 0)
            val = self.fields.set_field(self.diag_pin_field, 1)
        self.mcu_tmc.set_register("GCONF", val)
        # Enable tcoolthrs (if not already)
        self.coolthrs = self.fields.get_field("tcoolthrs")
        if self.coolthrs == 0:
            tc_val = self.fields.set_field("tcoolthrs", 0xfffff)
            self.mcu_tmc.set_register("TCOOLTHRS", tc_val)
        # Disable thigh
        reg = self.fields.lookup_register("thigh", None)
        if reg is not None:
            self.thigh = self.fields.get_field("thigh")
            th_val = self.fields.set_field("thigh", 0)
            self.mcu_tmc.set_register(reg, th_val)

    def handle_homing_move_end(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # Here we only restore the StallGuard-specific registers.
        logging.info("tmc VirtualPinHelper: homing_move_end")
        # Restore stealthchop/spreadcycle
        reg = self.fields.lookup_register("en_pwm_mode", None)
        if reg is None:
            tp_val = self.fields.set_field("tpwmthrs", self.pwmthrs)
            self.mcu_tmc.set_register("TPWMTHRS", tp_val)
            val = self.fields.set_field("en_spreadcycle", not self.en_pwm)
        else:
            self.fields.set_field("en_pwm_mode", self.en_pwm)
            val = self.fields.set_field(self.diag_pin_field, 0)
        self.mcu_tmc.set_register("GCONF", val)
        # Restore tcoolthrs
        tc_val = self.fields.set_field("tcoolthrs", self.coolthrs)
        self.mcu_tmc.set_register("TCOOLTHRS", tc_val)
        # Restore thigh
        reg = self.fields.lookup_register("thigh", None)
        if reg is not None:
            th_val = self.fields.set_field("thigh", self.thigh)
            self.mcu_tmc.set_register(reg, th_val)


######################################################################
# Config reading helpers
######################################################################

# Helper to initialize the wave table from config or defaults
def TMCWaveTableHelper(config, mcu_tmc):
    set_config_field = mcu_tmc.get_fields().set_config_field
    set_config_field(config, "mslut0", 0xAAAAB554)
    set_config_field(config, "mslut1", 0x4A9554AA)
    set_config_field(config, "mslut2", 0x24492929)
    set_config_field(config, "mslut3", 0x10104222)
    set_config_field(config, "mslut4", 0xFBFFFFFF)
    set_config_field(config, "mslut5", 0xB5BB777D)
    set_config_field(config, "mslut6", 0x49295556)
    set_config_field(config, "mslut7", 0x00404222)
    set_config_field(config, "w0", 2)
    set_config_field(config, "w1", 1)
    set_config_field(config, "w2", 1)
    set_config_field(config, "w3", 1)
    set_config_field(config, "x1", 128)
    set_config_field(config, "x2", 255)
    set_config_field(config, "x3", 255)
    set_config_field(config, "start_sin", 0)
    set_config_field(config, "start_sin90", 247)

# Helper to configure the microstep settings
def TMCMicrostepHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    stepper_name = " ".join(config.get_name().split()[1:])
    if not config.has_section(stepper_name):
        raise config.error(
            "Could not find config section '[%s]' required by tmc driver"
            % (stepper_name,))
    sconfig = config.getsection(stepper_name)
    steps = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
    mres = sconfig.getchoice('microsteps', steps)
    fields.set_field("mres", mres)
    fields.set_field("intpol", config.getboolean("interpolate", True))

# Helper for calculating TSTEP based values from velocity
def TMCtstepHelper(mcu_tmc, velocity, pstepper=None, config=None):
    if velocity <= 0.:
        return 0xfffff
    if pstepper is not None:
        step_dist = pstepper.get_step_dist()
    else:
        stepper_name = " ".join(config.get_name().split()[1:])
        sconfig = config.getsection(stepper_name)
        rotation_dist, steps_per_rotation = stepper.parse_step_distance(sconfig)
        step_dist = rotation_dist / steps_per_rotation
    mres = mcu_tmc.get_fields().get_field("mres")
    step_dist_256 = step_dist / (1 << mres)
    tmc_freq = mcu_tmc.get_tmc_frequency()
    threshold = int(tmc_freq * step_dist_256 / velocity + .5)
    return max(0, min(0xfffff, threshold))

# Helper to configure stealthChop-spreadCycle transition velocity
def TMCStealthchopHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    en_pwm_mode = False
    velocity = config.getfloat('stealthchop_threshold', None, minval=0.)
    tpwmthrs = 0xfffff

    if velocity is not None:
        en_pwm_mode = True
        tpwmthrs = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("tpwmthrs", tpwmthrs)

    reg = fields.lookup_register("en_pwm_mode", None)
    if reg is not None:
        fields.set_field("en_pwm_mode", en_pwm_mode)
    else:
        # TMC2208 uses en_spreadCycle
        fields.set_field("en_spreadcycle", not en_pwm_mode)


PWM_FREQ_TARGETS = {"tmc2130": 55e3,
                    "tmc2208": 55e3,
                    "tmc2209": 55e3,
                    "tmc2240": 20e3, # 2240s run very hot at high frequencies
                    "tmc2660": 55e3,
                    "tmc5160": 55e3}

######################################################################
# Homing Profile Definitions
######################################################################

# Low-noise / robust defaults for sensorless homing (StallGuard).
# These are intentionally conservative – SpreadCycle only, CoolStep OFF,
# THIGH disabled (forced via VirtualPinHelper), StallGuard filter ON.
# All values match TMC5160 field ranges as defined in tmc5160.py.
HOMING_DEFAULTS = {
    # CHOPCONF – SpreadCycle, conservative low-noise settings
    "toff":        3,    # moderate slow-decay time
    "tbl":         1,    # 24 clock blanking – good balance
    "hstrt":       0,    # minimal start hysteresis → quietest
    "hend":        4,    # hend=4 (encoded) → effective +1, slightly positive
    "tpfd":        4,    # passive fast-decay to reduce resonances
    "chm":         0,    # SpreadCycle (not constant-off-time)
    "vhighfs":     0,    # no full-step at high speed
    "vhighchm":    0,
    # COOLCONF – CoolStep OFF, sfilt OFF (user can enable via homing_sfilt)
    "semin":       0,    # CoolStep disabled
    "semax":       0,
    "seup":        0,
    "sedn":        0,
    "seimin":      0,
    "sfilt":       0,    # SG filter off by default; set homing_sfilt=1 to enable
    # IHOLD_IRUN
    "iholddelay":  8,    # slower hold current decay = quieter standstill
    # Current scaler default for homing (independent of driver_CS / RUN profile)
    "cs":          8,    # low CS → GlobalScaler does the heavy lifting; low noise
}

# Registers that hold homing-relevant chopper / SG settings.
# These are saved before homing and restored afterwards (RUN profile).


######################################################################
# Base TMC current / tuning helper
######################################################################

class BaseTMCCurrentHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.config_file = self.printer.lookup_object("configfile")

        self.current_directory = os.path.dirname(os.path.realpath(__file__))
        self.motor_db_file = os.path.join(self.current_directory,
                                          'motor_database.cfg')
        try:
            self.motor_db = self.config_file.read_config(self.motor_db_file)
        except Exception:
            raise config.error(
                "Cannot load config '%s'" % (self.motor_db_file,))
        for motor_section in self.motor_db.get_prefix_sections(''):
            self.printer.load_object(self.motor_db, motor_section.get_name())

        self.driver_name = config.get_name()
        self.driver_type = self.driver_name.split()[0]
        self.name = self.driver_name.split()[-1]
        self.motor = config.get('motor', None)
        if self.motor is not None:
            self.motor_name = "motor_constants " + self.motor

        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()

        stepper_driver_type = config.get("stepstick_type", None)
        sense_resistor_from_driver, step_driver_max_current = (
            stepstick_defs.STEPSTICK_DEFS.get(stepper_driver_type, (None, None))
        )

        override_sense_resistor = config.getfloat(
            "sense_resistor", None, minval=0.0,
        )
        if (override_sense_resistor is None
                and sense_resistor_from_driver is None):
            self.sense_resistor = self.DEFAULT_SENSE_RESISTOR
            self.config_file.warn(
                "config",
                f"""Neither 'stepper_driver_type' or 'sense_resistor' is """
                f"""defined for [{self.name}]. Using default value of """
                f"""{self.sense_resistor} ohm sense resistor. """
                f"""If this is incorrect, your drivers, steppers or board """
                f"""may be damaged.""",
                "sense_resistor",
            )
        else:
            self.sense_resistor = (
                override_sense_resistor or sense_resistor_from_driver
            )

        max_current = step_driver_max_current or self.DEFAULT_MAX_CURRENT

        # --- Run / Hold / Home current ---
        self.config_run_current = config.getfloat(
            "run_current", above=0.0, maxval=max_current
        )
        self.config_hold_current = config.getfloat(
            "hold_current", max_current, above=0.0, maxval=max_current
        )
        self.config_home_current = config.getfloat(
            "home_current",
            self.config_run_current,
            above=0.0,
            maxval=max_current,
        )
        self.current_change_dwell_time = config.getfloat(
            "current_change_dwell_time", 0.5, above=0.0
        )

        self.pwm_freq_target = config.getfloat(
            'pwm_freq_target',
            default=PWM_FREQ_TARGETS[self.driver_type],
            minval=10e3, maxval=100e3)

        self.chopper_freq_target = config.getfloat(
            'chopper_freq_target', default=None, minval=10e3, maxval=100e3)

        self.voltage = config.getfloat(
            'voltage', default=None, minval=0.0, maxval=60.0)

        self.extra_hysteresis = config.getint(
            'extra_hysteresis', default=0, minval=0, maxval=15)

        # --- Fixed driver parameters (user cfg override, driver_XXXX) ---
        # Backward compat: driver_TBL / driver_tbl both accepted
        self.tbl  = config.getint('driver_TBL',  default=None, minval=0, maxval=3)
        self.toff = config.getint('driver_TOFF',  default=None, minval=1, maxval=15)
        self.tpfd = config.getint('driver_TPFD',  default=None, minval=0, maxval=15)
        # driver_CS / driver_cs – both uppercase and lowercase accepted;
        # uppercase takes priority for backward compat.
        self.cs   = config.getint('driver_CS',    default=None, minval=0, maxval=31)
        if self.cs is None:
            # Fallback to lowercase variant (new, explicit opt-in)
            self.cs = config.getint('driver_cs',  default=0,    minval=0, maxval=31)

        self.hstrt = config.getint('driver_HSTRT', default=None, minval=0, maxval=7)
        self.hend  = config.getint('driver_HEND',  default=None, minval=0, maxval=15)

        self.sg4_thrs = config.getint(
            'driver_SGTHRS', default=None, minval=0, maxval=255)
        self.sgt = config.getint(
            'driver_SGT', default=None, minval=-64, maxval=63)

        self.overvoltage_vth = config.getfloat(
            'overvoltage_vth', default=None, minval=0.0, maxval=60.0)

        # ----------------------------------------------------------------
        # Homing-profile overrides (optional, homing_* keys)
        # Convention over configuration: if not set → HOMING_DEFAULTS
        # ----------------------------------------------------------------
        self.homing_toff  = config.getint(
            'homing_toff',  default=None, minval=1, maxval=15)
        self.homing_tbl   = config.getint(
            'homing_tbl',   default=None, minval=0, maxval=3)
        self.homing_hstrt = config.getint(
            'homing_hstrt', default=None, minval=0, maxval=7)
        self.homing_hend  = config.getint(
            'homing_hend',  default=None, minval=0, maxval=15)
        self.homing_tpfd  = config.getint(
            'homing_tpfd',  default=None, minval=0, maxval=15)
        self.homing_sfilt = config.getint(
            'homing_sfilt', default=None, minval=0, maxval=1)
        # homing_cs: optional fixed CS for homing profile.
        # Completely independent from driver_CS (RUN profile).
        # If not set → low-noise CS search is performed in _calc_homing_current().
        self.homing_cs = config.getint(
            'homing_cs', default=None, minval=0, maxval=31)

        if (self.sense_resistor is not None
                and self.motor is not None
                and self.voltage is not None):
            self.driver_tuning = True
            logging.info(f"tmc {self.name} ::: AutoTuning activated!")
        else:
            self.driver_tuning = None
            logging.info(f"tmc {self.name} ::: AutoTuning not active!")

        # Actual / requested currents
        self.req_run_current  = self.config_run_current
        self.req_hold_current = self.config_hold_current
        self.req_home_current = self.config_home_current

        # Register for G28-level events so snapshot/restore spans the
        # entire homing operation (all axes, all moves).
        printer = config.get_printer()
        printer.register_event_handler(
            "homing:homing_start", self._handle_homing_start)
        printer.register_event_handler(
            "homing:homing_end", self._handle_homing_end)
        printer.register_event_handler(
            "homing:home_rails_end", self._handle_home_rails_end)

        self.actual_current = self.req_run_current
        self.max_current    = max_current

        # Last current value for which tune_driver() completed a full run.
        # tune_driver() skips if new_current matches, unless force=True.
        self._last_tuned_current = None
        # True while HOMING profile is active
        self._homing_active = False
        # Axes requested in the current G28 cycle (set of ints: 0=X, 1=Y, 2=Z)
        self._homing_axes    = set()
        # Axes that have already completed homing in this cycle
        self._homed_axes     = set()


    # ------------------------------------------------------------------
    # Homing profile switching
    # ------------------------------------------------------------------

    def _load_homing_profile(self):
        """Switch to HOMING current + profile. No-op if already active.

        Uses print_time=None (immediate/synchronous) so that all homing
        registers are active on the MCU before the first homing move begins,
        with no MCU-queue wait per register.
        """
        if self._homing_active:
            return
        logging.info("tmc %s ::: loading HOMING profile", self.name)
        self._homing_active = True
        self._last_tuned_current = None
        self.actual_current = self.req_home_current
        self.apply_current(None)          # immediate – no queue lag
        self.tune_driver(self.req_home_current, immediate=True)

    def _load_run_profile(self):
        """Switch to RUN current + profile. No-op if already active.

        Uses print_time=None (immediate/synchronous) so that all run-profile
        registers are guaranteed to be written to the MCU before the next
        move (e.g. move to bed-centre for Z homing) is planned/executed.
        """
        if not self._homing_active:
            return
        logging.info("tmc %s ::: loading RUN profile", self.name)
        self._homing_active = False
        self._last_tuned_current = None
        self.actual_current = self.req_run_current
        self.apply_current(None)          # immediate – no queue lag
        self.tune_driver(self.req_run_current, immediate=True)

    def _handle_homing_start(self, homing_state):
        """G28 started: build axes state, then conditionally load HOMING profile.

        Rules:
          - Record which axes are being homed this cycle.
          - Reset the 'already homed' tracker for this cycle.
          - Only switch to HOMING profile when X (0) or Y (1) is among the
            requested axes.  A pure Z-only home (e.g. G28 Z) never needs
            the homing profile.
        """
        axes = set(homing_state.get_axes())
        self._homing_axes = axes
        self._homed_axes  = set()
        logging.info("tmc %s ::: homing_start axes=%s", self.name, axes)

        xy_axes = {0, 1}
        if axes & xy_axes:
            # X or Y (or both, or XYZ) → activate HOMING profile
            self._load_homing_profile()
        else:
            # Pure Z home → stay on RUN profile
            logging.info(
                "tmc %s ::: homing_start – Z-only, skipping HOMING profile",
                self.name)

    def _handle_homing_end(self, homing_state):
        """G28 finished (always via finally): restore RUN profile if still active.

        Covers the case where only X or only Y was homed (no Z follows to
        trigger an early switch-back).  No-op if already on RUN profile.
        """
        logging.info("tmc %s ::: homing_end homed_axes=%s",
                     self.name, self._homed_axes)
        self._load_run_profile()
        # Reset cycle state
        self._homing_axes = set()
        self._homed_axes  = set()

    def _handle_home_rails_end(self, homing_state, rails):
        """After each individual rail homing: update state and switch to RUN
        profile as soon as both X and Y have been homed.

        Logic:
          1. Determine which axes just finished homing from the rail list.
          2. Add them to _homed_axes.
          3. If X and Y were both requested AND both are now in _homed_axes
             → switch to RUN profile immediately, before any subsequent move
             (e.g. move to bed-centre for Z homing) is planned.
          4. If only X or only Y was requested (single-axis home) that axis
             finishing is the last XY axis → same switch.
        """
        # Axis index for each stepper name
        _axis_map = {
            "stepper_x":  0, "stepper_x1": 0,
            "stepper_y":  1, "stepper_y1": 1,
            "stepper_z":  2, "stepper_z1": 2, "stepper_z2": 2, "stepper_z3": 2,
        }
        finished = set()
        for rail in rails:
            for s in rail.get_steppers():
                axis = _axis_map.get(s.get_name())
                if axis is not None:
                    finished.add(axis)

        self._homed_axes |= finished
        logging.info(
            "tmc %s ::: home_rails_end finished=%s homed_so_far=%s "
            "homing_axes=%s",
            self.name, finished, self._homed_axes, self._homing_axes)

        # Determine which XY axes were requested in this cycle
        requested_xy = self._homing_axes & {0, 1}
        if not requested_xy:
            # Pure Z cycle – nothing to do
            return

        # Switch back to RUN as soon as all requested XY axes are done
        if requested_xy <= self._homed_axes:
            logging.info(
                "tmc %s ::: all requested XY axes homed – loading RUN profile",
                self.name)
            self._load_run_profile()

    # ------------------------------------------------------------------
    # tune_driver – see full implementation in the subclass section below
    # ------------------------------------------------------------------

    # ------------------------------------------------------------------
    # RUN profile
    # ------------------------------------------------------------------

    def _apply_run_profile(self, new_current=None, flush=True, immediate=False):
        """Apply the performance-tuned RUN profile (existing TuneDriver logic)."""
        if new_current is None:
            new_current = self.actual_current or self.config_run_current

        logging.info("tmc %s ::: _apply_run_profile current=%.3fA",
                     self.name, new_current)

        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.name)

        self._get_tmc_clock_frequency()
        self._configure_pwm(new_current)
        new_tbl, new_toff = self._configure_spreadcycle(new_current)
        self._configure_hysteresis(new_current, new_tbl, new_toff)
        self._configure_stallguard(new_current)
        self._configure_coolstep()
        self._configure_overvoltage()
        self._configure_highspeed(new_current)

        if flush:
            tuned_regs = ["CHOPCONF", "COOLCONF", "PWMCONF",
                          "IHOLD_IRUN", "TPWMTHRS", "TCOOLTHRS", "THIGH",
                          "GCONF", "GLOBALSCALER"]
            self._flush_registers(tuned_regs, immediate=immediate)

    # ------------------------------------------------------------------
    # HOMING profile
    # ------------------------------------------------------------------

    def _apply_homing_profile(self, homing_current=None, flush=True, immediate=False):
        """Apply low-noise HOMING profile with conservative chopper settings.

        Priority of values (high → low):
          homing_* cfg keys > global driver_* cfg keys > HOMING_DEFAULTS
        CoolStep is explicitly disabled.
        THIGH is already forced to 0 by VirtualPinHelper.
        """
        if homing_current is None:
            homing_current = self.req_home_current

        logging.info(
            "tmc %s ::: _apply_homing_profile current=%.3fA",
            self.name, homing_current)

        # --- Resolve CHOPCONF fields ---
        # Priority: homing_* cfg  →  HOMING_DEFAULTS
        # driver_* (RUN) values are intentionally NOT used as fallback.
        # toff
        if self.homing_toff is not None:
            toff = self.homing_toff
            logging.info("tmc %s ::: HOMING toff=%d (from homing_toff cfg)",
                         self.name, toff)
        else:
            toff = HOMING_DEFAULTS["toff"]
            logging.info("tmc %s ::: HOMING toff=%d (from HOMING_DEFAULTS)",
                         self.name, toff)

        # tbl
        if self.homing_tbl is not None:
            tbl = self.homing_tbl
            logging.info("tmc %s ::: HOMING tbl=%d (from homing_tbl cfg)",
                         self.name, tbl)
        else:
            tbl = HOMING_DEFAULTS["tbl"]
            logging.info("tmc %s ::: HOMING tbl=%d (from HOMING_DEFAULTS)",
                         self.name, tbl)

        # Datasheet constraint: TOFF=1 → TBL >= 2
        if toff == 1 and tbl < 2:
            logging.warning(
                "tmc %s ::: HOMING constraint: toff==1 requires tbl>=2, "
                "clamping tbl to 2", self.name)
            tbl = 2

        # hstrt
        if self.homing_hstrt is not None:
            hstrt = self.homing_hstrt
            logging.info("tmc %s ::: HOMING hstrt=%d (from homing_hstrt cfg)",
                         self.name, hstrt)
        else:
            hstrt = HOMING_DEFAULTS["hstrt"]
            logging.info("tmc %s ::: HOMING hstrt=%d (from HOMING_DEFAULTS)",
                         self.name, hstrt)

        # hend
        if self.homing_hend is not None:
            hend = self.homing_hend
            logging.info("tmc %s ::: HOMING hend=%d (from homing_hend cfg)",
                         self.name, hend)
        else:
            hend = HOMING_DEFAULTS["hend"]
            logging.info("tmc %s ::: HOMING hend=%d (from HOMING_DEFAULTS)",
                         self.name, hend)

        # Hysteresis constraint: (hstrt+1) + (hend-3) <= 16
        # Encoded hend range is 0-15, interpreted as -3..+12
        hysteresis_sum = (hstrt + 1) + (hend - 3)
        if hysteresis_sum > 16:
            logging.warning(
                "tmc %s ::: HOMING hysteresis constraint violated "
                "(hstrt+1)+(hend-3)=%d > 16; clamping hstrt to %d",
                self.name, hysteresis_sum, max(0, 16 - (hend - 3) - 1))
            hstrt = max(0, 16 - (hend - 3) - 1)

        # tpfd
        if self.homing_tpfd is not None:
            tpfd = self.homing_tpfd
            logging.info("tmc %s ::: HOMING tpfd=%d (from homing_tpfd cfg)",
                         self.name, tpfd)
        else:
            tpfd = HOMING_DEFAULTS["tpfd"]
            logging.info("tmc %s ::: HOMING tpfd=%d (from HOMING_DEFAULTS)",
                         self.name, tpfd)

        # sfilt (lives in COOLCONF)
        if self.homing_sfilt is not None:
            sfilt = self.homing_sfilt
            logging.info("tmc %s ::: HOMING sfilt=%d (from homing_sfilt cfg)",
                         self.name, sfilt)
        else:
            sfilt = HOMING_DEFAULTS["sfilt"]
            logging.info("tmc %s ::: HOMING sfilt=%d (from HOMING_DEFAULTS)",
                         self.name, sfilt)

        # --- Write CHOPCONF fields ---
        self.fields.set_field("toff",   toff)
        self.fields.set_field("tbl",    tbl)
        self.fields.set_field("hstrt",  hstrt)
        self.fields.set_field("hend",   hend)
        self.fields.set_field("tpfd",   tpfd)
        self.fields.set_field("chm",    HOMING_DEFAULTS["chm"])       # SpreadCycle
        self.fields.set_field("vhighfs", HOMING_DEFAULTS["vhighfs"])
        self.fields.set_field("vhighchm", HOMING_DEFAULTS["vhighchm"])

        # --- COOLCONF: CoolStep OFF, sfilt as configured ---
        self.fields.set_field("semin",  HOMING_DEFAULTS["semin"])
        self.fields.set_field("semax",  HOMING_DEFAULTS["semax"])
        self.fields.set_field("seup",   HOMING_DEFAULTS["seup"])
        self.fields.set_field("sedn",   HOMING_DEFAULTS["sedn"])
        self.fields.set_field("seimin", HOMING_DEFAULTS["seimin"])
        self.fields.set_field("sfilt",  sfilt)
        # Keep sgt from user config if provided
        if self.sgt is not None:
            logging.info("tmc %s ::: HOMING sgt=%d (from driver_SGT cfg)",
                         self.name, self.sgt)
            self.fields.set_field("sgt", self.sgt)

        # --- IHOLD_IRUN: iholddelay ---
        self.fields.set_field("iholddelay", HOMING_DEFAULTS["iholddelay"])

        # --- Current (GlobalScaler / CS) for homing ---
        self._apply_homing_current(homing_current)

        if flush:
            # Only flush CHOPCONF and COOLCONF here.
            # IHOLD_IRUN + GLOBALSCALER are written by apply_current() directly.
            # TCOOLTHRS is overwritten by VirtualPinHelper.homing_move_begin.
            homing_regs = ["CHOPCONF", "COOLCONF"]
            self._flush_registers(homing_regs, label="HOMING", immediate=immediate)

    # ------------------------------------------------------------------
    # Register flush helpers
    # ------------------------------------------------------------------

    def _flush_registers(self, reg_names, label="", immediate=False):
        """Write a list of shadow registers to hardware and log each one.

        immediate=True: write with print_time=None (synchronous, bypasses MCU
        queue delay).  Use for profile switches during homing so that registers
        are guaranteed to be active before the next move is planned.
        immediate=False (default): use get_last_move_time() as before.
        """
        if immediate:
            print_time = None
        else:
            toolhead = self.printer.lookup_object('toolhead', None)
            if toolhead is not None:
                print_time = toolhead.get_last_move_time()
            else:
                print_time = None

        tag = f"[{label}] " if label else ""
        for reg_name in reg_names:
            val = self.fields.registers.get(reg_name)
            if val is None:
                continue
            try:
                self.mcu_tmc.set_register(reg_name, val, print_time)
                logging.info(
                    "tmc %s ::: %sHW flush: %s = 0x%08x",
                    self.name, tag, reg_name, val)
            except Exception as e:
                logging.warning(
                    "tmc %s ::: %sHW flush FAILED for %s: %s",
                    self.name, tag, reg_name, e)

    # ------------------------------------------------------------------
    # Current helpers (public)
    # ------------------------------------------------------------------

    def get_max_current(self):
        return self.max_current

    def set_home_current(self, new_home_current):
        self.req_home_current = min(self.max_current, new_home_current)

    def set_run_current(self, new_run_current):
        self.req_run_current = min(self.max_current, new_run_current)

    def set_hold_current(self, new_hold_current):
        self.req_hold_current = new_hold_current

    def needs_current_changes(self, run_current, hold_current, force=False):
        if (run_current == self.actual_current
                and hold_current == self.req_hold_current
                and not force):
            return False
        return True

    def set_current(self, new_current, hold_current, print_time, force=False):
        if not self.needs_current_changes(new_current, hold_current, force):
            return
        if hold_current != self.req_hold_current:
            self.req_hold_current = hold_current
        self.actual_current = new_current
        self.apply_current(print_time)
        self.tune_driver(new_current, force=force)

    def set_driver_velocity_field(self, field, velocity):
        register = self.fields.lookup_register(field, None)
        if register is None:
            return
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.name)
        arg = TMCtstepHelper(self.mcu_tmc, velocity, pstepper=self.stepper)
        logging.info("tmc %s ::: set_driver_velocity_field %s=%s(%s)",
                     self.name, field, repr(arg), repr(velocity))
        self.fields.set_field(field, arg)

    # ------------------------------------------------------------------
    # RUN tuning sub-functions (unchanged from original)
    # ------------------------------------------------------------------

    def tune_driver(self, new_current=0, flush=True, force=False, immediate=False):
        """Apply RUN or HOMING profile based on new_current.

        Skips if new_current unchanged since last call, unless force=True
        (SET_TMC_CURRENT uses force=True to override stored currents).

        Profile selection:
          new_current == req_home_current  →  HOMING profile
          anything else                   →  RUN profile

        immediate=True: all register flushes use print_time=None so they
        are written synchronously.  Must be set when called from
        _load_homing_profile()/_load_run_profile() to guarantee the
        profile is active before the next move is planned.
        """
        if self.driver_tuning is None:
            return

        if new_current == 0:
            new_current = self.config_run_current

        if not force and new_current == self._last_tuned_current:
            logging.info(
                "tmc %s ::: tune_driver skip – already tuned for %.3fA",
                self.name, new_current)
            return

        self._last_tuned_current = new_current

        if self._homing_active:
            logging.info(
                "tmc %s ::: tune_driver → HOMING profile (%.3fA)",
                self.name, new_current)
            self._apply_homing_profile(homing_current=new_current, flush=flush,
                                       immediate=immediate)
        else:
            logging.info(
                "tmc %s ::: tune_driver → RUN profile (%.3fA)",
                self.name, new_current)
            self._apply_run_profile(new_current=new_current, flush=flush,
                                    immediate=immediate)
    DEFAULT_MAX_FREQUENCY = 12.5e6

    def _get_tmc_clock_frequency(self):
        try:
            self.driver_clock_frequency = (
                self.mcu_tmc.get_tmc_frequency() or self.DEFAULT_MAX_FREQUENCY)
        except AttributeError:
            self.driver_clock_frequency = self.DEFAULT_MAX_FREQUENCY
        logging.info(f"tmc {self.name} ::: driver clock: "
                     f"{self.driver_clock_frequency}")

    def _configure_pwm(self, new_current):
        motor_object = self.printer.lookup_object(self.motor_name)
        pwm_freq, calculated_freq = motor_object.pwmfreq(
            fclk=self.driver_clock_frequency, target=self.pwm_freq_target)
        logging.info(f"tmc {self.name} ::: Calculated Frequency: "
                     f"{calculated_freq / 1000} kHz")
        logging.info(f"tmc {self.name} ::: pwm_freq: {pwm_freq}")
        pwmgrad = motor_object.pwmgrad(
            volts=self.voltage, fclk=self.driver_clock_frequency)
        pwmofs  = motor_object.pwmofs(volts=self.voltage, current=new_current)
        logging.info(f"tmc {self.name} ::: pwmgrad: {pwmgrad}")
        logging.info(f"tmc {self.name} ::: pwmofs: {pwmofs}")
        self.fields.set_field("pwm_freq",     pwm_freq)
        self.fields.set_field("pwm_autoscale", True)
        self.fields.set_field("pwm_autograd",  True)
        self.fields.set_field("pwm_grad",      pwmgrad)
        self.fields.set_field("pwm_ofs",       pwmofs)
        self.fields.set_field("pwm_reg",       15)
        self.fields.set_field("pwm_lim",       4)
        self.fields.set_field("tpwmthrs",      0xfffff)

    def _configure_spreadcycle(self, new_current):
        motor_object = self.printer.lookup_object(self.motor_name)
        _, calculated_freq = motor_object.pwmfreq(
            fclk=self.driver_clock_frequency, target=self.pwm_freq_target)
        ncycles = int(math.ceil(self.driver_clock_frequency / calculated_freq))
        sdcycles = ncycles / 4
        tbl = self.tbl or 0
        tblank = 16.0 * (1.5 ** tbl) / self.driver_clock_frequency

        if self.toff is None:
            toff = 0
            tsd_duty = (24.0 + 32.0 * toff) / self.driver_clock_frequency
            duty_cycle_highside = (new_current * 0.7 / self.voltage
                                   + (tblank / (tblank + tsd_duty)))
            chop_freq_lowest = 1 / ((2 + 4 * duty_cycle_highside) * tsd_duty)
            target = self.chopper_freq_target or 20e3
            while chop_freq_lowest > target:
                toff += 1
                tsd_duty = (24.0 + 32.0 * toff) / self.driver_clock_frequency
                duty_cycle_highside = (new_current * 0.7 / self.voltage
                                       + (tblank / (tblank + tsd_duty)))
                chop_freq_lowest = 1 / (
                    (2 + 4 * duty_cycle_highside) * tsd_duty)
            toff = max(toff - 1, 0)
            tsd_duty = (24.0 + 32.0 * toff) / self.driver_clock_frequency
            duty_cycle_highside = (new_current * 0.7 / self.voltage
                                   + (tblank / (tblank + tsd_duty)))
            chop_freq_lowest = 1 / ((2 + 4 * duty_cycle_highside) * tsd_duty)
            logging.info(
                f"tmc {self.name} ::: toff: {toff}, "
                f"target chopper freq: {chop_freq_lowest}")
        else:
            toff = self.toff

        logging.info(
            f"tmc {self.name} ::: ncycles: {ncycles}, sdcycles: {sdcycles}")

        if toff == 1 and tbl == 0:
            tbl = 1
            tblank = 16.0 * (1.5 ** tbl) / self.driver_clock_frequency

        tsd      = (12.0 + 32.0 * toff) / self.driver_clock_frequency
        tsd_duty = (24.0 + 32.0 * toff) / self.driver_clock_frequency

        chop_freq_limit = 1 / (2 * tsd + 2 * tblank)
        duty_cycle_highside = (new_current * 0.7 / self.voltage
                               + (tblank / (tblank + tsd_duty)))
        chop_freq_lowest = 1 / ((2 + 4 * duty_cycle_highside) * tsd_duty)

        logging.info(
            f"tmc {self.name} ::: duty_cycle_highside: {duty_cycle_highside}")
        logging.info(
            f"tmc {self.name} ::: chop_freq_limit: {chop_freq_limit / 1000}")
        logging.info(
            f"tmc {self.name} ::: chop_freq_lowest: {chop_freq_lowest / 1000}")

        pfdcycles = (ncycles
                     - (tsd_duty * 2 - tblank) * self.driver_clock_frequency)
        tpfd = (max(0, min(15, int(math.ceil(pfdcycles / 128))))
                if self.tpfd is None else self.tpfd)

        logging.info(
            f"tmc {self.name} ::: tbl: {tbl}, "
            f"pfdcycles: {pfdcycles}, tpfd: {tpfd}")

        self.fields.set_field('tpfd', tpfd)
        self.fields.set_field('tbl',  tbl)
        self.fields.set_field('toff', toff)
        return tbl, toff

    def _configure_hysteresis(self, new_current, new_tbl, new_toff):
        if self.hstrt and self.hend:
            hstrt, hend = self.hstrt, self.hend
        else:
            motor_object = self.printer.lookup_object(self.motor_name)
            hstrt, hend = motor_object.hysteresis(
                name=self.name,
                volts=self.voltage,
                current=new_current,
                tbl=new_tbl,
                toff=new_toff,
                fclk=self.driver_clock_frequency,
                extra=self.extra_hysteresis,
                rsense=self.sense_resistor,
                scale=self.cs
            )
        logging.info(
            f"tmc {self.name} ::: hstrt: {hstrt}, hend: {hend}, "
            f"extra: {self.extra_hysteresis}")
        self.fields.set_field('hstrt', hstrt)
        self.fields.set_field('hend',  hend)

    def _configure_stallguard(self, new_current):
        pwmthrs = 0
        vmaxpwm = self._calculate_vmaxpwm_parameters(new_current)
        coolthrs = 0.75 * self.stepper.get_rotation_distance()[0]
        logging.info(f"tmc {self.name} ::: coolthrs: {coolthrs}")

        if self.fields.lookup_register("sg4_thrs", None) is not None:
            pwmthrs = max(0.2 * vmaxpwm, 1.125 * coolthrs)
            if self.sg4_thrs is not None:
                self.fields.set_field('sg4_thrs', self.sg4_thrs)
                self.fields.set_field('sg4_filt_en', True)
        elif self.fields.lookup_register("sgthrs", None) is not None:
            pwmthrs = max(0.2 * vmaxpwm, 1.125 * coolthrs)
            if self.sg4_thrs is not None:
                self.fields.set_field('sgthrs', self.sg4_thrs)
        else:
            pwmthrs = 0.5 * vmaxpwm

        if self.sgt is not None:
            self.fields.set_field('sgt', self.sgt)

        logging.info(f"tmc {self.name} ::: pwmthrs: {pwmthrs}")

    def _configure_coolstep(self):
        coolthrs = 0.75 * self.stepper.get_rotation_distance()[0]
        self.set_driver_velocity_field('tcoolthrs', coolthrs)
        self.fields.set_field('faststandstill', True)
        self.fields.set_field('small_hysteresis', False)
        self.fields.set_field('semin',     2)
        self.fields.set_field('semax',     4)
        self.fields.set_field('seup',      3)
        self.fields.set_field('sedn',      2)
        self.fields.set_field('seimin',    1)
        self.fields.set_field('sfilt',     0)
        self.fields.set_field('iholddelay', 12)

    def _configure_overvoltage(self):
        if self.overvoltage_vth is not None:
            vth = int(self.overvoltage_vth / 0.009732)
            self.fields.set_field('overvoltage_vth', vth)

    def _configure_highspeed(self, new_current):
        vmaxpwm = 1.2 * self._calculate_vmaxpwm_parameters(new_current)
        logging.info(f"tmc {self.name} ::: vmaxpwm: {vmaxpwm}")
        self.set_driver_velocity_field('thigh', vmaxpwm)
        self.fields.set_field('vhighfs', False)
        self.fields.set_field('vhighchm', False)
        self.fields.set_field('multistep_filt', True)

    def _calculate_vmaxpwm_parameters(self, new_current):
        motor_object = self.printer.lookup_object(self.motor_name)
        maxpwmrps = motor_object.maxpwmrps(
            volts=self.voltage, current=new_current)
        rdist = self.stepper.get_rotation_distance()[0]
        return maxpwmrps * rdist


# Helper to configure StallGuard and CoolStep minimum velocity
def TMCVcoolthrsHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat('coolstep_threshold', None, minval=0.)
    tcoolthrs = 0
    if velocity is not None:
        tcoolthrs = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("tcoolthrs", tcoolthrs)

# Helper to configure StallGuard and CoolStep maximum velocity and
# SpreadCycle-FullStepping (High velocity) mode threshold.
def TMCVhighHelper(config, mcu_tmc):
    fields = mcu_tmc.get_fields()
    velocity = config.getfloat('high_velocity_threshold', None, minval=0.)
    thigh = 0
    if velocity is not None:
        thigh = TMCtstepHelper(mcu_tmc, velocity, config=config)
    fields.set_field("thigh", thigh)
