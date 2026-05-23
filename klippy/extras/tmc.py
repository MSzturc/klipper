# Common helper code for TMC stepper drivers
#
# Copyright (C) 2018-2020  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math, collections
import stepper
from . import bulk_sensor


######################################################################
# Sense-resistor / stepstick lookup
######################################################################

# Resolve the sense-resistor value for a TMC driver config section.  The
# user may supply 'sense_resistor' explicitly, or a 'stepstick_type' that
# names a [stepstick <name>] section (carrier data shipped as config).  At
# least one of the two must be set; configs that omit both fail to load.
# Returns (sense_resistor, max_current) — max_current is None if no
# stepstick_type was given (callers fall back to a per-driver default).
def _lookup_stepstick(config, name):
    # [stepstick <name>] sections come from the config (included by the base
    # layer), so they normally exist before the driver inits.  Force-load the
    # one we need if section ordering left it uninstantiated -- the same lazy
    # resolve as [motor_constants <name>], with no file path in the firmware.
    printer = config.get_printer()
    section = 'stepstick ' + name
    obj = printer.lookup_object(section, None)
    if obj is None:
        try:
            obj = printer.load_object(config, section)
        except config.error:
            obj = None
    if obj is None:
        raise config.error(
            "Unknown stepstick_type '%s' in section '%s'. Define a "
            "[stepstick %s] section (THEOS-Configuration ships the carrier "
            "database in steppers/database/stepsticks.cfg)."
            % (name, config.get_name(), name))
    return obj


def resolve_sense_resistor(config, required=True):
    explicit = config.getfloat('sense_resistor', None, above=0.)
    stepstick = config.get('stepstick_type', None)
    lookup_sr = lookup_max = None
    if stepstick is not None:
        carrier = _lookup_stepstick(config, stepstick)
        lookup_sr, lookup_max = carrier.sense_resistor, carrier.max_current
    sense_resistor = explicit if explicit is not None else lookup_sr
    if sense_resistor is None and required:
        raise config.error(
            "Section '%s' must specify either 'sense_resistor' or "
            "'stepstick_type' so the driver knows the correct sense "
            "resistance.  See steppers/database/stepsticks.cfg for the "
            "list of supported stepstick boards."
            % (config.get_name(),))
    return sense_resistor, lookup_max


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
            # Caller signalled "only write the register if the user set it
            # explicitly".  Used by write-only registers like SHORT_CONF
            # that have no readable defaults to round-trip.
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
        self.mcu = mcu_tmc.get_mcu()
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
            # A comms error on a non-critical MCU should pause driver-status
            # polling rather than invoke_shutdown - the reconnect path will
            # restart checks once the MCU is back.
            if getattr(self.mcu, "is_non_critical", False):
                logging.info("Pausing TMC periodic check on '%s': %s",
                             self.stepper_name, str(e))
                self.stop_checks()
                return self.printer.get_reactor().NEVER
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
# Record driver status
######################################################################

class TMCStallguardDump:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.mcu_tmc = mcu_tmc
        self.mcu = self.mcu_tmc.get_mcu()
        self.fields = self.mcu_tmc.get_fields()
        self.sg2_supp = False
        self.sg4_reg_name = None
        # It is possible to support TMC2660, just disable it for now
        if not self.fields.all_fields.get("DRV_STATUS", None):
            return
        # Collect driver capabilities
        if self.fields.all_fields["DRV_STATUS"].get("sg_result", None):
            self.sg2_supp = True
        # New drivers have separate register for SG4 result
        if self.mcu_tmc.name_to_reg.get("SG_RESULT", 0):
            self.sg4_reg_name = "SG_RESULT"
        # 2240 supports both SG2 & SG4
        if self.sg4_reg_name is None:
            if self.mcu_tmc.name_to_reg.get("SG4_RESULT", 0):
                self.sg4_reg_name = "SG4_RESULT"
        # TMC2208
        if self.sg2_supp is None and self.sg4_reg_name is None:
            return
        self.optimized_spi = False
        # Bulk API
        self.samples = []
        self.query_timer = None
        self.error = None
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._dump, self._start, self._stop)
        api_resp = {'header': ('time', 'sg_result', 'cs_actual')}
        self.batch_bulk.add_mux_endpoint("tmc/stallguard_dump", "name",
                                         self.stepper_name, api_resp)
    def _start(self):
        self.error = None
        status = self.mcu_tmc.get_register_raw("DRV_STATUS")
        if status.get("spi_status"):
            self.optimized_spi = True
        reactor = self.printer.get_reactor()
        self.query_timer = reactor.register_timer(self._query_tmc,
                                                  reactor.NOW)
    def _stop(self):
        self.printer.get_reactor().unregister_timer(self.query_timer)
        self.query_timer = None
        self.samples = []
    def _query_tmc(self, eventtime):
        sg_result = -1
        cs_actual = -1
        recv_time = eventtime
        try:
            if self.optimized_spi or self.sg4_reg_name == "SG4_RESULT":
                #TMC2130/TMC5160/TMC2240
                status = self.mcu_tmc.get_register_raw("DRV_STATUS")
                reg_val = status["data"]
                cs_actual = self.fields.get_field("cs_actual", reg_val)
                sg_result = self.fields.get_field("sg_result", reg_val)
                is_stealth = self.fields.get_field("stealth", reg_val)
                recv_time = status["#receive_time"]
                if is_stealth and self.sg4_reg_name == "SG4_RESULT":
                    sg4_ret = self.mcu_tmc.get_register_raw("SG4_RESULT")
                    sg_result = sg4_ret["data"]
                    recv_time = sg4_ret["#receive_time"]
            else:
                # TMC2209
                if self.sg4_reg_name == "SG_RESULT":
                    sg4_ret = self.mcu_tmc.get_register_raw("SG_RESULT")
                    sg_result = sg4_ret["data"]
                    recv_time = sg4_ret["#receive_time"]
        except self.printer.command_error as e:
            self.error = e
            return self.printer.get_reactor().NEVER
        print_time = self.mcu.estimated_print_time(recv_time)
        self.samples.append((print_time, sg_result, cs_actual))
        if self.optimized_spi:
            return eventtime + 0.001
        # UART queried as fast as possible
        return eventtime + 0.005
    def _dump(self, eventtime):
            if self.error:
                raise self.error
            samples = self.samples
            self.samples = []
            return {"data": samples}


######################################################################
# G-Code command helpers
######################################################################

class TMCCommandHelper:
    def __init__(self, config, mcu_tmc, current_helper):
        self.printer = config.get_printer()
        self.stepper_name = ' '.join(config.get_name().split()[1:])
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.mcu = mcu_tmc.get_mcu()
        self.current_helper = current_helper
        self.fields = mcu_tmc.get_fields()
        self.stepper = None
        # Stepper phase tracking
        self.mcu_phase_offset = None
        # Stepper enable/disable tracking
        self.toff = None
        self.stepper_enable = self.printer.load_object(config, "stepper_enable")
        self.enable_mutex = self.printer.get_reactor().mutex()
        # DUMP_TMC support
        self.read_registers = self.read_translate = None
        # Common tmc helpers
        self.echeck_helper = TMCErrorCheck(config, mcu_tmc)
        self.record_helper = TMCStallguardDump(config, mcu_tmc)
        TMCMicrostepHelper(config, mcu_tmc)
        # Register callbacks
        self.printer.register_event_handler("stepper:sync_mcu_position",
                                            self._handle_sync_mcu_pos)
        self.printer.register_event_handler("stepper:set_dir_inverted",
                                            self._handle_sync_mcu_pos)
        self.printer.register_event_handler("klippy:mcu_identify",
                                            self._handle_mcu_identify)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        # Hook non-critical disconnect/reconnect so driver-status polling
        # pauses on disconnect and TMC registers get re-initialised when
        # the driver MCU returns.
        if self._is_noncritical_mcu():
            self.printer.register_event_handler(
                self.mcu.get_non_critical_disconnect_event_name(),
                self._handle_noncritical_disconnect)
            self.printer.register_event_handler(
                self.mcu.get_non_critical_reconnect_event_name(),
                self._handle_noncritical_reconnect)
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
    def _is_noncritical_mcu(self):
        return bool(getattr(self.mcu, "is_non_critical", False))
    def _handle_noncritical_disconnect(self):
        self.echeck_helper.stop_checks()
        self.mcu_phase_offset = None
    def _handle_noncritical_reconnect(self):
        if self.stepper is None:
            return
        enable_line = self.stepper_enable.lookup_enable(self.stepper_name)
        if not enable_line.is_motor_enabled():
            return
        try:
            with self.enable_mutex:
                if self.toff is not None:
                    self.fields.set_field("toff", self.toff)
                # Re-tune so the chopper / hysteresis fields land in the
                # bulk write below; the driver MCU lost them on the
                # disconnect.  No-op when autotune is not configured.
                self.current_helper.tune_driver(force=True)
                self._init_registers()
                did_reset = self.echeck_helper.start_checks()
            if did_reset:
                self.mcu_phase_offset = None
        except self.printer.command_error as e:
            logging.info("TMC %s reconnect recovery failed: %s",
                         self.name, str(e))
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
        cur = ch.get_current()
        prev_cur, prev_hold_cur, req_hold_cur, max_cur = cur[:4]
        prev_home_cur = cur[4] if len(cur) > 4 else None
        run_current = gcmd.get_float('CURRENT', None, above=0., maxval=max_cur)
        hold_current = gcmd.get_float('HOLDCURRENT', None,
                                      above=0., maxval=max_cur)
        home_current = gcmd.get_float('HOMECURRENT', None,
                                      above=0., maxval=max_cur)
        if (run_current is not None or hold_current is not None
                or home_current is not None):
            # Defer req_run_current / req_home_current mutation until
            # after a successful driver write. set_current() rolls back
            # actual_current and req_hold_current on apply_current()
            # failure, but cannot undo set_run_current/set_home_current
            # — touching those before the write means a transient
            # UART/SPI error would silently shift the target the next
            # post-home current restore writes back.
            new_run_current = (run_current if run_current is not None
                               else prev_cur)
            new_hold_current = (hold_current if hold_current is not None
                                else req_hold_cur)
            toolhead = self.printer.lookup_object('toolhead')
            print_time = toolhead.get_last_move_time()
            # force=True so apply_current() and tune_driver() always
            # re-run, even when the requested run current matches the
            # currently programmed value — autotune may have moved
            # chopper / hysteresis registers since the last apply, and
            # SET_TMC_CURRENT semantically commits whatever the user
            # just typed.
            ch.set_current(new_run_current, new_hold_current, print_time,
                           force=True)
            if run_current is not None:
                ch.set_run_current(run_current)
            if home_current is not None:
                ch.set_home_current(home_current)
            cur = ch.get_current()
            prev_cur, prev_hold_cur, req_hold_cur, max_cur = cur[:4]
            prev_home_cur = cur[4] if len(cur) > 4 else None
        # Report values
        parts = ["Run Current: %0.2fA" % (prev_cur,)]
        if prev_hold_cur is not None:
            parts.append("Hold Current: %0.2fA" % (prev_hold_cur,))
        if prev_home_cur is not None:
            parts.append("Home Current: %0.2fA" % (prev_home_cur,))
        gcmd.respond_info(" ".join(parts))
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
        if self.toff is not None:
            # Shared enable via comms handling
            self.fields.set_field("toff", self.toff)
        # Autotune before _init_registers so the chopper / hysteresis /
        # stallguard register fields land in the same bulk write that
        # _init_registers issues.  Without autotune (no motor/voltage)
        # this is a cheap no-op.
        self.current_helper.tune_driver()
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
    def _do_disable(self, print_time):
        if self.toff is not None:
            val = self.fields.set_field("toff", 0)
            reg_name = self.fields.lookup_register("toff")
            self.mcu_tmc.set_register(reg_name, val, print_time)
        self.echeck_helper.stop_checks()
    def _handle_stepper_enable(self, print_time, is_enable):
        if self._is_noncritical_mcu():
            logging.info("TMC '%s' stepper event: %s at %.6f",
                         self.stepper_name,
                         "enable" if is_enable else "disable",
                         print_time)
        def enable_disable_cb(eventtime):
            try:
                with self.enable_mutex:
                    if is_enable:
                        self._do_enable(print_time)
                        if self._is_noncritical_mcu():
                            logging.info("TMC '%s' enable sequence complete",
                                         self.stepper_name)
                    else:
                        self._do_disable(print_time)
            except self.printer.command_error as e:
                # Ignore comms errors on non-critical MCUs - the reconnect
                # path will re-init registers once the link is back.
                if self._is_noncritical_mcu():
                    logging.info("Ignoring TMC '%s' comms error on %s: %s",
                                 self.stepper_name,
                                 "enable" if is_enable else "disable",
                                 str(e))
                    return
                self.printer.invoke_shutdown(str(e))
        self.printer.get_reactor().register_callback(enable_disable_cb)
    # Initial startup handling
    def _handle_mcu_identify(self):
        # Lookup stepper object
        force_move = self.printer.lookup_object("force_move")
        self.stepper = force_move.lookup_stepper(self.stepper_name)
        # Note pulse duration and step_both_edge optimizations available
        self.stepper.setup_default_pulse_duration(.000000100, True)
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
        # Send init - skip if the driver MCU is a non-critical one that is
        # currently offline; the reconnect path will run _init_registers
        # once the MCU is back.
        try:
            if getattr(self.mcu, "non_critical_disconnected", False):
                logging.info(
                    "TMC %s skipping init - non-critical MCU '%s' is"
                    " disconnected", self.name, self.mcu.get_name())
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
        self._dirty_regs = collections.OrderedDict()
        self._prev_state = collections.OrderedDict()
        # Register virtual_endstop pin
        name_parts = config.get_name().split()
        ppins = self.printer.lookup_object("pins")
        ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)
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
    def _set_field(self, field_name, value):
        self._prev_state[field_name] = self.fields.get_field(field_name)
        reg_name = self.fields.lookup_register(field_name)
        self._dirty_regs[reg_name] = self.fields.set_field(field_name, value)
    def _send_fields(self):
        for reg, val in self._dirty_regs.items():
            self.mcu_tmc.set_register(reg, val)
        self._dirty_regs.clear()
    def handle_homing_move_begin(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        sg4_thrs = 0
        if self.fields.lookup_register("sg4_thrs", None) is not None:
            sg4_thrs = self.fields.get_field("sg4_thrs")
        # Enable/disable stealthchop
        reg = self.fields.lookup_register("en_pwm_mode", None)
        if reg is None:
            # On "stallguard4" drivers, "stealthchop" must be enabled
            self._set_field("tpwmthrs", 0)
            self._set_field("en_spreadcycle", 0)
        elif sg4_thrs:
            # TMC2240 using SG4, "stealthchop" must be enabled
            self._set_field("en_pwm_mode", 1)
            self._set_field("tpwmthrs", 0)
            self._set_field(self.diag_pin_field, 1)
        else:
            # On earlier drivers, "stealthchop" must be disabled
            self._set_field("en_pwm_mode", 0)
            self._set_field(self.diag_pin_field, 1)
        # Enable tcoolthrs (if not already)
        if self.fields.get_field("tcoolthrs") == 0:
            self._set_field("tcoolthrs", 0xfffff)
        # Disable thigh
        reg = self.fields.lookup_register("thigh", None)
        if reg is not None:
            self._set_field("thigh", 0)
        self._send_fields()
    def handle_homing_move_end(self, hmove):
        if self.mcu_endstop not in hmove.get_mcu_endstops():
            return
        # Restore previous state
        for field, val in list(self._prev_state.items()):
            self._set_field(field, val)
        self._send_fields()
        self._prev_state.clear()


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


######################################################################
# Base class for TMC current helpers (sensorless homing + autotuning)
######################################################################

# Per-driver-type stealthChop PWM frequency targets.  TMC2240 runs hot
# at the higher 55 kHz target, so it uses 20 kHz instead.
PWM_FREQ_TARGETS = {
    'tmc2130': 55e3,
    'tmc2208': 55e3,
    'tmc2209': 55e3,
    'tmc2240': 20e3,
    'tmc2660': 55e3,
    'tmc5160': 55e3,
}

# Conservative fallback if the driver doesn't expose a clock frequency
# via FFI (e.g. because the chelper does not know about it).  Source:
# TMC5160A page 122 ("Clock oscillator frequency").
DEFAULT_TMC_CLOCK_FREQUENCY = 12.5e6

# Per-stepper homing profile defaults.  Applied around every homing
# move to swap the run-time chopper/coolstep/iholddelay configuration
# for a low-noise / robust-StallGuard configuration.  Each value is
# overridden by the corresponding `homing_*` config option when set.
# CHOPCONF: SpreadCycle, conservative low-noise.
# COOLCONF: CoolStep off (semin/semax/seup/sedn/seimin all 0); sfilt
# off by default — set `homing_sfilt: 1` to enable the StallGuard
# filter when the SG signal is noisy.
# IHOLD_IRUN: longer iholddelay decays the hold current more slowly,
# producing a quieter standstill during the brief homing pause.
# `cs`: low-noise CS for the homing IRUN/IHOLD search; only consumed
# by drivers that implement _calc_homing_current (TMC5160, TMC2240).
HOMING_DEFAULTS = {
    'toff':       3,
    'tbl':        1,
    'hstrt':      0,
    'hend':       4,
    'tpfd':       4,
    'chm':        0,
    'vhighfs':    0,
    'vhighchm':   0,
    'semin':      0,
    'semax':      0,
    'seup':       0,
    'sedn':       0,
    'seimin':     0,
    'sfilt':      0,
    'iholddelay': 8,
    'cs':         8,
}

# Registers that tune_driver()'s _configure_* helpers may dirty.  After
# tuning every shadow field is up to date but only IRUN/IHOLD/GLOBALSCALER
# are flushed (via apply_current); the chopper / coolstep / pwm / threshold
# registers must be flushed explicitly or the tuned values would only land
# at the next _init_registers (driver enable / non-critical reconnect).
TUNE_FLUSH_REGS = (
    "GCONF",
    "CHOPCONF", "COOLCONF", "PWMCONF", "IHOLD_IRUN",
    "TPWMTHRS", "TCOOLTHRS", "THIGH",
    "SGTHRS", "SG4_THRS", "OTW_OV_VTH",
    "DRV_CONF",   # VS-aware filt_isense default lives here
)


# Shared state machine used by every TMC driver CurrentHelper so the same
# "run / hold / home" current model is visible to the homing state machine.
# Three current concepts are tracked independently:
#   * config_*  - the value parsed from the config file
#   * req_*     - the current the user/gcode last requested (SET_TMC_CURRENT
#                 moves this, the config default initialises it)
#   * actual_*  - the current the driver is actually programmed to right now;
#                 this swings between req_run_current and req_home_current
#                 around a homing cycle, and is restored to req_run_current
#                 at the end of the homing sequence.
# Subclasses implement apply_current(print_time) - the driver-specific
# path that programs actual_current into the hardware.
class BaseTMCCurrentHelper:
    def __init__(self, config, mcu_tmc, max_current):
        self.printer = config.get_printer()
        self.driver_type = config.get_name().split()[0]
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.max_current = max_current
        # Configured defaults
        self.config_run_current = config.getfloat(
            'run_current', above=0., maxval=max_current)
        self.config_hold_current = config.getfloat(
            'hold_current', max_current, above=0., maxval=max_current)
        self.config_home_current = config.getfloat(
            'home_current', self.config_run_current,
            above=0., maxval=max_current)
        self.current_change_dwell_time = config.getfloat(
            'current_change_dwell_time', 0.5, above=0.)
        # Requested values start at config defaults
        self.req_run_current = self.config_run_current
        self.req_hold_current = self.config_hold_current
        self.req_home_current = self.config_home_current
        # Actual value currently programmed on the driver
        self.actual_current = self.req_run_current
        # ------------------------------------------------------------
        # Autotuning configuration.  Tuning is opt-in: it activates only
        # when the user supplies both 'motor:' (referencing a
        # [motor_constants <name>] section) and 'voltage:'.  Without
        # those, every per-field config option below still works as a
        # plain register override; the only thing that gets skipped is
        # the chopper / hysteresis / stallguard derivation.
        self.motor = config.get('motor', None)
        # The [motor_constants <name>] sections are config-defined (shipped in
        # THEOS-Configuration's steppers/database/motors.cfg and pulled in by
        # the base layer), so they are instantiated like any other config
        # object; tune_driver() resolves the named motor at runtime.
        self.voltage = config.getfloat('voltage', None,
                                       above=0., maxval=60.)
        self.pwm_freq_target = config.getfloat(
            'pwm_freq_target',
            PWM_FREQ_TARGETS.get(self.driver_type, 55e3),
            minval=10e3, maxval=100e3)
        self.chopper_freq_target = config.getfloat(
            'chopper_freq_target', None, minval=10e3, maxval=100e3)
        self.extra_hysteresis = config.getint('extra_hysteresis', 0,
                                              minval=0, maxval=15)
        # Driver-register overrides — None means "let autotune decide".
        # (driver_cs is parsed by per-driver subclasses and assigned to
        # self.cs before BaseTMCCurrentHelper sees it; if a subclass did
        # not set it, fall back to None here so autotune treats it as
        # auto-compute.)
        self.tbl = config.getint('driver_TBL', None, minval=0, maxval=3)
        self.toff = config.getint('driver_TOFF', None, minval=1, maxval=15)
        self.tpfd = config.getint('driver_TPFD', None, minval=0, maxval=15)
        if not hasattr(self, 'cs'):
            self.cs = None
        self.hstrt = config.getint('driver_HSTRT', None, minval=0, maxval=7)
        self.hend = config.getint('driver_HEND', None, minval=0, maxval=15)
        self.sg4_thrs = config.getint('driver_SGTHRS', None,
                                      minval=0, maxval=255)
        self.sgt = config.getint('driver_SGT', None,
                                 minval=-64, maxval=63)
        self.overvoltage_vth = config.getfloat('overvoltage_vth', None,
                                               minval=0., maxval=60.)
        # Cached state populated by tune_driver().
        self.driver_clock_frequency = DEFAULT_TMC_CLOCK_FREQUENCY
        self.stepper = None
        self._last_tuned_current = None
        self._autotune_skip_logged = False
        self._skipped_fields = set()
        # TOFF=1 (shortest off-time) requires a minimum blank time: TMC2209
        # datasheet §5.5.1 mandates TBL>=2; the TMC5160 allows TBL>=1.
        self.min_tbl_at_min_toff = {'tmc2209': 2}.get(self.driver_type, 1)
        # ------------------------------------------------------------
        # Per-stepper homing profile overrides.  Each is None by
        # default so the corresponding HOMING_DEFAULTS value applies.
        # `homing_cs` is consumed only by drivers with a low-noise CS
        # search (TMC5160, TMC2240); other drivers just use it as the
        # CS bits during the homing window.
        self.homing_toff = config.getint('homing_toff', None,
                                         minval=1, maxval=15)
        self.homing_tbl = config.getint('homing_tbl', None,
                                        minval=0, maxval=3)
        self.homing_hstrt = config.getint('homing_hstrt', None,
                                          minval=0, maxval=7)
        self.homing_hend = config.getint('homing_hend', None,
                                         minval=0, maxval=15)
        self.homing_tpfd = config.getint('homing_tpfd', None,
                                         minval=0, maxval=15)
        self.homing_sfilt = config.getint('homing_sfilt', None,
                                          minval=0, maxval=1)
        self.homing_cs = config.getint('homing_cs', None,
                                       minval=0, maxval=31)
        self._homing_active = False
        # Shadow-field snapshot taken by _apply_homing_profile() so the
        # post-homing path can put chopper / coolstep / iholddelay back to
        # their pre-homing values even when autotune is off or the
        # _last_tuned_current cache shortcuts the run-profile re-derive.
        self._run_profile_snapshot = None
        # ------------------------------------------------------------
        # Tuning goal — selects the auto-derivation strategy used by
        # the _configure_* helpers when a field is not explicitly
        # pinned via driver_X.
        #   * performance — max acceleration / torque (current behaviour)
        #   * balanced    — daily-driver, hours-printable, few artefacts
        #   * silent      — minimum audible noise + maximum vibration smoothing
        self.tuning_goal = config.getchoice(
            'tuning_goal',
            {'performance': 'performance',
             'balanced':    'balanced',
             'silent':      'silent'},
            'balanced')
        # ------------------------------------------------------------
        # PWM/StealthChop pin-reads
        self.pwm_freq = config.getint('driver_PWM_FREQ', None,
                                       minval=0, maxval=3)
        self.pwm_autoscale = config.getboolean('driver_PWM_AUTOSCALE', None)
        self.pwm_autograd = config.getboolean('driver_PWM_AUTOGRAD', None)
        self.pwm_grad = config.getint('driver_PWM_GRAD', None,
                                       minval=0, maxval=255)
        self.pwm_ofs = config.getint('driver_PWM_OFS', None,
                                      minval=0, maxval=255)
        self.pwm_reg_pin = config.getint('driver_PWM_REG', None,
                                          minval=1, maxval=15)
        self.pwm_lim_pin = config.getint('driver_PWM_LIM', None,
                                          minval=0, maxval=15)
        # tpwmthrs has three pin paths in order of precedence (highest first):
        #   1. driver_TPWMTHRS  — raw register pin (user sets exact TSTEP value)
        #   2. stealthchop_threshold — velocity-form pin; TMCStealthchopHelper
        #      converts it to TSTEP at config-time, but we must re-read it here
        #      so _configure_pwm sees the velocity-form value too.  Without
        #      that, the TMCStealthchopHelper write is silently overwritten by
        #      the goal-derived default in _configure_pwm.
        #   3. goal-default (_derive_tpwmthrs)
        self.tpwmthrs_pin = config.getint('driver_TPWMTHRS', None,
                                           minval=0, maxval=0xfffff)
        self.stealthchop_threshold = config.getfloat(
            'stealthchop_threshold', None, minval=0.)
        # GCONF flag pin-reads
        self.faststandstill = config.getboolean('driver_FASTSTANDSTILL', None)
        self.small_hysteresis = config.getboolean('driver_SMALL_HYSTERESIS',
                                                   None)
        self.multistep_filt = config.getboolean('driver_MULTISTEP_FILT', None)
        # CoolStep pin-reads
        self.semin = config.getint('driver_SEMIN', None, minval=0, maxval=15)
        self.semax = config.getint('driver_SEMAX', None, minval=0, maxval=15)
        self.seup = config.getint('driver_SEUP', None, minval=0, maxval=3)
        self.sedn = config.getint('driver_SEDN', None, minval=0, maxval=3)
        self.seimin = config.getboolean('driver_SEIMIN', None)
        self.sfilt = config.getboolean('driver_SFILT', None)
        self.iholddelay = config.getint('driver_IHOLDDELAY', None,
                                         minval=0, maxval=15)
        # Velocity-threshold and high-speed pin-reads
        self.tcoolthrs_pin = config.getint('driver_TCOOLTHRS', None,
                                            minval=0, maxval=0xfffff)
        self.thigh_pin = config.getint('driver_THIGH', None,
                                        minval=0, maxval=0xfffff)
        self.vhighfs = config.getboolean('driver_VHIGHFS', None)
        self.vhighchm = config.getboolean('driver_VHIGHCHM', None)
        # Velocity-form pins (already read by TMCVcoolthrsHelper /
        # TMCVhighHelper at config-time; we re-read them here so the
        # autotune path can apply pin-precedence: raw > velocity > goal)
        self.coolstep_threshold = config.getfloat('coolstep_threshold', None,
                                                   minval=0.)
        self.high_velocity_threshold = config.getfloat(
            'high_velocity_threshold', None, minval=0.)
        # Conflict detection at the config boundary.
        # Setting both the raw register pin and the velocity-form pin for the
        # same threshold register is ambiguous; raise an error so the user
        # is told which knob to use rather than silently letting raw win.
        if self.tcoolthrs_pin is not None and self.coolstep_threshold is not None:
            raise config.error(
                "TMC [%s]: driver_TCOOLTHRS and coolstep_threshold are "
                "mutually exclusive — use one or the other. "
                "driver_TCOOLTHRS sets the raw TSTEP register value; "
                "coolstep_threshold sets it from a velocity in mm/s. "
                "Remove one of the two options." % (config.get_name(),))
        if self.thigh_pin is not None and self.high_velocity_threshold is not None:
            raise config.error(
                "TMC [%s]: driver_THIGH and high_velocity_threshold are "
                "mutually exclusive — use one or the other. "
                "driver_THIGH sets the raw TSTEP register value; "
                "high_velocity_threshold sets it from a velocity in mm/s. "
                "Remove one of the two options." % (config.get_name(),))
        # DRV_CONF VS-aware defaults
        self.filt_isense = config.getint('driver_FILT_ISENSE', None,
                                          minval=0, maxval=3)
        # Pinning a driver_<FIELD> for a register field this driver lacks is a
        # config error, caught here at config-load rather than as a late
        # set_field KeyError when autotune runs at stepper-enable time.
        self._reject_unsupported_pins(config)
    # Introspection --------------------------------------------------------
    def needs_home_current_change(self):
        return self.actual_current != self.req_home_current
    def needs_run_current_change(self):
        return self.actual_current != self.req_run_current
    def needs_hold_current_change(self, hold_current):
        return hold_current != self.req_hold_current
    # Requested-value mutators --------------------------------------------
    def set_home_current(self, new_home_current):
        self.req_home_current = min(self.max_current, new_home_current)
    def set_run_current(self, new_run_current):
        self.req_run_current = min(self.max_current, new_run_current)
    def set_hold_current(self, new_hold_current):
        self.req_hold_current = new_hold_current
    # Actual-value tracker -------------------------------------------------
    def set_actual_current(self, current):
        self.actual_current = current
    # Current programming -------------------------------------------------
    # set_current() is the unified entry point used both by homing
    # transitions and by the SET_TMC_CURRENT gcode command.  The
    # in-memory state (actual_current, req_hold_current) has to be
    # updated before apply_current() because subclass apply_current()
    # implementations read those fields back out to compute the
    # register values they program.  If apply_current() then raises
    # (driver-write error mid-batch), roll the in-memory state back so
    # a follow-up swap — including the post-home rollback to
    # run_current — actually re-issues instead of being short-circuited
    # by needs_*_current_change() reporting "already there".
    def set_current(self, run_current, hold_current, print_time, force=False):
        needs_run = run_current != self.actual_current
        needs_hold = self.needs_hold_current_change(hold_current)
        if not force and not needs_run and not needs_hold:
            return
        prev_actual = self.actual_current
        prev_hold = self.req_hold_current
        if needs_hold:
            self.set_hold_current(hold_current)
        self.set_actual_current(run_current)
        try:
            self.apply_current(print_time)
        except Exception:
            self.set_actual_current(prev_actual)
            if needs_hold:
                self.set_hold_current(prev_hold)
            raise
        # Re-tune chopper / hysteresis / stallguard for the new current.
        # Skips internally if autotune is not configured.  The tune
        # rewrites a number of register fields and flushes them, so
        # callers who forced a re-apply (SET_TMC_CURRENT) get a fresh
        # chopper profile that actually lands in hardware.
        self.tune_driver(run_current, force=force, print_time=print_time)
    # Homing-transition helpers -------------------------------------------
    # set_current_for_homing(pre_homing=True) swaps to the homing
    # profile (low-noise chopper + req_home_current);
    # set_current_for_homing(pre_homing=False) restores the run profile
    # (autotuned chopper + req_run_current).  Returns the dwell time
    # the driver requires after the swap (caller collects the max dwell
    # across all rails in a batch so the toolhead only dwells once).
    # The chopper-profile swap is unconditional even when home_current
    # equals run_current — homing-noise gain is independent of the
    # current swap.
    def set_current_for_homing(self, print_time, pre_homing):
        if pre_homing:
            self._homing_active = True
            dwell = 0.
            if self.needs_home_current_change():
                self.set_current(self.req_home_current,
                                 self.req_hold_current, print_time)
                dwell = self.current_change_dwell_time
            else:
                # home_current == actual_current → set_current() would
                # short-circuit, leaving IRUN/IHOLD/GLOBALSCALER at run
                # values for the homing window.  Drive apply_current()
                # directly so the per-driver _homing_active branch
                # writes the low-noise homing CS/GS/IRUN.
                self.apply_current(print_time)
            self._apply_homing_profile(print_time)
            return dwell
        # Post-homing: restore run profile first (so the run-tuned
        # chopper writes are in the bulk that follows), then swap
        # current back if needed.
        self._homing_active = False
        # Put chopper / coolstep / iholddelay back to their pre-homing
        # shadow values.  Runs unconditionally so the run profile is
        # restored even if autotune is disabled or the post-homing
        # tune_driver shortcuts via the _last_tuned_current cache.
        # Any subsequent tune_driver call in this path may overwrite
        # the restored fields with newly-derived values at the same
        # print_time — that is the intended behaviour when autotune
        # is active.
        self._restore_run_profile(print_time)
        dwell = 0.
        if self.needs_run_current_change():
            self.set_current(self.req_run_current,
                             self.req_hold_current, print_time)
            dwell = self.current_change_dwell_time
        else:
            # Same current as before homing.  apply_current() now
            # sees _homing_active=False and overwrites the homing
            # IRUN/IHOLD/GLOBALSCALER with run-profile values; the
            # forced tune_driver re-applies the run-profile chopper /
            # hysteresis / stallguard / coolstep over the homing ones
            # and flushes them at the same print_time so the swap is
            # atomic with the IRUN/IHOLD restore.
            self.apply_current(print_time)
            self.tune_driver(self.req_run_current, force=True,
                             print_time=print_time)
        return dwell
    def _apply_homing_profile(self, print_time=None):
        # Write the homing chopper / coolstep / iholddelay fields and
        # flush every register touched.  CS selection (the IRUN/IHOLD
        # bits during the homing window) is handled per-driver in
        # apply_current() paths that branch on self._homing_active.
        # Snapshot the pre-homing shadow value of every field we touch
        # so _restore_run_profile() can put them back unconditionally —
        # independent of whether autotune is active and independent of
        # the _last_tuned_current cache.
        snapshot = {}
        dirty = set()
        for field, default in HOMING_DEFAULTS.items():
            if field == 'cs':
                continue  # per-driver _calc_homing_current handles CS
            reg_name = self.fields.lookup_register(field, None)
            if reg_name is None:
                continue  # field absent on this driver type
            snapshot[field] = (self.fields.get_field(field), reg_name)
            override = getattr(self, 'homing_' + field, None)
            if override is None:
                override = default
            self.fields.set_field(field, override)
            dirty.add(reg_name)
        self._run_profile_snapshot = snapshot
        for reg in dirty:
            self.mcu_tmc.set_register(reg, self.fields.registers[reg],
                                      print_time)
    def _restore_run_profile(self, print_time=None):
        # Reverse of _apply_homing_profile().  Writes the snapshotted
        # pre-homing shadow values back into the field cache and flushes
        # every touched register at print_time.  Subsequent tune_driver
        # / apply_current calls in the post-homing path may overwrite
        # these with freshly-tuned values; that is intentional — this
        # restore guarantees a sane baseline even when autotune is off
        # (motor/voltage unset → tune_driver returns early) or the
        # _last_tuned_current cache would skip the re-tune.
        snapshot = self._run_profile_snapshot
        if not snapshot:
            return
        self._run_profile_snapshot = None
        dirty = set()
        for field, (value, reg_name) in snapshot.items():
            self.fields.set_field(field, value)
            dirty.add(reg_name)
        for reg in dirty:
            self.mcu_tmc.set_register(reg, self.fields.registers[reg],
                                      print_time)
    def get_homing_cs(self):
        return self.homing_cs if self.homing_cs is not None \
            else HOMING_DEFAULTS['cs']
    def _autotune_field_set_present(self):
        # Autotune derives the full StealthChop2 + CoolStep register set.
        # pwm_autograd (PWMCONF) and semin (COOLCONF) are present together only
        # on TMC2209/5160/2240; TMC2130 (pwm_ampl, no autograd), TMC2208 (no
        # COOLCONF) and TMC2660 (no PWMCONF) lack one or both — so they gate the
        # capable family precisely.
        return (self.fields.lookup_register("pwm_autograd", None) is not None
                and self.fields.lookup_register("semin", None) is not None)
    # Autotuning ----------------------------------------------------------
    # Recomputes chopper / PWM / hysteresis / stallguard / overvoltage
    # parameters for the current operating point, writes them to the
    # field shadow, and flushes the touched registers to hardware so the
    # tuned values take effect immediately rather than at the next driver
    # init.  Callers either (a) hand the new active current directly
    # (set_current path), (b) leave it 0 to use config_run_current
    # (driver-init / _do_enable path), or (c) pass force=True to re-apply
    # without a current change (SET_TMC_CURRENT / post-homing path).
    # `print_time` schedules the flush in the motion queue; pass None for
    # init / reconnect paths where there is no print-time anchor.
    # Returns silently when autotune is not configured (motor/voltage
    # missing) or unsupported (driver lacks PWMCONF, e.g. TMC2660).
    def tune_driver(self, new_current=0, force=False, print_time=None):
        if self.motor is None or self.voltage is None:
            return
        # Autotune derives the full StealthChop2 + CoolStep register set, so it
        # runs on the capable family (TMC2209/5160/2240) and skips drivers that
        # lack it (TMC2130/2208/2660).  Optional fields the 2209 does not expose
        # (tpfd, thigh, vhigh*, ...) are dropped per-write by
        # _set_field_if_present rather than gating the whole driver out.
        if not self._autotune_field_set_present():
            if not self._autotune_skip_logged:
                logging.info("tmc %s: autotune disabled — driver lacks the"
                             " StealthChop2/CoolStep field set (pwm_autograd+semin)",
                             self.name)
                self._autotune_skip_logged = True
            return
        # While the homing profile is active the chopper/coolstep
        # fields are deliberately set to HOMING_DEFAULTS; tuning would
        # immediately overwrite them.  The post-homing path explicitly
        # clears _homing_active before re-tuning, so this guard never
        # blocks a legitimate run-profile re-derive.
        if self._homing_active:
            return
        if new_current == 0:
            new_current = self.config_run_current
        if (not force and self._last_tuned_current is not None
                and abs(self._last_tuned_current - new_current) < 1e-6):
            return
        # The autotune math depends on the per-driver effective sense
        # resistor.  Subclasses (TMC2130/2660/5160) set self.sense_resistor
        # directly; TMC2240 derives it from Rref/KIFS at init time.  If a
        # subclass forgot, log and bail rather than crashing.
        if not hasattr(self, 'sense_resistor'):
            logging.info("tmc %s autotune skipped: no sense_resistor", self.name)
            return
        # The motor object holds the back-EMF / inductance / resistance
        # spec.  Resolved on first use; reported once if missing.
        try:
            motor_object = self.printer.lookup_object(
                "motor_constants " + self.motor)
        except self.printer.config_error as e:
            logging.error("tmc %s autotune: motor lookup failed (%s)",
                          self.name, str(e))
            return
        # Stepper rotation distance is needed for stallguard / coolstep
        # velocity thresholds.  Resolved lazily — _do_enable runs after
        # mcu_identify so by the time tune_driver runs the stepper is
        # up.
        if self.stepper is None:
            try:
                force_move = self.printer.lookup_object("force_move")
                self.stepper = force_move.lookup_stepper(self.name)
            except self.printer.config_error:
                # Pre-connect tuning attempts (rare); retry on next call.
                return
        try:
            self.driver_clock_frequency = (
                self.mcu_tmc.get_tmc_frequency()
                or DEFAULT_TMC_CLOCK_FREQUENCY)
        except AttributeError:
            self.driver_clock_frequency = DEFAULT_TMC_CLOCK_FREQUENCY
        logging.info("tmc %s autotune: tuning for %.3fA at %.0fV (clock %.3f MHz)",
                     self.name, new_current, self.voltage,
                     self.driver_clock_frequency / 1e6)
        self._configure_pwm(motor_object, new_current)
        new_tbl, new_toff = self._configure_spreadcycle(motor_object,
                                                        new_current)
        self._configure_hysteresis(motor_object, new_current,
                                   new_tbl, new_toff)
        self._configure_stallguard(new_current)
        self._configure_coolstep()
        self._configure_overvoltage()
        self._configure_highspeed(motor_object, new_current)
        self._configure_drvconf()
        # Flush every register the _configure_* helpers may have dirtied.
        # Without this, tuned chopper / PWM / threshold values stay only
        # in the shadow cache and reach the hardware at the next bulk
        # _init_registers (driver enable / non-critical reconnect) — so
        # post-homing retunes and SET_TMC_CURRENT force-reapplies would
        # not actually change driver behaviour until the next enable.
        for reg in TUNE_FLUSH_REGS:
            if reg in self.fields.registers:
                self.mcu_tmc.set_register(reg, self.fields.registers[reg],
                                          print_time)
        if self._skipped_fields and not self._autotune_skip_logged:
            logging.info("tmc %s: autotune for %s — skipping fields absent on"
                         " this driver: %s", self.name, self.driver_type,
                         ", ".join(sorted(self._skipped_fields)))
            self._autotune_skip_logged = True
        # Cache the operating point only after the tune has actually
        # landed in hardware.  An earlier failure path (motor lookup,
        # stepper resolution, sense_resistor missing) returns without
        # touching the cache, so a subsequent retry with the same
        # current still runs the tune end-to-end instead of being
        # short-circuited by a "tuning has been attempted" flag.
        self._last_tuned_current = new_current
    def _set_velocity_field(self, field, velocity):
        # tcoolthrs / thigh accept a TSTEP threshold; convert from a
        # rotation-distance velocity.  Skips silently if the field is
        # absent on this driver.
        if self.fields.lookup_register(field, None) is None:
            return
        tstep = TMCtstepHelper(self.mcu_tmc, velocity, pstepper=self.stepper)
        self.fields.set_field(field, tstep)
    # driver_<OPTION> → register field for fields that some autotune-family
    # drivers lack (TMC2209 has none of these). Pinning one for an absent field
    # is a config error rather than a silent no-op.
    _OPTIONAL_PIN_FIELDS = {
        'driver_TPFD': 'tpfd', 'driver_SGT': 'sgt', 'driver_THIGH': 'thigh',
        'driver_VHIGHFS': 'vhighfs', 'driver_VHIGHCHM': 'vhighchm',
        'driver_FASTSTANDSTILL': 'faststandstill',
        'driver_SMALL_HYSTERESIS': 'small_hysteresis', 'driver_SFILT': 'sfilt',
    }

    def _reject_unsupported_pins(self, config):
        for pin_name, field_name in self._OPTIONAL_PIN_FIELDS.items():
            if (config.get(pin_name, None) is not None
                    and self.fields.lookup_register(field_name, None) is None):
                raise config.error(
                    "%s: %s is not supported on %s (driver has no '%s' register"
                    " field)" % (config.get_name(), pin_name, self.driver_type,
                                 field_name))
    def _set_field_if_present(self, field_name, value):
        # Write a tuning field only when this driver actually has it. Optional
        # fields (tpfd, thigh, vhighfs, vhighchm, faststandstill,
        # small_hysteresis, sfilt, sgt) are absent on the TMC2209/2208 register
        # set; autotune skips them here. Explicit driver_<FIELD> pins for absent
        # fields are rejected at config-load time (see __init__), so reaching
        # this point with an absent field is always an autotune-derived value
        # that is safe to drop.
        if self.fields.lookup_register(field_name, None) is None:
            self._skipped_fields.add(field_name)
            return
        self.fields.set_field(field_name, value)
    def _configure_pwm(self, motor_object, new_current):
        # Per-field: user pin wins; otherwise derive from motor model or goal.
        if self.pwm_freq is not None:
            pwm_freq = self.pwm_freq
            calc_freq = motor_object.pwmfreq_to_hz(
                pwm_freq, fclk=self.driver_clock_frequency)
        else:
            pwm_freq, calc_freq = motor_object.pwmfreq(
                fclk=self.driver_clock_frequency,
                target=self.pwm_freq_target)
        self._calc_freq = calc_freq  # consumed by _configure_spreadcycle

        pwmgrad = (self.pwm_grad if self.pwm_grad is not None
                   else motor_object.pwmgrad(
                       volts=self.voltage,
                       fclk=self.driver_clock_frequency))
        pwmofs = (self.pwm_ofs if self.pwm_ofs is not None
                  else motor_object.pwmofs(
                      volts=self.voltage, current=new_current))

        pwm_autoscale = (self.pwm_autoscale if self.pwm_autoscale is not None
                         else True)
        pwm_autograd = (self.pwm_autograd if self.pwm_autograd is not None
                        else True)
        pwm_reg = (self.pwm_reg_pin if self.pwm_reg_pin is not None
                   else self._derive_pwm_reg())
        pwm_lim = (self.pwm_lim_pin if self.pwm_lim_pin is not None
                   else self._derive_pwm_lim())
        # Pin precedence: raw driver_TPWMTHRS > stealthchop_threshold velocity
        # > goal-default.
        if self.tpwmthrs_pin is not None:
            tpwmthrs = self.tpwmthrs_pin
        elif self.stealthchop_threshold is not None:
            tpwmthrs = TMCtstepHelper(self.mcu_tmc, self.stealthchop_threshold,
                                      pstepper=self.stepper)
        else:
            tpwmthrs = self._derive_tpwmthrs(motor_object, new_current)

        # Set en_pwm_mode (or en_spreadcycle for TMC2208/2209) symmetrically:
        # 1 when TPWMTHRS allows StealthChop to engage (threshold < 0xfffff),
        # 0 when TPWMTHRS is pegged at 0xfffff ("never cross into StealthChop").
        # The write must be symmetric so a driver_TPWMTHRS: 0xfffff pin can
        # override an en_pwm_mode=1 already written by TMCStealthchopHelper —
        # otherwise the register state is incoherent (StealthChop "armed" but
        # threshold set to never trigger).
        reg = self.fields.lookup_register("en_pwm_mode", None)
        if reg is not None:
            self.fields.set_field("en_pwm_mode",
                                  1 if tpwmthrs != 0xfffff else 0)
        else:
            # TMC2208/2209 family: en_spreadcycle=0 enables StealthChop
            if self.fields.lookup_register("en_spreadcycle", None) is not None:
                self.fields.set_field("en_spreadcycle",
                                      0 if tpwmthrs != 0xfffff else 1)

        logging.info("tmc %s autotune (goal=%s): pwm_freq=%d (~%.1f kHz)"
                     " pwmgrad=%d pwmofs=%d pwm_reg=%d pwm_lim=%d"
                     " tpwmthrs=%d",
                     self.name, self.tuning_goal, pwm_freq, calc_freq / 1e3,
                     pwmgrad, pwmofs, pwm_reg, pwm_lim, tpwmthrs)

        self.fields.set_field("pwm_freq", pwm_freq)
        self.fields.set_field("pwm_autoscale", pwm_autoscale)
        self.fields.set_field("pwm_autograd", pwm_autograd)
        self.fields.set_field("pwm_grad", pwmgrad)
        self.fields.set_field("pwm_ofs", pwmofs)
        self.fields.set_field("pwm_reg", pwm_reg)
        self.fields.set_field("pwm_lim", pwm_lim)
        self.fields.set_field("tpwmthrs", tpwmthrs)

    def _derive_pwm_reg(self):
        # PI response speed per goal
        return {'performance': 15, 'balanced': 8, 'silent': 4}[self.tuning_goal]

    def _derive_pwm_lim(self):
        # Transition cap per goal
        return {'performance': 4, 'balanced': 8, 'silent': 12}[self.tuning_goal]

    def _derive_small_hysteresis(self):
        # Silent uses small_hysteresis for smoother microstep transitions at
        # the cost of slight torque-ripple sensitivity.
        return self.tuning_goal == 'silent'

    def _derive_tpwmthrs(self, motor_object, current):
        # StealthChop crossover velocity, in TSTEP units, per tuning goal:
        #   performance: 0xfffff (StealthChop never)
        #   balanced:    0.3 * vmaxpwm (StealthChop low-speed only)
        #   silent:      1.2 * vmaxpwm (StealthChop until physical limit)
        if self.tuning_goal == 'performance':
            return 0xfffff
        maxpwmrps = motor_object.maxpwmrps(
            volts=self.voltage, current=current,
            fclk=self.driver_clock_frequency)
        rdist = self.stepper.get_rotation_distance()[0]
        ratio = 0.3 if self.tuning_goal == 'balanced' else 1.2
        velocity = ratio * maxpwmrps * rdist
        # Convert velocity to TSTEP via the existing helper
        return TMCtstepHelper(self.mcu_tmc, velocity, pstepper=self.stepper)

    def _configure_spreadcycle(self, motor_object, new_current):
        calc_freq = self._calc_freq    # set by _configure_pwm just above
        ncycles = int(math.ceil(self.driver_clock_frequency / calc_freq))
        # Goal-aware comparator blank time (AN-001: 1-2 us starting point; the
        # TMC5160 drives external MOSFETs whose switching ringing the blank
        # time must cover — see AN-006).  performance favours a higher chopper
        # frequency (lower TBL); the quiet goals favour ringing margin and
        # wave quality (higher TBL).  A driver_TBL pin overrides; the TOFF=1
        # / min_tbl_at_min_toff correction below still applies.
        if self.tbl is not None:
            tbl = self.tbl
        else:
            tbl = {'performance': 1, 'balanced': 2, 'silent': 2}[self.tuning_goal]
        tblank = 16.0 * (1.5 ** tbl) / self.driver_clock_frequency
        # If the user pinned driver_TOFF, honour it; otherwise search for
        # the smallest TOFF whose lowest chopper frequency stays at or
        # below chopper_freq_target (default 20 kHz so we sit just above
        # the audible band).
        if self.toff is None:
            # Goal-specific chopper frequency defaults: performance=20 kHz
            # (just above audible), balanced=35 kHz (reduced SpreadCycle
            # hissing), silent=45 kHz (ultrasonic).  A user-configured
            # chopper_freq_target always wins.
            _goal_chopper_defaults = {
                'performance': 20e3, 'balanced': 35e3, 'silent': 45e3}
            target = (self.chopper_freq_target
                      or _goal_chopper_defaults[self.tuning_goal])
            toff = 0
            while True:
                tsd_duty = (24.0 + 32.0 * toff) / self.driver_clock_frequency
                duty = (new_current * 0.7 / self.voltage
                        + tblank / (tblank + tsd_duty))
                chop_low = 1. / ((2. + 4. * duty) * tsd_duty)
                if chop_low <= target or toff >= 15:
                    break
                toff += 1
            # Back off by one so chop_low sits just above the audible-band
            # target rather than just below.  Floor at 1 — TOFF=0 is the
            # CHOPCONF driver-disable code per the TMC datasheet, and
            # the user-facing driver_TOFF is also bounded minval=1.  At
            # very high chopper_freq_target / current combinations the
            # first iteration already meets the constraint and the
            # decrement would otherwise land on 0.
            toff = max(toff - 1, 1)
        else:
            toff = self.toff
        # TOFF=1 (shortest off-time) requires a minimum blank time per the
        # datasheet (CHOPCONF): TMC2209 mandates TBL>=2, TMC5160 allows TBL>=1.
        # When BOTH fields are user-pinned, raise an error instead of silently
        # correcting — silent correction is only safe when autotune chose one
        # or both values.
        if toff == 1 and tbl < self.min_tbl_at_min_toff:
            if self.toff is not None and self.tbl is not None:
                raise self.printer.config_error(
                    "tmc %s: driver_TOFF=1 requires driver_TBL>=%d on %s per the"
                    " datasheet (CHOPCONF); raise driver_TBL or remove one of the"
                    " pins and let autotune choose."
                    % (self.name, self.min_tbl_at_min_toff, self.driver_type))
            tbl = self.min_tbl_at_min_toff
            tblank = 16.0 * (1.5 ** tbl) / self.driver_clock_frequency
        tsd_duty = (24.0 + 32.0 * toff) / self.driver_clock_frequency
        # Allocate the remaining cycle time to TPFD (passive fast decay).
        # The (×2 - tblank) accounts for the two slow-decay phases per
        # cycle minus blanking already counted.
        # Silent goal forces TPFD=0: passive fast decay interacts with
        # ultrasonic StealthChop and can introduce audible resonances, so it
        # is disabled entirely for the silent goal.
        pfdcycles = (ncycles
                     - (tsd_duty * 2. - tblank) * self.driver_clock_frequency)
        if self.tpfd is not None:
            tpfd = self.tpfd
        elif self.tuning_goal == 'silent':
            tpfd = 0
        else:
            tpfd = max(0, min(15, int(math.ceil(pfdcycles / 128.))))
        logging.info("tmc %s autotune: tbl=%d toff=%d tpfd=%d",
                     self.name, tbl, toff, tpfd)
        self._set_field_if_present("tpfd", tpfd)
        self.fields.set_field("tbl", tbl)
        self.fields.set_field("toff", toff)
        return tbl, toff
    def _hysteresis_scale(self, current):
        # Current-scale (CS) used for the hysteresis derivation. Default: the
        # explicit driver_cs override, or None to let motor_constants.hysteresis
        # auto-derive from the sense resistor. The vsense-based current path
        # overrides this so hstrt/hend match the real programmed CS.
        return self.cs
    def _configure_hysteresis(self, motor_object, new_current,
                              new_tbl, new_toff):
        # Allow partial pinning: if the user set only one of driver_HSTRT /
        # driver_HEND, the pinned field wins and the unpinned field falls
        # back to the autotune-derived value.  HSTRT and HEND are independent
        # CHOPCONF sub-fields with no hardware constraint that mandates
        # setting them as a pair; partial overrides
        # are safe and allow fine-grained tuning on top of autotune.
        hstrt_auto, hend_auto = motor_object.hysteresis(
            name=self.name, extra=self.extra_hysteresis,
            fclk=self.driver_clock_frequency, volts=self.voltage,
            current=new_current, tbl=new_tbl, toff=new_toff,
            rsense=self.sense_resistor,
            scale=self._hysteresis_scale(new_current))
        hstrt = self.hstrt if self.hstrt is not None else hstrt_auto
        hend = self.hend if self.hend is not None else hend_auto
        self.fields.set_field("hstrt", hstrt)
        self.fields.set_field("hend", hend)
    def _configure_stallguard(self, new_current):
        if self.fields.lookup_register("sg4_thrs", None) is not None:
            if self.sg4_thrs is not None:
                self.fields.set_field("sg4_thrs", self.sg4_thrs)
                self.fields.set_field("sg4_filt_en", True)
        elif self.fields.lookup_register("sgthrs", None) is not None:
            if self.sg4_thrs is not None:
                self.fields.set_field("sgthrs", self.sg4_thrs)
        if self.sgt is not None:
            self._set_field_if_present("sgt", self.sgt)
        # tcoolthrs precedence: raw driver_TCOOLTHRS > velocity-form
        # coolstep_threshold > goal-default
        if self.tcoolthrs_pin is not None:
            self.fields.set_field("tcoolthrs", self.tcoolthrs_pin)
        elif self.coolstep_threshold is not None:
            self._set_velocity_field("tcoolthrs", self.coolstep_threshold)
        else:
            # Goal-default: 0.75 rev/s — below this StallGuard becomes
            # noisy due to insufficient back-EMF.  Same for all goals.
            coolthrs = 0.75 * self.stepper.get_rotation_distance()[0]
            self._set_velocity_field("tcoolthrs", coolthrs)
    def _configure_coolstep(self):
        # Pin-respecting CoolStep configuration.  Goal-aware defaults; silent
        # sets semin=0 to disable CoolStep entirely (no current-modulation
        # noise).
        defaults = self._coolstep_defaults_for_goal()

        semin = self.semin if self.semin is not None else defaults['semin']
        semax = self.semax if self.semax is not None else defaults['semax']
        seup = self.seup if self.seup is not None else defaults['seup']
        sedn = self.sedn if self.sedn is not None else defaults['sedn']
        seimin = (self.seimin if self.seimin is not None
                  else defaults['seimin'])
        sfilt = self.sfilt if self.sfilt is not None else defaults['sfilt']
        iholddelay = (self.iholddelay if self.iholddelay is not None
                      else defaults['iholddelay'])

        # GCONF flags (faststandstill, small_hysteresis) are handled here
        # rather than in a dedicated _configure_gconf so the writes ride
        # along with the CoolStep register flush.
        faststandstill = (self.faststandstill if self.faststandstill is not None
                          else True)
        small_hysteresis = (self.small_hysteresis
                            if self.small_hysteresis is not None
                            else self._derive_small_hysteresis())

        self._set_field_if_present("faststandstill", faststandstill)
        self._set_field_if_present("small_hysteresis", small_hysteresis)
        self.fields.set_field("semin", semin)
        self.fields.set_field("semax", semax)
        self.fields.set_field("seup", seup)
        self.fields.set_field("sedn", sedn)
        self.fields.set_field("seimin", seimin)
        self._set_field_if_present("sfilt", sfilt)
        self.fields.set_field("iholddelay", iholddelay)

    def _coolstep_defaults_for_goal(self):
        # CoolStep defaults per goal.  Silent sets semin=0 which disables
        # CoolStep entirely; the other fields then have no effect but are
        # written for deterministic register state.
        if self.tuning_goal == 'silent':
            return {'semin': 0, 'semax': 0, 'seup': 0, 'sedn': 0,
                    'seimin': 1, 'sfilt': 1, 'iholddelay': 12}
        if self.tuning_goal == 'balanced':
            return {'semin': 2, 'semax': 4, 'seup': 3, 'sedn': 2,
                    'seimin': 1, 'sfilt': 1, 'iholddelay': 10}
        # performance
        return {'semin': 2, 'semax': 4, 'seup': 3, 'sedn': 2,
                'seimin': 1, 'sfilt': 0, 'iholddelay': 12}
    def _configure_overvoltage(self):
        if self.overvoltage_vth is not None:
            # 0.009732 V/LSB per TMC2240 datasheet.  Guard against drivers
            # that lack the field (e.g. TMC5160 has no OTW_OV_VTH register).
            if self.fields.lookup_register("overvoltage_vth", None) is None:
                return
            vth = int(self.overvoltage_vth / 0.009732)
            self.fields.set_field("overvoltage_vth", vth)
    def _configure_highspeed(self, motor_object, new_current):
        # Pin-respecting high-speed mode + thigh.
        if self.tuning_goal == 'performance':
            maxpwmrps = motor_object.maxpwmrps(
                volts=self.voltage, current=new_current,
                fclk=self.driver_clock_frequency)
            rdist = self.stepper.get_rotation_distance()[0]
            thigh_default_velocity = 1.2 * maxpwmrps * rdist
        else:
            thigh_default_velocity = None  # balanced/silent: no threshold

        # THIGH precedence: raw > velocity > goal-default
        if self.thigh_pin is not None:
            self._set_field_if_present("thigh", self.thigh_pin)
        elif self.high_velocity_threshold is not None:
            self._set_velocity_field("thigh", self.high_velocity_threshold)
        elif thigh_default_velocity is not None:
            self._set_velocity_field("thigh", thigh_default_velocity)
        else:
            # balanced/silent: no high-velocity threshold; matches
            # Klipper upstream TMCVhighHelper default of THIGH=0 (CoolStep
            # window remains open up to physical limits).
            self._set_field_if_present("thigh", 0)

        # vhighfs / vhighchm: Goal-aware (only performance enables)
        vhighfs_default = (self.tuning_goal == 'performance')
        vhighchm_default = (self.tuning_goal == 'performance')
        vhighfs = (self.vhighfs if self.vhighfs is not None
                   else vhighfs_default)
        vhighchm = (self.vhighchm if self.vhighchm is not None
                    else vhighchm_default)
        self._set_field_if_present("vhighfs", vhighfs)
        self._set_field_if_present("vhighchm", vhighchm)

        # multistep_filt rides with the high-speed register flush; it is a
        # GCONF flag but logically tied to high-speed motion smoothing.
        multistep_filt = (self.multistep_filt if self.multistep_filt is not None
                          else True)
        self.fields.set_field("multistep_filt", multistep_filt)
    def _configure_drvconf(self):
        # VS-aware filt_isense default. filt_isense reduces sense-line
        # ringing artefacts; at high VS (>52V on TMC5160) PCB ringing is
        # large enough that the 1us filter measurably cleans up hysteresis
        # regulation.
        if self.fields.lookup_register("filt_isense", None) is None:
            return  # not a TMC5160-class driver
        if self.filt_isense is not None:
            value = self.filt_isense
        else:
            value = 1 if self.voltage and self.voltage > 52.0 else 0
        self.fields.set_field("filt_isense", value)
    # Subclass hook --------------------------------------------------------
    def apply_current(self, print_time):
        raise NotImplementedError(
            "BaseTMCCurrentHelper subclass must implement apply_current()")
