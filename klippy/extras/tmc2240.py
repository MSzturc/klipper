# TMC2240 configuration
#
# Copyright (C) 2018-2023  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2023  Alex Voinea <voinea.dragos.alexandru@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import bus, tmc, tmc2130, tmc_uart

TMC_FREQUENCY=12500000.

Registers = {
    "GCONF":            0x00,
    "GSTAT":            0x01,
    "IFCNT":            0x02,
    "NODECONF":         0x03,
    "IOIN":             0x04,
    "DRV_CONF":         0x0A,
    "GLOBALSCALER":     0x0B,
    "IHOLD_IRUN":       0x10,
    "TPOWERDOWN":       0x11,
    "TSTEP":            0x12,
    "TPWMTHRS":         0x13,
    "TCOOLTHRS":        0x14,
    "THIGH":            0x15,
    "DIRECT_MODE":      0x2D,
    "ENCMODE":          0x38,
    "X_ENC":            0x39,
    "ENC_CONST":        0x3A,
    "ENC_STATUS":       0x3B,
    "ENC_LATCH":        0x3C,
    "ADC_VSUPPLY_AIN":  0x50,
    "ADC_TEMP":         0x51,
    "OTW_OV_VTH":       0x52,
    "MSLUT0":           0x60,
    "MSLUT1":           0x61,
    "MSLUT2":           0x62,
    "MSLUT3":           0x63,
    "MSLUT4":           0x64,
    "MSLUT5":           0x65,
    "MSLUT6":           0x66,
    "MSLUT7":           0x67,
    "MSLUTSEL":         0x68,
    "MSLUTSTART":       0x69,
    "MSCNT":            0x6A,
    "MSCURACT":         0x6B,
    "CHOPCONF":         0x6C,
    "COOLCONF":         0x6D,
    "DRV_STATUS":       0x6F,
    "PWMCONF":          0x70,
    "PWM_SCALE":        0x71,
    "PWM_AUTO":         0x72,
    "SG4_THRS":         0x74,
    "SG4_RESULT":       0x75,
    "SG4_IND":          0x76,
}

ReadRegisters = [
    "GCONF", "GSTAT", "IOIN", "DRV_CONF", "GLOBALSCALER", "IHOLD_IRUN",
    "TPOWERDOWN", "TSTEP", "TPWMTHRS", "TCOOLTHRS", "THIGH", "ADC_VSUPPLY_AIN",
    "ADC_TEMP", "OTW_OV_VTH", "MSCNT", "MSCURACT", "CHOPCONF", "COOLCONF",
    "DRV_STATUS", "PWMCONF", "PWM_SCALE", "PWM_AUTO", "SG4_THRS", "SG4_RESULT",
    "SG4_IND"
]

Fields = {}
Fields["COOLCONF"] = {
    "semin":                    0x0F << 0,
    "seup":                     0x03 << 5,
    "semax":                    0x0F << 8,
    "sedn":                     0x03 << 13,
    "seimin":                   0x01 << 15,
    "sgt":                      0x7F << 16,
    "sfilt":                    0x01 << 24
}
Fields["CHOPCONF"] = {
    "toff":                     0x0F << 0,
    "hstrt":                    0x07 << 4,
    "hend":                     0x0F << 7,
    "fd3":                      0x01 << 11,
    "disfdcc":                  0x01 << 12,
    "chm":                      0x01 << 14,
    "tbl":                      0x03 << 15,
    "vhighfs":                  0x01 << 18,
    "vhighchm":                 0x01 << 19,
    "tpfd":                     0x0F << 20, # midrange resonances
    "mres":                     0x0F << 24,
    "intpol":                   0x01 << 28,
    "dedge":                    0x01 << 29,
    "diss2g":                   0x01 << 30,
    "diss2vs":                  0x01 << 31
}
Fields["DRV_STATUS"] = {
    "sg_result":                0x3FF << 0,
    "s2vsa":                    0x01 << 12,
    "s2vsb":                    0x01 << 13,
    "stealth":                  0x01 << 14,
    "fsactive":                 0x01 << 15,
    "cs_actual":                0x1F << 16,
    "stallguard":               0x01 << 24,
    "ot":                       0x01 << 25,
    "otpw":                     0x01 << 26,
    "s2ga":                     0x01 << 27,
    "s2gb":                     0x01 << 28,
    "ola":                      0x01 << 29,
    "olb":                      0x01 << 30,
    "stst":                     0x01 << 31
}
Fields["GCONF"] = {
    "faststandstill":           0x01 << 1,
    "en_pwm_mode":              0x01 << 2,
    "multistep_filt":           0x01 << 3,
    "shaft":                    0x01 << 4,
    "diag0_error":              0x01 << 5,
    "diag0_otpw":               0x01 << 6,
    "diag0_stall":              0x01 << 7,
    "diag1_stall":              0x01 << 8,
    "diag1_index":              0x01 << 9,
    "diag1_onstate":            0x01 << 10,
    "diag0_pushpull":           0x01 << 12,
    "diag1_pushpull":           0x01 << 13,
    "small_hysteresis":         0x01 << 14,
    "stop_enable":              0x01 << 15,
    "direct_mode":              0x01 << 16
}
Fields["GSTAT"] = {
    "reset":                    0x01 << 0,
    "drv_err":                  0x01 << 1,
    "uv_cp":                    0x01 << 2,
    "register_reset":           0x01 << 3,
    "vm_uvlo":                  0x01 << 4
}
Fields["GLOBALSCALER"] = {
    "globalscaler":             0xFF << 0
}
Fields["IHOLD_IRUN"] = {
    "ihold":                    0x1F << 0,
    "irun":                     0x1F << 8,
    "iholddelay":               0x0F << 16,
    "irundelay":                0x0F << 24
}
Fields["IOIN"] = {
    "step":                     0x01 << 0,
    "dir":                      0x01 << 1,
    "encb":                     0x01 << 2,
    "enca":                     0x01 << 3,
    "drv_enn":                  0x01 << 4,
    "encn":                     0x01 << 5,
    "uart_en":                  0x01 << 6,
    "comp_a":                   0x01 << 8,
    "comp_b":                   0x01 << 9,
    "comp_a1_a2":               0x01 << 10,
    "comp_b1_b2":               0x01 << 11,
    "output":                   0x01 << 12,
    "ext_res_det":              0x01 << 13,
    "ext_clk":                  0x01 << 14,
    "adc_err":                  0x01 << 15,
    "silicon_rv":               0x07 << 16,
    "version":                  0xFF << 24
}
Fields["MSLUT0"] = { "mslut0": 0xffffffff }
Fields["MSLUT1"] = { "mslut1": 0xffffffff }
Fields["MSLUT2"] = { "mslut2": 0xffffffff }
Fields["MSLUT3"] = { "mslut3": 0xffffffff }
Fields["MSLUT4"] = { "mslut4": 0xffffffff }
Fields["MSLUT5"] = { "mslut5": 0xffffffff }
Fields["MSLUT6"] = { "mslut6": 0xffffffff }
Fields["MSLUT7"] = { "mslut7": 0xffffffff }
Fields["MSLUTSEL"] = {
    "x3":                       0xFF << 24,
    "x2":                       0xFF << 16,
    "x1":                       0xFF << 8,
    "w3":                       0x03 << 6,
    "w2":                       0x03 << 4,
    "w1":                       0x03 << 2,
    "w0":                       0x03 << 0,
}
Fields["MSLUTSTART"] = {
    "start_sin":                0xFF << 0,
    "start_sin90":              0xFF << 16,
    "offset_sin90":             0xFF << 24,
}
Fields["MSCNT"] = {
    "mscnt":                    0x3ff << 0
}
Fields["MSCURACT"] = {
    "cur_a":                    0x1ff << 0,
    "cur_b":                    0x1ff << 16
}
Fields["PWM_AUTO"] = {
    "pwm_ofs_auto":             0xff << 0,
    "pwm_grad_auto":            0xff << 16
}
Fields["PWMCONF"] = {
    "pwm_ofs":                  0xFF << 0,
    "pwm_grad":                 0xFF << 8,
    "pwm_freq":                 0x03 << 16,
    "pwm_autoscale":            0x01 << 18,
    "pwm_autograd":             0x01 << 19,
    "freewheel":                0x03 << 20,
    "pwm_meas_sd_enable":       0x01 << 22,
    "pwm_dis_reg_stst":         0x01 << 23,
    "pwm_reg":                  0x0F << 24,
    "pwm_lim":                  0x0F << 28
}
Fields["PWM_SCALE"] = {
    "pwm_scale_sum":            0x3ff << 0,
    "pwm_scale_auto":           0x1ff << 16
}
Fields["TPOWERDOWN"] = {
    "tpowerdown":               0xff << 0
}
Fields["TPWMTHRS"] = {
    "tpwmthrs":                 0xfffff << 0
}
Fields["TCOOLTHRS"] = {
    "tcoolthrs":                0xfffff << 0
}
Fields["TSTEP"] = {
    "tstep":                    0xfffff << 0
}
Fields["THIGH"] = {
    "thigh":                    0xfffff << 0
}
Fields["DRV_CONF"] = {
    "current_range":            0x03 << 0,
    "slope_control":            0x03 << 4
}
Fields["ADC_VSUPPLY_AIN"] = {
    "adc_vsupply":              0x1fff << 0,
    "adc_ain":                  0x1fff << 16
}
Fields["ADC_TEMP"] = {
    "adc_temp":                 0x1fff << 0
}
Fields["OTW_OV_VTH"] = {
    "overvoltage_vth":          0x1fff << 0,
    "overtempprewarning_vth":   0x1fff << 16
}
Fields["SG4_THRS"] = {
    "sg4_thrs":                 0xFF << 0,
    "sg4_filt_en":              0x01 << 8,
    "sg4_angle_offset":         0x01 << 9
}
Fields["SG4_RESULT"] = {
    "sg4_result":               0x3FF << 0
}
Fields["SG4_IND"] = {
    "sg4_ind_0":                0xFF << 0,
    "sg4_ind_1":                0xFF << 8,
    "sg4_ind_2":                0xFF << 16,
    "sg4_ind_3":                0xFF << 24
}


SignedFields = ["cur_a", "cur_b", "sgt", "pwm_scale_auto", "offset_sin90"]

FieldFormatters = dict(tmc2130.FieldFormatters)
FieldFormatters.update({
    "s2vsa":            (lambda v: "1(ShortToSupply_A!)" if v else ""),
    "s2vsb":            (lambda v: "1(ShortToSupply_B!)" if v else ""),
    "adc_temp":         (lambda v: "0x%04x(%.1fC)" % (v, ((v - 2038) / 7.7))),
    "adc_vsupply":      (lambda v: "0x%04x(%.3fV)" % (v, v * 0.009732)),
    "adc_ain":          (lambda v: "0x%04x(%.3fmV)" % (v, v * 0.3052)),
    "overvoltage_vth":  (lambda v: "0x%04x(%.3fV)" % (v, v * 0.009732)),
    "overtempprewarning_vth": (lambda v:
                               "0x%04x(%.1fC)" % (v, ((v - 2038) / 7.7))),
})


######################################################################
# TMC stepper current config helper
######################################################################

# Floor for the homing-profile GLOBALSCALER on TMC2240.  Same rationale
# as TMC5160 — below ~31 the regulator becomes coarse, hurting
# StallGuard cleanliness more than the small under-current it produces.
HOMING_GLOBALSCALER_MIN_ROBUST_2240 = 31

class TMC2240CurrentHelper(tmc.BaseTMCCurrentHelper):
    def __init__(self, config, mcu_tmc):
        # Rref determines the per-range full-scale current on the TMC2240
        # and is hardware-fixed by the carrier board.  Mandatory: there
        # is no safe default.  Evaluated before the base class parses
        # run/hold/home because max_current depends on it.
        self.Rref = config.getfloat('rref',
                                    minval=12000., maxval=60000.)
        # TMC2240 has no physical sense resistor (Rref + KIFS table
        # define the I-to-cs relationship), so stepstick_type's
        # sense_resistor entry is meaningless for this driver.  The
        # max_current cap, however, is still a real hardware limit of
        # the carrier board — pass required=False so a stepstick_type
        # without sense_resistor is accepted, then cap max_cur with the
        # board limit so run/hold/home_current and SET_TMC_CURRENT can
        # never exceed the stepstick's rated current.
        _, lookup_max = tmc.resolve_sense_resistor(config, required=False)
        max_cur = self._get_ifs_rms_for(3)
        if lookup_max is not None:
            max_cur = min(max_cur, lookup_max)
        super().__init__(config, mcu_tmc, max_cur)
        # Auto-pick the smallest CURRENT_RANGE that covers every current
        # the driver can be programmed to during normal operation:
        # req_run_current, req_home_current, and req_hold_current.  An
        # auto-range derived from req_run_current alone silently
        # under-delivers home_current whenever home_current exceeds the
        # active range's ifs_rms — _calc_homing_current solves IRUN/GS
        # against _get_ifs_rms(active_range) and saturates at the range
        # ceiling without raising.  The user can still lock a higher
        # range for headroom; the config minval ties to the auto-picked
        # floor so users cannot under-range any of the three currents.
        # Cap req_hold_current at req_run_current for range selection:
        # when hold_current is omitted the base class defaults it to
        # max_current (a sentinel, not a real user request), which would
        # otherwise force auto_range to 3 regardless of run_current.
        auto_range = self._calc_current_range(
            max(self.req_run_current,
                self.req_home_current,
                min(self.req_hold_current, self.req_run_current)))
        current_range = config.getint('current_range', auto_range,
                                      minval=auto_range, maxval=3)
        self.fields.set_field("current_range", current_range)
        # When 'driver_cs' is unset (default) the IRUN bits are computed
        # from the requested current; when set, the user-specified value
        # is used directly and GLOBALSCALER is solved for it.  Mirrors
        # the TMC5160 driver_cs convention.
        self.cs = config.getint('driver_cs', None, minval=0, maxval=31)
        # Effective sense resistor for the autotune hysteresis math.
        # TMC2240 has no physical Rsens; its KIFS table at the active
        # current_range plus Rref defines the same I-to-cs relationship
        # as Rsens does on TMC5160.  Solving Rsens_equiv from
        # IRUN_RMS = (cs+1) * VREF_5160 / (32 * sqrt(2) * Rsens) versus
        # IRUN_RMS = (cs+1) * ifs_rms / 32 yields:
        self.sense_resistor = 0.325 / (math.sqrt(2.) * self._get_ifs_rms())
        gscaler, irun, ihold = self._calc_current(
            self.req_run_current, self.req_hold_current)
        self.fields.set_field("globalscaler", gscaler)
        self.fields.set_field("ihold", ihold)
        self.fields.set_field("irun", irun)
    def _get_ifs_rms_for(self, current_range):
        KIFS = [11750., 24000., 36000., 36000.]
        return (KIFS[current_range] / self.Rref) / math.sqrt(2.)
    def _get_ifs_rms(self, current_range=None):
        if current_range is None:
            current_range = self.fields.get_field("current_range")
        return self._get_ifs_rms_for(current_range)
    def _calc_current_range(self, current):
        for current_range in range(4):
            if current <= self._get_ifs_rms(current_range):
                break
        return current_range
    def _calc_globalscaler(self, current):
        # Solve GLOBALSCALER given the IRUN value chosen by
        # _calc_current_bits.  Ceiling rounding keeps the resulting RMS
        # current at or above the requested value, never below.
        cs = self._calc_current_bits(current)
        ifs_rms = self._get_ifs_rms()
        globalscaler = int(math.ceil(
            (current * 256. * 32.) / (ifs_rms * (1. + cs))))
        if self.cs is not None and (globalscaler < 32 or globalscaler > 256):
            # See TMC5160._calc_globalscaler for the full rationale.
            # In short: with user-pinned driver_cs the auto-CS guarantee
            # of "GS within [32, 256]" is gone, and silent-clamping in
            # either direction multi-x mis-programs the actual current.
            raise self.printer.config_error(
                "TMC %s: driver_cs=%d cannot deliver %.3fA at the"
                " active current_range (ifs_rms=%.3fA; raw"
                " GLOBALSCALER=%d, valid range 32..256). Choose a"
                " different driver_cs, raise current_range, or omit"
                " driver_cs to auto-pick."
                % (self.name, cs, current, ifs_rms, globalscaler))
        globalscaler = max(32, globalscaler)
        if globalscaler >= 256:
            globalscaler = 0
        return globalscaler
    def _calc_current_bits(self, current):
        if self.cs is None:
            # Auto: pick the smallest IRUN that, with GLOBALSCALER fixed
            # at its maximum, can deliver the requested RMS current at
            # the active current_range.  TMC2240's KIFS table already
            # speaks RMS, so no sqrt(2)/VREF factors here.
            ifs_rms = self._get_ifs_rms()
            cs = int(math.ceil(32. * current / ifs_rms) - 1)
        else:
            cs = self.cs
        return max(0, min(31, cs))
    def _calc_current(self, run_current, hold_current):
        gscaler = self._calc_globalscaler(run_current)
        irun = self._calc_current_bits(run_current)
        # Scale IHOLD as a fraction of IRUN so hold/run ratios survive
        # driver-cs changes; floor-clamp to IRUN so a misconfiguration
        # cannot raise hold above run.
        ihold = int(min((hold_current / run_current) * irun, irun))
        return gscaler, irun, ihold
    def _calc_homing_current(self, homing_current):
        # Low-noise homing CS / GLOBALSCALER pick — TMC2240 mirror of
        # the TMC5160 method.  TMC2240 uses ifs_rms (already RMS;
        # KIFS-based) instead of VREF/sense_resistor, so the formulas
        # drop the VREF and sqrt(2) factors.  Picks the smallest CS
        # that fits the requested current at GS in
        # [HOMING_GLOBALSCALER_MIN_ROBUST_2240..255], starting from
        # HOMING_DEFAULTS['cs'].  Sets IRUN == IHOLD for cleanest
        # StallGuard signal during the homing window.
        ifs_rms = self._get_ifs_rms()
        if self.homing_cs is not None:
            cs = max(0, min(31, self.homing_cs))
        else:
            cs_default = max(0, min(31, tmc.HOMING_DEFAULTS['cs']))
            cs = cs_default
            for candidate in range(cs_default, 32):
                gs_candidate = (homing_current * 256. * 32.
                                / (ifs_rms * (candidate + 1)))
                if gs_candidate <= 255.:
                    cs = candidate
                    break
            else:
                cs = 31
        gs_raw = int(round(homing_current * 256. * 32.
                           / (ifs_rms * (cs + 1))))
        if gs_raw >= 256:
            globalscaler = 0  # 0 encodes 256 (full scale)
        elif gs_raw < HOMING_GLOBALSCALER_MIN_ROBUST_2240:
            globalscaler = HOMING_GLOBALSCALER_MIN_ROBUST_2240
        else:
            globalscaler = gs_raw
        irun = max(0, min(31, cs))
        ihold = irun
        logging.info(
            "tmc %s homing: cs=%d gs=%d (raw %d) irun=%d ihold=%d "
            "ifs_rms=%.3fA (target %.3fA)",
            self.name, cs,
            globalscaler if globalscaler else 256, gs_raw,
            irun, ihold, ifs_rms, homing_current)
        return globalscaler, irun, ihold
    def _calc_current_from_field(self, field_name):
        ifs_rms = self._get_ifs_rms()
        globalscaler = self.fields.get_field("globalscaler")
        if not globalscaler:
            globalscaler = 256
        bits = self.fields.get_field(field_name)
        return globalscaler * (bits + 1) * ifs_rms / (256. * 32.)
    def get_current(self):
        ifs_rms = self._get_ifs_rms()
        run_current = self._calc_current_from_field("irun")
        hold_current = self._calc_current_from_field("ihold")
        # Cap at min(active-range full-scale, stepstick max).  ifs_rms
        # alone caps to the active CURRENT_RANGE; self.max_current adds
        # the stepstick_type / Rref-derived board limit so SET_TMC_CURRENT
        # cannot drive a higher run current than the carrier supports.
        return (run_current, hold_current, self.req_hold_current,
                min(ifs_rms, self.max_current), self.req_home_current)
    def apply_current(self, print_time):
        # _homing_active explicit (not derived) so the homing-profile
        # path runs even when home_current == run_current.
        if self._homing_active:
            gscaler, irun, ihold = self._calc_homing_current(
                self.actual_current)
        else:
            gscaler, irun, ihold = self._calc_current(
                self.actual_current, self.req_hold_current)
        val = self.fields.set_field("globalscaler", gscaler)
        self.mcu_tmc.set_register("GLOBALSCALER", val, print_time)
        self.fields.set_field("ihold", ihold)
        val = self.fields.set_field("irun", irun)
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)


######################################################################
# TMC2240 printer object
######################################################################

class TMC2240:
    def __init__(self, config):
        # Setup mcu communication
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        if config.get("uart_pin", None) is not None:
            # use UART for communication
            self.mcu_tmc = tmc_uart.MCU_TMC_uart(config, Registers, self.fields,
                                                 7, TMC_FREQUENCY)
        else:
            # Use SPI bus for communication
            self.mcu_tmc = tmc2130.MCU_TMC_SPI(config, Registers, self.fields,
                                               TMC_FREQUENCY)
        # Allow virtual pins to be created
        tmc.TMCVirtualPinHelper(config, self.mcu_tmc)
        # Register commands
        self.current_helper = TMC2240CurrentHelper(config, self.mcu_tmc)
        cmdhelper = tmc.TMCCommandHelper(config, self.mcu_tmc,
                                         self.current_helper)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_status = cmdhelper.get_status
        # Setup basic register values
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        self.fields.set_config_field(config, "offset_sin90", 0)
        tmc.TMCStealthchopHelper(config, self.mcu_tmc)
        tmc.TMCVcoolthrsHelper(config, self.mcu_tmc)
        tmc.TMCVhighHelper(config, self.mcu_tmc)
        # Allow other registers to be set from the config
        set_config_field = self.fields.set_config_field
        #   GCONF
        set_config_field(config, "multistep_filt", True)
        #   CHOPCONF
        set_config_field(config, "toff", 3)
        set_config_field(config, "hstrt", 5)
        set_config_field(config, "hend", 2)
        set_config_field(config, "fd3", 0)
        set_config_field(config, "disfdcc", 0)
        set_config_field(config, "chm", 0)
        set_config_field(config, "tbl", 2)
        set_config_field(config, "vhighfs", 0)
        set_config_field(config, "vhighchm", 0)
        set_config_field(config, "tpfd", 4)
        set_config_field(config, "diss2g", 0)
        set_config_field(config, "diss2vs", 0)
        #   COOLCONF
        set_config_field(config, "semin", 0)
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "sfilt", 0)
        #   IHOLDIRUN
        set_config_field(config, "iholddelay", 6)
        set_config_field(config, "irundelay", 4)
        #   PWMCONF
        set_config_field(config, "pwm_ofs", 29)
        set_config_field(config, "pwm_grad", 0)
        set_config_field(config, "pwm_freq", 0)
        set_config_field(config, "pwm_autoscale", True)
        set_config_field(config, "pwm_autograd", True)
        set_config_field(config, "freewheel", 0)
        set_config_field(config, "pwm_reg", 4)
        set_config_field(config, "pwm_lim", 12)
        #   TPOWERDOWN
        set_config_field(config, "tpowerdown", 10)
        #   SG4_THRS
        set_config_field(config, "sg4_thrs", 0)
        set_config_field(config, "sg4_angle_offset", 1)
        #   DRV_CONF
        set_config_field(config, "slope_control", 0)

def load_config_prefix(config):
    return TMC2240(config)
