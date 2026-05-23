# TMC5160 configuration
#
# Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from . import bus, tmc, tmc2130

TMC_FREQUENCY=12000000.

Registers = {
    "GCONF":            0x00,
    "GSTAT":            0x01,
    "IFCNT":            0x02,
    "SLAVECONF":        0x03,
    "IOIN":             0x04,
    "X_COMPARE":        0x05,
    "OTP_READ":         0x07,
    "FACTORY_CONF":     0x08,
    "SHORT_CONF":       0x09,
    "DRV_CONF":         0x0A,
    "GLOBALSCALER":     0x0B,
    "OFFSET_READ":      0x0C,
    "IHOLD_IRUN":       0x10,
    "TPOWERDOWN":       0x11,
    "TSTEP":            0x12,
    "TPWMTHRS":         0x13,
    "TCOOLTHRS":        0x14,
    "THIGH":            0x15,
    "RAMPMODE":         0x20,
    "XACTUAL":          0x21,
    "VACTUAL":          0x22,
    "VSTART":           0x23,
    "A1":               0x24,
    "V1":               0x25,
    "AMAX":             0x26,
    "VMAX":             0x27,
    "DMAX":             0x28,
    "D1":               0x2A,
    "VSTOP":            0x2B,
    "TZEROWAIT":        0x2C,
    "XTARGET":          0x2D,
    "VDCMIN":           0x33,
    "SW_MODE":          0x34,
    "RAMP_STAT":        0x35,
    "XLATCH":           0x36,
    "ENCMODE":          0x38,
    "X_ENC":            0x39,
    "ENC_CONST":        0x3A,
    "ENC_STATUS":       0x3B,
    "ENC_LATCH":        0x3C,
    "ENC_DEVIATION":    0x3D,
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
    "LOST_STEPS":       0x73,
}

ReadRegisters = [
    "GCONF", "CHOPCONF", "GSTAT", "DRV_STATUS", "FACTORY_CONF", "IOIN",
    "LOST_STEPS", "MSCNT", "MSCURACT", "OTP_READ", "PWM_SCALE",
    "PWM_AUTO", "TSTEP"
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
Fields["DRV_CONF"] = {
    "bbmtime":                  0x1F << 0,
    "bbmclks":                  0x0F << 8,
    "otselect":                 0x03 << 16,
    "drvstrength":              0x03 << 18,
    "filt_isense":              0x03 << 20,
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
Fields["FACTORY_CONF"] = {
    "factory_conf":             0x1F << 0
}
Fields["SHORT_CONF"] = {
    "s2vs_level":               0x0F << 0,
    "s2g_level":                0x0F << 8,
    "short_filter":             0x03 << 16,
    "shortdelay":               0x01 << 18,
}
Fields["GCONF"] = {
    "recalibrate":              0x01 << 0,
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
    "diag1_steps_skipped":      0x01 << 11,
    "diag0_int_pushpull":       0x01 << 12,
    "diag1_poscomp_pushpull":   0x01 << 13,
    "small_hysteresis":         0x01 << 14,
    "stop_enable":              0x01 << 15,
    "direct_mode":              0x01 << 16,
    "test_mode":                0x01 << 17
}
Fields["GSTAT"] = {
    "reset":                    0x01 << 0,
    "drv_err":                  0x01 << 1,
    "uv_cp":                    0x01 << 2
}
Fields["GLOBALSCALER"] = {
    "globalscaler":             0xFF << 0
}
Fields["IHOLD_IRUN"] = {
    "ihold":                    0x1F << 0,
    "irun":                     0x1F << 8,
    "iholddelay":               0x0F << 16
}
Fields["IOIN"] = {
    "refl_step":                0x01 << 0,
    "refr_dir":                 0x01 << 1,
    "encb_dcen_cfg4":           0x01 << 2,
    "enca_dcin_cfg5":           0x01 << 3,
    "drv_enn":                  0x01 << 4,
    "enc_n_dco_cfg6":           0x01 << 5,
    "sd_mode":                  0x01 << 6,
    "swcomp_in":                0x01 << 7,
    "version":                  0xFF << 24
}
Fields["LOST_STEPS"] = {
    "lost_steps":               0xfffff << 0
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
}
Fields["MSCNT"] = {
    "mscnt":                    0x3ff << 0
}
Fields["MSCURACT"] = {
    "cur_a":                    0x1ff << 0,
    "cur_b":                    0x1ff << 16
}
Fields["OTP_READ"] = {
    "otp_fclktrim":             0x1f << 0,
    "otp_s2_level":             0x01 << 5,
    "otp_bbm":                  0x01 << 6,
    "otp_tbl":                  0x01 << 7
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
    "pwm_reg":                  0x0F << 24,
    "pwm_lim":                  0x0F << 28
}
Fields["PWM_SCALE"] = {
    "pwm_scale_sum":            0xff << 0,
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

SignedFields = ["cur_a", "cur_b", "sgt", "xactual", "vactual", "pwm_scale_auto"]

FieldFormatters = dict(tmc2130.FieldFormatters)
FieldFormatters.update({
    "s2vsa":            (lambda v: "1(ShortToSupply_A!)" if v else ""),
    "s2vsb":            (lambda v: "1(ShortToSupply_B!)" if v else ""),
})


######################################################################
# TMC stepper current config helper
######################################################################

VREF = 0.325
MAX_CURRENT = 10.000 # Maximum dependent on board, but 10 is safe sanity check

# Lowest GLOBALSCALER value the homing-profile current search will
# accept.  Below ~31 the regulator becomes coarse and noisy; clamping
# up sacrifices a small amount of accuracy at the absolute current
# value but produces a much cleaner StallGuard signal.
HOMING_GLOBALSCALER_MIN_ROBUST = 31


def _validate_short_conf(config, voltage, s2vs_level_pin, s2g_level_pin):
    """VS-aware SHORT_CONF validation.

    Raises config.error if VS > 52 V and driver_S2G_LEVEL < 12 (datasheet §6.3).
    Emits a logging.warning if VS > 52 V but SHORT_CONF is not explicitly set.
    """
    if (voltage is not None and voltage > 52.0
            and s2g_level_pin is not None and s2g_level_pin < 12):
        raise config.error(
            "TMC5160 [%s]: at VS=%.1fV (>52V) the driver_S2G_LEVEL "
            "must be >=12 per datasheet \xa76.3 to avoid false short-"
            "to-GND triggers; you set %d. Either raise driver_S2G_LEVEL "
            "to >=12, or lower the supply voltage."
            % (config.get_name(), voltage, s2g_level_pin))
    if (voltage is not None and voltage > 52.0
            and s2vs_level_pin is None and s2g_level_pin is None):
        logging.warning(
            "TMC5160 [%s]: VS=%.1fV (>52V) but SHORT_CONF is not "
            "explicitly programmed. The chip's OTP defaults may "
            "permit false short-to-GND triggers at this voltage. "
            "Set both driver_S2VS_LEVEL (>=4) and driver_S2G_LEVEL "
            "(>=12) to silence this warning. (Datasheet \xa76.3)",
            config.get_name(), voltage)


def _validate_chopconf(config):
    """Guard CHOPCONF boundary constraints unconditionally.

    The TOFF=1/TBL=0 validation in tune_driver / _configure_spreadcycle only
    fires for autotune-ON configs (motor: + voltage: present).  A user with
    autotune OFF who pins driver_TOFF=1 + driver_TBL=0 would otherwise get
    silent mis-programming.  That combination is a TMC5160 datasheet limit
    (§5.2 CHOPCONF), not an autotune artefact — it must be enforced at
    config-load time regardless of autotune state.
    """
    toff_pin = config.getint('driver_TOFF', None, minval=1, maxval=15)
    tbl_pin  = config.getint('driver_TBL',  None, minval=0, maxval=3)

    # TOFF=1 with TBL=0 is invalid per TMC5160 datasheet §5.2 CHOPCONF.
    # Only raise when both are explicitly user-pinned; autotune chose one or
    # both, in which case _configure_spreadcycle handles the correction.
    if toff_pin == 1 and tbl_pin == 0:
        raise config.error(
            "tmc5160 %s: driver_TOFF=1 with driver_TBL=0 is invalid per"
            " the TMC5160 datasheet (\xa75.2 CHOPCONF); set driver_TBL"
            " to 1 or higher, or remove driver_TBL and let autotune"
            " choose." % (config.get_name(),))


class TMC5160CurrentHelper(tmc.BaseTMCCurrentHelper):
    def __init__(self, config, mcu_tmc):
        # Resolve before super().__init__ so a stepstick_type with a
        # lower current ceiling caps the effective max_current the base
        # class uses to validate run/hold/home_current at config-time.
        sense_resistor, lookup_max = tmc.resolve_sense_resistor(config)
        max_current = (MAX_CURRENT if lookup_max is None
                       else min(MAX_CURRENT, lookup_max))
        super().__init__(config, mcu_tmc, max_current)
        self.sense_resistor = sense_resistor
        # When 'driver_cs' is unset (default) the IRUN bits are computed
        # from the requested current; when set, the user-specified value
        # is used directly and GLOBALSCALER is solved for it.
        self.cs = config.getint('driver_cs', None, minval=0, maxval=31)
        gscaler, irun, ihold = self._calc_current(
            self.req_run_current, self.req_hold_current)
        self.fields.set_field("globalscaler", gscaler)
        self.fields.set_field("ihold", ihold)
        self.fields.set_field("irun", irun)
    def _calc_globalscaler(self, current):
        # Solve GLOBALSCALER given the IRUN value chosen by
        # _calc_current_bits.  Ceiling rounding keeps the resulting RMS
        # current at or above the requested value, never below.
        cs = self._calc_current_bits(current)
        globalscaler = int(math.ceil(
            (current * 256. * math.sqrt(2.) * self.sense_resistor * 32.)
            / (VREF * (1. + cs))))
        if self.cs is not None and (globalscaler < 32 or globalscaler > 256):
            # User-pinned driver_cs.  The auto-CS path's "smallest CS
            # that fits at GS<=256" guarantee is gone, so any
            # current+cs combination whose raw GLOBALSCALER falls
            # outside the encodable [32, 256] window is unrepresentable.
            # Silent-clamping below 32 over-currents (driver delivers
            # 32/256 instead of the requested fraction); silent-folding
            # above 256 to the GS=0 (full-scale) encoding under-currents
            # by a large factor (delivered I_RMS scales with cs+1, not
            # the requested current).  Both directions are silent
            # multi-x mis-programs at register-write time.
            raise self.printer.config_error(
                "TMC %s: driver_cs=%d cannot deliver %.3fA at"
                " sense_resistor=%.4f (raw GLOBALSCALER=%d, valid"
                " range 32..256). Choose a different driver_cs"
                " or omit the option to auto-pick."
                % (self.name, cs, current, self.sense_resistor,
                   globalscaler))
        globalscaler = max(32, globalscaler)
        if globalscaler >= 256:
            globalscaler = 0
        return globalscaler
    def _calc_current_bits(self, current):
        if self.cs is None:
            # Auto: pick the smallest IRUN that, with GLOBALSCALER fixed
            # at its maximum, can deliver Ipeak = current * sqrt(2) given
            # the configured sense resistor.  Subtracting 1 converts
            # "32 levels" to the 0..31 register encoding.
            ipeak = current * math.sqrt(2.)
            cs = int(math.ceil(self.sense_resistor * 32. * ipeak / 0.32) - 1)
        else:
            cs = self.cs
        return max(0, min(31, cs))
    def _calc_current(self, run_current, hold_current):
        gscaler = self._calc_globalscaler(run_current)
        irun = self._calc_current_bits(run_current)
        # Scale IHOLD as a fraction of IRUN so that hold_current/run_current
        # ratios survive driver-CS changes.  Floor-clamped to IRUN so a
        # misconfiguration can never raise hold above run.
        ihold = int(min((hold_current / run_current) * irun, irun))
        return gscaler, irun, ihold
    def _calc_homing_current(self, homing_current):
        # Low-noise homing CS / GLOBALSCALER pick.  Goal: maximise
        # GLOBALSCALER (lower bits-quantisation noise on IRUN) by using
        # the smallest CS that fits the requested homing current at
        # GS <= 255.  Independent of driver_cs (run profile).  Sets
        # IRUN == IHOLD for the cleanest StallGuard signal.
        Ipeak = homing_current * math.sqrt(2.)
        Rsens = self.sense_resistor
        if self.homing_cs is not None:
            cs = max(0, min(31, self.homing_cs))
        else:
            # Start at HOMING_DEFAULTS['cs']; auto-bump only when the
            # requested current cannot fit at that CS (would force
            # GS > 255).
            cs_default = max(0, min(31, tmc.HOMING_DEFAULTS['cs']))
            cs = cs_default
            for candidate in range(cs_default, 32):
                gs_candidate = (Ipeak * 32. * 256. * Rsens
                                / ((candidate + 1) * VREF))
                if gs_candidate <= 255.:
                    cs = candidate
                    break
            else:
                cs = 31
        gs_raw = int(round(Ipeak * 32. * 256. * Rsens / ((cs + 1) * VREF)))
        if gs_raw >= 256:
            globalscaler = 0  # 0 encodes 256 (full scale) per datasheet
        elif gs_raw < HOMING_GLOBALSCALER_MIN_ROBUST:
            # Sub-floor would be coarse; clamp up.  Resulting current
            # will be slightly below target — safer than below-floor
            # quantisation noise.
            globalscaler = HOMING_GLOBALSCALER_MIN_ROBUST
        else:
            globalscaler = gs_raw
        irun = max(0, min(31, cs))
        ihold = irun
        logging.info(
            "tmc %s homing: cs=%d gs=%d (raw %d) irun=%d ihold=%d (target %.3fA)",
            self.name, cs,
            globalscaler if globalscaler else 256, gs_raw,
            irun, ihold, homing_current)
        return globalscaler, irun, ihold
    def _calc_current_from_field(self, field_name):
        globalscaler = self.fields.get_field("globalscaler")
        if not globalscaler:
            globalscaler = 256
        bits = self.fields.get_field(field_name)
        return (globalscaler * (bits + 1) * VREF
                / (256. * 32. * math.sqrt(2.) * self.sense_resistor))
    def get_current(self):
        run_current = self._calc_current_from_field("irun")
        hold_current = self._calc_current_from_field("ihold")
        return (run_current, hold_current, self.req_hold_current,
                self.max_current, self.req_home_current)
    def apply_current(self, print_time):
        # _homing_active selects formula explicitly because comparing
        # actual_current to req_home_current is unreliable when
        # home_current and run_current are equal (e.g. a 0.5A Z stepper
        # uses the same value for both and would otherwise look
        # already-applied).
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
# TMC5160 printer object
######################################################################

class TMC5160:
    def __init__(self, config):
        # Setup mcu communication
        self.fields = tmc.FieldHelper(Fields, SignedFields, FieldFormatters)
        self.mcu_tmc = tmc2130.MCU_TMC_SPI(config, Registers, self.fields,
                                           TMC_FREQUENCY)
        # Allow virtual pins to be created
        tmc.TMCVirtualPinHelper(config, self.mcu_tmc)
        # Register commands
        self.current_helper = TMC5160CurrentHelper(config, self.mcu_tmc)
        cmdhelper = tmc.TMCCommandHelper(config, self.mcu_tmc,
                                         self.current_helper)
        cmdhelper.setup_register_dump(ReadRegisters)
        self.get_phase_offset = cmdhelper.get_phase_offset
        self.get_status = cmdhelper.get_status
        # Setup basic register values
        tmc.TMCWaveTableHelper(config, self.mcu_tmc)
        tmc.TMCStealthchopHelper(config, self.mcu_tmc)
        # driver_TPWMTHRS raw pin must override the stealthchop_threshold-
        # derived value written by TMCStealthchopHelper when autotune is
        # disabled (tune_driver returns early without motor:/voltage:, so
        # _configure_pwm never runs for autotune-OFF configs).
        # set_config_field with default=None writes only when the user set
        # the field explicitly, so a config without driver_TPWMTHRS is
        # unaffected.
        set_config_field = self.fields.set_config_field
        set_config_field(config, "tpwmthrs", None)
        # Mirror the _configure_pwm en_pwm_mode symmetry rule here for the
        # autotune-OFF path: when TPWMTHRS != 0xfffff StealthChop may engage
        # (en_pwm_mode=1); when TPWMTHRS == 0xfffff it never triggers
        # (en_pwm_mode=0).  Without this, autotune-OFF + driver_TPWMTHRS
        # leaves en_pwm_mode at the chip's reset value, which is incoherent
        # with the user-pinned threshold.  driver_EN_PWM_MODE still wins via
        # set_config_field's getint path if the user sets it explicitly.
        _tpwmthrs_pin = config.getint("driver_TPWMTHRS", None,
                                      minval=0, maxval=0xfffff)
        if _tpwmthrs_pin is not None:
            set_config_field(config, "en_pwm_mode",
                             1 if _tpwmthrs_pin != 0xfffff else 0)
        tmc.TMCVcoolthrsHelper(config, self.mcu_tmc)
        # driver_TCOOLTHRS raw pin must override the coolstep_threshold-
        # derived value written by TMCVcoolthrsHelper when autotune is
        # disabled.  Mutual-exclusion with coolstep_threshold is already
        # enforced in BaseTMCCurrentHelper.__init__ (config error).
        set_config_field(config, "tcoolthrs", None)
        tmc.TMCVhighHelper(config, self.mcu_tmc)
        # driver_THIGH raw pin must override the high_velocity_threshold-
        # derived value written by TMCVhighHelper when autotune is disabled.
        # Mutual-exclusion with high_velocity_threshold is already enforced
        # in BaseTMCCurrentHelper.__init__ (config error).
        set_config_field(config, "thigh", None)
        # Allow other registers to be set from the config
        #   GCONF
        set_config_field(config, "multistep_filt", True)
        # FASTSTANDSTILL and SMALL_HYSTERESIS (GCONF flags) are only written
        # inside _configure_coolstep(), which runs as part of tune_driver().
        # When autotune is disabled those helpers never run, leaving the
        # fields at their hardware reset values (both 0).  Provide static
        # pin-respecting defaults so autotune-OFF configs are not stuck at
        # reset values: faststandstill=True matches the autotune default;
        # small_hysteresis=False is the conservative reset-value default
        # (autotune would otherwise derive 0/1 from tuning_goal).
        set_config_field(config, "faststandstill", True)
        set_config_field(config, "small_hysteresis", False)
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
        set_config_field(config, "semin", 0)    # page 52
        set_config_field(config, "seup", 0)
        set_config_field(config, "semax", 0)
        set_config_field(config, "sedn", 0)
        set_config_field(config, "seimin", 0)
        set_config_field(config, "sgt", 0)
        set_config_field(config, "sfilt", 0)
        #   DRV_CONF
        set_config_field(config, "drvstrength", 0)
        set_config_field(config, "bbmclks", 4)
        set_config_field(config, "bbmtime", 0)
        # filt_isense static default must reflect voltage even when motor: is
        # absent — tune_driver returns early without motor, so
        # _configure_drvconf never runs for a voltage-only config.  Re-read
        # voltage here (before BaseTMCCurrentHelper reads it in __init__) so
        # the static shadow is correct from the first _init_registers call.
        _filt_voltage = config.getfloat('voltage', None, minval=0., maxval=60.)
        _filt_isense_pin = config.getint('driver_FILT_ISENSE', None,
                                         minval=0, maxval=3)
        if _filt_isense_pin is not None:
            _filt_isense_default = _filt_isense_pin
        elif _filt_voltage is not None and _filt_voltage > 52.0:
            _filt_isense_default = 1
        else:
            _filt_isense_default = 0
        set_config_field(config, "filt_isense", _filt_isense_default)
        set_config_field(config, "otselect", 0)
        #   SHORT_CONF — write-only register; the chip's OTP defaults are
        # not readable, so we only program it when the user has set both
        # of the level fields explicitly.  Setting individual sub-fields
        # without both s2vs_level and s2g_level would corrupt the chip's
        # short-detection thresholds, so that combination is rejected.
        s2vs_level_pin = config.getint("driver_S2VS_LEVEL", None, 4, 15)
        s2g_level_pin = config.getint("driver_S2G_LEVEL", None, 2, 15)
        voltage = config.getfloat('voltage', None, minval=0., maxval=60.)

        # VS-aware SHORT_CONF validation
        _validate_short_conf(config, voltage, s2vs_level_pin, s2g_level_pin)
        # CHOPCONF boundary constraints (also enforced for autotune-OFF)
        _validate_chopconf(config)

        if s2vs_level_pin is not None and s2g_level_pin is not None:
            set_config_field(config, "s2vs_level", 6)
            set_config_field(config, "s2g_level", 6)
            set_config_field(config, "short_filter", 1)
            set_config_field(config, "shortdelay", 0)
        elif any(config.get("driver_%s" % field, None) is not None
                 for field in Fields["SHORT_CONF"].keys()):
            raise config.error(
                "driver_S2VS_LEVEL and driver_S2G_LEVEL must both be set "
                "to update SHORT_CONF on TMC5160 [%s]" % (config.get_name(),))
        #   IHOLDIRUN
        set_config_field(config, "iholddelay", 6)
        #   PWMCONF
        set_config_field(config, "pwm_ofs", 30)
        set_config_field(config, "pwm_grad", 0)
        set_config_field(config, "pwm_freq", 0)
        set_config_field(config, "pwm_autoscale", True)
        set_config_field(config, "pwm_autograd", True)
        set_config_field(config, "freewheel", 0)
        set_config_field(config, "pwm_reg", 4)
        set_config_field(config, "pwm_lim", 12)
        #   TPOWERDOWN
        set_config_field(config, "tpowerdown", 10)

def load_config_prefix(config):
    return TMC5160(config)
