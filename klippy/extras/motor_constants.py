# Motor database — stepper-motor specifications used by the TMC autotuning
# subsystem (see klippy/extras/tmc.py).
#
# R is coil resistance, Ohms
# L is coil inductance, Henries
# T is holding torque, Nm (be careful about units here)
# I is nominal rated current, Amps
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging


class MotorConstants:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.R = config.getfloat('resistance', above=0.)
        self.L = config.getfloat('inductance', above=0.)
        self.T = config.getfloat('holding_torque', above=0.)
        self.S = config.getint('steps_per_revolution', minval=1)
        self.I = config.getfloat('max_current', above=0.)
        # cbemf is the back-EMF constant of the motor in Volts per
        # radian/second.  Source: TMC5160A page 64 ("Understanding the
        # back EMF constant of a motor").
        self.cbemf = self.T / (2.0 * self.I)

    # First approximation for PWM_GRAD.  Source: TMC5160A page 63.
    def pwmgrad(self, fclk=12.5e6, steps=0, volts=24.0):
        if steps == 0:
            steps = self.S
        return int(math.ceil(self.cbemf * 2 * math.pi * fclk * 1.46
                             / (volts * 256.0 * steps)))

    # Velocity-based scaling.  Source: TMC5160A page 63.
    # current = target RMS current; defaults to motor rating.
    def pwmofs(self, volts=24.0, current=0.0):
        I = current if current > 0.0 else self.I
        return int(math.ceil(374 * self.R * I / volts))

    # Maximum revolutions per second before PWM saturates.
    def maxpwmrps(self, fclk=12.5e6, steps=0, volts=24.0, current=0.0):
        if steps == 0:
            steps = self.S
        # volts must be forwarded to pwmgrad; without it pwmgrad silently uses
        # its 24V default and produces a wrong threshold for 48V / 56V configs.
        return ((255 - self.pwmofs(volts, current))
                / (math.pi * self.pwmgrad(fclk, steps, volts)))

    # Calculates the PWM frequency given clock frequency and target.
    # Source: TMC5160A page 60 ("Choices of PWM frequency for stealthChop").
    def pwmfreq(self, fclk=12.5e6, target=55e3):
        best = None
        for prescaler, factor in [(3, 2./410), (2, 2./512),
                                  (1, 2./683), (0, 2./1024)]:
            calculated_freq = fclk * factor
            if calculated_freq < target:
                return prescaler, round(calculated_freq, 1)
            best = (prescaler, round(calculated_freq, 1))
        # No generated frequency is below the target (target is very low);
        # return the lowest available prescaler setting rather than 0,
        # which would cause a ZeroDivisionError in _configure_spreadcycle.
        return best

    # Inverse of pwmfreq(): given a chosen prescaler enum, return the
    # actual chopper frequency in Hz.  Used by _configure_pwm when the
    # user pins driver_PWM_FREQ — the rest of autotune still needs the
    # corresponding calc_freq.
    def pwmfreq_to_hz(self, prescaler, fclk=12.5e6):
        factor = {3: 2./410, 2: 2./512, 1: 2./683, 0: 2./1024}.get(prescaler)
        if factor is None:
            raise ValueError("pwm_freq prescaler must be 0..3, got %r"
                             % (prescaler,))
        return round(fclk * factor, 1)

    # Compute the chopper hysteresis (hstrt, hend) for the given operating
    # point.  Returns the two register values.  See TMC5160A datasheet for
    # the underlying formulas.
    def hysteresis(self, name, extra, fclk, volts, current,
                   tbl, toff, rsense, scale):
        I = (current if current > 0.0 else self.I) * math.sqrt(2)
        tblank = 16.0 * (1.5 ** tbl) / fclk
        tsd = (12.0 + 32.0 * toff) / fclk
        dcoilblank = volts * tblank / self.L
        dcoilsd = self.R * I * 2.0 * tsd / self.L
        # `scale` is either an explicit driver_cs override (0..31) or
        # None to auto-compute from sense resistor and Ipeak.  cs=0 is
        # a valid pinned value (lowest 1/32 of full-scale) — only None
        # triggers auto-compute.
        if scale is not None:
            cs = scale
        else:
            cs = max(0, min(31, int(math.ceil(rsense * 32 * I / 0.32) - 1)))
        hysteresis = extra + int(
            max(0.5 + ((dcoilblank + dcoilsd) * 2 * 248 * (cs + 1) / I) / 32
                - 8, -2))
        hstrt = max(min(hysteresis, 8), 1)
        hend = min(hysteresis - hstrt, 12)
        logging.info("tmc %s autotune: Ipeak=%.3fA tblank=%.2eus tsd=%.2eus"
                     " cs=%d hysteresis=%d hstrt=%d hend=%d",
                     name, I, tblank * 1e6, tsd * 1e6,
                     cs, hysteresis, hstrt - 1, hend + 3)
        return hstrt - 1, hend + 3


def load_config_prefix(config):
    return MotorConstants(config)
