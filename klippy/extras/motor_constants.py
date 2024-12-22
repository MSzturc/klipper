import math, logging
# Motor database, contains specifications for stepper motors.

# R is coil resistance, Ohms
# L is coil inductance, Henries
# T is holding torque, Nm (be careful about units here)
# I is nominal rated current, Amps


class MotorConstants:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.R = config.getfloat('resistance', minval=0.)
        self.L = config.getfloat('inductance', minval=0.)
        self.T = config.getfloat('holding_torque', minval=0.)
        self.S = config.getint('steps_per_revolution', minval=0)
        self.I = config.getfloat('max_current', minval=0.)
        self.cbemf = self.T / (2.0 * self.I)
    def pwmgrad(self, fclk=12.5e6, steps=0, volts=24.0):
        if steps==0:
            steps=self.S
        return int(math.ceil(self.cbemf * 2 * math.pi * fclk  * 1.46 / (volts * 256.0 * steps)))
    def pwmofs(self, volts=24.0, current=0.0):
        I = current if current > 0.0 else self.I
        return int(math.ceil(374 * self.R * I / volts))
    # Maximum revolutions per second before PWM maxes out.
    def maxpwmrps(self, fclk=12.5e6, steps=0, volts=24.0, current=0.0):
        if steps==0:
            steps=self.S
        return (255 - self.pwmofs(volts, current)) / ( math.pi * self.pwmgrad(fclk, steps))
    def hysteresis(self,name, extra, fclk, volts, current, tbl, toff, rsense,scale):
        if current > 0.0:
            I = current * math.sqrt(2)
        else:
            I = self.I
        
        logging.info(f"tmc {name} ::: calculating hysteresis")
        logging.info(f"tmc {name} ::: Ipeak: {I}")
        logging.info(f"tmc {name} ::: Voltage: {volts}")
        logging.info(f"tmc {name} ::: Sense Resistor: {rsense}")

        tblank = 16.0 * (1.5 ** tbl) / fclk
        logging.info(f"tmc {name} ::: tblank: {tblank}")
        tsd = (12.0 + 32.0 * toff) / fclk
        logging.info(f"tmc {name} ::: tsd: {tsd}")
        dcoilblank = volts * tblank / self.L
        logging.info(f"tmc {name} ::: dcoilblank: {dcoilblank}")
        dcoilsd = self.R * I * 2.0 * tsd / self.L
        logging.info(f"tmc {name} ::: dcoilsd: {dcoilsd}")
        if scale > 0:
            cs = scale
        else:
            cs = max(0, min(31, int(math.ceil(rsense * 32 * I / 0.32) - 1)))
        logging.info(f"tmc {name} ::: cs: {cs}")
        hysteresis = extra + int(max(0.5 + ((dcoilblank + dcoilsd) * 2 * 248 * (cs + 1) / I ) / 32 - 8, -2))
        logging.info(f"tmc {name} ::: hysteresis: {hysteresis}")
        hstrt = max(min(hysteresis, 8), 1)
        hend = min(hysteresis - hstrt, 12)

        return hstrt - 1, hend + 3


def load_config_prefix(config):
    return MotorConstants(config)

