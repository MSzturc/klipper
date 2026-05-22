# Stepstick / driver-carrier database — sense-resistor and current limit per
# carrier board, used by the TMC autotuning subsystem (see klippy/extras/tmc.py).
#
# A [stepstick <name>] section is matched against the `stepstick_type` option of
# a TMC stepper section; resolve_sense_resistor() reads sense_resistor from it
# and bounds run_current with max_current. The carrier data itself ships as
# config (THEOS-Configuration steppers/database/stepsticks.cfg) — this module is
# only the parser.
#
# This file may be distributed under the terms of the GNU GPLv3 license.


class StepStick:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.sense_resistor = config.getfloat('sense_resistor', above=0.)
        self.max_current = config.getfloat('max_current', above=0.)


def load_config_prefix(config):
    return StepStick(config)
