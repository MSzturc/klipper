# Lookup table for common stepper-driver carrier boards ("stepsticks").
#
# Each entry maps a board identifier (matched against the
# `stepstick_type` config option of a TMC stepper section) to a
# `(sense_resistor, max_current)` pair.  The TMC autotuning subsystem
# uses this table to populate `sense_resistor` and to bound `run_current`
# when those values were not given explicitly in the config.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

STEPSTICK_DEFS = {
    "REFERENCE_WOTT":         (0.11,  1.2),
    "REFERENCE_2209":         (0.11,  2.0),
    "REFERENCE_5160":         (0.075, 3.0),
    "KRAKEN_2160_8A":         (0.022, 8.0),
    "KRAKEN_2160_3A":         (0.075, 3.0),
    "BTT_2240":               (0.11,  2.1),
    "BTT_EZ_5160_PRO":        (0.075, 2.5),
    "BTT_EZ_5160_RGB":        (0.05,  4.7),
    "BTT_EZ_6609":            (0.11,  2.0),
    "BTT_5160T":              (0.022, 10.6),
    "WOTT_2209":              (0.11,  1.7),
    "COREVUS_2209":           (0.1,   3.0),
    "COREVUS_2160_OLD":       (0.03,  3.0),
    "COREVUS_2160_5A":        (0.03,  5.0),
    "COREVUS_2160":           (0.05,  3.0),
    "FYSETC_2225":            (0.11,  1.4),
    "FYSETC_5161":            (0.06,  3.5),
    "MKS_2226":               (0.17,  2.5),
    "MELLOW_FLY_5160":        (0.11,  3.0),
    "MELLOW_FLY_HV_5160_Pro": (0.033, 6.0),
}
