"""Pytest fixtures and mocks for TMC autotune unit tests.

These mocks isolate the TMC autotune logic from Klipper's runtime
(MCU communication, printer reactor, config parser) so the
_configure_* methods can be exercised as pure functions.
"""
import pytest
from collections import OrderedDict

_MISSING = object()  # Sentinel for "no default provided"


class MockConfig:
    """Minimal Klipper config-like object backed by a flat dict."""

    def __init__(self, name="tmc5160 stepper_x", values=None):
        self._name = name
        self._values = dict(values or {})
        self._printer = MockPrinter()
        self._fileconfig = MockFileConfig()

    def get_name(self):
        return self._name

    def get_printer(self):
        return self._printer

    def _get(self, parser, key, default, **kwargs):
        if key in self._values:
            v = self._values[key]
            return parser(v) if v is not None else None
        # Return _MISSING (not the caller's default) so callers can distinguish
        # "key absent" from "caller provided None as explicit default".
        # This mirrors real Klipper configfile.py behaviour where getint(k, None)
        # returns None when the option is absent.
        if default is _MISSING:
            return _MISSING
        return default

    def get(self, key, default=_MISSING, **kwargs):
        v = self._get(str, key, default)
        if v is _MISSING:
            raise self.error("Missing required config option '%s'" % key)
        return v

    def getint(self, key, default=_MISSING, minval=None, maxval=None, **kwargs):
        v = self._get(int, key, default)
        if v is _MISSING:
            raise self.error("Missing required config option '%s'" % key)
        return v

    def getfloat(self, key, default=_MISSING, above=None, minval=None,
                 maxval=None, **kwargs):
        v = self._get(float, key, default)
        if v is _MISSING:
            raise self.error("Missing required config option '%s'" % key)
        return v

    def getboolean(self, key, default=_MISSING, **kwargs):
        v = self._get(bool, key, default)
        if v is _MISSING:
            raise self.error("Missing required config option '%s'" % key)
        return v

    def getchoice(self, key, choices, default=_MISSING):
        v = self._values.get(key, _MISSING)
        if v is _MISSING:
            if default is _MISSING:
                raise self.error("Missing required config option '%s'" % key)
            v = default
        if v not in choices:
            raise self.error("Choice '%s' not in %s for option '%s'"
                             % (v, list(choices.keys()), key))
        return choices[v]

    @property
    def error(self):
        return MockConfigError


class MockConfigError(Exception):
    pass


class MockPrinter:
    config_error = MockConfigError

    def lookup_object(self, name, default=None):
        return default

    def load_object(self, config, name):
        return None

    def get_reactor(self):
        return None

    def register_event_handler(self, name, cb):
        pass


class MockFileConfig:
    def has_section(self, name):
        return False


class MockMcuTmc:
    """Minimal mcu_tmc that exposes a FieldHelper and records register writes."""

    def __init__(self, fields):
        self._fields = fields
        self.register_writes = []  # (reg_name, value, print_time)
        self._tmc_freq = 12.5e6

    def get_fields(self):
        return self._fields

    def get_mcu(self):
        return MockMcu()

    def get_tmc_frequency(self):
        return self._tmc_freq

    def set_register(self, reg_name, value, print_time=None):
        self.register_writes.append((reg_name, value, print_time))


class MockMcu:
    is_non_critical = False
    non_critical_disconnected = False

    def get_name(self):
        return "mcu"


class MockMotorConstants:
    """Synthetic motor for deterministic test outputs.

    Defaults model an LDO-42STH48-2504AC at 56 V — the canonical
    THEOS T100/T250 X/Y motor.  Override via constructor for other
    test motors (LDO-2804, OMC pancake).
    """

    def __init__(self, R=1.4, L=3e-3, holding=0.55, max_current=2.5, S=200):
        self.R = R
        self.L = L
        self.T = holding
        self.I = max_current
        self.S = S

    def pwmgrad(self, fclk=12.5e6, steps=0, volts=24.0):
        import math
        cbemf = self.T / (2 * self.I)
        return int(math.ceil(cbemf * 2 * math.pi * fclk * 1.46
                             / (volts * 256 * (steps or self.S))))

    def pwmofs(self, volts=24.0, current=0.0):
        import math
        I = current if current > 0 else self.I
        return int(math.ceil(374 * self.R * I / volts))

    def maxpwmrps(self, fclk=12.5e6, steps=0, volts=24.0, current=0.0):
        import math
        return ((255 - self.pwmofs(volts, current))
                / (math.pi * self.pwmgrad(fclk, steps or self.S, volts)))

    def pwmfreq(self, fclk=12.5e6, target=55e3):
        for prescaler, factor in [(3, 2./410), (2, 2./512),
                                  (1, 2./683), (0, 2./1024)]:
            f = fclk * factor
            if f < target:
                return prescaler, round(f, 1)
        return (0, round(fclk * 2./1024, 1))

    def hysteresis(self, name, extra, fclk, volts, current,
                   tbl, toff, rsense, scale):
        # Return deterministic test values; real formula tested elsewhere
        return 5, 2

    def commutation_time(self, voltage, current):
        # Task 7: returns 8 µs so that commutation_cycles = 8e-6 * 12.5e6 = 100
        # cycles, which with tbl_cycles+1 floor places dc_time in [30, 200]
        # as required by TestPhase2DCCTRL.test_dc_time_auto_derive_performance.
        return 8e-6

    def pwmfreq_to_hz(self, prescaler, fclk=12.5e6):
        # Mirrors MotorConstants.pwmfreq_to_hz — see motor_constants.py.
        # Needed for tune_invocation when driver_PWM_FREQ is pinned.
        factor = {3: 2./410, 2: 2./512, 1: 2./683, 0: 2./1024}.get(prescaler)
        if factor is None:
            raise ValueError("pwm_freq prescaler must be 0..3, got %r"
                             % (prescaler,))
        return round(fclk * factor, 1)


class MockStepper:
    """Minimal stepper for autotune tests that need rotation distance."""

    def __init__(self, rotation_distance=40.0, steps_per_rotation=200,
                 microsteps=16):
        self._rotation_distance = rotation_distance
        self._steps_per_rotation = steps_per_rotation
        self._microsteps = microsteps

    def get_rotation_distance(self):
        return self._rotation_distance, self._steps_per_rotation

    def get_step_dist(self):
        return (self._rotation_distance
                / (self._steps_per_rotation * self._microsteps))


@pytest.fixture
def mock_motor():
    return MockMotorConstants()


# Minimal field definitions needed for tune_invocation tests.
# Subset of tmc5160.Fields — only the registers that _configure_pwm and
# TMCtstepHelper touch.  Kept inline so tune_invocation never has to
# import tmc5160 (which pulls in bus → mcu → serial — unavailable on
# Windows without the full Klipper runtime).
_TUNE_TEST_FIELDS = {
    "PWMCONF": {
        "pwm_ofs":        0xFF << 0,
        "pwm_grad":       0xFF << 8,
        "pwm_freq":       0x03 << 16,
        "pwm_autoscale":  0x01 << 18,
        "pwm_autograd":   0x01 << 19,
        "freewheel":      0x03 << 20,
        "pwm_reg":        0x0F << 24,
        "pwm_lim":        0x0F << 28,
    },
    "TPWMTHRS": {
        "tpwmthrs":       0xfffff << 0,
    },
    "CHOPCONF": {
        # mres is read by TMCtstepHelper to compute the per-microstep
        # step distance.  Default value 0 = 256 µsteps (full microstepping).
        "mres":           0x0F << 24,
        "vhighfs":        0x01 << 18,
        "vhighchm":       0x01 << 19,
        # tbl is read by _configure_dcstep to determine blank cycles.
        "tbl":            0x03 << 15,
        # toff is needed for the TOFF=1/TBL=0 validation path.
        "toff":           0x0F << 0,
        # tpfd is written by _configure_spreadcycle.
        "tpfd":           0x0F << 20,
    },
    "DCCTRL": {
        "dc_time":        0x3FF << 0,
        "dc_sg":          0xFF  << 16,
    },
    "GCONF": {
        "faststandstill":   0x01 << 1,
        "en_pwm_mode":      0x01 << 2,
        "multistep_filt":   0x01 << 3,
        "small_hysteresis": 0x01 << 14,
    },
    "COOLCONF": {
        "semin":          0x0F << 0,
        "seup":           0x03 << 5,
        "semax":          0x0F << 8,
        "sedn":           0x03 << 13,
        "seimin":         0x01 << 15,
        "sfilt":          0x01 << 24,
    },
    "TCOOLTHRS": {
        "tcoolthrs":      0xfffff << 0,
    },
    "THIGH": {
        "thigh":          0xfffff << 0,
    },
    "IHOLDIRUN": {
        "iholddelay":     0x0F << 16,
    },
    "DRV_CONF": {
        "bbmtime":        0x1F << 0,
        "bbmclks":        0x0F << 8,
        "otselect":       0x03 << 16,
        "drvstrength":    0x03 << 18,
        "filt_isense":    0x03 << 20,
    },
}
_TUNE_TEST_SIGNED_FIELDS = []

# Module-level storage for the FieldHelper from the most recent tune_invocation.
# _last_tune_fields() is exported so test helpers like _read_dcctrl_field can
# retrieve field values that are not included in tune_invocation's return dict.
_last_tune_fields_ref = [None]


def _last_tune_fields():
    """Return the FieldHelper from the most recent tune_invocation call."""
    return _last_tune_fields_ref[0]


def _load_tmc_module():
    """Import extras.tmc without triggering the full Klipper runtime.

    tmc.py depends on 'stepper', 'extras.bulk_sensor', and
    'extras.stepstick_defs' — all of which pull in MCU / serial layers
    that don't exist on Windows.  We stub them out so only the pure-
    Python autotune logic gets loaded.
    """
    import sys
    import importlib.util
    import types
    import os

    if 'extras.tmc' in sys.modules:
        return sys.modules['extras.tmc']

    # Ensure 'extras' package namespace exists
    if 'extras' not in sys.modules:
        extras_pkg = types.ModuleType('extras')
        extras_pkg.__path__ = [
            os.path.join(os.path.dirname(__file__), '..')]
        extras_pkg.__package__ = 'extras'
        sys.modules['extras'] = extras_pkg

    # Stub out the three heavy imports that tmc.py does at module level
    for stub_name in ('stepper', 'extras.bulk_sensor', 'extras.stepstick_defs'):
        if stub_name not in sys.modules:
            stub = types.ModuleType(stub_name)
            if stub_name == 'extras.stepstick_defs':
                stub.STEPSTICK_DEFS = {}
            sys.modules[stub_name] = stub

    tmc_path = os.path.join(os.path.dirname(__file__), '..', 'tmc.py')
    spec = importlib.util.spec_from_file_location(
        'extras.tmc', os.path.abspath(tmc_path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules['extras.tmc'] = mod
    spec.loader.exec_module(mod)
    return mod


def _load_tmc5160_module():
    """Import extras.tmc5160 without triggering the full Klipper runtime.

    tmc5160.py imports extras.bus and extras.tmc2130, which pull in MCU/
    serial layers unavailable on Windows.  We stub those out so the
    module-level Fields dict and _validate_short_conf can be accessed.

    Must be called after _load_tmc_module() has populated extras.tmc.
    """
    import sys
    import importlib.util
    import types
    import os

    if 'extras.tmc5160' in sys.modules:
        return sys.modules['extras.tmc5160']

    # Ensure extras.tmc is loaded first (provides BaseTMCCurrentHelper etc.)
    _load_tmc_module()

    # Stub extras.bus (SPI/UART transport — not needed for validation)
    if 'extras.bus' not in sys.modules:
        sys.modules['extras.bus'] = types.ModuleType('extras.bus')

    # Stub extras.tmc2130 (only FieldFormatters and MCU_TMC_SPI are used at
    # module level in tmc5160.py)
    if 'extras.tmc2130' not in sys.modules:
        stub_tmc2130 = types.ModuleType('extras.tmc2130')
        stub_tmc2130.FieldFormatters = {}
        stub_tmc2130.MCU_TMC_SPI = object
        sys.modules['extras.tmc2130'] = stub_tmc2130

    tmc5160_path = os.path.join(os.path.dirname(__file__), '..', 'tmc5160.py')
    spec = importlib.util.spec_from_file_location(
        'extras.tmc5160', os.path.abspath(tmc5160_path))
    mod = importlib.util.module_from_spec(spec)
    sys.modules['extras.tmc5160'] = mod
    spec.loader.exec_module(mod)
    return mod


def tune_invocation(motor, tuning_goal='balanced', pins=None, voltage=56.0,
                    run_current=1.768, stealthchop_threshold=None,
                    pre_field_writes=None):
    """Construct a thin BaseTMCCurrentHelper-like proxy, run _configure_pwm,
    and return the resulting PWMCONF + TPWMTHRS field-shadow values as a dict.

    Approach (b): manually assembles only the attributes that _configure_pwm
    and its _derive_* helpers need, then calls the method directly.  Full
    TMC5160CurrentHelper.__init__ is bypassed (it needs a real MCU and
    sense-resistor lookup); only the pure-math configure path is exercised.

    `stealthchop_threshold`: when set, simulates TMCStealthchopHelper having
    previously set en_pwm_mode=1 in the field shadow before _configure_pwm
    runs.

    `pre_field_writes`: optional dict of {field_name: value} written into the
    FieldHelper shadow before _configure_pwm runs, for scenarios where a
    prior init path has already set certain bits.

    SHORT_CONF validation: _validate_short_conf from tmc5160 is called here
    (before the proxy is configured) so that the test suite can exercise the
    config-time validation path without a full TMC5160 init.
    """
    tmc = _load_tmc_module()
    tmc5160 = _load_tmc5160_module()

    # Invoke VS-aware SHORT_CONF validation. Extract S2VS/S2G pins directly
    # from the pins dict (same semantics as TMC5160.__init__'s config.getint
    # calls — None when not provided).
    _pins = pins or {}
    _s2vs = (_pins['driver_S2VS_LEVEL']
             if 'driver_S2VS_LEVEL' in _pins else None)
    _s2g = (_pins['driver_S2G_LEVEL']
            if 'driver_S2G_LEVEL' in _pins else None)
    # Build a minimal MockConfig solely for config.error / config.get_name()
    # — _validate_short_conf calls these when raising or warning.
    _val_cfg = MockConfig(values={})
    tmc5160._validate_short_conf(_val_cfg, voltage, _s2vs, _s2g)

    fields = tmc.FieldHelper(_TUNE_TEST_FIELDS, _TUNE_TEST_SIGNED_FIELDS)
    mcu_tmc = MockMcuTmc(fields)

    # Simulate TMCStealthchopHelper having set en_pwm_mode=1 prior to
    # _configure_pwm (this is what happens when stealthchop_threshold is
    # configured on a real driver).
    if stealthchop_threshold is not None:
        fields.set_field("en_pwm_mode", 1)

    # Apply any caller-supplied pre-field writes into the shadow before the
    # _configure_* methods run.
    if pre_field_writes:
        for fname, fval in pre_field_writes.items():
            fields.set_field(fname, fval)

    # Build a proxy that satisfies BaseTMCCurrentHelper._configure_pwm.
    proxy = object.__new__(tmc.BaseTMCCurrentHelper)
    proxy.name = "stepper_x"
    proxy.fields = fields
    proxy.mcu_tmc = mcu_tmc
    proxy.driver_clock_frequency = 12.5e6
    proxy.voltage = voltage
    proxy.tuning_goal = tuning_goal
    proxy.pwm_freq_target = 55e3  # default from tmc.PWM_FREQ_TARGETS

    # Resolve pin overrides (mirrors BaseTMCCurrentHelper.__init__ Step 3).
    proxy.pwm_freq = _pins.get('driver_PWM_FREQ', None)
    proxy.pwm_autoscale = _pins.get('driver_PWM_AUTOSCALE', None)
    proxy.pwm_autograd = _pins.get('driver_PWM_AUTOGRAD', None)
    proxy.pwm_grad = _pins.get('driver_PWM_GRAD', None)
    proxy.pwm_ofs = _pins.get('driver_PWM_OFS', None)
    proxy.pwm_reg_pin = _pins.get('driver_PWM_REG', None)
    proxy.pwm_lim_pin = _pins.get('driver_PWM_LIM', None)
    proxy.tpwmthrs_pin = _pins.get('driver_TPWMTHRS', None)
    # GCONF / multistep pin overrides
    proxy.faststandstill = _pins.get('driver_FASTSTANDSTILL', None)
    proxy.small_hysteresis = _pins.get('driver_SMALL_HYSTERESIS', None)
    proxy.multistep_filt = _pins.get('driver_MULTISTEP_FILT', None)
    # CoolStep pin overrides
    proxy.semin = _pins.get('driver_SEMIN', None)
    proxy.semax = _pins.get('driver_SEMAX', None)
    proxy.seup = _pins.get('driver_SEUP', None)
    proxy.sedn = _pins.get('driver_SEDN', None)
    proxy.seimin = _pins.get('driver_SEIMIN', None)
    proxy.sfilt = _pins.get('driver_SFILT', None)
    proxy.iholddelay = _pins.get('driver_IHOLDDELAY', None)
    # High-velocity / threshold pin overrides
    proxy.tcoolthrs_pin = _pins.get('driver_TCOOLTHRS', None)
    proxy.thigh_pin = _pins.get('driver_THIGH', None)
    proxy.vhighfs = _pins.get('driver_VHIGHFS', None)
    proxy.vhighchm = _pins.get('driver_VHIGHCHM', None)
    # Velocity-form pins — passed via pins= for test convenience but stored
    # on the proxy as floats (not driver_X register pins).
    proxy.coolstep_threshold = _pins.get('coolstep_threshold', None)
    proxy.high_velocity_threshold = _pins.get('high_velocity_threshold', None)
    # StallGuard threshold pins (used by _configure_stallguard)
    proxy.sg4_thrs = _pins.get('driver_SGTHRS', None)
    proxy.sgt = _pins.get('driver_SGT', None)
    # DRV_CONF pin-reads
    proxy.filt_isense = _pins.get('driver_FILT_ISENSE', None)
    # DCCTRL pin-reads
    proxy.dc_time = _pins.get('driver_DC_TIME', None)
    proxy.dc_sg = _pins.get('driver_DC_SG', None)
    # stealthchop_threshold velocity-form pin for tpwmthrs.  The
    # stealthchop_threshold= kwarg takes precedence over the pins= dict so
    # tests can supply it directly without polluting the pins dict (which is
    # reserved for driver_* register pins).
    _sc_thresh = stealthchop_threshold if stealthchop_threshold is not None \
        else _pins.get('stealthchop_threshold', None)
    proxy.stealthchop_threshold = _sc_thresh
    # printer needed for config_error in _configure_spreadcycle
    proxy.printer = MockPrinter()
    # _configure_spreadcycle attributes (goal-chopper + TPFD)
    proxy.tbl = _pins.get('driver_TBL', None)
    proxy.toff = _pins.get('driver_TOFF', None)
    proxy.tpfd = _pins.get('driver_TPFD', None)
    proxy.chopper_freq_target = None   # user-configurable; None = use goal default
    proxy.extra_hysteresis = 0
    proxy.hstrt = _pins.get('driver_HSTRT', None)
    proxy.hend = _pins.get('driver_HEND', None)
    proxy.sense_resistor = 0.075       # BTT Kraken default; needed by hysteresis
    # otselect is handled by tmc5160.py's set_config_field at config-time;
    # autotune never modifies it.  Mimic set_config_field by writing the
    # pin value directly into the field shadow before tune_driver runs.
    if 'driver_OTSELECT' in _pins:
        fields.set_field("otselect", _pins['driver_OTSELECT'])

    # MockStepper: canonical T100 X/Y axis — 40 mm rotation distance,
    # 200 steps/rev, 16 microsteps.
    proxy.stepper = MockStepper()

    # Run the full configure pipeline:
    #   _configure_pwm           — PWM/StealthChop fields
    #   _configure_spreadcycle   — TOFF/TPFD/TBL/HSTRT/HEND
    #   _configure_coolstep      — CoolStep + IHOLDDELAY
    #   _configure_stallguard    — TCOOLTHRS + SGTHRS/SGT
    #   _configure_highspeed     — THIGH + VHIGHFS/VHIGHCHM + multistep_filt
    #   _configure_drvconf       — DRV_CONF VS-aware filt_isense
    #   _configure_dcstep        — DCCTRL dc_time/dc_sg
    proxy._configure_pwm(motor, run_current)
    proxy._configure_spreadcycle(motor, run_current)
    proxy._configure_coolstep()
    proxy._configure_stallguard(run_current)
    proxy._configure_highspeed(motor, run_current)
    # dcStep+TOFF<3 validation (mirrors tune_driver check)
    _toff_final = fields.get_field("toff")
    _vhighfs_final = fields.get_field("vhighfs")
    _vhighchm_final = fields.get_field("vhighchm")
    if _vhighfs_final and _vhighchm_final and _toff_final < 3:
        raise proxy.printer.config_error(
            "tmc %s: dcStep requires TOFF>=3 per the TMC5160 datasheet"
            " (§13.2); current TOFF=%d." % (proxy.name, _toff_final))
    proxy._configure_drvconf()
    proxy._configure_dcstep(motor, run_current)

    # Store FieldHelper for _last_tune_fields() accessor used by test helpers.
    _last_tune_fields_ref[0] = fields

    # Return PWMCONF + tpwmthrs + CoolStep/GCONF/threshold fields as a flat dict.
    result = {f: fields.get_field(f)
              for f in _TUNE_TEST_FIELDS.get('PWMCONF', {})}
    result['tpwmthrs'] = fields.get_field('tpwmthrs')
    result['faststandstill'] = fields.get_field('faststandstill')
    result['small_hysteresis'] = fields.get_field('small_hysteresis')
    result['multistep_filt'] = fields.get_field('multistep_filt')
    result['semin'] = fields.get_field('semin')
    result['semax'] = fields.get_field('semax')
    result['seup'] = fields.get_field('seup')
    result['sedn'] = fields.get_field('sedn')
    result['seimin'] = fields.get_field('seimin')
    result['sfilt'] = fields.get_field('sfilt')
    result['iholddelay'] = fields.get_field('iholddelay')
    result['tcoolthrs'] = fields.get_field('tcoolthrs')
    result['thigh'] = fields.get_field('thigh')
    result['vhighfs'] = fields.get_field('vhighfs')
    result['vhighchm'] = fields.get_field('vhighchm')
    result['filt_isense'] = fields.get_field('filt_isense')
    result['otselect'] = fields.get_field('otselect')
    result['en_pwm_mode'] = fields.get_field('en_pwm_mode')
    return result


@pytest.fixture
def autotune_off_config():
    """Config with no motor:/voltage: → autotune disabled, static defaults active."""
    return MockConfig(values={
        "run_current": 1.0,
        "stepstick_type": "KRAKEN_2160_3A",
    })


@pytest.fixture
def autotune_on_config():
    """Config with motor + voltage → autotune active, balanced default."""
    return MockConfig(values={
        "run_current": 1.768,
        "voltage": 56.0,
        "motor": "ldo-42sth48-2504ac",
        "stepstick_type": "KRAKEN_2160_3A",
    })
