"""TMC2209 field-aware autotune behaviour."""
import sys, os
import pytest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))
from .conftest import (tune_invocation, _TUNE_TEST_FIELDS_2209,
                       _last_tune_fields, _load_tmc_module,
                       MockConfig, MockConfigError, MockMcuTmc,
                       _TUNE_TEST_FIELDS)


def _tune_2209(mock_motor, **kw):
    return tune_invocation(motor=mock_motor, fields=_TUNE_TEST_FIELDS_2209,
                           driver_type='tmc2209', **kw)


class TestFieldAwareSkip:
    def test_absent_fields_recorded_skipped_no_crash(self, mock_motor):
        # 2209 lacks these; the balanced pipeline must skip them silently and
        # record them, not KeyError. (sgt is only touched when pinned, so it is
        # excluded here — see TestRejectUnsupportedPins for the pinned case.)
        r = _tune_2209(mock_motor)  # must not raise
        for absent in ("tpfd", "thigh", "vhighfs", "vhighchm",
                       "faststandstill", "small_hysteresis", "sfilt"):
            assert absent in r["_skipped_fields"]

    def test_present_fields_are_derived(self, mock_motor):
        r = _tune_2209(mock_motor)
        assert r["pwm_reg"] is not None and r["pwm_lim"] is not None
        assert r["semin"] is not None
        assert r["en_spreadcycle"] is not None  # 2209 uses en_spreadcycle
        # Prove the configure pipeline actually wrote the present-field
        # registers (tune_invocation does NOT call _configure_hysteresis, so
        # hstrt/hend are out of scope here; sgthrs needs a pin, see below).
        f = _last_tune_fields()
        for present in ("toff", "tbl", "semin", "iholddelay", "tcoolthrs"):
            reg = f.lookup_register(present)
            assert reg in f.registers, "%s register %s not written" % (present, reg)


class TestSetFieldIfPresentUnit:
    def _proxy(self):
        tmc = _load_tmc_module()
        proxy = object.__new__(tmc.BaseTMCCurrentHelper)
        proxy.fields = tmc.FieldHelper({"CHOPCONF": {"toff": 0x0F}}, [])
        proxy._skipped_fields = set()
        return proxy

    def test_writes_when_present(self):
        p = self._proxy()
        p._set_field_if_present("toff", 5)
        assert p.fields.get_field("toff") == 5
        assert "toff" not in p._skipped_fields

    def test_skips_and_records_when_absent(self):
        p = self._proxy()
        p._set_field_if_present("tpfd", 4)  # absent on this field set
        assert "tpfd" in p._skipped_fields


class TestGoalBehavior2209:
    @pytest.mark.parametrize("goal,reg,lim", [
        ('performance', 15, 4), ('balanced', 8, 8), ('silent', 4, 12)])
    def test_pwm_reg_lim_per_goal(self, mock_motor, goal, reg, lim):
        r = _tune_2209(mock_motor, tuning_goal=goal)
        assert r["pwm_reg"] == reg
        assert r["pwm_lim"] == lim

    def test_performance_no_crash_with_absent_vhigh(self, mock_motor):
        # performance enables vhighfs/vhighchm on a 5160; on a 2209 those
        # fields are absent and must be skipped, not crash.
        r = _tune_2209(mock_motor, tuning_goal='performance')  # must not raise
        assert r["vhighfs"] is None and r["vhighchm"] is None


def _make_helper(field_set, driver_type, values):
    tmc = _load_tmc_module()
    fields = tmc.FieldHelper(field_set, [])
    proxy = object.__new__(tmc.BaseTMCCurrentHelper)
    proxy.fields = fields
    proxy.driver_type = driver_type
    cfg = MockConfig(name="%s stepper_x" % driver_type, values=values)
    return proxy, cfg


class TestRejectUnsupportedPins:
    @pytest.mark.parametrize("pin", [
        "driver_THIGH", "driver_SGT", "driver_TPFD", "driver_VHIGHFS",
        "driver_VHIGHCHM", "driver_FASTSTANDSTILL",
        "driver_SMALL_HYSTERESIS", "driver_SFILT"])
    def test_pinned_absent_field_raises(self, pin):
        proxy, cfg = _make_helper(_TUNE_TEST_FIELDS_2209, "tmc2209", {pin: 1})
        with pytest.raises(MockConfigError, match="(?i)%s|unsupported" % pin):
            proxy._reject_unsupported_pins(cfg)

    def test_present_field_pin_ok_on_5160(self):
        proxy, cfg = _make_helper(_TUNE_TEST_FIELDS, "tmc5160",
                                  {"driver_THIGH": 1000})
        proxy._reject_unsupported_pins(cfg)  # must not raise

    def test_init_wires_in_validation(self):
        # Construct a real BaseTMCCurrentHelper so the test fails if __init__
        # forgets to call _reject_unsupported_pins (a direct call would pass).
        tmc = _load_tmc_module()
        fields = tmc.FieldHelper(_TUNE_TEST_FIELDS_2209, [])
        mcu_tmc = MockMcuTmc(fields)
        cfg = MockConfig(name="tmc2209 stepper_x",
                         values={"run_current": 0.8, "driver_THIGH": 1000})
        with pytest.raises(MockConfigError):
            tmc.BaseTMCCurrentHelper(cfg, mcu_tmc, max_current=2.0)


class TestMinTblAtMinToff:
    def test_2209_toff1_forces_tbl_2(self, mock_motor):
        # driver_TOFF=1 with TBL auto-chosen must bump TBL to >=2 on 2209.
        tune_invocation(fields=_TUNE_TEST_FIELDS_2209, driver_type='tmc2209',
                        motor=mock_motor, pins={'driver_TOFF': 1})
        assert _last_tune_fields().get_field('tbl') >= 2

    def test_5160_toff1_forces_tbl_1(self, mock_motor):
        tune_invocation(motor=mock_motor, pins={'driver_TOFF': 1})
        assert _last_tune_fields().get_field('tbl') >= 1


class TestHysteresisScale:
    def test_base_default_returns_cs(self):
        from .conftest import _load_tmc_module
        tmc = _load_tmc_module()
        proxy = object.__new__(tmc.BaseTMCCurrentHelper)
        proxy.cs = 7
        assert proxy._hysteresis_scale(1.0) == 7

    def test_2130_helper_returns_programmed_cs_when_unset(self):
        # tmc2130.TMCCurrentHelper overrides the hook: with cs unset it must
        # return the IRUN bits the driver is actually programmed to, not None.
        from .conftest import _load_tmc_module
        tmc = _load_tmc_module()
        import importlib, types, sys, os
        # Load tmc2130 with bus stubbed (no MCU on the host).
        if 'extras.bus' not in sys.modules:
            sys.modules['extras.bus'] = types.ModuleType('extras.bus')
        path = os.path.join(os.path.dirname(__file__), '..', 'tmc2130.py')
        spec = importlib.util.spec_from_file_location('extras.tmc2130',
                                                      os.path.abspath(path))
        mod = importlib.util.module_from_spec(spec)
        sys.modules['extras.tmc2130'] = mod
        spec.loader.exec_module(mod)
        h = object.__new__(mod.TMCCurrentHelper)
        h.cs = None
        h.sense_resistor = 0.11
        h.req_hold_current = 0.4
        scale = h._hysteresis_scale(0.8)
        assert isinstance(scale, int) and 0 <= scale <= 31

    def test_configure_hysteresis_passes_hook_scale(self):
        # _configure_hysteresis must hand _hysteresis_scale(current) to
        # motor.hysteresis() as `scale` — not self.cs directly. Recording motor
        # captures the scale it receives.
        from .conftest import _load_tmc_module, _TUNE_TEST_FIELDS_2209

        class RecMotor:
            scale = "unset"
            def hysteresis(self, name, extra, fclk, volts, current,
                           tbl, toff, rsense, scale):
                RecMotor.scale = scale
                return 5, 2

        tmc = _load_tmc_module()
        proxy = object.__new__(tmc.BaseTMCCurrentHelper)
        proxy.fields = tmc.FieldHelper(_TUNE_TEST_FIELDS_2209, [])
        proxy.name = "stepper_x"
        proxy.extra_hysteresis = 0
        proxy.driver_clock_frequency = 12e6
        proxy.voltage = 24.0
        proxy.sense_resistor = 0.11
        proxy.cs = None
        proxy.hstrt = None
        proxy.hend = None
        proxy._hysteresis_scale = lambda current: 9  # sentinel
        proxy._configure_hysteresis(RecMotor(), 0.8, 1, 3)
        assert RecMotor.scale == 9
