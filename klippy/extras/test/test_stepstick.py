"""The stepstick carrier database is config-driven: each carrier is a
[stepstick <name>] section parsed by extras/stepstick.py, and
resolve_sense_resistor looks it up as a printer object. The old hardcoded
STEPSTICK_DEFS dict is gone."""
import os
import sys
import importlib.util
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from .conftest import MockConfig, MockConfigError, _load_tmc_module


def _load_stepstick():
    path = os.path.join(os.path.dirname(__file__), "..", "stepstick.py")
    spec = importlib.util.spec_from_file_location(
        "extras.stepstick", os.path.abspath(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _StepStickStub:
    def __init__(self, sense_resistor, max_current):
        self.sense_resistor = sense_resistor
        self.max_current = max_current


class TestStepstickParser:
    def test_parses_sense_resistor_and_max_current(self):
        stepstick = _load_stepstick()
        cfg = MockConfig(name="stepstick KRAKEN_2160_8A",
                         values={"sense_resistor": 0.022, "max_current": 8.0})
        ss = stepstick.load_config_prefix(cfg)
        assert ss.name == "KRAKEN_2160_8A"
        assert ss.sense_resistor == 0.022
        assert ss.max_current == 8.0


class TestResolveSenseResistor:
    def _cfg(self, name=None, sr=None, mx=None, **values):
        if name is not None:
            values["stepstick_type"] = name
        cfg = MockConfig(values=values)
        if name is not None and sr is not None:
            cfg.get_printer().objects["stepstick " + name] = \
                _StepStickStub(sr, mx)
        return cfg

    def test_resolves_from_stepstick_object(self):
        m = _load_tmc_module()
        sr, mx = m.resolve_sense_resistor(
            self._cfg("KRAKEN_2160_8A", 0.022, 8.0))
        assert sr == 0.022
        assert mx == 8.0

    def test_explicit_sense_resistor_wins(self):
        # An explicit sense_resistor beats the carrier's value, but the
        # carrier's max_current still bounds run_current.
        m = _load_tmc_module()
        sr, mx = m.resolve_sense_resistor(
            self._cfg("KRAKEN_2160_8A", 0.022, 8.0, sense_resistor=0.05))
        assert sr == 0.05
        assert mx == 8.0

    def test_unknown_stepstick_raises(self):
        m = _load_tmc_module()
        with pytest.raises(MockConfigError):
            m.resolve_sense_resistor(self._cfg("NOT_A_REAL_BOARD"))

    def test_neither_given_raises_when_required(self):
        m = _load_tmc_module()
        with pytest.raises(MockConfigError):
            m.resolve_sense_resistor(self._cfg())

    def test_neither_given_returns_none_when_optional(self):
        m = _load_tmc_module()
        sr, mx = m.resolve_sense_resistor(self._cfg(), required=False)
        assert sr is None
        assert mx is None
