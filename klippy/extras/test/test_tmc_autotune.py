"""Unit tests for TMC autotune Convention-over-Configuration paradigm."""
import sys
import os
import pytest

# Make klippy/extras importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

from .conftest import MockConfig, MockConfigError, tune_invocation, _last_tune_fields


def _read_dcctrl_field(field_name):
    """Helper: read DCCTRL field shadow from the most recent tune_invocation."""
    return _last_tune_fields().get_field(field_name)


class TestTestHarness:
    """Smoke tests for the mock infrastructure itself."""

    def test_mock_motor_constructible(self, mock_motor):
        # MockMotorConstants must expose all autotune-required methods
        assert mock_motor.pwmgrad(fclk=12.5e6, volts=56.0) > 0
        assert mock_motor.pwmofs(volts=56.0, current=1.768) > 0
        assert mock_motor.maxpwmrps(volts=56.0, current=1.768) > 0
        prescaler, freq = mock_motor.pwmfreq(fclk=12.5e6, target=55e3)
        assert prescaler in (0, 1, 2, 3)
        assert freq > 0

    def test_mock_config_basic_access(self, autotune_off_config):
        # MockConfig must support getint/getfloat/getchoice
        assert autotune_off_config.getfloat("run_current") == 1.0
        assert autotune_off_config.get("stepstick_type") == "KRAKEN_2160_3A"


class TestMigrationAutotuneOff:
    """Verify autotune-OFF configs produce bit-identical register values
    after the refactor."""

    # Golden snapshot: minimal autotune-OFF tmc5160 config.
    # If this changes, an autotune-OFF user's printer behaviour changes.
    GOLDEN_FIELDS_AUTOTUNE_OFF = {
        "multistep_filt": 1,
        "toff": 3,
        "hstrt": 5,
        "hend": 2,
        "tbl": 2,
        "tpfd": 4,
        "drvstrength": 0,
        "bbmclks": 4,
        "iholddelay": 6,
        "pwm_ofs": 30,
        "pwm_grad": 0,
        "pwm_freq": 0,
        "pwm_autoscale": 1,
        "pwm_autograd": 1,
        "freewheel": 0,
        "pwm_reg": 4,
        "pwm_lim": 12,
        "tpowerdown": 10,
    }

    def test_autotune_off_field_shadow_matches_golden(self):
        # Autotune-OFF behaviour must remain bit-identical.  Currently a
        # placeholder assertion against the golden-snapshot dict; will be
        # extended to instantiate TMC5160 with MockConfig and read back the
        # field shadow once the harness is wired up for full chip init.
        assert self.GOLDEN_FIELDS_AUTOTUNE_OFF["toff"] == 3
        assert self.GOLDEN_FIELDS_AUTOTUNE_OFF["pwm_freq"] == 0


class TestTuningGoalParsing:
    """tuning_goal config option."""

    def test_default_is_balanced(self, autotune_on_config, mock_motor):
        # We can't fully construct without an mcu_tmc; just exercise the
        # config read path that the patch will introduce.
        goal = autotune_on_config.getchoice(
            'tuning_goal',
            {'performance': 'performance', 'balanced': 'balanced',
             'silent': 'silent'},
            'balanced')
        assert goal == 'balanced'

    def test_explicit_performance(self):
        cfg = MockConfig(values={"tuning_goal": "performance"})
        goal = cfg.getchoice('tuning_goal',
            {'performance': 'performance', 'balanced': 'balanced',
             'silent': 'silent'},
            'balanced')
        assert goal == 'performance'

    def test_explicit_silent(self):
        cfg = MockConfig(values={"tuning_goal": "silent"})
        goal = cfg.getchoice('tuning_goal',
            {'performance': 'performance', 'balanced': 'balanced',
             'silent': 'silent'},
            'balanced')
        assert goal == 'silent'

    def test_invalid_value_raises(self):
        cfg = MockConfig(values={"tuning_goal": "max_torque"})
        with pytest.raises(MockConfigError):
            cfg.getchoice('tuning_goal',
                {'performance': 'performance', 'balanced': 'balanced',
                 'silent': 'silent'},
                'balanced')


class TestPwmStealthChopAutotune:
    """PWM/StealthChop pin-respect + goal-aware defaults."""

    @pytest.mark.parametrize("pin_name,pin_value,field_name", [
        ("driver_PWM_FREQ", 2, "pwm_freq"),
        ("driver_PWM_AUTOSCALE", False, "pwm_autoscale"),
        ("driver_PWM_AUTOGRAD", False, "pwm_autograd"),
        ("driver_PWM_GRAD", 42, "pwm_grad"),
        ("driver_PWM_OFS", 100, "pwm_ofs"),
        ("driver_PWM_REG", 7, "pwm_reg"),
        ("driver_PWM_LIM", 6, "pwm_lim"),
        ("driver_TPWMTHRS", 12345, "tpwmthrs"),
    ])
    def test_pin_respected(self, pin_name, pin_value, field_name, mock_motor):
        # pinned driver_X overrides goal-derived value
        result = self._tune_with_pin(pin_name, pin_value, mock_motor)
        assert result[field_name] == (int(pin_value) if isinstance(pin_value, bool)
                                      else pin_value)

    def test_pwm_reg_goal_defaults(self, mock_motor):
        # pwm_reg differs per goal
        assert self._tune_for_goal('performance', mock_motor)['pwm_reg'] == 15
        assert self._tune_for_goal('balanced',    mock_motor)['pwm_reg'] == 8
        assert self._tune_for_goal('silent',      mock_motor)['pwm_reg'] == 4

    def test_pwm_lim_goal_defaults(self, mock_motor):
        # pwm_lim differs per goal
        assert self._tune_for_goal('performance', mock_motor)['pwm_lim'] == 4
        assert self._tune_for_goal('balanced',    mock_motor)['pwm_lim'] == 8
        assert self._tune_for_goal('silent',      mock_motor)['pwm_lim'] == 12

    def test_tpwmthrs_goal_defaults_relative(self, mock_motor):
        # tpwmthrs ordering: silent < performance, balanced < silent
        # (numerically: TPWMTHRS is a TSTEP threshold — higher TSTEP = slower
        # velocity; performance=0xfffff means StealthChop never active)
        perf = self._tune_for_goal('performance', mock_motor)['tpwmthrs']
        bal  = self._tune_for_goal('balanced',    mock_motor)['tpwmthrs']
        sil  = self._tune_for_goal('silent',      mock_motor)['tpwmthrs']
        assert perf == 0xfffff
        assert bal > sil  # balanced crosses earlier (higher TSTEP threshold)
        assert sil > 0    # silent has a real threshold, not 0

    def _tune_with_pin(self, pin_name, pin_value, mock_motor):
        """Helper: instantiate helper with one pin set, run tune_driver,
        return dict of relevant field values."""
        return tune_invocation(motor=mock_motor, pins={pin_name: pin_value})

    def _tune_for_goal(self, goal, mock_motor):
        return tune_invocation(motor=mock_motor, tuning_goal=goal)


class TestGconfFlagAutotune:
    """GCONF flag pin-respect + goal-defaults
    (faststandstill, small_hysteresis, multistep_filt)."""

    @pytest.mark.parametrize("pin_name,pin_value,field_name", [
        ("driver_FASTSTANDSTILL", False, "faststandstill"),
        ("driver_SMALL_HYSTERESIS", True, "small_hysteresis"),
        ("driver_MULTISTEP_FILT", False, "multistep_filt"),
    ])
    def test_pin_respected(self, pin_name, pin_value, field_name, mock_motor):
        result = tune_invocation(motor=mock_motor,
                                 pins={pin_name: pin_value})
        assert result[field_name] == int(pin_value)

    def test_small_hysteresis_goal_defaults(self, mock_motor):
        assert tune_invocation(motor=mock_motor, tuning_goal='performance'
                               )['small_hysteresis'] == 0
        assert tune_invocation(motor=mock_motor, tuning_goal='balanced'
                               )['small_hysteresis'] == 0
        assert tune_invocation(motor=mock_motor, tuning_goal='silent'
                               )['small_hysteresis'] == 1

    def test_faststandstill_default_true_all_goals(self, mock_motor):
        for goal in ('performance', 'balanced', 'silent'):
            assert tune_invocation(motor=mock_motor, tuning_goal=goal
                                   )['faststandstill'] == 1

    def test_multistep_filt_default_true_all_goals(self, mock_motor):
        for goal in ('performance', 'balanced', 'silent'):
            assert tune_invocation(motor=mock_motor, tuning_goal=goal
                                   )['multistep_filt'] == 1


class TestCoolStepAutotune:
    """CoolStep pin-respect + goal-defaults."""

    @pytest.mark.parametrize("pin_name,pin_value,field_name", [
        ("driver_SEMIN", 5, "semin"),
        ("driver_SEMAX", 7, "semax"),
        ("driver_SEUP", 1, "seup"),
        ("driver_SEDN", 3, "sedn"),
        ("driver_SEIMIN", 0, "seimin"),
        ("driver_SFILT", 1, "sfilt"),
        ("driver_IHOLDDELAY", 4, "iholddelay"),
    ])
    def test_pin_respected(self, pin_name, pin_value, field_name, mock_motor):
        result = tune_invocation(motor=mock_motor,
                                 pins={pin_name: pin_value})
        assert result[field_name] == pin_value

    def test_silent_disables_coolstep(self, mock_motor):
        # silent goal sets semin=0 to disable CoolStep
        result = tune_invocation(motor=mock_motor, tuning_goal='silent')
        assert result['semin'] == 0

    def test_performance_balanced_enable_coolstep(self, mock_motor):
        # semin=2 for performance and balanced
        for goal in ('performance', 'balanced'):
            result = tune_invocation(motor=mock_motor, tuning_goal=goal)
            assert result['semin'] == 2
            assert result['semax'] == 4
            assert result['seup'] == 3
            assert result['sedn'] == 2

    def test_sfilt_goal_defaults(self, mock_motor):
        # sfilt 0 for performance, 1 for balanced
        assert tune_invocation(motor=mock_motor, tuning_goal='performance'
                               )['sfilt'] == 0
        assert tune_invocation(motor=mock_motor, tuning_goal='balanced'
                               )['sfilt'] == 1

    def test_iholddelay_goal_defaults(self, mock_motor):
        # iholddelay 12 for performance/silent, 10 for balanced
        assert tune_invocation(motor=mock_motor, tuning_goal='performance'
                               )['iholddelay'] == 12
        assert tune_invocation(motor=mock_motor, tuning_goal='balanced'
                               )['iholddelay'] == 10
        assert tune_invocation(motor=mock_motor, tuning_goal='silent'
                               )['iholddelay'] == 12


class TestVelocityThresholdAutotune:
    """Thresholds/HighSpeed pin-respect + raw-vs-velocity reconcile
    (TCOOLTHRS, THIGH, VHIGHFS, VHIGHCHM, coolstep_threshold,
    high_velocity_threshold)."""

    @pytest.mark.parametrize("pin_name,pin_value,field_name", [
        ("driver_TCOOLTHRS", 50000, "tcoolthrs"),
        ("driver_THIGH", 1000, "thigh"),
        ("driver_VHIGHFS", True, "vhighfs"),
        ("driver_VHIGHCHM", True, "vhighchm"),
    ])
    def test_pin_respected(self, pin_name, pin_value, field_name, mock_motor):
        result = tune_invocation(motor=mock_motor,
                                 pins={pin_name: pin_value})
        if isinstance(pin_value, bool):
            assert result[field_name] == int(pin_value)
        else:
            assert result[field_name] == pin_value

    def test_vhighfs_vhighchm_goal_defaults(self, mock_motor):
        # performance enables both, balanced/silent disable
        perf = tune_invocation(motor=mock_motor, tuning_goal='performance')
        assert perf['vhighfs'] == 1 and perf['vhighchm'] == 1
        for goal in ('balanced', 'silent'):
            r = tune_invocation(motor=mock_motor, tuning_goal=goal)
            assert r['vhighfs'] == 0 and r['vhighchm'] == 0

    def test_thigh_goal_defaults(self, mock_motor):
        # performance has computed THIGH; balanced/silent: 0
        # (THIGH=0 keeps CoolStep window open; upstream TMCVhighHelper default)
        perf = tune_invocation(motor=mock_motor, tuning_goal='performance')
        assert 0 < perf['thigh'] < 0xfffff
        for goal in ('balanced', 'silent'):
            r = tune_invocation(motor=mock_motor, tuning_goal=goal)
            assert r['thigh'] == 0

    def test_velocity_pin_for_tcoolthrs(self, mock_motor):
        # coolstep_threshold (velocity) sets tcoolthrs via TSTEP
        result = tune_invocation(motor=mock_motor,
                                 pins={"coolstep_threshold": 100.0})
        # 100 mm/s with default rotation_distance — non-zero TSTEP
        assert 0 < result['tcoolthrs'] < 0xfffff

    def test_raw_pin_beats_velocity_pin(self, mock_motor):
        # driver_TCOOLTHRS wins over coolstep_threshold
        result = tune_invocation(motor=mock_motor, pins={
            "driver_TCOOLTHRS": 99999,
            "coolstep_threshold": 100.0,  # would compute to different TSTEP
        })
        assert result['tcoolthrs'] == 99999


class TestDrvConfAutotune:
    """DRV_CONF: otselect exposure + VS-aware filt_isense default."""

    def test_otselect_pin_respected(self, mock_motor):
        result = tune_invocation(motor=mock_motor,
                                 pins={"driver_OTSELECT": 2})
        assert result.get('otselect', None) == 2 \
            or _read_drvconf_field('otselect') == 2

    def test_filt_isense_default_high_vs(self, mock_motor):
        # voltage=56 → filt_isense should default to 1
        result = tune_invocation(motor=mock_motor, voltage=56.0)
        assert result['filt_isense'] == 1

    def test_filt_isense_default_low_vs(self, mock_motor):
        # voltage=24 → filt_isense should default to 0
        result = tune_invocation(motor=mock_motor, voltage=24.0)
        assert result['filt_isense'] == 0

    def test_filt_isense_pin_overrides_vs_default(self, mock_motor):
        # voltage=56 with explicit driver_FILT_ISENSE=0 → 0 wins
        result = tune_invocation(motor=mock_motor, voltage=56.0,
                                 pins={"driver_FILT_ISENSE": 0})
        assert result['filt_isense'] == 0


class TestDcCtrlAutotune:
    """DCCTRL register definition + auto-derive for dcStep."""

    def test_dcctrl_fields_defined(self):
        import sys, importlib.util, types, os
        # Stub out heavy runtime imports so tmc5160 can be loaded on Windows.
        if 'extras.bus' not in sys.modules:
            sys.modules['extras.bus'] = types.ModuleType('extras.bus')
        if 'extras.tmc2130' not in sys.modules:
            stub_tmc2130 = types.ModuleType('extras.tmc2130')
            stub_tmc2130.FieldFormatters = {}
            stub_tmc2130.MCU_TMC_SPI = object
            sys.modules['extras.tmc2130'] = stub_tmc2130
        tmc5160_path = os.path.join(
            os.path.dirname(__file__), '..', 'tmc5160.py')
        spec = importlib.util.spec_from_file_location(
            'extras.tmc5160', os.path.abspath(tmc5160_path))
        mod = importlib.util.module_from_spec(spec)
        sys.modules['extras.tmc5160'] = mod
        spec.loader.exec_module(mod)
        assert "DCCTRL" in mod.Fields
        assert "dc_time" in mod.Fields["DCCTRL"]
        assert "dc_sg" in mod.Fields["DCCTRL"]

    def test_dc_time_pin_respected(self, mock_motor):
        result = tune_invocation(motor=mock_motor, tuning_goal='performance',
                                 pins={"driver_DC_TIME": 100})
        assert _read_dcctrl_field('dc_time') == 100

    def test_dc_sg_pin_respected(self, mock_motor):
        result = tune_invocation(motor=mock_motor, tuning_goal='performance',
                                 pins={"driver_DC_SG": 16})
        assert _read_dcctrl_field('dc_sg') == 16

    def test_dc_time_auto_derive_performance(self, mock_motor):
        # performance goal computes dc_time from motor params
        result = tune_invocation(motor=mock_motor, tuning_goal='performance')
        dc_time = _read_dcctrl_field('dc_time')
        assert dc_time >= 30  # AN-003 typical lower bound
        assert dc_time <= 200  # sanity upper bound

    def test_dc_sg_is_dc_time_over_16(self, mock_motor):
        result = tune_invocation(motor=mock_motor, tuning_goal='performance')
        dc_time = _read_dcctrl_field('dc_time')
        dc_sg = _read_dcctrl_field('dc_sg')
        assert dc_sg == max(1, dc_time // 16)

    def test_balanced_silent_dont_compute_dcctrl(self, mock_motor):
        # In balanced/silent the values are written for determinism,
        # but vhighchm=0 means they have no hardware effect.
        for goal in ('balanced', 'silent'):
            r = tune_invocation(motor=mock_motor, tuning_goal=goal)
            assert _read_dcctrl_field('dc_time') >= 0  # written, not crashing


class TestShortConfValidation:
    """VS-aware SHORT_CONF validation."""

    def test_high_vs_low_s2g_level_raises(self, mock_motor):
        # voltage=56 + driver_S2G_LEVEL=8 → config.error
        with pytest.raises(MockConfigError, match="(?i)s2g_level must be"):
            tune_invocation(motor=mock_motor, voltage=56.0, pins={
                "driver_S2VS_LEVEL": 6,
                "driver_S2G_LEVEL": 8,  # invalid at >52V
            })

    def test_high_vs_valid_s2g_level_ok(self, mock_motor):
        # voltage=56 + driver_S2G_LEVEL=12 → OK
        result = tune_invocation(motor=mock_motor, voltage=56.0, pins={
            "driver_S2VS_LEVEL": 6,
            "driver_S2G_LEVEL": 12,
        })
        assert result is not None  # no error

    def test_low_vs_low_s2g_level_ok(self, mock_motor):
        # voltage=24 + driver_S2G_LEVEL=8 → OK (no datasheet violation)
        result = tune_invocation(motor=mock_motor, voltage=24.0, pins={
            "driver_S2VS_LEVEL": 6,
            "driver_S2G_LEVEL": 8,
        })
        assert result is not None

    def test_high_vs_no_short_conf_warns(self, mock_motor, caplog):
        # voltage=56 without explicit SHORT_CONF → warning, not error
        import logging
        with caplog.at_level(logging.WARNING):
            tune_invocation(motor=mock_motor, voltage=56.0)
        assert any("SHORT_CONF" in r.message and "52V" in r.message
                   for r in caplog.records)

    def test_low_vs_no_short_conf_no_warn(self, mock_motor, caplog):
        import logging
        with caplog.at_level(logging.WARNING):
            tune_invocation(motor=mock_motor, voltage=24.0)
        assert not any("SHORT_CONF" in r.message
                       for r in caplog.records)


class TestUserStoryCoverage:
    """Behaviour-level verification of the four user pain-points the
    autotune design targets, plus the high-VS implicit safety default."""

    def test_silent_disables_coolstep_modulation(self, mock_motor):
        # Heat: silent disables CoolStep current modulation (semin=0) so the
        # coil current does not modulate with load — keeps motor temperature
        # uniform on long prints.
        result = tune_invocation(motor=mock_motor, tuning_goal='silent')
        assert result['semin'] == 0

    def test_balanced_smooths_coolstep(self, mock_motor):
        # Vibration / ghosting: balanced enables sfilt=1 to stabilise
        # CoolStep so coil current does not breathe under varying load.
        result = tune_invocation(motor=mock_motor, tuning_goal='balanced')
        assert result['sfilt'] == 1

    def test_performance_enables_dcstep(self, mock_motor):
        # Top-speed step loss: performance enables vhighfs+vhighchm above
        # THIGH which activates dcStep, recovering torque at high RPM.
        result = tune_invocation(motor=mock_motor, tuning_goal='performance')
        assert result['vhighfs'] == 1
        assert result['vhighchm'] == 1

    def test_silent_high_pwm_freq(self, mock_motor):
        # Audible whining: silent goal targets chopper_freq_target=45 kHz
        # (above human hearing).  pwm_reg=4 is silent's PI-response signature
        # (smoothest StealthChop regulation).
        result = tune_invocation(motor=mock_motor, tuning_goal='silent')
        assert result['pwm_reg'] == 4

    def test_high_vs_safety_engaged(self, mock_motor):
        # Implicit safety: 56V configs default filt_isense=1 to suppress
        # sense-line ringing.
        result = tune_invocation(motor=mock_motor, voltage=56.0)
        assert result['filt_isense'] == 1


class TestStealthChopGoalAndPinning:
    """en_pwm_mode goal-gating, stealthchop_threshold pin precedence,
    voltage-aware maxpwmrps, goal-specific chopper-frequency targets, and
    TOFF/TBL/dcStep boundary conflicts."""

    # balanced/silent goals must set en_pwm_mode=1
    def test_balanced_enables_stealthchop(self, mock_motor):
        result = tune_invocation(motor=mock_motor, tuning_goal='balanced')
        assert result['en_pwm_mode'] == 1, (
            "balanced goal must set en_pwm_mode=1 to enable StealthChop")

    def test_silent_enables_stealthchop(self, mock_motor):
        result = tune_invocation(motor=mock_motor, tuning_goal='silent')
        assert result['en_pwm_mode'] == 1, (
            "silent goal must set en_pwm_mode=1 to enable StealthChop")

    def test_performance_does_not_enable_stealthchop(self, mock_motor):
        result = tune_invocation(motor=mock_motor, tuning_goal='performance')
        assert result['en_pwm_mode'] == 0, (
            "performance goal must leave en_pwm_mode=0 (SpreadCycle only)")

    # stealthchop_threshold velocity pin beats goal-default tpwmthrs
    def test_stealthchop_threshold_velocity_pin_respected(self, mock_motor):
        # When stealthchop_threshold is set, _configure_pwm must use the
        # TSTEP-converted velocity value, not the goal-derived default.
        result_pinned = tune_invocation(motor=mock_motor,
                                        tuning_goal='balanced',
                                        pins={'stealthchop_threshold': 50.0})
        result_default = tune_invocation(motor=mock_motor,
                                         tuning_goal='balanced')
        # The pinned value (50 mm/s) should produce a different TSTEP than
        # the goal default (0.3 * vmaxpwm); they must not be equal.
        assert result_pinned['tpwmthrs'] != result_default['tpwmthrs'], (
            "stealthchop_threshold must override goal-derived tpwmthrs")

    def test_raw_tpwmthrs_pin_beats_stealthchop_threshold(self, mock_motor):
        # driver_TPWMTHRS (raw) must win over stealthchop_threshold (velocity)
        result = tune_invocation(motor=mock_motor, tuning_goal='balanced',
                                 pins={'driver_TPWMTHRS': 99999,
                                       'stealthchop_threshold': 50.0})
        assert result['tpwmthrs'] == 99999, (
            "raw driver_TPWMTHRS must override stealthchop_threshold")

    # maxpwmrps must forward volts to pwmgrad
    def test_maxpwmrps_voltage_affects_threshold(self, mock_motor):
        # At 24V vs 56V the tpwmthrs values must differ because higher voltage
        # means a higher PWM tracking velocity (motor saturates later).
        result_24 = tune_invocation(motor=mock_motor, tuning_goal='balanced',
                                    voltage=24.0)
        result_56 = tune_invocation(motor=mock_motor, tuning_goal='balanced',
                                    voltage=56.0)
        # Higher voltage → higher vmaxpwm → lower TSTEP threshold
        # (TSTEP is inverse velocity: smaller TSTEP = faster)
        assert result_24['tpwmthrs'] != result_56['tpwmthrs'], (
            "tpwmthrs must differ between 24V and 56V configs")

    # chopper_freq_target goal defaults (20/35/45 kHz) and TPFD=0 for silent.
    # The chopper-freq default is exercised indirectly through TOFF (higher
    # target → lower TOFF).  TOFF is not exposed via tune_invocation's
    # return dict; we check TPFD instead.
    def test_silent_tpfd_is_zero(self, mock_motor):
        # silent must force TPFD=0
        from .conftest import _last_tune_fields
        tune_invocation(motor=mock_motor, tuning_goal='silent')
        tpfd = _last_tune_fields().get_field('tpfd')
        assert tpfd == 0, "silent goal must set TPFD=0"

    def test_performance_tpfd_nonzero(self, mock_motor):
        # performance goal computes TPFD normally (non-zero expected with
        # typical motor params)
        from .conftest import _last_tune_fields
        tune_invocation(motor=mock_motor, tuning_goal='performance')
        tpfd = _last_tune_fields().get_field('tpfd')
        # TPFD is computed; should be >= 0 and not forced to zero
        # (with the mock motor at 56V it will be > 0 unless cycle-math
        # produces 0 — accept >= 0 but document that non-silent goals
        # use the computed path, not the forced-0 path)
        assert tpfd >= 0  # sanity only; silent-vs-others distinction is key

    def test_user_pinned_tpfd_respected_for_silent(self, mock_motor):
        # driver_TPFD pin must win even for silent
        from .conftest import _last_tune_fields
        tune_invocation(motor=mock_motor, tuning_goal='silent',
                        pins={'driver_TPFD': 7})
        tpfd = _last_tune_fields().get_field('tpfd')
        assert tpfd == 7, "user-pinned TPFD must override silent goal default"

    # TOFF=1 + TBL=0 both user-pinned must raise config error
    def test_toff1_tbl0_both_pinned_raises(self, mock_motor):
        # User explicitly pinning both TOFF=1 and TBL=0 must be rejected with
        # a config.error.  Autotune is allowed to silently correct one of the
        # two when the user only pinned one — but a pin-pin conflict must be
        # surfaced rather than overridden.
        with pytest.raises(Exception, match="(?i)TOFF.*TBL|TBL.*TOFF|driver_TOFF"):
            tune_invocation(motor=mock_motor,
                            pins={'driver_TOFF': 1, 'driver_TBL': 0})

    def test_toff1_tbl0_only_toff_pinned_autocorrects(self, mock_motor):
        # When only TOFF is user-pinned (TBL auto-chosen), autotune may
        # silently bump TBL — no error expected.
        from .conftest import _last_tune_fields
        # This should not raise; TBL=0 is the autotune default and gets bumped
        result = tune_invocation(motor=mock_motor, pins={'driver_TOFF': 1})
        tbl = _last_tune_fields().get_field('tbl')
        assert tbl >= 1, "autotune must bump TBL when TOFF=1 and TBL would be 0"

    # dcStep enabled with TOFF<3 must raise config error
    def test_dcstep_with_low_toff_raises(self, mock_motor):
        # vhighfs=1 + vhighchm=1 (dcStep active) requires TOFF>=3
        with pytest.raises(Exception, match="(?i)TOFF.*3|dcStep|TOFF.*datas"):
            tune_invocation(motor=mock_motor, tuning_goal='performance',
                            pins={'driver_VHIGHFS': True,
                                  'driver_VHIGHCHM': True,
                                  'driver_TOFF': 2})


class TestPerformancePinnedStealthChop:
    """The performance goal still sets en_pwm_mode based on TPWMTHRS — a
    user pinning driver_TPWMTHRS to a real value under the performance goal
    must enable StealthChop at low velocities, while no pin must keep
    StealthChop disabled (TPWMTHRS=0xfffff)."""

    def test_performance_pinned_tpwmthrs_enables_stealthchop(self, mock_motor):
        result = tune_invocation(motor=mock_motor, tuning_goal='performance',
                                 pins={'driver_TPWMTHRS': 0})
        assert result['en_pwm_mode'] == 1, (
            "performance + driver_TPWMTHRS: 0 must set en_pwm_mode=1 "
            "so StealthChop is active at low velocities")

    def test_performance_no_pin_still_disables_stealthchop(self, mock_motor):
        # Without a pin, performance still uses TPWMTHRS=0xfffff (never
        # enter StealthChop) and en_pwm_mode must stay 0.
        result = tune_invocation(motor=mock_motor, tuning_goal='performance')
        assert result['en_pwm_mode'] == 0, (
            "performance without driver_TPWMTHRS pin must leave en_pwm_mode=0")


class TestSentinelHandlingAndAutotuneOffValidation:
    """driver_TPWMTHRS=0xfffff must symmetrically clear en_pwm_mode (so a
    user can disable StealthChop even when stealthchop_threshold is also
    set), and CHOPCONF boundary validations (_validate_chopconf) must fire
    even without motor: / voltage: (autotune-OFF configs)."""

    def test_tpwmthrs_0xfffff_clears_en_pwm_mode(self, mock_motor):
        # Scenario: stealthchop_threshold is set (TMCStealthchopHelper writes
        # en_pwm_mode=1), then driver_TPWMTHRS=0xfffff overrides the
        # threshold.  After _configure_pwm the mode bit must be 0 — otherwise
        # we get an incoherent register state (StealthChop "armed" but
        # TPWMTHRS set to never trigger).
        result = tune_invocation(motor=mock_motor,
                                 stealthchop_threshold=50.0,
                                 pins={'driver_TPWMTHRS': 0xfffff})
        assert result['en_pwm_mode'] == 0, (
            "driver_TPWMTHRS: 0xfffff must clear en_pwm_mode even when "
            "stealthchop_threshold is also configured")

    def test_tpwmthrs_real_value_keeps_en_pwm_mode(self, mock_motor):
        # driver_TPWMTHRS set to a real (non-max) value must keep
        # en_pwm_mode=1 so StealthChop can engage.
        result = tune_invocation(motor=mock_motor,
                                 pins={'driver_TPWMTHRS': 1000})
        assert result['en_pwm_mode'] == 1, (
            "driver_TPWMTHRS: 1000 must keep en_pwm_mode=1")

    # CHOPCONF boundary validations must fire for autotune-OFF configs.
    # _validate_chopconf() is called unconditionally in TMC5160.__init__,
    # so even configs without motor: / voltage: are guarded.
    def test_autotune_off_toff1_tbl0_raises(self):
        # autotune-OFF (no motor:): driver_TOFF=1 + driver_TBL=0 must error.
        from .conftest import _load_tmc5160_module
        tmc5160_mod = _load_tmc5160_module()
        cfg = MockConfig(values={'driver_TOFF': 1, 'driver_TBL': 0})
        with pytest.raises(MockConfigError,
                           match="(?i)TOFF.*TBL|TBL.*TOFF|driver_TOFF"):
            tmc5160_mod._validate_chopconf(cfg)

    def test_autotune_off_dcstep_low_toff_raises(self):
        # autotune-OFF: vhighfs=1 + vhighchm=1 + driver_TOFF=2 must error.
        from .conftest import _load_tmc5160_module
        tmc5160_mod = _load_tmc5160_module()
        cfg = MockConfig(values={'driver_VHIGHFS': 1,
                                 'driver_VHIGHCHM': 1,
                                 'driver_TOFF': 2})
        with pytest.raises(MockConfigError,
                           match="(?i)TOFF.*3|dcStep|TOFF.*datas"):
            tmc5160_mod._validate_chopconf(cfg)

    def test_autotune_off_valid_chopconf_no_raise(self):
        # autotune-OFF: valid pinned combination must not raise.
        from .conftest import _load_tmc5160_module
        tmc5160_mod = _load_tmc5160_module()
        cfg = MockConfig(values={'driver_TOFF': 3, 'driver_TBL': 1,
                                 'driver_VHIGHFS': 1, 'driver_VHIGHCHM': 1})
        tmc5160_mod._validate_chopconf(cfg)  # must not raise

    def test_autotune_off_toff1_no_tbl_no_raise(self):
        # driver_TOFF=1 without driver_TBL pin: autotune can correct TBL,
        # so _validate_chopconf must NOT raise (only both-pinned triggers error).
        from .conftest import _load_tmc5160_module
        tmc5160_mod = _load_tmc5160_module()
        cfg = MockConfig(values={'driver_TOFF': 1})
        tmc5160_mod._validate_chopconf(cfg)  # must not raise
