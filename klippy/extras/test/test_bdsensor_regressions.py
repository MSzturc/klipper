import importlib.util
import os
import sys
import types


REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", ".."))


def _load_bdsensor_module():
    if "extras.BDsensor" in sys.modules:
        return sys.modules["extras.BDsensor"]

    extras_dir = os.path.join(REPO_ROOT, "klippy", "extras")
    extras_pkg = sys.modules.setdefault("extras", types.ModuleType("extras"))
    extras_pkg.__path__ = [extras_dir]
    extras_pkg.__package__ = "extras"

    sys.modules.setdefault("chelper", types.ModuleType("chelper"))

    mcu_stub = types.ModuleType("mcu")
    mcu_stub.MCU = object
    mcu_stub.MCU_trsync = object
    sys.modules.setdefault("mcu", mcu_stub)

    manual_probe_stub = types.ModuleType("extras.manual_probe")

    class ProbeResult(tuple):
        def __new__(cls, *values):
            return tuple.__new__(cls, values)

        @property
        def bed_z(self):
            return self[2]

    manual_probe_stub.ProbeResult = ProbeResult
    sys.modules.setdefault("extras.manual_probe", manual_probe_stub)
    sys.modules.setdefault("extras.probe", types.ModuleType("extras.probe"))

    path = os.path.join(extras_dir, "BDsensor.py")
    spec = importlib.util.spec_from_file_location("extras.BDsensor", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules["extras.BDsensor"] = module
    spec.loader.exec_module(module)
    return module


class FakeGCode:
    def __init__(self, stop_on=None):
        self.messages = []
        self.scripts = []
        self._stop_on = stop_on

    def respond_info(self, msg):
        self.messages.append(msg)

    def run_script_from_command(self, script):
        self.scripts.append(script)
        if self._stop_on is not None and self._stop_on(script):
            raise _ScriptSentinel(script)


class _ScriptSentinel(Exception):
    pass


class FakeReactor:
    def monotonic(self):
        return 0.0


class FakeToolhead:
    def __init__(self):
        self.position = [0., 0., 5.]
        self.set_positions = []
        self.homed_axes = ''

    def get_position(self):
        return list(self.position)

    def set_position(self, position):
        self.position = list(position)
        self.set_positions.append(list(position))

    def get_status(self, curtime):
        return {'homed_axes': self.homed_axes}

    def wait_moves(self):
        pass

    def dwell(self, t):
        pass


class FakePrinter:
    def __init__(self, toolhead):
        self.toolhead = toolhead
        self.reactor = FakeReactor()

    def lookup_object(self, name, default=None):
        if name == "toolhead":
            return self.toolhead
        return default

    def get_reactor(self):
        return self.reactor

    def command_error(self, msg):
        return RuntimeError(msg)


class FakeGcmd:
    def __init__(self, ints=None):
        self._ints = ints or {}
        self.messages = []

    def get_int(self, key, default=None):
        return self._ints.get(key, default)

    def respond_info(self, msg):
        self.messages.append(msg)

    def respond_raw(self, msg):
        self.messages.append(msg)


def _distinct_equal_pin(value):
    return str(bytearray(value.encode("ascii")), "ascii")


def test_same_sda_endstop_pin_uses_value_equality_when_finishing_home():
    bdsensor = _load_bdsensor_module()
    wrapper = object.__new__(bdsensor.BDsensorEndstopWrapper)
    toolhead = FakeToolhead()
    wrapper.printer = FakePrinter(toolhead)
    wrapper.gcode = FakeGCode()
    wrapper.sda_pin_num = _distinct_equal_pin("PG1")
    wrapper.endstop_pin_num = _distinct_equal_pin("PG1")
    assert wrapper.sda_pin_num == wrapper.endstop_pin_num
    assert wrapper.sda_pin_num is not wrapper.endstop_pin_num
    wrapper.switch_mode = 0
    wrapper.homing = 1
    wrapper.position_endstop = 0.6
    wrapper.endstop_bdsensor_offset = 0
    wrapper.stow_on_each_sample = True
    wrapper.I2C_BD_send = lambda cmd: None
    wrapper.BD_Sensor_Read = lambda fore_r: 0.6

    wrapper.multi_probe_end()

    assert toolhead.set_positions[-1][2] == 0.6
    assert wrapper.endstop_bdsensor_offset == 0


def test_firmware_live_adjust_remembers_previous_sensor_sample():
    path = os.path.join(REPO_ROOT, "src", "BD_sensor.c")
    with open(path, encoding="utf-8") as f:
        source = f.read()

    assert "sensor_z_old = sensor_z;" in source
    assert "sensor_z = sensor_z_old;" not in source


def test_firmware_declares_and_uses_live_z_motor_limit():
    path = os.path.join(REPO_ROOT, "src", "BD_sensor.c")
    with open(path, encoding="utf-8") as f:
        source = f.read()

    assert "#define NUM_Z_MOTOR  6" in source
    assert "if (dat >= NUM_Z_MOTOR)" in source
    assert "if (z_index >= NUM_Z_MOTOR)" in source


def _read_bd_sensor_c():
    path = os.path.join(REPO_ROOT, "src", "BD_sensor.c")
    with open(path, encoding="utf-8") as f:
        return f.read()


def test_firmware_adjust_z_move_guards_null_stepper():
    source = _read_bd_sensor_c()
    import re
    m = re.search(
        r"void\s+adjust_z_move\s*\([^)]*\)\s*\{(.*?)\n\}\n", source, re.DOTALL)
    assert m, "adjust_z_move body not found"
    body = m.group(1)
    lookup_count = body.count("stepper_oid_lookup_bd")
    null_check_count = body.count("if (!s)") + body.count("if(!s)")
    assert null_check_count >= lookup_count, (
        "Every stepper_oid_lookup_bd() call in adjust_z_move must be "
        "followed by a NULL check; found %d lookups but only %d checks"
        % (lookup_count, null_check_count))


def test_firmware_division_paths_guard_zero_steps_per_mm():
    source = _read_bd_sensor_c()
    assert "step_adj[0].steps_per_mm <= 0" in source, (
        "adjust_z_move and adust_Z_calc divide by step_adj[0].steps_per_mm; "
        "both paths must guard against the zero/negative case")
    assert source.count("step_adj[0].steps_per_mm <= 0") >= 2, (
        "Expected guard in both adjust_z_move and adust_Z_calc")


def test_firmware_timer_bd_uinit_clears_correction_state():
    source = _read_bd_sensor_c()
    import re
    m = re.search(
        r"void\s+timer_bd_uinit\s*\([^)]*\)\s*\{(.*?)\n\}\n",
        source, re.DOTALL)
    assert m, "timer_bd_uinit body not found"
    body = m.group(1)
    assert "diff_step = 0" in body or "diff_step=0" in body, (
        "timer_bd_uinit must reset diff_step on shutdown")
    assert "adjusted_step = 0" in body or "adjusted_step=0" in body, (
        "timer_bd_uinit must reset adjusted_step on shutdown")


def test_firmware_clamps_rt_sample_time_to_positive_range():
    source = _read_bd_sensor_c()
    import re
    m = re.search(
        r"cmd\s*==\s*CMD_RT_SAMPLE_TIME[^{]*\{([^}]*)\}", source)
    assert m, "CMD_RT_SAMPLE_TIME branch not found"
    branch = m.group(1)
    compact = "".join(branch.split())
    assert "dat<1" in compact or "dat<=0" in compact, (
        "CMD_RT_SAMPLE_TIME must lower-bound dat so RT_SAMPLE_TIME never "
        "becomes zero (timer reschedule storm). Branch was: %r" % (branch,))


def test_BD_calibrate_initializes_toolhead_before_collision_branch():
    bdsensor = _load_bdsensor_module()
    wrapper = object.__new__(bdsensor.BDsensorEndstopWrapper)
    toolhead = FakeToolhead()
    toolhead.homed_axes = ''
    wrapper.printer = FakePrinter(toolhead)
    wrapper.gcode = FakeGCode(stop_on=lambda s: s != "BED_MESH_CLEAR")
    wrapper.bdversion = "V1.0.5"
    wrapper.switch_mode = 1
    wrapper.collision_calibrate = 1
    wrapper.collision_calibrating = 0
    wrapper.g28_cmd = "G28"

    try:
        wrapper.BD_calibrate(FakeGcmd())
    except _ScriptSentinel:
        pass
    except AttributeError as e:
        if "toolhead" in str(e).lower():
            raise AssertionError(
                "BD_calibrate accessed self.toolhead before initialization: %s"
                % (e,)) from e
        raise

    assert "BED_MESH_CLEAR" in wrapper.gcode.scripts
    assert any(s.startswith("G28") for s in wrapper.gcode.scripts)


def test_process_M102_switch_mode_sends_integer_commands():
    bdsensor = _load_bdsensor_module()
    wrapper = object.__new__(bdsensor.BDsensorEndstopWrapper)
    toolhead = FakeToolhead()
    wrapper.printer = FakePrinter(toolhead)
    wrapper.gcode = FakeGCode()
    wrapper.process_m102 = 0
    wrapper.position_endstop = 0.6

    sent = []
    wrapper.I2C_BD_send = lambda cmd, data=0: sent.append((cmd, data))

    wrapper.process_M102(FakeGcmd(ints={'S': -9}))

    assert sent, "process_M102 with S=-9 sent no I2C commands"
    for cmd, data in sent:
        assert isinstance(cmd, int), (
            "I2C_BD_send received non-int cmd %r (data=%r); switch-mode "
            "path must pass position_endstop as data, not as a string cmd"
            % (cmd, data))


def test_handle_command_error_preserves_original_exception_context():
    bdsensor = _load_bdsensor_module()

    class Helper(bdsensor.BDPrinterProbe):
        def __init__(self):
            self.multi_probe_pending = True

        def multi_probe_end(self):
            raise ValueError("inner failure")

    helper = Helper()
    try:
        helper._handle_command_error()
    except ValueError:
        return
    except Exception as e:
        if str(e) == "Multi-probe end":
            raise AssertionError(
                "_handle_command_error masked the original exception "
                "with a generic 'Multi-probe end' Exception")
        raise
