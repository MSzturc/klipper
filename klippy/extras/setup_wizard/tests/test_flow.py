import os, sys, unittest
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
from setup_wizard import SetupWizard  # noqa: E402

WORKSPACE = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
CONFIG_ROOT = os.path.join(WORKSPACE, "THEOS-Configuration", "config")
MAINSAIL = os.path.join(WORKSPACE, "THEOS-Configuration", "mainsail.cfg")


def setUpModule():
    with open(MAINSAIL, "w") as f:
        f.write("[virtual_sdcard]\npath: ~/printer_data/gcodes\n"
                "[pause_resume]\n[display_status]\n")


def tearDownModule():
    if os.path.exists(MAINSAIL):
        os.remove(MAINSAIL)


class FakeGcode:
    def __init__(self):
        self.out = []
        self.cmds = {}

    def register_command(self, name, func, when_not_ready=False, desc=None):
        self.cmds[name] = func

    def respond_info(self, msg, log=True):
        self.out.append(msg)


class FlowTest(unittest.TestCase):
    def _wizard(self):
        w = SetupWizard.__new__(SetupWizard)
        w.gcode = FakeGcode()
        w.config_root = CONFIG_ROOT
        w.printer_cfg = os.path.join(HERE, "_out_printer.cfg")
        w.reset_state()
        return w

    def test_easy_flow_reaches_generation(self):
        w = self._wizard()
        w.answer("printer", "t250")
        w.answer("board", "btt-kraken")     # integrated -> no free slots
        self.assertEqual(w.pending_driver_groups(), [])
        w.answer("toolhead", "dualhorn")
        w.answer("bed", "ender-3")
        text = w.finish()                   # assembles + validates
        # render emits the printer's leaves, not the meta-def include itself
        self.assertIn("[include config/kinematics/corexy.cfg]", text)

    def test_accessory_prompt_offered_after_bed(self):
        w = self._wizard()
        w.answer("printer", "t250")
        w.answer("board", "btt-kraken")
        w.answer("toolhead", "dualhorn")
        w.gcode.out = []
        w.answer("bed", "ender-3")
        w._advance("bed")
        joined = "\n".join(w.gcode.out)
        self.assertIn("action:prompt_begin Input Shaper Sensor", joined)
        self.assertIn("FYSETC Nozzle Input Shaper", joined)
        self.assertIn("VALUE=none", joined)

    def test_toolhead_prompt_excludes_incompatible_printer(self):
        w = self._wizard()
        w.answer("printer", "t250")
        w.answer("board", "btt-kraken")
        w.gcode.out = []
        w._advance("board")   # integrated board -> prompts the toolhead
        joined = "\n".join(w.gcode.out)
        self.assertIn("action:prompt_begin Toolhead", joined)
        self.assertIn("DualHorn", joined)
        self.assertIn("Scorpio", joined)
        self.assertNotIn("Standard", joined)

    def test_bed_prompt_printbed_title_and_uniform_style(self):
        w = self._wizard()
        w.answer("printer", "t250")
        w.answer("board", "btt-kraken")
        w.answer("toolhead", "dualhorn")
        w.gcode.out = []
        w._advance("toolhead")   # -> bed prompt
        out = w.gcode.out
        self.assertIn("action:prompt_begin Printbed", out)
        btns = [ln for ln in out if ln.startswith("action:prompt_button")]
        self.assertTrue(btns)
        self.assertTrue(all(b.endswith("|primary") for b in btns), out)

    def test_accessory_selection_included(self):
        w = self._wizard()
        w.answer("printer", "t250")
        w.answer("board", "btt-kraken")
        w.answer("toolhead", "dualhorn")
        w.answer("bed", "ender-3")
        w.answer("accessory", "fysetc-nis")
        text = w.finish()
        self.assertIn("config/accessories/fysetc-nis/fysetc-nis.cfg", text)

    def test_accessory_none_includes_nothing(self):
        w = self._wizard()
        w.answer("printer", "t250")
        w.answer("board", "btt-kraken")
        w.answer("toolhead", "dualhorn")
        w.answer("bed", "ender-3")
        w.answer("accessory", "none")
        text = w.finish()
        self.assertNotIn("accessories/", text)

    def test_slot_board_detected(self):
        # Foundation for the deferred slot path: free-slot groups are detected
        # even though provisioning is not wired into generation yet.
        w = self._wizard()
        w.answer("printer", "t250")
        w.set_board_meta(os.path.join(HERE, "fixtures", "slot-board",
                                      "config.cfg"))
        self.assertEqual(w.pending_driver_groups(), ["xy", "e", "z"])

    def test_slot_board_stops_after_board(self):
        # A free-slot board stops immediately after board selection — no
        # further questions, a clear message instead.
        w = self._wizard()
        w.answer("printer", "t250")
        w.answers["board"] = "slot-demo"
        w.set_board_meta(os.path.join(HERE, "fixtures", "slot-board",
                                      "config.cfg"))
        w.gcode.out = []
        w._advance("board")
        joined = "\n".join(w.gcode.out)
        self.assertIn("Manual driver setup required", joined)
        self.assertNotIn("Toolhead?", joined)


if __name__ == "__main__":
    unittest.main()
