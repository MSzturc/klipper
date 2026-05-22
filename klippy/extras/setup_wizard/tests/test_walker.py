import os
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_PARENT = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, PKG_PARENT)
from setup_wizard import walker  # noqa: E402

WORKSPACE = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
CONFIG_ROOT = os.path.join(WORKSPACE, "THEOS-Configuration", "config")


class WalkerTest(unittest.TestCase):
    def test_toolhead_dualhorn(self):
        path = os.path.join(CONFIG_ROOT, "toolheads", "dualhorn",
                            "toolhead.cfg")
        includes, constants = walker.walk_meta(path)
        names = [os.path.basename(p) for p in includes]
        self.assertEqual(names, [
            "part_fan_cpap.cfg", "hotend_fan.cfg", "side_blower_fan_dual.cfg",
            "t250-bmg.cfg", "rapido-uhf.cfg",
        ])
        self.assertEqual(constants.get("toolhead_label"), "DualHorn")

    def test_ignores_commented_includes(self):
        body = "[constants]\na: 1\n#[include nope.cfg]\n[include real.cfg]\n"
        tmp = tempfile.NamedTemporaryFile(mode="w", suffix=".cfg",
                                          delete=False, dir=CONFIG_ROOT)
        try:
            tmp.write(body); tmp.close()
            includes, constants = walker.walk_meta(tmp.name)
            self.assertEqual([os.path.basename(p) for p in includes],
                             ["real.cfg"])
            self.assertEqual(constants.get("a"), "1")
        finally:
            os.unlink(tmp.name)

    def test_collect_stepper_suffixes_t250(self):
        path = os.path.join(CONFIG_ROOT, "printers", "t250", "printer.cfg")
        suffixes = walker.collect_stepper_suffixes(path)
        self.assertEqual(suffixes, {"x", "x1", "y", "y1", "z", "z1", "z2"})

    def test_collect_stepper_suffixes_t100(self):
        path = os.path.join(CONFIG_ROOT, "printers", "t100", "printer.cfg")
        suffixes = walker.collect_stepper_suffixes(path)
        self.assertEqual(suffixes, {"x", "y", "z"})

    def test_inline_overrides_captures_meta_body(self):
        # The printer meta's own override sections (not includes/constants)
        # must be captured so the generator can re-emit them.
        t250 = walker.inline_overrides(
            os.path.join(CONFIG_ROOT, "printers", "t250", "printer.cfg"))
        self.assertIn("[input_shaper]", t250)
        t100 = walker.inline_overrides(
            os.path.join(CONFIG_ROOT, "printers", "t100", "printer.cfg"))
        self.assertIn("driver_SGTHRS", t100)
        self.assertIn("homing_positive_dir", t100)


if __name__ == "__main__":
    unittest.main()
