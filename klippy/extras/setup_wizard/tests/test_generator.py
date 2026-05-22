import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
from setup_wizard import generator  # noqa: E402

WORKSPACE = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
CONFIG_ROOT = os.path.join(WORKSPACE, "THEOS-Configuration", "config")
MAINSAIL = os.path.join(WORKSPACE, "THEOS-Configuration", "mainsail.cfg")


def setUpModule():
    # mainsail.cfg is a runtime symlink (mainsail-config) absent from the repo;
    # essentials.cfg includes it, so provide a stand-in for the static parse.
    with open(MAINSAIL, "w") as f:
        f.write("[virtual_sdcard]\npath: ~/printer_data/gcodes\n"
                "[pause_resume]\n[display_status]\n")


def tearDownModule():
    if os.path.exists(MAINSAIL):
        os.remove(MAINSAIL)


def parse_pair(v):
    a, b = v.split(",")
    return float(a), float(b)


class GeneratorTest(unittest.TestCase):
    def _t250_selection(self):
        # Minimal seed only: every domain constant comes from the meta-defs.
        return generator.Selection(
            config_root=CONFIG_ROOT,
            printer="t250", board="btt-kraken", toolhead="dualhorn",
            bed="ender-3", constants={"printer": "t250"})

    def _t100_selection(self):
        return generator.Selection(
            config_root=CONFIG_ROOT,
            printer="t100", board="btt-skr-pico", toolhead="standard",
            bed="ender-2-pro", constants={"printer": "t100"})

    def test_render_has_leaf_includes_seam_and_overrides(self):
        text = generator.render(self._t250_selection())
        self.assertIn("[include config/kinematics/corexy.cfg]", text)
        self.assertIn("[include config/hotends/rapido-uhf.cfg]", text)
        self.assertIn("[include config/boards/btt-kraken/config.cfg]", text)
        # the printer-agnostic base layer is always emitted
        self.assertIn("[include config/base/essentials.cfg]", text)
        # the stock probe rides along inside the printer meta-def
        self.assertIn("[include config/probes/bdsensor.cfg]", text)
        # the printer meta's inline override sections survive into the output
        self.assertIn("[input_shaper]", text)
        self.assertIn("YOUR OVERRIDES", text)
        # wizard-only metadata never leaks into the output
        self.assertNotIn("printer_label", text)
        self.assertNotIn("has_toolheadboard", text)
        self.assertNotIn("compatible_printers", text)

    def test_t250_render_parses_and_matches_legacy_mesh(self):
        # With only {printer: t250} seeded, the assembled config (incl. the
        # base-layer macros) parses and yields the known mesh box.
        text = generator.render(self._t250_selection())
        fc = generator.validate(text, CONFIG_ROOT)
        mn = parse_pair(fc.get("bed_mesh", "mesh_min"))
        mx = parse_pair(fc.get("bed_mesh", "mesh_max"))
        self.assertAlmostEqual(mn[0], 16.875, places=3)
        self.assertAlmostEqual(mx[1], 197.0, places=3)

    def test_base_render_omits_accel_modules_without_accessory(self):
        # shaketune/resonance_tester ride along with the accel accessory, not
        # the base layer, so a config without one boots clean.
        fc = generator.validate(generator.render(self._t250_selection()),
                                CONFIG_ROOT)
        self.assertFalse(fc.has_section("shaketune"))
        self.assertFalse(fc.has_section("resonance_tester"))

    def test_accessory_render_enables_shaketune_and_resonance(self):
        sel = self._t250_selection()
        sel.accessories = ["fysetc-nis"]
        fc = generator.validate(generator.render(sel), CONFIG_ROOT)
        self.assertTrue(fc.has_section("shaketune"))
        self.assertTrue(fc.has_section("resonance_tester"))
        self.assertTrue(fc.has_section("adxl345"))

    def test_t250_probe_carries_required_z_offset(self):
        # BDsensor.load_config reads z_offset with no default, so the assembled
        # config must supply it. The legacy template did this via a [BDsensor]
        # override; in the modular model the probe leaf must carry it itself.
        fc = generator.validate(generator.render(self._t250_selection()),
                                CONFIG_ROOT)
        self.assertTrue(fc.has_section("BDsensor"))
        self.assertEqual(float(fc.get("BDsensor", "z_offset")), 0.0)

    def test_t100_render_parses_with_overrides(self):
        # Full t100 render incl. base-layer macros must parse (the macros need
        # print_volume_y/center_x — supplied by the t100 printer meta), and the
        # stock sensorless override must survive into the output.
        text = generator.render(self._t100_selection())
        fc = generator.validate(text, CONFIG_ROOT)
        self.assertTrue(fc.has_section("extruder"))
        self.assertEqual(fc.get("tmc2209 stepper_x", "driver_SGTHRS"), "90")


if __name__ == "__main__":
    unittest.main()
