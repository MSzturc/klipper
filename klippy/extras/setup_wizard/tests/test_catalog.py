import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
from setup_wizard import catalog  # noqa: E402

WORKSPACE = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
CONFIG_ROOT = os.path.join(WORKSPACE, "THEOS-Configuration", "config")


class CatalogTest(unittest.TestCase):
    def test_discover_toolheads(self):
        opts = {o["id"]: o for o in catalog.discover(CONFIG_ROOT, "toolheads")}
        self.assertIn("dualhorn", opts)
        self.assertEqual(opts["dualhorn"]["label"], "DualHorn")

    def test_discover_printers(self):
        opts = {o["id"]: o for o in catalog.discover(CONFIG_ROOT, "printers")}
        self.assertIn("t250", opts)
        self.assertEqual(opts["t250"]["label"], "T250")
        self.assertEqual(opts["t250"]["constants"]["has_toolheadboard"],
                         "false")

    def test_discover_beds(self):
        opts = {o["id"]: o for o in catalog.discover(CONFIG_ROOT, "beds")}
        self.assertIn("ender-3", opts)
        self.assertEqual(opts["ender-3"]["label"], "Ender 3")

    def test_discover_accessories(self):
        opts = {o["id"]: o
                for o in catalog.discover(CONFIG_ROOT, "accessories")}
        self.assertEqual(opts["fysetc-nis"]["label"],
                         "FYSETC Nozzle Input Shaper")
        self.assertEqual(opts["fysetc-pis"]["label"],
                         "FYSETC Portable Input Shaper")

    def test_toolheads_compatible_with_t250(self):
        ids = {o["id"]
               for o in catalog.compatible(CONFIG_ROOT, "toolheads", "t250")}
        self.assertEqual(ids, {"dualhorn", "scorpio"})

    def test_toolheads_compatible_with_t100(self):
        ids = {o["id"]
               for o in catalog.compatible(CONFIG_ROOT, "toolheads", "t100")}
        self.assertEqual(ids, {"standard"})

    def test_beds_compatible_per_printer(self):
        self.assertEqual(
            {o["id"] for o in catalog.compatible(CONFIG_ROOT, "beds", "t250")},
            {"ender-3"})
        self.assertEqual(
            {o["id"] for o in catalog.compatible(CONFIG_ROOT, "beds", "t100")},
            {"ender-2-pro"})


if __name__ == "__main__":
    unittest.main()
