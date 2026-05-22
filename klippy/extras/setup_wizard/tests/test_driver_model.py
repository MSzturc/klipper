import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
from setup_wizard import driver_model, catalog  # noqa: E402

WORKSPACE = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
CONFIG_ROOT = os.path.join(WORKSPACE, "THEOS-Configuration", "config")


def board(name):
    return os.path.join(CONFIG_ROOT, "boards", name, "config.cfg")


def printer(name):
    return os.path.join(CONFIG_ROOT, "printers", name, "printer.cfg")


class DriverModelTest(unittest.TestCase):
    def test_board_slots(self):
        self.assertEqual(driver_model.board_slots(board("btt-kraken")),
                         {"X", "X1", "Y", "Y1", "Z", "Z1", "Z2", "E"})
        self.assertEqual(driver_model.board_slots(board("btt-skr-pico")),
                         {"X", "Y", "Z", "E"})

    def test_board_drivers(self):
        kraken = driver_model.board_drivers(board("btt-kraken"))
        self.assertEqual(set(kraken),
                         {"X", "X1", "Y", "Y1", "Z", "Z1", "Z2", "E"})
        self.assertTrue(all(v == "tmc5160" for v in kraken.values()))
        pico = driver_model.board_drivers(board("btt-skr-pico"))
        self.assertEqual(set(pico), {"X", "Y", "Z", "E"})
        self.assertTrue(all(v == "tmc2209" for v in pico.values()))

    def test_required_slots(self):
        self.assertEqual(driver_model.required_slots(printer("t250")),
                         {"X", "X1", "Y", "Y1", "Z", "Z1", "Z2", "E"})
        self.assertEqual(driver_model.required_slots(printer("t100")),
                         {"X", "Y", "Z", "E"})

    def test_boards_for_printer(self):
        t250 = {b["id"] for b in catalog.boards_for_printer(CONFIG_ROOT, "t250")}
        self.assertIn("btt-kraken", t250)
        self.assertNotIn("btt-skr-pico", t250)
        t100 = {b["id"] for b in catalog.boards_for_printer(CONFIG_ROOT, "t100")}
        self.assertIn("btt-kraken", t100)
        self.assertIn("btt-skr-pico", t100)
        # ADXL/accessory boards have no _STEP aliases -> never offered.
        for b in t250 | t100:
            self.assertTrue(driver_model.board_slots(board(b)))


class FreeSlotGroupTest(unittest.TestCase):
    def _fixture(self, name):
        return os.path.join(HERE, "fixtures", name, "config.cfg")

    def test_integrated_board_has_no_questions(self):
        self.assertEqual(
            driver_model.free_slot_groups(printer("t250"), board("btt-kraken")),
            [])

    def test_slot_board_asks_all_groups(self):
        self.assertEqual(
            driver_model.free_slot_groups(printer("t250"),
                                          self._fixture("slot-board")),
            ["xy", "e", "z"])

    def test_hybrid_board_asks_only_free_groups(self):
        # XY soldered, E + Z free -> per-slot derivation, not per-board.
        self.assertEqual(
            driver_model.free_slot_groups(printer("t250"),
                                          self._fixture("hybrid-board")),
            ["e", "z"])


if __name__ == "__main__":
    unittest.main()
