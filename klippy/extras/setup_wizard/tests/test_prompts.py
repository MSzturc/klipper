import os, sys, unittest
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "..", "..")))
from setup_wizard import prompts  # noqa: E402


class PromptsTest(unittest.TestCase):
    def test_dialog_lines(self):
        lines = prompts.dialog(
            title="Toolhead?",
            text="Choose your toolhead",
            buttons=[("T250 Stock",
                      "WIZARD_ANSWER KEY=toolhead VALUE=t250-stock", "primary"),
                     ("Dragonfly",
                      "WIZARD_ANSWER KEY=toolhead VALUE=dragonfly",
                      "secondary")],
            footer=[("Back", "WIZARD_BACK", "info")])
        self.assertEqual(lines[0], "action:prompt_begin Toolhead?")
        self.assertIn("action:prompt_text Choose your toolhead", lines)
        self.assertIn(
            "action:prompt_button T250 Stock|"
            "WIZARD_ANSWER KEY=toolhead VALUE=t250-stock|primary", lines)
        self.assertIn(
            "action:prompt_footer_button Back|WIZARD_BACK|info", lines)
        self.assertEqual(lines[-1], "action:prompt_show")
