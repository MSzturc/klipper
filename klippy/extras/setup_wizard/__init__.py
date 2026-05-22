# THEOS setup wizard host extra.
#
# Drives an action:prompt question flow and generates a thin leaf-include
# printer.cfg. All hardware compatibility is derived from the config tree
# (see driver_model): boards are filtered by required-vs-offered driver slots,
# and the driver type of an integrated slot is read from the board's [tmc####]
# sections. Free-slot detection (free_slot_groups) runs at board selection;
# full slot provisioning + the per-group driver question are a planned
# follow-up (see "Scope — Iteration 1"), so a board with free slots stops with
# a clear message rather than writing an unusable config.
import os
import logging
from . import catalog, driver_model, generator, prompts

EASY_STEPS = ["printer", "board", "toolhead", "hotend", "bed", "accessory"]


class SetupWizard:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        # config_root: the THEOS-Configuration config tree, symlinked into
        # printer_data/config/config on the device.
        self.config_root = os.path.expanduser(
            config.get('config_root', '~/printer_data/config/config'))
        self.printer_cfg = os.path.expanduser(
            config.get('printer_cfg', '~/printer_data/config/printer.cfg'))
        self.reset_state()
        self.gcode.register_command(
            'SETUP_WIZARD', self.cmd_SETUP_WIZARD, when_not_ready=True,
            desc="Start the THEOS configuration wizard")
        self.gcode.register_command(
            'WIZARD_ANSWER', self.cmd_WIZARD_ANSWER, when_not_ready=True,
            desc="Answer the current wizard question")

    # --- state ---
    def reset_state(self):
        self.answers = {}
        self.board_meta = None
        self.driver_groups = []        # free-slot groups detected on the board

    def answer(self, key, value):
        self.answers[key] = value
        if key == "board":
            self.set_board_meta(os.path.join(self.config_root, "boards",
                                             value, "config.cfg"))

    def set_board_meta(self, meta_path):
        self.board_meta = meta_path
        printer_meta = os.path.join(self.config_root, "printers",
                                    self.answers["printer"], "printer.cfg")
        self.driver_groups = driver_model.free_slot_groups(printer_meta,
                                                           meta_path)

    def pending_driver_groups(self):
        return self.driver_groups

    # --- generation ---
    def _selection(self):
        # Only the printer-id seed; every domain constant comes from meta-defs.
        acc = self.answers.get("accessory")
        accessories = [] if acc in (None, "none") else [acc]
        return generator.Selection(
            config_root=self.config_root,
            printer=self.answers["printer"],
            board=self.answers["board"],
            toolhead=self.answers["toolhead"],
            hotend=self.answers["hotend"],
            bed=self.answers["bed"],
            accessories=accessories,
            constants={"printer": self.answers["printer"]})

    def finish(self):
        sel = self._selection()
        text = generator.render(sel)
        generator.validate(text, self.config_root)   # pre-write gate
        for w in sel.warnings:
            logging.warning("setup_wizard: %s", w)
        return text

    def _write(self, text):
        # One-shot: back up an existing printer.cfg, then write fresh.
        if os.path.exists(self.printer_cfg):
            import time
            os.rename(self.printer_cfg,
                      self.printer_cfg + time.strftime("-%Y%m%d_%H%M%S"))
        with open(self.printer_cfg, "w") as f:
            f.write(text)

    # --- gcode commands (action:prompt I/O) ---
    def _emit(self, lines):
        for ln in lines:
            self.gcode.respond_info(ln)

    def cmd_SETUP_WIZARD(self, gcmd):
        self.reset_state()
        self._emit(self._prompt_for("printer"))

    def cmd_WIZARD_ANSWER(self, gcmd):
        key = gcmd.get("KEY")
        self.answer(key, gcmd.get("VALUE"))
        self._advance(key)

    def _advance(self, key):
        # A board with free (non-integrated) slots cannot be auto-provisioned
        # yet — stop right after board selection with a clear message instead
        # of asking the remaining questions (see "Scope — Iteration 1").
        if key == "board" and self.driver_groups:
            self._emit(prompts.dialog(
                "Manual driver setup required",
                "This board has free driver slots (%s) without soldered "
                "drivers. Automatic stepstick provisioning is a planned "
                "follow-up; configure those drivers manually for now."
                % ", ".join(self.driver_groups)))
            return
        nxt = self._next_step(key)
        if nxt is not None:
            self._emit(self._prompt_for(nxt))
            return
        self._write(self.finish())
        self._emit(["action:prompt_end"])
        self.gcode.respond_info("Wizard done. Restarting...")
        self.gcode.run_script_from_command("RESTART")

    def _next_step(self, answered_key):
        idx = EASY_STEPS.index(answered_key)
        return EASY_STEPS[idx + 1] if idx + 1 < len(EASY_STEPS) else None

    def _prompt_for(self, step):
        if step == "printer":
            opts = catalog.discover(self.config_root, "printers")
            return prompts.dialog(
                "Printer", "Select your Printer:",
                [(o["label"], "WIZARD_ANSWER KEY=printer VALUE=%s" % o["id"],
                  "primary") for o in opts])
        if step == "board":
            opts = catalog.boards_for_printer(self.config_root,
                                              self.answers["printer"])
            return prompts.dialog(
                "Board", "Select your Printer Mainboard:",
                [(o["label"], "WIZARD_ANSWER KEY=board VALUE=%s" % o["id"],
                  "primary") for o in opts])
        if step == "toolhead":
            opts = catalog.compatible(self.config_root, "toolheads",
                                      self.answers["printer"])
            return prompts.dialog(
                "Toolhead", "Select your Toolhead:",
                [(o["label"], "WIZARD_ANSWER KEY=toolhead VALUE=%s" % o["id"],
                  "primary") for o in opts])
        if step == "hotend":
            opts = catalog.hotends_for_board(self.config_root,
                                             self.answers["board"])
            return prompts.dialog(
                "Hotend", "Select your Hotend:",
                [(o["label"], "WIZARD_ANSWER KEY=hotend VALUE=%s" % o["id"],
                  "primary") for o in opts])
        if step == "bed":
            opts = catalog.compatible(self.config_root, "beds",
                                      self.answers["printer"])
            return prompts.dialog(
                "Printbed", "Select your Printbed:",
                [(o["label"], "WIZARD_ANSWER KEY=bed VALUE=%s" % o["id"],
                  "primary") for o in opts])
        if step == "accessory":
            opts = catalog.discover(self.config_root, "accessories")
            buttons = [(o["label"],
                        "WIZARD_ANSWER KEY=accessory VALUE=%s" % o["id"],
                        "primary") for o in opts]
            buttons.append(("None",
                            "WIZARD_ANSWER KEY=accessory VALUE=none",
                            "primary"))
            return prompts.dialog(
                "Input Shaper Sensor", "Select your Input Shaper Sensor:",
                buttons)
        return []


def load_config(config):
    return SetupWizard(config)
