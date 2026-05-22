# Assemble a thin, leaf-include printer.cfg from a wizard Selection, then
# statically parse it through the fork parser before it is written.
import os
import sys
from . import walker

# Constants that are wizard metadata, not runtime config — never emitted.
_META_CONSTANTS = ("has_toolheadboard", "compatible_printers")


class Selection:
    def __init__(self, config_root, printer, board, toolhead, hotend, bed,
                 accessories=None, constants=None):
        self.config_root = config_root
        self.printer = printer
        self.board = board
        self.toolhead = toolhead
        self.hotend = hotend
        self.bed = bed
        self.accessories = list(accessories or [])
        self.constants = dict(constants or {})
        self.warnings = []
        # Note: the stock probe is part of the printer meta-def (it is not a
        # wizard question). Slot-driver provisioning (drivers={...}) is a
        # planned follow-up — see "Scope — Iteration 1".


def _rel(config_root, path):
    """Path relative to config_root's parent -> '[include config/...]'."""
    rel = os.path.relpath(path, os.path.dirname(config_root))
    return rel.replace(os.sep, "/")


def _meta(config_root, dimension, ident, meta_file):
    return os.path.join(config_root, dimension, ident, meta_file)


def render(sel):
    """Return the printer.cfg text for a Selection."""
    root = sel.config_root
    sections = []      # (title, [include lines])
    overrides = []     # (title, verbatim override-section text)
    merged = {}

    def add(title, includes, constants):
        lines = ["[include %s]" % _rel(root, p) for p in includes]
        sections.append((title, lines))
        for k, v in constants.items():
            if k.endswith("_label") or k in _META_CONSTANTS:
                continue
            if k in merged and merged[k] != v:
                sel.warnings.append(
                    "constant '%s' set twice (%r -> %r)" % (k, merged[k], v))
            merged[k] = v

    def add_meta(title, meta_path):
        inc, c = walker.walk_meta(meta_path)
        add(title, inc, c)
        body = walker.inline_overrides(meta_path)
        if body:
            overrides.append((title, body))

    # Printer-agnostic base layer (macros + shaketune + pa_test) — always.
    add("Base", [os.path.join(root, "base", "essentials.cfg")], {})
    add_meta("Printer: %s" % sel.printer,
             _meta(root, "printers", sel.printer, "printer.cfg"))
    add("Board", [_meta(root, "boards", sel.board, "config.cfg")], {})
    add_meta("Toolhead: %s" % sel.toolhead,
             _meta(root, "toolheads", sel.toolhead, "toolhead.cfg"))
    # The hotend is a self-contained leaf: its [extruder]/[firmware_retraction]
    # and its wiring sub-include live inside hotend.cfg, pulled in as one line
    # (not walked/inlined) so the wiring sub-module is never surfaced.
    add("Hotend: %s" % sel.hotend,
        [_meta(root, "hotends", sel.hotend, "hotend.cfg")], {})
    add_meta("Bed: %s" % sel.bed, _meta(root, "beds", sel.bed, "bed.cfg"))
    for acc in sel.accessories:
        add_meta("Accessory: %s" % acc,
                 _meta(root, "accessories", acc, "accessory.cfg"))

    merged.update(sel.constants)   # user-supplied constants win

    out = []
    for title, lines in sections:
        out.append("# --- %s ---" % title)
        out.extend(lines)
        out.append("")
    out.append("[constants]")
    for k in sorted(merged):
        out.append("%s: %s" % (k, merged[k]))
    out.append("")
    # Meta-def override sections (e.g. printer [input_shaper] / sensorless
    # SGTHRS) emitted verbatim — walk_meta does not carry these.
    for title, body in overrides:
        out.append("# --- %s (stock overrides) ---" % title)
        out.append(body)
        out.append("")
    out.append("# --- YOUR OVERRIDES (edit freely) ---")
    out.append("# e.g. [extruder] / pressure_advance: ...")
    out.append("")
    return "\n".join(out)


def validate(text, config_root):
    """Static parse gate: raise on a structurally broken config. Writes to a
    temp file beside config_root so the emitted '[include config/...]' lines
    resolve the same way they will at runtime in printer_data/config/."""
    import tempfile
    klippy = os.path.abspath(os.path.join(config_root, "..", "..",
                                          "klipper", "klippy"))
    if klippy not in sys.path:
        sys.path.insert(0, klippy)
    import configfile
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".cfg", delete=False,
        dir=os.path.dirname(config_root))
    try:
        tmp.write(text); tmp.close()
        reader = configfile.ConfigFileReader()
        data = reader.read_config_file(tmp.name)
        return reader.build_fileconfig_with_includes(data, tmp.name)
    finally:
        os.unlink(tmp.name)
