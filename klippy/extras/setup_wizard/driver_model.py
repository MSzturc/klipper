# Driver derivation — the heart of the compatibility model. Nothing is
# declared; everything is read from the existing config building blocks:
#   * required slots  = single-token [stepper_*] motion sections the printer's
#                       axis/Z leaves define (recursively), uppercased, plus 'E'
#                       unless the printer declares has_toolheadboard (then the
#                       extruder driver lives on the toolhead board, not here).
#   * board slots     = <PREFIX>_STEP pin aliases the board declares.
#   * driver type     = the tmc#### of [tmc#### stepper_<p>] / [tmc#### extruder]
#                       in the board cfg; absence (with a pin alias present)
#                       means a free slot the wizard asks about.
import os
import re
from . import walker

_STEP_ALIAS_RE = re.compile(r"\b([A-Za-z][A-Za-z0-9]*)_STEP\b")
_HEATER_ALIAS_RE = re.compile(r"\b([A-Za-z][A-Za-z0-9]*)_HEATER\b")
_TMC_SECT_RE = re.compile(r"^(tmc\d+)\s+(stepper_\w+|extruder)$", re.IGNORECASE)
_TRUE = ("true", "1", "yes", "on")

# Wiring convention: which slot belongs to which driver question group.
_SLOT_GROUP = {"X": "xy", "X1": "xy", "Y": "xy", "Y1": "xy",
               "E": "e", "Z": "z", "Z1": "z", "Z2": "z"}
_GROUP_ORDER = ("xy", "e", "z")


def _section_headers(path):
    out = []
    with open(path, "r") as f:
        for raw in f:
            line = walker._strip_comment(raw)
            m = walker._SECT_RE.match(line)
            if m:
                out.append(m.group(1).strip())
    return out


def board_slots(board_cfg):
    """Offered driver slots = prefixes of <PREFIX>_STEP pin aliases.
    Comments are stripped per line so a commented-out alias is not counted."""
    slots = set()
    with open(board_cfg, "r") as f:
        for raw in f:
            line = walker._strip_comment(raw)
            slots.update(m.group(1).upper()
                         for m in _STEP_ALIAS_RE.finditer(line))
    return slots


def board_heaters(board_cfg):
    """Extruder heater slots a board offers = prefixes of <PREFIX>_HEATER pin
    aliases, excluding the bed heater. Mirrors board_slots for the indexed-alias
    convention (E, E1, ...), which is why E1_HEATER must not carry a _2 suffix."""
    out = set()
    with open(board_cfg, "r") as f:
        for raw in f:
            line = walker._strip_comment(raw)
            out.update(m.group(1).upper()
                       for m in _HEATER_ALIAS_RE.finditer(line))
    out.discard("BED")
    return out


def collect_heater_aliases(path, _seen=None):
    """Recursively follow includes from a hotend leaf and return the set of
    <PREFIX>_HEATER alias prefixes its wiring references (excl. the bed):
    default_wiring -> {E}, dual_wiring -> {E, E1}. Symmetric to board_heaters,
    so a hotend fits a board when its heater set is covered by the board's."""
    if _seen is None:
        _seen = set()
    path = os.path.abspath(path)
    if path in _seen or not os.path.isfile(path):
        return set()
    _seen.add(path)
    out = set()
    base = os.path.dirname(path)
    with open(path, "r") as f:
        for raw in f:
            line = walker._strip_comment(raw)
            m = walker._SECT_RE.match(line)
            if m:
                inc = walker._INCLUDE_RE.match(m.group(1).strip())
                if inc:
                    spec = inc.group(1).strip()
                    if not spec.startswith("if:"):
                        child = os.path.normpath(os.path.join(base, spec))
                        out |= collect_heater_aliases(child, _seen)
                continue
            out.update(m.group(1).upper()
                       for m in _HEATER_ALIAS_RE.finditer(line))
    out.discard("BED")
    return out


def board_drivers(board_cfg):
    """Integrated drivers as {slot_prefix: tmc_type} from the board's
    [tmc#### stepper_*] / [tmc#### extruder] sections."""
    out = {}
    for header in _section_headers(board_cfg):
        m = _TMC_SECT_RE.match(header)
        if not m:
            continue
        tmc, name = m.group(1).lower(), m.group(2).lower()
        prefix = "E" if name == "extruder" else name[len("stepper_"):].upper()
        out[prefix] = tmc
    return out


def required_slots(printer_cfg):
    """Driver slots a printer meta-definition requires from the main board."""
    req = {s.upper() for s in walker.collect_stepper_suffixes(printer_cfg)}
    _, consts = walker.walk_meta(printer_cfg)
    # Without a toolhead board the extruder driver must sit on the main board,
    # so the main board has to provide an E slot. A toolhead board carries the
    # extruder driver itself, so the main board does not need one.
    if consts.get("has_toolheadboard", "false").strip().lower() not in _TRUE:
        req.add("E")
    return req


def board_covers(board_cfg, required):
    """True when the board offers every required slot (set cover, not count)."""
    return set(required) <= board_slots(board_cfg)


def free_slot_groups(printer_cfg, board_cfg):
    """Ordered driver-question groups (xy/e/z) that have at least one
    required-but-not-integrated slot — i.e. a free slot the wizard must ask
    about. Integrated boards (all required slots soldered) return []."""
    required = required_slots(printer_cfg)
    integrated = set(board_drivers(board_cfg))
    groups = []
    for slot in required:
        if slot in integrated:
            continue
        g = _SLOT_GROUP.get(slot)
        if g and g not in groups:
            groups.append(g)
    return [g for g in _GROUP_ORDER if g in groups]
