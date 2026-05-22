# Dimension discovery + labels for the wizard. Board filtering lives in
# driver_model (boards_for_printer) and is re-exported here for the flow.
import os
from . import walker

_META_FILE = {
    "printers": "printer.cfg",
    "toolheads": "toolhead.cfg",
    "beds": "bed.cfg",
    "boards": "config.cfg",
    "accessories": "accessory.cfg",
    "drivers": "driver.cfg",
    "hotends": "hotend.cfg",
}
_LABEL_KEY = {
    "printers": "printer_label",
    "toolheads": "toolhead_label",
    "beds": "bed_label",
    "boards": "board_label",
    "accessories": "accessory_label",
    "drivers": "driver_label",
    "hotends": "hotend_label",
}


def discover(config_root, dimension):
    """List a dimension's options as {id, label, dir, meta, constants}."""
    base = os.path.join(config_root, dimension)
    out = []
    if not os.path.isdir(base):
        return out
    fname = _META_FILE.get(dimension, "config.cfg")
    lkey = _LABEL_KEY.get(dimension, dimension.rstrip("s") + "_label")
    for entry in sorted(os.listdir(base)):
        meta = os.path.join(base, entry, fname)
        if not os.path.isfile(meta):
            continue
        _, constants = walker.walk_meta(meta)
        out.append({
            "id": entry,
            "label": constants.get(lkey, entry),
            "dir": os.path.join(base, entry),
            "meta": meta,
            "constants": constants,
        })
    return out


def compatible(config_root, dimension, printer_id):
    """Options of a dimension whose `compatible_printers` constant lists the
    selected printer. Mounts (toolheads) and frames (beds) are printer-specific
    and not derivable from the config, so each declares its fit explicitly."""
    out = []
    for o in discover(config_root, dimension):
        ids = [s.strip()
               for s in o["constants"].get("compatible_printers", "").split(",")
               if s.strip()]
        if printer_id in ids:
            out.append(o)
    return out


def hotends_for_board(config_root, board_id):
    """Hotends whose heater need is covered by the board's heater pins. A
    dual-cartridge hotend (e.g. STD6 V2, {E, E1}) is hidden on a single-heater
    board ({E}). The fit is derived, not declared (see driver_model)."""
    from . import driver_model
    board_cfg = os.path.join(config_root, "boards", board_id, "config.cfg")
    offered = driver_model.board_heaters(board_cfg)
    return [h for h in discover(config_root, "hotends")
            if driver_model.collect_heater_aliases(h["meta"]) <= offered]


def boards_for_printer(config_root, printer_id):
    """Boards whose offered slots cover the printer's required slots
    (noise-reduction filter only — see spec section 5)."""
    from . import driver_model
    printer_meta = os.path.join(config_root, "printers", printer_id,
                                "printer.cfg")
    required = driver_model.required_slots(printer_meta)
    return [b for b in discover(config_root, "boards")
            if driver_model.board_covers(b["meta"], required)]
