# Provenance-preserving include walker for wizard meta-definitions.
#
# Klipper's configfile parser merges includes and discards which file a value
# came from, so the wizard reads meta-definitions itself: walk_meta returns the
# meta-def's *direct* [include ...] targets (the leaf files the wizard emits one
# per line) plus its own [constants]; collect_stepper_suffixes follows includes
# recursively to find the motion steppers an axis/Z config defines.
import os
import re

_SECT_RE = re.compile(r"^\s*\[([^\]]+)\]\s*$")
_INCLUDE_RE = re.compile(r"^include\s+(.*)$")
_OPT_RE = re.compile(r"^\s*([^:#]+?)\s*[:=]\s*(.*?)\s*$")
# Single-token motion stepper: [stepper_x]. NOT [tmc5160 stepper_x] (two tokens).
_STEPPER_SECT_RE = re.compile(r"^stepper_(\w+)$")


def _strip_comment(line):
    pos = line.find('#')
    return line if pos < 0 else line[:pos]


def walk_meta(path):
    """Return (includes, constants) for a meta-definition .cfg.

    includes: ordered absolute, normalized paths of the meta-def's direct,
              non-conditional [include ...] targets.
    constants: dict of the meta-def's own [constants] options.
    Conditional includes ([include if:...]) are skipped — meta-defs use plain
    includes; composition is the wizard's job.
    """
    base = os.path.dirname(os.path.abspath(path))
    includes = []
    constants = {}
    section = None
    with open(path, "r") as f:
        for raw in f:
            line = _strip_comment(raw)
            m = _SECT_RE.match(line)
            if m:
                header = m.group(1).strip()
                inc = _INCLUDE_RE.match(header)
                if inc:
                    spec = inc.group(1).strip()
                    if spec.startswith("if:"):
                        section = None
                        continue
                    includes.append(os.path.normpath(os.path.join(base, spec)))
                    section = None
                else:
                    section = header.lower()
                continue
            if section == "constants":
                o = _OPT_RE.match(line)
                if o:
                    constants[o.group(1).strip()] = o.group(2).strip()
    return includes, constants


def collect_stepper_suffixes(path, _seen=None):
    """Recursively follow includes from `path` and return the set of suffixes
    of single-token [stepper_<suffix>] motion sections (lowercase).

    Two-token headers like 'tmc5160 stepper_x' or 'autotune_tmc stepper_x' are
    driver/tuning sections, not motion steppers, and are ignored. Conditional
    includes are skipped (meta-defs use plain includes).

    Note: this follows EVERY include the printer meta-def pulls, not only the
    axis/Z leaves. That is correct as long as the only files defining a
    single-token [stepper_*] are the axis/Z motion configs (true today). A
    future leaf that introduces an unrelated single-token [stepper_*] would be
    counted as a required slot — keep motion steppers confined to axis/Z."""
    if _seen is None:
        _seen = set()
    path = os.path.abspath(path)
    if path in _seen or not os.path.isfile(path):
        return set()
    _seen.add(path)
    suffixes = set()
    base = os.path.dirname(path)
    with open(path, "r") as f:
        for raw in f:
            line = _strip_comment(raw)
            m = _SECT_RE.match(line)
            if not m:
                continue
            header = m.group(1).strip()
            inc = _INCLUDE_RE.match(header)
            if inc:
                spec = inc.group(1).strip()
                if spec.startswith("if:"):
                    continue
                child = os.path.normpath(os.path.join(base, spec))
                suffixes |= collect_stepper_suffixes(child, _seen)
                continue
            sm = _STEPPER_SECT_RE.match(header)
            if sm:
                suffixes.add(sm.group(1).lower())
    return suffixes


def inline_overrides(path):
    """Return the verbatim text of a meta-def's own override sections — every
    section that is neither [constants] nor an [include ...] line (spec
    section 4 'eigene Defaults', e.g. a printer meta's [input_shaper] or the
    t100 [tmc2209 ...] driver_SGTHRS overrides). walk_meta only returns
    includes + constants, so the generator emits this body separately;
    otherwise these sections would be silently dropped from printer.cfg."""
    out = []
    keep = False
    with open(path, "r") as f:
        for raw in f:
            line = raw.rstrip("\n")
            m = _SECT_RE.match(_strip_comment(raw))
            if m:
                header = m.group(1).strip()
                if header.lower() == "constants" or _INCLUDE_RE.match(header):
                    keep = False
                    continue
                keep = True
                out.append(line)
                continue
            if keep:
                out.append(line)
    return "\n".join(out).strip()
