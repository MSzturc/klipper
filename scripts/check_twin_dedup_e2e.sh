#!/bin/bash
# End-to-end byte-identity gate for AWD twin-stepper deduplication.  Runs
# klippy in batch mode against the AWD config twice -- once with dedup
# active, once disabled via KLIPPER_DISABLE_TWIN_DEDUP.
#
# The raw klippy -o dump is the framed serial byte stream; its block framing
# depends on host thread timing and is not reproducible run-to-run.  The
# decoded MCU command stream (parsedump.py) is fully deterministic, so the
# comparison is done there: decode both dumps, sort, and diff.  Any real
# divergence in the MCU commands -- a wrong mirror, a broken handoff, an
# unhandled force_move, a twin that emits nothing -- makes the diff fail.
#
# Requires DICTDIR to point at a directory containing atmega2560.dict.

set -eu
REPO="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO"
DICT="${DICTDIR:?set DICTDIR to the dir containing atmega2560.dict}/atmega2560.dict"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

# Extract the gcode body from the .test file (drop directives/comments).
grep -vE '^(CONFIG|DICTIONARY|GCODE|SHOULD_FAIL|#|[[:space:]]*$)' \
    test/klippy/awd_corexy.test > "$OUT/moves.gcode"

# Run once with dedup active, once with it disabled.  Logs carry
# non-deterministic timestamps -- keep them out of the comparison.
env python3 klippy/klippy.py test/klippy/awd_corexy.cfg \
    -i "$OUT/moves.gcode" -o "$OUT/dump.a" -d "$DICT" -l "$OUT/log.a"
env KLIPPER_DISABLE_TWIN_DEDUP=1 python3 klippy/klippy.py \
    test/klippy/awd_corexy.cfg \
    -i "$OUT/moves.gcode" -o "$OUT/dump.b" -d "$DICT" -l "$OUT/log.b"

# Guard against a false green: each run must produce a non-empty serial dump.
for d in a b; do
    [ -s "$OUT/dump.$d" ] || {
        echo "check_twin_dedup_e2e: FAIL -- run '$d' produced no MCU dump" >&2
        exit 1
    }
done

# Decode the framed serial dumps into the MCU command stream and sort
# (same-clock messages may be emitted in a different order between runs).
python3 klippy/parsedump.py "$DICT" "$OUT/dump.a" | sort > "$OUT/dec.a"
python3 klippy/parsedump.py "$DICT" "$OUT/dump.b" | sort > "$OUT/dec.b"

# Guard against a false green: the decoded stream must actually contain
# stepper motion -- a config that emitted no queue_step would let two empty
# decodes satisfy the diff.
grep -q 'queue_step' "$OUT/dec.a" || {
    echo "check_twin_dedup_e2e: FAIL -- decoded stream has no queue_step" >&2
    exit 1
}

if cmp -s "$OUT/dec.a" "$OUT/dec.b"; then
    echo "check_twin_dedup_e2e: PASS (decoded MCU command stream identical)"
else
    echo "check_twin_dedup_e2e: FAIL -- dedup changed the MCU command stream" >&2
    diff "$OUT/dec.a" "$OUT/dec.b" | head -40 >&2 || true
    exit 1
fi
