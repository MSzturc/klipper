#!/bin/bash
# Build and run the standalone TDD suite for the cruise fast-path in
# klippy/chelper/itersolve.c.  Not part of c_helper.so -- gates structural
# changes to is_linear plumbing and the itersolve_gen_steps_range_cruise
# helper.

set -eu

REPO="$(cd "$(dirname "$0")/.." && pwd)"
CHELPER="$REPO/klippy/chelper"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

CC="${CC:-gcc}"
CFLAGS="-Wall -Wextra -O2 -g -fPIC -pthread -I$CHELPER"

# Source list: every kinematic + itersolve + the helpers they pull in.
SRCS=(
    "$CHELPER/test_itersolve_cruise.c"
    "$CHELPER/itersolve.c"
    "$CHELPER/trapq.c"
    "$CHELPER/stepcompress.c"
    "$CHELPER/msgblock.c"
    "$CHELPER/pyhelper.c"
    "$CHELPER/integrate.c"
    "$CHELPER/kin_cartesian.c"
    "$CHELPER/kin_corexy.c"
    "$CHELPER/kin_corexz.c"
    "$CHELPER/kin_delta.c"
    "$CHELPER/kin_deltesian.c"
    "$CHELPER/kin_extruder.c"
    "$CHELPER/kin_generic.c"
    "$CHELPER/kin_idex.c"
    "$CHELPER/kin_polar.c"
    "$CHELPER/kin_rotary_delta.c"
    "$CHELPER/kin_shaper.c"
    "$CHELPER/kin_winch.c"
)

build_and_run() {
    local label="$1"; shift
    local bin="$OUT/test_itersolve_cruise.$label"
    $CC $CFLAGS "$@" "${SRCS[@]}" -lm -lpthread -o "$bin"
    printf '== run [%s] ==\n' "$label"
    "$bin"
}

build_and_run plain
build_and_run asan -fsanitize=address,undefined -fno-omit-frame-pointer

echo "check_itersolve_cruise: PASS"
