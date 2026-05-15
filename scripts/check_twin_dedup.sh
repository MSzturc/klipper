#!/bin/bash
# Build and run the standalone TDD suite for AWD twin-stepper deduplication.
# Not part of c_helper.so -- gates the stepcompress mirror, the itersolve
# dispatch, and the steppersync twin-pair handoff.

set -eu

REPO="$(cd "$(dirname "$0")/.." && pwd)"
CHELPER="$REPO/klippy/chelper"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

CC="${CC:-gcc}"
CFLAGS="-Wall -Wextra -O2 -g -fPIC -pthread -I$CHELPER"

SRCS=(
    "$CHELPER/test_twin_dedup.c"
    "$CHELPER/stepcompress.c"
    "$CHELPER/itersolve.c"
    "$CHELPER/steppersync.c"
    "$CHELPER/trapq.c"
    "$CHELPER/msgblock.c"
    "$CHELPER/pyhelper.c"
    "$CHELPER/integrate.c"
    "$CHELPER/kin_corexy.c"
    "$CHELPER/kin_cartesian.c"
    "$CHELPER/kin_shaper.c"
)

build_and_run() {
    local label="$1"; shift
    local extra_flags=()
    local runner=()
    # TSan on WSL2 kernels aborts with "unexpected memory mapping" unless ASLR
    # is disabled for the process.  setarch -R disables ASLR per-process without
    # requiring root and lets TSan initialize correctly.
    if [[ "$label" == "tsan" ]]; then
        extra_flags=(-fsanitize=thread -fno-omit-frame-pointer)
        # The global qm_pool has a pre-existing, benign race in slab_pool_alloc's
        # carve-fresh-chunk slow path (documented in slab_pool.h).  Suppress it
        # so the gate reliably catches new races, e.g. in the twin-handoff path.
        runner=(env "TSAN_OPTIONS=suppressions=$REPO/scripts/twin_dedup.tsan-supp" setarch -R)
    else
        extra_flags=("$@")
    fi
    local bin="$OUT/test_twin_dedup.$label"
    $CC $CFLAGS "${extra_flags[@]}" "${SRCS[@]}" -lm -lpthread -o "$bin"
    printf '== run [%s] ==\n' "$label"
    "${runner[@]}" "$bin"
}

build_and_run plain
build_and_run asan -fsanitize=address,undefined -fno-omit-frame-pointer
build_and_run tsan

echo "check_twin_dedup: PASS"
