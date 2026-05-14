#!/bin/bash
# Build and run the standalone slab-pool TDD suite for msgblock + history_steps.
# Not part of c_helper.so — exists purely to gate changes to the pool helpers
# (klippy/chelper/slab_pool.h) and the message_alloc/message_free contract.

set -eu

REPO="$(cd "$(dirname "$0")/.." && pwd)"
CHELPER="$REPO/klippy/chelper"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

CC="${CC:-gcc}"
CFLAGS="-Wall -Wextra -O2 -g -fPIC -pthread -I$CHELPER"

# ASan run catches leaks in slab_pool_destroy and use-after-free across the
# pool boundary.  Run twice: once with stock flags, once with ASan.

build_and_run() {
    local label="$1"; shift
    local bin="$OUT/test_msgblock_pool.$label"
    $CC $CFLAGS "$@" \
        "$CHELPER/test_msgblock_pool.c" \
        "$CHELPER/msgblock.c" \
        "$CHELPER/pyhelper.c" \
        -o "$bin"
    printf '== run [%s] ==\n' "$label"
    "$bin"
}

build_and_run plain
build_and_run asan -fsanitize=address,undefined -fno-omit-frame-pointer

echo "check_msgblock_pool: PASS"
