#!/usr/bin/env bash
# Shortcut to run the TMC autotune unit tests with sensible defaults.
#
# Usage: bash scripts/run-tmc-tests.sh [pytest args...]

set -euo pipefail

cd "$(dirname "$0")/.."

if ! command -v python3 >/dev/null 2>&1; then
    echo "error: python3 not found in PATH" >&2
    exit 1
fi

if ! python3 -c "import pytest" >/dev/null 2>&1; then
    echo "error: pytest not installed; install with: pip install pytest" >&2
    exit 1
fi

exec python3 -m pytest klippy/extras/test/ -v "$@"
