#!/bin/sh
# Developer test runner — Kalman simulation + firmware build.
# Usage: ./test.sh
# Must be run from the repository root.

if [ ! -f "src/version.h" ]; then
    echo "Error: This script must be run from the repository root."
    exit 1
fi

PASS=0
FAIL=0
KALMAN_BIN=/tmp/imuf_kalman_test

separator() { echo "=========================================================="; }

# --- 1. Native Kalman filter simulation ---
separator
echo "Kalman simulation (native x86)"
separator

echo "Compiling test_kalman..."
if (cd test_kalman && gcc -o "$KALMAN_BIN" libfixmath/fix16.c test.c -O0 -lpthread 2>&1); then
    echo "Running test_kalman..."
    if (cd test_kalman && "$KALMAN_BIN") > /dev/null 2>&1; then
        echo "✅ Kalman simulation passed."
        PASS=$((PASS + 1))
    else
        echo "❌ Kalman simulation FAILED (non-zero exit)."
        FAIL=$((FAIL + 1))
    fi
else
    echo "❌ Kalman compile FAILED."
    FAIL=$((FAIL + 1))
fi

echo ""

# --- 2. Firmware build (GCC 13 toolchain, download if needed) ---
separator
echo "Firmware build — arm-gnu-toolchain 13.3"
separator

if ./build.sh; then
    PASS=$((PASS + 1))
else
    echo "❌ Firmware build FAILED."
    FAIL=$((FAIL + 1))
fi

echo ""
separator
echo "Results: ${PASS} passed, ${FAIL} failed."
separator

[ "$FAIL" -eq 0 ]
