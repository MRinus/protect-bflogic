#!/bin/bash
# Runs test_sqnt44044_0dB.conf against real hardware (hw:ADDA,0) for a
# few seconds and reads BruteFIR's own internal peak meter over its cli
# port -- the reliable way to check the real digital output level,
# instead of the Mytek's coarse front-panel LEDs.
#
# Real audio will play through the Mytek while this runs. Confirm no
# other BruteFIR instance holds the device first (pgrep -af brutefir),
# and warn whoever is nearby before running.

set -euo pipefail
cd "$(dirname "$0")"

if [ ! -f tones/tone_0dbfs_1k_2ch_input.raw ]; then
    echo "Tone file missing, generating..." >&2
    python3 generate_test_tones.py
fi

if pgrep -f 'brutefir.*test_sqnt44044_0dB.conf' >/dev/null; then
    echo "A test instance is already running -- kill it first." >&2
    exit 1
fi

/usr/local/bin/brutefir -nodefault test_sqnt44044_0dB.conf > /tmp/brutefir_test_run.log 2>&1 &
bfpid=$!
echo "started brutefir pid $bfpid"

cleanup() {
    kill "$bfpid" 2>/dev/null || true
    sleep 1
    kill -9 "$bfpid" 2>/dev/null || true
}
trap cleanup EXIT

sleep 2
{ printf 'rpk;\n'; sleep 2; printf 'ppk;\n'; sleep 1; } | timeout 8 nc -q2 localhost 3001
