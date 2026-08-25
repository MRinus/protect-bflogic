#!/bin/bash
# Plays a known-dBFS 1kHz tone straight to hw:ADDA,0, bypassing BruteFIR
# entirely -- for visually checking the Mytek's front-panel meter against
# a known reference level. Non-destructive, doesn't touch any real config.
#
# IMPORTANT: confirm no BruteFIR instance already holds hw:ADDA,0 first
# (pgrep -af brutefir / fuser -v /dev/snd/pcmC0D0p), and warn whoever is
# about to watch the meter before playback starts -- this makes real
# sound on real speakers.
#
# Usage: play_direct_bypass.sh [0|-10] [--loop]   (dBFS level, default 0)
# --loop: replays the tone back-to-back until Ctrl+C (or SIGTERM), for
# trimpot calibration where you need a steady tone while adjusting and
# reading a meter -- otherwise it's just the one ~5s burst.

set -euo pipefail
cd "$(dirname "$0")"

level="0"
loop=""
for arg in "$@"; do
    case "$arg" in
        --loop) loop="1" ;;
        0|-10)  level="$arg" ;;
        *)      echo "Usage: $0 [0|-10] [--loop]" >&2; exit 1 ;;
    esac
done

case "$level" in
    0)   file="tones/tone_0dbfs_1k_8ch.raw" ;;
    -10) file="tones/tone_minus10dbfs_1k_8ch.raw" ;;
esac

if [ ! -f "$file" ]; then
    echo "Tone file missing, generating..." >&2
    python3 generate_test_tones.py
fi

if [ -n "$loop" ]; then
    trap 'echo "Stopped."; exit 0' INT TERM
    echo "Looping ${level} dBFS 1kHz to all 8 channels of hw:ADDA,0 -- Ctrl+C to stop ..."
    while true; do
        aplay -q -D hw:ADDA,0 -f S32_LE -r 44100 -c 8 "$file"
    done
else
    echo "Playing ${level} dBFS 1kHz to all 8 channels of hw:ADDA,0 ..."
    aplay -D hw:ADDA,0 -f S32_LE -r 44100 -c 8 "$file"
fi
