#!/usr/bin/env python3
"""
Regenerates the raw PCM test tones used by the 0dBFS-headroom test rig in
this directory (test_sqnt44044_0dB.conf / play_direct_bypass.sh). Not
checked in as binary blobs on purpose -- just re-run this to get them.

Writes into ./tones/ (created if missing):
  tone_0dbfs_1k_8ch.raw     - 5s, 8ch, 0 dBFS 1kHz, for direct hw:ADDA,0
                              bypass playback (play_direct_bypass.sh 0)
  tone_minus10dbfs_1k_8ch.raw - same, at -10 dBFS (play_direct_bypass.sh -10)
  tone_0dbfs_1k_2ch_input.raw - 1s, 2ch, 0 dBFS 1kHz, exactly 1000 cycles
                              so it loops seamlessly -- used as the "file"
                              input for test_sqnt44044_0dB*.conf
All S32_LE, 44100 Hz.
"""
import sys
import numpy as np

SR = 44100
FREQ = 1000
INT32_MAX = 2147483647


def make_tone(seconds, amp_db, nch):
    n = SR * seconds
    t = np.arange(n) / SR
    amp = (10 ** (amp_db / 20)) * INT32_MAX
    tone = (amp * np.sin(2 * np.pi * FREQ * t)).astype(np.int32)
    return np.tile(tone[:, None], (1, nch)).astype("<i4")


def make_loop_tone(cycles, amp_db, nch):
    # exact integer number of cycles at FREQ -> seamless loop point
    n = int(round(cycles * SR / FREQ))
    t = np.arange(n) / SR
    amp = (10 ** (amp_db / 20)) * INT32_MAX
    tone = (amp * np.sin(2 * np.pi * FREQ * t)).astype(np.int32)
    return np.tile(tone[:, None], (1, nch)).astype("<i4")


def main():
    out_dir = "tones"
    import os
    os.makedirs(out_dir, exist_ok=True)

    make_tone(5, 0.0, 8).tofile(f"{out_dir}/tone_0dbfs_1k_8ch.raw")
    make_tone(5, -10.0, 8).tofile(f"{out_dir}/tone_minus10dbfs_1k_8ch.raw")
    make_loop_tone(1000, 0.0, 2).tofile(f"{out_dir}/tone_0dbfs_1k_2ch_input.raw")

    print(f"wrote 3 tone files into ./{out_dir}/")


if __name__ == "__main__":
    main()
