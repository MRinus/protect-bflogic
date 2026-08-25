# protect.bflogic

A [BruteFIR](https://github.com/atorger/brutefir) logic module that adds
per-channel peak/RMS voltage limiting for speaker and amplifier
protection, applied right before D/A conversion.

It was built for a real 4-channel active system (electrostatic panels +
sealed woofers, crossed over in BruteFIR) where different channels feed
physically different, independently vulnerable loads — an amp that
clips, a step-up transformer that saturates, a woofer with a mechanical
Xmax limit — and a single global limiter can't express any of that. This
module lets you describe the actual physical signal chain instead, and
have it compute the correct digital threshold for you.

**Status: not yet validated against real hardware.** See
[Status](#status) below before relying on this for anything that can be
damaged by a mistake.

## Why

BruteFIR has no built-in per-channel, physically-referenced limiting.
The alternatives are either a single global ceiling (wrong the moment
two channels feed different amps/transducers) or hand-computing a dBFS
threshold for every checkpoint yourself, then redoing that math by hand
every time a cable, adapter, or amplifier changes.

This module instead lets you describe the chain in physical units —
DAC output voltage, amplifier gain in dB, transformer loss — and give
each limit checkpoint a real voltage ceiling (Vrms). It converts that to
the correct normalized threshold internally, at startup, from those
parameters. Change an amp or a cable, change one gain number, not a
threshold you calculated by hand.

## Concept

- **`chain`** describes a physical signal path from the DAC's full-scale
  output (0 dBFS) through zero or more gain stages (amplifiers,
  transformers, adapters — gain in dB, can be negative for lossy
  stages) to some point in the system.
- **`group`** attaches a set of BruteFIR output channels to one chain,
  and defines one or more **`limit`** checkpoints on it. Each limit has
  a real physical ceiling (`max_vrms`) at its point in the chain; the
  module derives the digital threshold itself:

  ```
  threshold_lin = max_vrms / (dac_max_output_vrms * chain_gain_lin)
  ```

- A limit can be **frequency-weighted** (`ref_freq_hz` + `slope_db_oct`)
  for physically frequency-dependent limits — e.g. transformer core
  saturation (ceiling rises ~6 dB/octave) or a sealed-box woofer's Xmax
  above box resonance (~12 dB/octave). Implemented as a cascade of
  single-pole filters on that checkpoint's detector input only.
- A limit can be **peak** or **RMS** (independent attack/release, or
  window + release, respectively).
- Every checkpoint in a group runs in parallel each sample; the applied
  gain reduction is the minimum across all of them — the strictest
  limit wins automatically.
- The module logs its computed dBFS threshold for every checkpoint at
  startup (`fprintf(stderr, ...)`) — **always check this log** against
  your own expectations before trusting a new config.

See [`examples/protect_example_config.txt`](examples/protect_example_config.txt)
for a fully commented, real-world chain/group/limit config.

## Hook point

The module implements `bfevents.output_timed(buf, channel)` — the last
point in BruteFIR's pipeline before the signal reaches the output
device, after convolution, crossover, delays, and volume. It sees the
final signal.

**Important:** at this hook, BruteFIR's internal buffer for integer
output formats (e.g. `S32_LE`) is **not** normalized to ±1.0 — it carries
the raw integer magnitude (±2^31 for `S32_LE`), regardless of what
BruteFIR's general documentation says about integer format scaling
elsewhere in the pipeline. The module compensates via an `output_scale`
config field (default `2147483647.0`, i.e. `S32_LE`) — set this
explicitly if your config uses a different output format.

## Build

Requires a checkout of the [BruteFIR source](https://github.com/atorger/brutefir)
(only `src/bfmod.h` is needed — nothing links against BruteFIR itself;
the module is `dlopen`'d by BruteFIR at runtime via its `modules_path`
config setting):

```sh
make BRUTEFIR_SRC=/path/to/brutefir
```

This produces `protect.bflogic`. Copy or symlink it into the
`modules_path` directory your BruteFIR config points at.

Built and tested against BruteFIR 1.1.2 (pthread-based). Should work
against any BruteFIR version whose `bfmod.h` still exposes
`output_timed()` with the same signature — not verified against older
fork()-based BruteFIR releases.

## Config

Load it like any other logic module:

```
logic: "protect" {
    output_scale: 2147483647.0;

    chain: "example_chain" {
        dac_max_output_vrms: 6.91;
        { gain_db: 20.0; };   # e.g. an amplifier stage
    };

    group: {
        channels: "left", "right";
        chain: "example_chain";
        limit: {
            name: "amp_clip"; max_vrms: 40.0; type: "peak";
            attack_ms: 0.3; release_ms: 200.0;
        };
    };
};
```

Note: BruteFIR's config lexer only understands `#` line comments, no
`/* */` block comments.

## Testing

`tests/` has a software-only dry-test setup: synthetic tones at known
levels (some deliberately above/below each configured threshold),
played through BruteFIR with this module loaded and file output instead
of a real device, then checked against BruteFIR's own CLI peak meter.
This is how the two bugs below were originally found — run this (or
something like it) against any change before trusting it with real
speakers.

```sh
python3 tests/generate_test_tones.py
tests/measure_peak_via_cli.sh   # needs a real BruteFIR + audio device
```

`tests/play_direct_bypass.sh` plays a tone straight to the output device
with the module bypassed, as a reference point.

## Status

- Dry-tested in software (synthetic tones, file output, no hardware)
  against known expected levels — confirmed correct gain/threshold
  behavior for both peak and RMS checkpoints, frequency-weighted and
  not.
- **Not yet validated against real hardware** (e.g. an oscilloscope
  check of actual output voltage under a triggered limit). Treat any
  config's computed thresholds as unverified until you've done that
  check yourself.
- Two real bugs were found and fixed during dry-testing, worth knowing
  about if you're auditing the source:
  1. Limit gain state initialized to `0.0` instead of `1.0`, causing a
     spurious full mute on every start/reload until one full release
     time constant elapsed.
  2. The `output_timed()` buffer scaling issue described above
     (integer magnitude, not ±1.0-normalized) — without the fix, every
     real signal permanently collapses gain to zero regardless of
     level.
- All example values in `examples/protect_example_config.txt` marked
  "provisional" are estimates from public component data, not measured —
  replace them with your own measured or manufacturer-confirmed values
  before relying on them.

## License

ISC, matching BruteFIR's own license. See [LICENSE](LICENSE).
