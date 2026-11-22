# protect.bflogic

A [BruteFIR](https://github.com/atorger/brutefir) logic module for speaker
and amplifier protection. It limits the peak or RMS voltage per output
channel, directly before the D/A conversion.

The module was written for one real active system with 5 output channels
(READ, live config 2026-11-22):

| channels | load | amplifier |
|---|---|---|
| `esll`, `eslr` | electrostatic loudspeaker (ESL) panels with a 1:60 step-up transformer | tube amplifier |
| `subl`, `subr` | sealed woofer towers | Class-D amplifier (BTL) |
| `alkoven` | alcove fill woofer | NC400 Class-D amplifier |

Each channel feeds a different load that can fail in a different way: an
amplifier clips, a step-up transformer saturates, a woofer reaches its
mechanical excursion limit. One global limiter cannot express this. With
this module you describe the physical signal chain, and the module
calculates the digital threshold for each limit.

**Status:** live on the real system since 2026-08-27 (READ). The absolute
accuracy of the thresholds is not yet verified with an oscilloscope
(OPEN). Read [Status](#status) before you use the module on anything that
a mistake can damage.

## Contents

- [Why](#why)
- [Concept](#concept)
- [Units: what `max_vrms` means](#units-what-max_vrms-means)
- [Frequency weighting](#frequency-weighting)
- [Lookahead](#lookahead)
- [Channel link](#channel-link)
- [Hook point](#hook-point)
- [Calibration caveat: where is the volume control?](#calibration-caveat-where-is-the-volume-control)
- [Build](#build)
- [Config](#config)
- [Runtime control via the CLI](#runtime-control-via-the-cli)
- [Tests](#tests)
- [Status](#status)
- [Example deployment](#example-deployment)
- [License](#license)

## Why

BruteFIR has no per-channel limiting that refers to physical units. The
alternatives are:

- one global ceiling. This is wrong as soon as two channels feed
  different amplifiers or transducers.
- a dBFS threshold that you calculate by hand for each checkpoint. You
  must calculate it again each time a cable, adapter or amplifier
  changes.

This module uses physical units instead: the DAC output voltage, the
amplifier gain in dB, the loss of an adapter. Each limit checkpoint gets a
real voltage ceiling. At startup the module converts it to the digital
threshold. When an amplifier or a cable changes, you change one gain
value, not a calculated threshold.

## Concept

- A **`chain`** describes a physical signal path. It starts at the DAC
  full-scale output (0 dBFS) and goes through zero or more gain stages
  (amplifiers, transformers, adapters). Each stage has a gain in dB. A
  lossy stage has a negative gain.
- A **`group`** connects a set of BruteFIR output channels to one chain.
  It contains one or more **`limit`** checkpoints. Each limit has a
  physical ceiling (`max_vrms`) at its point in the chain. The module
  calculates the digital threshold from it:

  ```
  threshold_lin = max_vrms / (dac_max_output_vrms * chain_gain_lin)
  ```

- A limit can refer to the **first N stages of its chain only**
  (`stage: N;`, counted from 1). Use this when several limits in one group
  sit at different points of the same chain. Example: a transformer limit
  after the full chain, and the rated-power limit of the amplifier after
  stage 2. Without `stage` the limit uses the full chain. `stage`
  *replaces* the reference point. It does not add to the chain gain.
- A limit is **`peak`** (attack and release time) or **`rms`** (window
  and RMS release time).
- All checkpoints of a group run in parallel for each sample. The module
  applies the smallest gain of all checkpoints. So the strictest limit
  always wins, and you do not set a priority order.
- At startup the module writes the calculated dBFS threshold of each
  checkpoint to stderr. **Always check this log** against your own
  expectation before you trust a new config. `tests/check_config.sh` shows
  the same log offline, without BruteFIR (see [Tests](#tests)).

[`examples/protect_example_config.txt`](examples/protect_example_config.txt)
is a fully commented config of a real system (see
[Example deployment](#example-deployment)).

## Units: what `max_vrms` means

`dac_max_output_vrms` is the RMS voltage of a 0 dBFS sine at the DAC
output. The module compares the threshold with the magnitude of each
sample (0 dBFS = 1.0). So:

- `max_vrms` is the **RMS value of a sine** at the limit.
- The real **peak** ceiling at that point is √2 × `max_vrms`, for any
  signal (CALCULATED from the source).

Example for unweighted limits (READ, live config 2026-11-21):
`max_vrms: 1768` stops the stator voltage at about 2500 V peak. An
unweighted `max_vrms: 971` would stop it at about 1373 V peak. For a
weighted limit the ceiling depends on the frequency and on the detector:
see [Frequency weighting](#frequency-weighting). Example: the live
1-pole `"abs"` limit with `max_vrms: 971` at 80 Hz lets an 80 Hz sine
through up to about 1195 Vrms (MEASURED offline, fundamental at the
output), not 971 Vrms.

## Frequency weighting

A peak limit can be frequency-weighted, for limits that change with the
frequency. The field `weighting` selects one of three detectors:

| `weighting` | detector | fields | use |
|---|---|---|---|
| `"abs"` (default, if the field is absent) | `\|x\|`, then 1 or 2 one-pole low-passes at `ref_freq_hz` | `ref_freq_hz`, `slope_db_oct` 6 or 12 | old behaviour, kept unchanged |
| `"signed"` | 1 or 2 one-pole low-passes on the signed signal, then `\|.\|` | `ref_freq_hz`, `slope_db_oct` 6 or 12 | woofer excursion, transformer saturation |
| `"integrator"` | leaky integrator on the signed signal, then `\|.\|`, plus an exact flux backstop | `ref_freq_hz`, `hp_corner_hz` | transformer core flux |

Each weighting acts only on the detector of its own checkpoint. Other
checkpoints in the same group do not change.

### `"abs"` and `"signed"`

Both use a cascade of N one-pole low-passes with the same corner
frequency `ref_freq_hz`, with N = `slope_db_oct`/6. N is 1 (6 dB/octave)
or 2 (12 dB/octave). The allowed voltage rises with the frequency above
`ref_freq_hz`.

- With `"signed"`, 2 poles give the transfer function from voltage to
  diaphragm excursion of a sealed box (2 equal poles = Q 0.5; the real box
  has Qtc 0.49, CALCULATED). The allowed peak voltage is
  `max_vrms × (1 + (f/fc)²)` (CALCULATED).
- With `"abs"`, the module rectifies first. The DC part of the rectified
  signal passes the low-pass without attenuation, so the weighting acts
  only at very low frequencies. Example (CALCULATED, holographer
  2026-11-20): a 1-pole `"abs"` limit at 80 Hz has a nearly flat ceiling
  below 100 Hz. With `max_vrms: 971` it lets an 80 Hz sine through up to
  about 1195 Vrms (MEASURED offline). `"abs"` stays the default so that old configs give the
  same result. For new limits, use `"signed"` or `"integrator"`.

`"signed"` needs `ref_freq_hz` and `slope_db_oct`. The module refuses the
config otherwise.

### `"integrator"`

Core flux is the integral of the voltage. So the voltage that a core can
take falls by 6 dB/octave towards low frequencies. The detector is a
leaky integrator on the signed signal (a one-pole low-pass at
`hp_corner_hz`, a few Hz; this corner stops DC drift), then `|.|`. The
module normalises it to gain 1 at `ref_freq_hz`. So `max_vrms` is the
ceiling **at `ref_freq_hz`**:

```
limit: { name: "stepup_flux"; max_vrms: 971.0; type: "peak";
         weighting: "integrator"; ref_freq_hz: 80.0; hp_corner_hz: 2.0;
         attack_ms: 0.5; release_ms: 3000.0; };
# startup log: ceiling 971.0 V @ 80.0 Hz, 486.0 V @ 40 Hz,
#              243.9 V @ 20 Hz, 123.7 V @ 10 Hz
```

Rules:

- `type` must be `"peak"`.
- `ref_freq_hz` and `hp_corner_hz` are required, with
  `0.1 <= hp_corner_hz <= ref_freq_hz/4`.
- `slope_db_oct` must be absent or 6.

Above `ref_freq_hz` the ceiling rises by 6 dB/octave. At every frequency
the ceiling is at or below the ceiling of a 1-pole `"signed"` limit with
the same `ref_freq_hz` (MEASURED offline, test I6).

**Add the integrator next to an existing transformer limit. Never remove
that limit "because the integrator covers it".** It does not cover it:

- The real core does not follow the 1/f model exactly. In the example
  deployment the 2 % THD point at 130 Hz is 1503 V (MEASURED, REW sweep
  2026-08-27). The integrator (971 V at 80 Hz) allows 1578 V there
  (CALCULATED, +0.4 dB).
- The default `"abs"` 1-pole weighting is stricter than the integrator
  above about 100 Hz. Fundamental at the limit (MEASURED offline):
  1286 V against 1583 V at 130 Hz, and 1412 V against 3888 V at 320 Hz.

The strictest limit wins for each sample. So the integrator never makes
the protection weaker.

The smooth attack/release gain alone does **not** limit the output flux.
The gain acts on the voltage, but the flux integrates over the past.
Example (MEASURED offline): a 20 Hz burst with a hard onset reached 2.8×
the flux ceiling. So an exact **flux backstop** runs on the final output
of the channel, after all limits, the link and the lookahead. It tracks
the output flux. It reduces each sample that would push the flux over
the ceiling. So the output flux never goes above the ceiling (MEASURED
offline, tests I3 and I5).

The backstop acts per sample, so it causes distortion. A long
`release_ms` keeps the smooth gain steady across the flux peaks, so the
backstop acts less often. 20 Hz tone at 971 V, steady state, 1 ms
lookahead (MEASURED offline):

| `release_ms` | THD+N while limiting | fundamental at the output |
|---|---|---|
| 300 | 9.7 % | 249.7 V |
| 1000 | 4.1 % | 246.0 V |
| 3000 | 1.8 % | 244.6 V |

All three hold the flux exactly at the ceiling.

The backstop never makes a sample larger and never flips its sign
(MEASURED offline, test I9, all runs). The module tracks the output flux
also while the limit is off (`enabled: 0;` or `lmc off`). It does not
reset the flux when the limit is switched on again. So after loud bass
with the limit off, `lmc on` can find the flux above the ceiling. The
backstop then mutes the channel (gain 0) until the flux has decayed below
the ceiling. The time constant is `1/(2π·hp_corner_hz)`: 80 ms at 2 Hz
(CALCULATED; MEASURED offline, test I8).

A channel link follows only the smooth gain of its partner, not the
backstop. While the backstop acts on an ESL channel, the linked follower
channel (live: `alkoven`) does not get the same reduction. For those
samples the balance between the ESL and the follower is briefly wrong.

## Lookahead

Optional top-level field `lookahead_ms: <ms>;`, 0 to 10 ms. Default 0 =
off. With 0, the output is bit-identical to the module without lookahead
(MEASURED offline). Live since 2026-11-17 with 1 ms (READ, holographer).

With a lookahead of D samples (D = `lookahead_ms` × sample rate, rounded):

- The module delays **every** output channel that it sees by D samples,
  also channels without a group. So the time alignment between the
  outputs stays the same.
- Each peak limit calculates the gain that each sample needs. It takes
  the minimum over the last D+1 samples and averages that minimum over
  D+1 samples. The gain on the delayed sample is therefore never higher
  than that sample needs. For unweighted limits the result is
  sample-exact: no overshoot. The gain falls linearly over D samples
  *before* the peak, instead of reacting after it.
- Peak limits do not use `attack_ms` in this mode. The ramp length is D.
- Weighted limits use the same method on their weighted detector. In
  reality the gain acts before the weighting filter, so for them the
  bound is close, but not exact (MEASURED offline).
- RMS limits keep their normal smoothing. The module applies their gain
  to the delayed sample.

The startup log shows the lookahead in ms and in samples.

## Channel link

Live since 2026-11-10 (READ, holographer). Design: architect `8a51a27`,
"protect Channel-Link Design" (2026-11-08), stage 1. A link couples the
gain *reduction* of output channels across groups. The live link (READ,
holographer 2026-11-22): the alcove channel follows the left ESL by the
same dB when the left ESL is limited.

```
link: "alk_fill" {
    channels: "esll", "alkoven";  # 2..8 members, each in exactly one group
    drive: "esll";                # optional subset; default: all members drive
    ratio: 1.0, 1.0;              # optional, per member, 0..1 (dB scale)
    offset_db: 0.0, 0.0;          # optional, per member, 0..40 dB dead zone
    enabled: 1;                   # optional; at runtime: lmc protect link on|off
};
```

- Each member keeps its own limits. In addition it applies
  `min(own gain, 10^(-ratio × max(0, A_partner_dB - offset) / 20))`.
  `A_partner` is the reduction of the driving partners in the *previous*
  block.
- A link can only reduce more, never less. The invariant
  `|y_link| <= |y_nolink|` holds for each sample (MEASURED offline, test
  T1a).
- The follower lags by 1 block: 23 ms at 44.1 kHz to 5.3 ms at 192 kHz
  (CALCULATED, 1024-sample blocks). It ramps linearly over the block, with
  no steps.
- The link is lock-free: two parity slots per channel, relaxed atomics.
  The per-block barrier between the BruteFIR filter threads
  (`bfrun.c synch_filter_processes`) orders the write before the read.
- Config errors stop the start (`preinit` returns -1, BruteFIR exits with
  `BF_EXIT_INVALID_CONFIG`):
  - unknown or duplicate names;
  - a member without a group;
  - a channel in two links;
  - a `drive` channel that is not a member;
  - wrong length or range of `ratio` or `offset_db`;
  - unknown or duplicate fields;
  - more than 4 links or more than 8 members.
- **Also without links:** a channel in two `group`s is a config error.
  Before the link version, the second group silently replaced the first.
- Without a `link` block, the processing path, the startup log and every
  `lmc` output are byte-identical to the module before the link
  (MEASURED offline, test T1b).

Not implemented: a sample-exact link (the rest of design stage 2). The
lookahead (stage 2a) is implemented, see [Lookahead](#lookahead).

## Hook point

The module implements `bfevents.output_timed(buf, channel)`. This is the
last point in the BruteFIR pipeline before the output device: after
convolution, crossover, delays and volume. The module sees the final
signal.

**Important:** at this hook, for integer output formats (for example
`S32_LE`), the BruteFIR buffer is **not** normalised to ±1.0. It carries
the raw integer magnitude (±2^31 for `S32_LE`). The general BruteFIR
documentation says otherwise for other points of the pipeline. The
module corrects this with the field `output_scale` (default
`2147483647.0`, for `S32_LE`). Set it when your config uses a different
output format. (MEASURED, dry test 2026-08-22.)

## Calibration caveat: where is the volume control?

`dac_max_output_vrms` is a fixed value in the config. It is correct only
while the real gain between the module's buffer and the physical DAC
output does not change at runtime.

BruteFIR's own volume control (`scale`/`fscale`) acts *before* the hook
of this module (see [Hook point](#hook-point)). If you use it, the
module sees the attenuated signal, and the chain model stays correct at
every volume setting.

The risk is a **hardware or ALSA volume or gain control after the
hook** (for example on the DAC) that is not a chain stage.
`dac_max_output_vrms` assumes that this control stays at the setting
that you measured with. If someone can change it at runtime, every
threshold is wrong from that moment. Keep that control fixed in
hardware, or model it as a chain stage.

## Build

You need a checkout of the [BruteFIR source](https://github.com/atorger/brutefir).
The build uses only `src/bfmod.h`. Nothing links against BruteFIR.
BruteFIR loads the module at runtime (`dlopen`) from its `modules_path`.

```sh
make BRUTEFIR_SRC=/path/to/brutefir
```

The result is `protect.bflogic`. Copy it, or link it, into the
`modules_path` directory of your BruteFIR config.

Built and tested with BruteFIR 1.1.2 (pthread-based). Other versions:
the module should work with any `bfmod.h` that has `output_timed()` with
the same signature (HYPOTHESIS). Not tested with older fork()-based
BruteFIR versions.

## Config

Load the module like any other logic module:

```
logic: "protect" {
    output_scale: 2147483647.0;
    lookahead_ms: 1.0;              # optional, default 0 = off

    chain: "example_chain" {
        dac_max_output_vrms: 6.91;
        { gain_db: 20.0; };         # for example an amplifier stage
    };

    group: {
        channels: "left", "right";
        chain: "example_chain";
        limit: {
            name: "amp_clip"; max_vrms: 40.0; type: "peak";
            attack_ms: 0.3; release_ms: 200.0;
            always_on: 1;           # optional, see below
        };
    };
};
```

Limit fields that are not shown above:

- `ref_freq_hz`, `slope_db_oct`, `weighting`, `hp_corner_hz`: see
  [Frequency weighting](#frequency-weighting).
- `stage: N;`: see [Concept](#concept).
- `window_ms`, `rms_release_ms`: for `type: "rms"`.
- `enabled: 0;` starts the limit switched off.
- `always_on: 1;` forbids `lmc off` for this limit. Use it for amplifier
  clipping and thermal limits. `always_on: 1` with `enabled: 0` is a
  config error.

The BruteFIR config lexer knows only `#` line comments. It does not
know `/* */` block comments.

## Runtime control via the CLI

When BruteFIR runs, use its CLI. `lmc` takes the module's index or its
config name without quotes:

```
lmc protect                 # status table, idle checkpoints hidden
lmc protect all             # status table, every checkpoint shown
lmc protect json            # the same data as a JSON array, for a web UI or script
lmc protect reset           # set every overs counter to 0 (gain and filter state stay)
lmc protect list            # every limit: on/off, type, weighting (WEIGHT: abs/signed/integ, "-" = unweighted), threshold
lmc protect on <name>|all   # switch a limit on (its state is cleared first)
lmc protect off <name>|all  # switch a limit off (refused, see below)
lmc protect link [list]     # links and their state
lmc protect link on|off <name>|all
```

- `lm` shows the current index of each logic module (for example
  `1: "protect"`). The index can change between configs, so the name is
  safer.
- `off` is refused for an `always_on` limit. It is also refused for the
  last enabled limit of a group that is not `always_on`. So the
  excursion/THD protection of a group can never be switched off
  completely.
- `link off` removes only the additional reduction of the link. The own
  limits of each channel stay. So `link off` is always allowed.
- The status table and JSON show link fields only when links exist.

For each checkpoint the table and JSON show:

- `overs`: the number of samples above the threshold since the last
  reset or start. The table also shows it in ms, because the raw count is
  easy to read as "N separate events" instead of a duration.
- `gain`: the current smoothed gain reduction, linear and in dB.
  `1.000`/`0.00dB` means no reduction now.

## Tests

### Offline suite (no BruteFIR, no audio device)

Linux bash and gcc. From Git Bash on Windows, the script starts itself
again inside WSL.

```sh
bash tests/run_tests.sh   # build (-Wall -Wextra), harness, T1..T11, TSan, lookahead, integrator
```

The suite needs two reference inputs (see `ref/README.md`):

- `ref/bf112/bfmod.h`: the BruteFIR 1.1.2 header. It is not in the repo.
  Copy `src/bfmod.h` from BruteFIR tag v1.1.2, or set `BF_INC_DIR`.
- `ref/bflogic_protect.deployed.c` (in the repo): the module that T1b and
  the warning check compare against. Override it with `REF_SRC`.

If an input is missing, the run stops with exit 2 and a message.

`tests/harness.c` includes the module source and emulates BruteFIR
around it: the config token stream, preinit/init, and `output_timed` for
each channel and block. It runs in three modes: sequential, one thread
per channel with a pthread barrier, and with semaphores exactly like
`bfrun.c synch_filter_processes`. `tests/make_cfgs.py` writes the
configs in `tests/cfg/` (committed). The integrator tests are in
`tests/integ/`, the lookahead tests in `tests/la/`.

Result on this branch (MEASURED, 2026-11-22): 186/186 pass, integrator
69/69 pass.

To check a config offline, run:

```sh
bash tests/check_config.sh <brutefir-config>
```

It feeds the `logic: "protect" { ... }` block to the module and prints
"accepted" or "REFUSED" and the startup log. It does not start BruteFIR.

### Dry test with BruteFIR

`tests/` also has a dry test with a real BruteFIR. Synthetic tones at
known levels (some above, some below each threshold) go through
BruteFIR with this module, to a file instead of a device. BruteFIR's own
CLI peak meter checks the result. This test found the two bugs in
[Status](#status). Run it, or a similar test, before you trust a change
on real speakers.

```sh
python3 tests/generate_test_tones.py
tests/measure_peak_via_cli.sh   # needs a real BruteFIR and an audio device
```

`tests/play_direct_bypass.sh` plays a tone directly to the output device,
without the module, as a reference.

**These two scripts start BruteFIR or play sound. Run them only with the
go of the system owner.**

## Status

| part | state |
|---|---|
| protect v1–v3 (chains, groups, peak/RMS, `"abs"` weighting, `stage`, `lmc`) | live since 2026-08-27 (READ) |
| `weighting: "signed"`, `lmc on/off/list`, `always_on` (v3, fad1221) | live since 2026-11-01 (READ, holographer commit 0dc3763) |
| channel link (54ecb2b) | live since 2026-11-10 (READ) |
| lookahead (a6e7122) | live since 2026-11-17, 1 ms (READ) |
| `weighting: "integrator"` + flux backstop (Cue-020) | live since 2026-11-22 20:37 (READ). Startup log and `lmc list` checked live at 44.1 kHz (MEASURED, holographer); other rates offline in the harness only |

- **Absolute accuracy of the thresholds: not verified with an
  oscilloscope (OPEN).** That check would measure the real output voltage
  at the moment a limit acts. Listening confirms the *behaviour*: limits
  act and release as designed. Listening does not confirm that each
  `max_vrms` value is correct in volts. Treat calculated thresholds
  accordingly until you do that check.
- Listening on the real system (STATED): in one song the step-up
  transformer limit acted first and then the rated-power limit of the
  tube amplifier. This matches the crossover frequency of the two
  checkpoints that the design predicts.
- The dry test found two bugs. Both are fixed:
  1. The limit gain state started at 0.0 instead of 1.0. Each start or
     reload muted the output for one release time constant.
  2. The buffer scale at `output_timed()` (see [Hook point](#hook-point)).
     Without the fix, every real signal pulled the gain to zero.
- [0.0.2](https://github.com/MRinus/protect-bflogic/releases/tag/0.0.2)
  added the `lmc` modes `all`, `json` and `reset` and the aligned status
  table.

## Example deployment

`examples/protect_example_config.txt` is a real config of 2026-08-27
(READ). It shows the **4-channel state of that date**: two ESL panels and
two sealed woofer towers, crossed over in BruteFIR (90 Hz, 20th-order
Neville-Thiele). The alcove channel, the link, the lookahead and the
`"signed"`/`"integrator"` weightings came later and are not in it. Use it
for the config syntax and the modelling method. Do not copy its numbers.

- **Woofer chain**: one gain stage: BruteFIR → DAC (direct XLR) → Class-D
  amplifier in BTL mode. Four limits in one group:
  - `driver_thd_2pct`: the 2 % THD point (MEASURED, REW sweep), an
    audible-distortion criterion;
  - `xmax`: the mechanical excursion limit (CALCULATED from the
    Thiele-Small parameters of the driver), weighted 12 dB/octave above
    the box resonance;
  - `amp_clip`: amplifier clipping (READ, datasheet);
  - `driver_thermal`: driver thermal limit, RMS (CALCULATED, datasheet).

  The THD criterion and the mechanical limit are separate limits. They
  are different phenomena and they act at different levels.
- **ESL chain**: three gain stages: DAC → passive balanced-to-unbalanced
  adapter (−6 dB, *not* a transformer) → tube amplifier line input → the
  1:60 step-up transformer of the panel to the stator. Three limits in
  one group:
  - `stepup_1_60_thd_2pct`: the step-up transformer limit, 971 V at
    80 Hz, weighted 6 dB/octave (MEASURED, REW THD sweep 80–300 Hz,
    full chain).
  - `tri_rated_power`: the rated power of the tube amplifier (`stage: 2`,
    not weighted; READ, datasheet).
  - `els_xmax`: a flat peak ceiling at the stator (full chain, not
    weighted). It is **not** an excursion limit. Its value is an
    ASSUMPTION: there is no maker data (OPEN question to the system
    owner, 2026-11-21). The live config uses `max_vrms: 1768`, which is
    about 2500 V peak (READ, see [Units](#units-what-max_vrms-means)).
    The example file uses the same value since 2026-11-22. Before, it had
    `max_vrms: 2500` (2026-08-27), which is about 3536 V peak
    (CALCULATED).

## License

ISC, the same license as BruteFIR. See [LICENSE](LICENSE).
