#!/bin/bash
# integ_tests.sh - weighting "integrator" (Cue-020), offline. Linux bash + gcc.
# Called by tests/run_tests.sh (OUT, ROOT, HERE, INC set), or alone:
#   OUT=/tmp/x ROOT=<repo> INC=-I<bf112 headers> bash tests/integ/integ_tests.sh
# Nothing here touches BruteFIR, holographer or an audio device.
set -u
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=${ROOT:-$(cd "$here/../.." && pwd)}
OUT=${OUT:-/tmp/protect-integ-tests}
INC=${INC:--I$ROOT/ref/bf112}
mkdir -p "$OUT/integ"
O=$OUT/integ
pass=0; fail=0
ok()  { echo "  PASS $*"; pass=$((pass + 1)); }
bad() { echo "  FAIL $*"; fail=$((fail + 1)); }
# field value from a "key=value ..." line
fv() { echo "$1" | tr ' ' '\n' | sed -n "s/^$2=//p"; }
# awk float compare: cmp "<expr>"
fc() { awk "BEGIN { exit !($1) }"; }

gcc -std=gnu99 -O2 -Wall -Wextra -pthread $INC -DMODULE_SRC="\"$ROOT/src/bflogic_protect.c\"" \
    -o "$O/integ_unit" "$here/integ_unit.c" -lm 2> "$O/build.log" \
  || { echo "  FAIL build integ_unit"; cat "$O/build.log"; exit 1; }
gcc -std=gnu99 -O2 -pthread $INC -DMODULE_SRC="\"$ROOT/src/bflogic_protect.c\"" \
    -o "$O/harness" "$here/../harness.c" -lm 2>> "$O/build.log" \
  || { echo "  FAIL build harness"; cat "$O/build.log"; exit 1; }
U=$O/integ_unit
: > "$O/all_runs.txt"
# every run goes through run_u: its result line is kept, and I9 checks the
# invariants (|y| <= |x|, no sign flip) over ALL runs at the end
run_u() { local r; r=$("$U" "$@" 2>/dev/null); echo "$r" >> "$O/all_runs.txt"; echo "$r"; }

CEIL20=243.9      # 971 V @ 80 Hz -> ceiling at 20 Hz (hp corner 2 Hz), see startup log
for la in 0 1.0; do
  sed "s/@LA@/$la/" "$here/cfg_integ_tpl.txt" > "$O/cfg_$la.txt"
  for en in 0 1; do
    sed "s/@LA@/$la/; s/@EN@/$en/" "$here/cfg_integ2_tpl.txt" > "$O/cfg2_${la}_en$en.txt"
  done
done

echo; echo "-- I0 startup log --"
"$O/harness" preinit "$O/cfg_0.txt" > /dev/null 2> "$O/log.txt"
grep -q "ceiling 971.0 V @ 80.0 Hz, 486.0 V @ 40 Hz, 243.9 V @ 20 Hz, 123.7 V @ 10 Hz" "$O/log.txt" \
  && ok "log shows the ceiling per frequency: $(grep -o 'ceiling.*' "$O/log.txt")" \
  || { bad "startup log line missing"; cat "$O/log.txt"; }

for la in 0 1.0; do for rate in 48000 192000; do
  C=$O/cfg_$la.txt
  tag="la=${la}ms rate=$rate"
  echo; echo "-- $tag --"

  # I1: 80 Hz at the ceiling (971 V) passes
  r=$(run_u "$C" $rate 80 971 3 0.5 0 0.2 2>/dev/null)
  # Steady state must be unchanged (within 1 mV = 1e-6 relative). During the fade-in the input flux of
  # this tone reaches 1.0001 x ceiling (fade transient), so the backstop
  # trims a few samples there; that count is printed, not hidden.
  fc "$(fv "$r" steady_dev_V) <= 0.001 && $(fv "$r" thdn_pct) < 0.01 && $(fv "$r" flux_clips) <= 10" \
    && ok "I1 80 Hz 971 V passes (steady |y-x| $(fv "$r" steady_dev_V) V, fundamental $(fv "$r" fund_peak_V) V, THD+N $(fv "$r" thdn_pct) %; fade-in: $(fv "$r" flux_clips) samples trimmed, max $(fv "$r" max_dev_V) V)" \
    || bad "I1 80 Hz 971 V: $r"

  # I2: 80 Hz at -1 dB passes bit-identical (only delayed by D)
  r=$(run_u "$C" $rate 80 865.4 3 0.5 0 0.2 2>/dev/null)
  [ "$(fv "$r" identical)" = 1 ] && ok "I2 80 Hz -1 dB (865.4 V) bit-identical, D=$(fv "$r" D)" \
    || bad "I2 80 Hz -1 dB: $r"

  # I3: 20 Hz at 971 V must be limited to the 20 Hz ceiling
  r=$(run_u "$C" $rate 20 971 3 0.5 0 0.2 2>/dev/null)
  fp=$(fv "$r" fund_peak_V)
  fc "$(fv "$r" limited_samples) > 0 && $(fv "$r" det_out_max_ratio) <= 1.000000001 && $fp > $CEIL20 * 0.966 && $fp < $CEIL20 * 1.035" \
    && ok "I3 20 Hz 971 V limited: fundamental $fp V (ceiling $CEIL20 V), output flux max $(fv "$r" det_out_max_ratio) x ceiling, THD+N $(fv "$r" thdn_pct) %" \
    || bad "I3 20 Hz 971 V: $r"

  # I3b: 20 Hz at -1 dB below its ceiling passes bit-identical
  r=$(run_u "$C" $rate 20 217.4 3 0.5 0 0.2 2>/dev/null)
  [ "$(fv "$r" identical)" = 1 ] && ok "I3b 20 Hz 217.4 V (-1 dB below $CEIL20 V) bit-identical" \
    || bad "I3b 20 Hz -1 dB: $r"

  # I4: no DC drift. a) 20 s of 80 Hz -1 dB plus 0.5 V DC stays bit-identical
  r=$(run_u "$C" $rate 80 865.4 20 0.5 0.5 0.2 2>/dev/null)
  [ "$(fv "$r" identical)" = 1 ] && ok "I4a 80 Hz -1 dB + 0.5 V DC for 20 s bit-identical (flux max $(fv "$r" det_out_max_ratio) x ceiling)" \
    || bad "I4a DC: $r"
  # b) after a hard-onset burst the integrator state decays (tau = 80 ms)
  r=$(run_u "$C" $rate 20 971 1 0 0 1.0 2>/dev/null)
  fc "$(fv "$r" tail_z_ratio) < 1e-4" \
    && ok "I4b flux state 1 s after a hard-onset burst: $(fv "$r" tail_z_ratio) of its burst maximum" \
    || bad "I4b decay: $r"

  # I5: exact backstop on hard onsets (zero phase, far above the ceiling)
  for f in 20 40 80; do
    r=$(run_u "$C" $rate $f 3000 1 0 0 0.2 2>/dev/null)
    fc "$(fv "$r" det_out_max_ratio) <= 1.000000001 && $(fv "$r" flux_clips) > 0" \
      && ok "I5 hard onset $f Hz 3000 V: output flux max $(fv "$r" det_out_max_ratio) x ceiling (backstop samples $(fv "$r" flux_clips), input flux $(fv "$r" det_in_max_ratio) x)" \
      || bad "I5 hard onset $f Hz: $r"
  done

  # I8 (sec-dude P-020-1): limit off -> loud LF -> limit on. The flux state
  # is tracked while off, so at "on" it can be far above the ceiling. The
  # module must never grow or flip a sample (I9), and 0.5 s after "on" the
  # output flux must be back under the ceiling.
  C2=$O/cfg2_${la}_en1.txt; C0=$O/cfg2_${la}_en0.txt
  r=$(OFF_AT=0 ON_AT=2.1 run_u "$C2" $rate 20 3000 2 0 0 1.0)
  fc "$(fv "$r" off_refused) == 0 && $(fv "$r" grow_V) <= 0 && $(fv "$r" sign_flips) == 0 && $(fv "$r" det_after_on_ratio) <= 1.000000001" \
    && ok "I8a off, 20 Hz 3000 V, on in silence: grow $(fv "$r" grow_V) V, flips $(fv "$r" sign_flips), flux 0.5 s after on $(fv "$r" det_after_on_ratio) x" \
    || bad "I8a: $r"
  r=$(OFF_AT=0 ON_AT=2.0 run_u "$C2" $rate 20 3000 4 0 0 0.5)
  fc "$(fv "$r" off_refused) == 0 && $(fv "$r" grow_V) <= 0 && $(fv "$r" sign_flips) == 0 && $(fv "$r" det_after_on_ratio) <= 1.000000001" \
    && ok "I8b off, 20 Hz 3000 V, on mid-tone: grow $(fv "$r" grow_V) V, flips $(fv "$r" sign_flips), flux 0.5 s after on $(fv "$r" det_after_on_ratio) x, y max $(fv "$r" y_max_V) V" \
    || bad "I8b: $r"
  r=$(ON_AT=2.0 run_u "$C0" $rate 20 971 4 0 0 0.5)
  fc "$(fv "$r" grow_V) <= 0 && $(fv "$r" sign_flips) == 0 && $(fv "$r" det_after_on_ratio) <= 1.000000001" \
    && ok "I8c \"enabled: 0;\" in the config, 20 Hz 971 V, lmc on mid-tone: grow $(fv "$r" grow_V) V, flips $(fv "$r" sign_flips), flux 0.5 s after on $(fv "$r" det_after_on_ratio) x" \
    || bad "I8c: $r"
done; done

# N-020-3: float buffers (realsize 4)
echo; echo "-- I3/I5 with float buffers (realsize 4) --"
for rate in 48000 192000; do
  r=$(REALSIZE=4 run_u "$O/cfg_1.0.txt" $rate 20 971 3 0.5 0 0.2)
  fc "$(fv "$r" limited_samples) > 0 && $(fv "$r" det_out_max_ratio) <= 1.000001" \
    && ok "I3f rate=$rate 20 Hz 971 V, float: fundamental $(fv "$r" fund_peak_V) V, output flux max $(fv "$r" det_out_max_ratio) x ceiling" \
    || bad "I3f rate=$rate: $r"
  r=$(REALSIZE=4 run_u "$O/cfg_1.0.txt" $rate 20 3000 1 0 0 0.2)
  fc "$(fv "$r" det_out_max_ratio) <= 1.000001 && $(fv "$r" flux_clips) > 0" \
    && ok "I5f rate=$rate hard onset 20 Hz 3000 V, float: output flux max $(fv "$r" det_out_max_ratio) x ceiling" \
    || bad "I5f rate=$rate: $r"
done

# I6: never weaker than the existing 1-pole weighting (ref 80 Hz, 6 dB/oct):
# a tone at the existing limit's ceiling 971*sqrt(1+(f/80)^2) is limited
# by the integrator at every frequency up to 320 Hz.
echo; echo "-- I6 stricter than the existing 80 Hz 1-pole weighting --"
for f in 10 20 40 80 160 320; do
  v=$(awk "BEGIN { printf \"%.1f\", 971 * sqrt(1 + ($f / 80) ^ 2) }")
  r=$(run_u "$O/cfg_0.txt" 48000 $f $v 3 0.5 0 0.2 2>/dev/null)
  fc "$(fv "$r" limited_samples) > 0" \
    && ok "I6 $f Hz at $v V (existing ceiling) is limited by the integrator (fundamental $(fv "$r" fund_peak_V) V)" \
    || bad "I6 $f Hz: $r"
done

# I7: config checks (preinit must refuse)
echo; echo "-- I7 config errors are refused --"
reject() {  # $1 = description, $2 = sed expression on the template
  sed "s/@LA@/0/; $2" "$here/cfg_integ_tpl.txt" > "$O/bad.txt"
  "$O/harness" preinit "$O/bad.txt" > /dev/null 2> "$O/bad.log"
  [ $? -eq 3 ] && ok "I7 refused: $1 ($(grep -o 'PROTECT: limit.*' "$O/bad.log" | head -1 | cut -c1-90))" \
               || bad "I7 accepted: $1"
}
reject "integrator without hp_corner_hz"   's/ hp_corner_hz: 2.0;//'
reject "integrator without ref_freq_hz"    's/ref_freq_hz: 80.0; //'
reject "hp_corner_hz above ref_freq_hz/4"  's/hp_corner_hz: 2.0/hp_corner_hz: 25.0/'
reject "slope_db_oct 12 with integrator"   's/hp_corner_hz: 2.0;/hp_corner_hz: 2.0; slope_db_oct: 12.0;/'
reject "hp_corner_hz with weighting signed" 's/weighting: "integrator";/weighting: "signed"; slope_db_oct: 6.0;/'
reject "unknown weighting"                 's/"integrator"/"integral"/'
reject "type rms with integrator"          's/type: "peak"; weighting/type: "rms"; weighting/'
reject "hp_corner_hz below 0.1 Hz"         's/hp_corner_hz: 2.0/hp_corner_hz: 0.05/'
sed "s/@LA@/0/; s/hp_corner_hz: 2.0;/hp_corner_hz: 2.0; slope_db_oct: 6.0;/" "$here/cfg_integ_tpl.txt" > "$O/ok6.txt"
"$O/harness" preinit "$O/ok6.txt" > /dev/null 2>&1 && ok "I7 accepted: slope_db_oct 6 with integrator" || bad "I7 slope 6 refused"

# I9 (sec-dude): over ALL runs above, the module never made a sample larger
# than its input and never flipped its sign
echo; echo "-- I9 invariants over all runs --"
nrun=$(grep -c 'rate=' "$O/all_runs.txt")
viol=$(awk '{ g = ""; s = ""; for (i = 1; i <= NF; i++) { split($i, kv, "="); if (kv[1] == "grow_V") g = kv[2]; if (kv[1] == "sign_flips") s = kv[2] }
             if (g == "" || s == "" || g + 0 > 0 || s + 0 != 0) print }' "$O/all_runs.txt")
[ "$nrun" -gt 0 ] && [ -z "$viol" ] && ok "I9 $nrun runs: |y| <= |x| for every sample, no sign flip" \
  || { bad "I9 invariant violated or no runs ($nrun):"; echo "$viol" | head -5; }

echo; echo "[integ] $pass PASS, $fail FAIL"
[ $fail -eq 0 ]
