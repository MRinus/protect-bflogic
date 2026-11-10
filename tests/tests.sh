#!/bin/bash
# tests.sh - T1..T11 (called by run_tests.sh with OUT, ROOT, HERE set).
set -u
cd "$HERE"
H=$OUT/harness_new; R=$OUT/harness_ref; TS=$OUT/harness_tsan; T8=$OUT/harness_t8
C=$HERE/cfg
RATES="44100 48000 88200 96000 176400 192000"
pass=0; fail=0
ok()  { echo "  PASS $*"; pass=$((pass+1)); }
bad() { echo "  FAIL $*"; fail=$((fail+1)); }
sub() {  # run a harness test that prints its own PASS/FAIL lines
  local log=$OUT/sub.log
  "$@" > "$log" 2>"$log.err"; local rc=$?
  grep -v '^  \[' "$log"
  local p f; p=$(grep -c '^  PASS' "$log"); f=$(grep -c '^  FAIL' "$log")
  pass=$((pass+p)); fail=$((fail+f))
  if [ $rc -ne 0 ] && [ "$f" -eq 0 ]; then bad "harness exit $rc: $*"; sed 's/^/    /' "$log.err" | tail -5; fi
}
hash_of() { "$@" 2>/dev/null | sed -n 's/^hash=\([0-9a-f]*\) faults=\([0-9]*\).*/\1 \2/p'; }

echo "-- T6 config errors -> preinit -1 (BruteFIR: BF_EXIT_INVALID_CONFIG) --"
while IFS=$'\t' read -r name msg; do
  out=$("$H" preinit "$C/err/$name.txt" 2>&1); rc=$?
  if [ $rc -eq 3 ] && echo "$out" | grep -qF "$msg"; then ok "T6 $name rejected: $(echo "$out" | grep -F "$msg" | head -1 | sed 's/^PROTECT: //')"
  else bad "T6 $name: rc=$rc, expected message '$msg'; got: $(echo "$out" | tail -2 | tr '\n' ' ')"; fi
done < "$C/err/EXPECT"
for f in simple_nolink t2 t5 t4_o0 t4_o2 ok_link_before_group ok_disabled ok_design_D live_nolink live_alk_nolink live_alk_link gen_sq48_link; do
  out=$("$H" preinit "$C/$f.txt" 2>&1); rc=$?
  [ $rc -eq 0 ] && ok "T6 valid config $f accepted$(echo "$out" | grep -F "PROTECT: link '" | sed 's/^PROTECT: / -> /' | head -1)" \
                || bad "T6 valid config $f rejected: $(echo "$out" | tail -2 | tr '\n' ' ')"
done
out=$("$R" preinit "$C/err/chan_two_groups.txt" 2>&1); rc=$?
[ $rc -eq 0 ] && echo "    (reference module accepts chan_two_groups silently: rc=$rc - the side finding, now fixed)"

echo; echo "-- T1b no link: new module bit-identical to the deployed one --"
for cfg in live_nolink live_alk_nolink; do
  for rate in $RATES; do
    nb=3000; [ "$rate" = 48000 ] && nb=20000
    for rs in 8 4; do
      a=$(hash_of "$R" run "$C/$cfg.txt" random seq $rate $nb $rs 2147483647)
      b=$(hash_of "$H" run "$C/$cfg.txt" random seq $rate $nb $rs 2147483647)
      [ -n "$a" ] && [ "${a%% *}" = "${b%% *}" ] && ok "T1b $cfg $rate Hz realsize=$rs $nb blocks: hash ${a%% *} identical" \
                                                 || bad "T1b $cfg $rate realsize=$rs: ref '$a' new '$b'"
    done
  done
done
"$R" lmc "$C/live_nolink.txt" 48000 2000 random "" all json list "off xmax_a" "on xmax_a14" reset > "$OUT/lmc_ref.txt" 2>/dev/null
"$H" lmc "$C/live_nolink.txt" 48000 2000 random "" all json list "off xmax_a" "on xmax_a14" reset > "$OUT/lmc_new.txt" 2>/dev/null
cmp -s "$OUT/lmc_ref.txt" "$OUT/lmc_new.txt" && ok "T1b lmc output (status/all/json/list/off/on/reset) byte-identical ($(wc -c < "$OUT/lmc_new.txt") bytes)" \
  || { bad "T1b lmc output differs"; diff "$OUT/lmc_ref.txt" "$OUT/lmc_new.txt" | head; }
"$R" preinit "$C/live_nolink.txt" > /dev/null 2> "$OUT/log_ref.txt"; "$H" preinit "$C/live_nolink.txt" > /dev/null 2> "$OUT/log_new.txt"
cmp -s "$OUT/log_ref.txt" "$OUT/log_new.txt" && ok "T1b startup log byte-identical ($(wc -l < "$OUT/log_new.txt") lines)" || { bad "T1b startup log differs"; diff "$OUT/log_ref.txt" "$OUT/log_new.txt" | head; }

echo; echo "-- T1a invariant: |y_link| <= |y_nolink| sample-exact, detectors bit-identical --"
sub "$H" t1a "$C/live_alk_link.txt" "$C/live_alk_nolink.txt" 48000 100000 seq
sub "$H" t1a "$C/live_alk_link.txt" "$C/live_alk_nolink.txt" 48000 20000 thr
for rate in 44100 88200 96000 176400 192000; do sub "$H" t1a "$C/live_alk_link.txt" "$C/live_alk_nolink.txt" $rate 10000 seq; done
sub "$H" t1a "$C/t4_o0.txt" "$C/simple_nolink.txt" 48000 20000 seq

echo; echo "-- T2 alkoven follows esll 1:1 (all rates), T5 ratio 0.5, T4 stereo offset 0/2 --"
for rate in $RATES; do sub "$H" follow t2 "$C/t2.txt" $rate seq 1.0 0.0 6; done
sub "$H" follow t2 "$C/t2.txt" 48000 thr 1.0 0.0 6
sub "$H" follow t5 "$C/t5.txt" 48000 seq 0.5 0.0 6
sub "$H" follow t5 "$C/t5.txt" 192000 seq 0.5 0.0 6
sub "$H" follow t4 "$C/t4_o0.txt" 48000 seq 1.0 0.0 6
sub "$H" follow t4 "$C/t4_o2.txt" 48000 seq 1.0 2.0 6
sub "$H" follow t4 "$C/t4_o2.txt" 48000 seq 1.0 2.0 1.5

echo; echo "-- T3 drive subset: alkoven over its own limit does not pull esll --"
sub "$H" t3 "$C/t2.txt" "$C/simple_nolink.txt" 48000

echo; echo "-- T7 threads: seq == thr (pthread barrier) == sem (bfrun-like), faults 0; TSan --"
for cfg in live_alk_link ok_design_D; do
  s=$(hash_of "$H" run "$C/$cfg.txt" random seq 48000 100000 8 2147483647)
  for m in thr sem; do
    t=$(hash_of "$H" run "$C/$cfg.txt" random $m 48000 100000 8 2147483647)
    [ -n "$s" ] && [ "$s" = "$t" ] && [ "${t#* }" = 0 ] && ok "T7 $cfg $m 100000 blocks: hash ${t%% *} == seq, link_faults ${t#* }" \
                                      || bad "T7 $cfg $m: seq '$s' vs '$t'"
  done
done
for m in thr sem; do
  log=$OUT/tsan_$m.log
  TSAN_OPTIONS="halt_on_error=0 exitcode=66" setarch "$(uname -m)" -R "$TS" run "$C/live_alk_link.txt" random $m 48000 100000 8 2147483647 > "$log.out" 2> "$log"
  rc=$?
  n=$(grep -c "WARNING: ThreadSanitizer" "$log")
  s=$(hash_of "$H" run "$C/live_alk_link.txt" random seq 48000 100000 8 2147483647)
  t=$(sed -n 's/^hash=\([0-9a-f]*\) faults=\([0-9]*\).*/\1 \2/p' "$log.out")
  [ $rc -eq 0 ] && [ "$n" -eq 0 ] && [ "$s" = "$t" ] && ok "T7 TSan $m 100000 blocks: 0 reports, hash identical ($(sed -n 's/.*wall=//p' "$log.out"))" \
                                             || { bad "T7 TSan $m: rc=$rc reports=$n hash '$t' vs '$s'"; head -30 "$log"; }
done

for m in thr sem; do  # float buffers (realsize 4) through the link path
  s=$(hash_of "$H" run "$C/live_alk_link.txt" random seq 48000 20000 4 2147483647)
  t=$(hash_of "$H" run "$C/live_alk_link.txt" random $m 48000 20000 4 2147483647)
  [ -n "$s" ] && [ "$s" = "$t" ] && [ "${t#* }" = 0 ] && ok "T7 realsize=4 (float) live_alk_link $m 20000 blocks: hash ${t%% *} == seq, link_faults 0" \
                                    || bad "T7 realsize=4 $m: seq '$s' vs '$t'"
done

echo; echo "-- T10b lmc output with links: JSON valid, link fields present --"
"$H" lmc "$C/live_alk_link.txt" 48000 3000 random json > "$OUT/lmc_link_json.txt" 2>/dev/null
sed -n '2,$p' "$OUT/lmc_link_json.txt" > "$OUT/lmc_link.json"
if command -v python3 >/dev/null; then
  r=$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); L=[o for o in d if o["link"]]; print(len(d), len(L), sorted({o["channel"] for o in L}), all("link_gain_db" in o and "link_faults" in o for o in d))' "$OUT/lmc_link.json" 2>&1)
  echo "$r" | grep -q "\['alkoven', 'esll', 'eslr'\] True" && ok "T10b lmc json with links parses ($r)" || bad "T10b lmc json: $r"
else
  bad "T10b python3 missing for JSON check"
fi
"$H" lmc "$C/live_alk_link.txt" 48000 3000 random "link list" "link off esl_main" "link list" "link on nosuch" > "$OUT/lmc_link_cmds.txt" 2>/dev/null
grep -q "esl_main      off" "$OUT/lmc_link_cmds.txt" && grep -q "name not found" "$OUT/lmc_link_cmds.txt" \
  && ok "T10b lmc link list/off/on: state shown, unknown name reported" || { bad "T10b lmc link cmds"; cat "$OUT/lmc_link_cmds.txt"; }

echo; echo "-- T8 real time: no malloc/printf in output_timed; cost per block --"
for rate in 192000 48000; do
  sub "$T8" t8 "$C/live_alk_link.txt" $rate 20000
  grep '^T8TIME' "$OUT/sub.log" | cut -d' ' -f2- | sort > "$OUT/t8_link.txt"
  sub "$T8" t8 "$C/live_alk_nolink.txt" $rate 20000
  grep '^T8TIME' "$OUT/sub.log" | cut -d' ' -f2- | sort > "$OUT/t8_nolink.txt"
  extra=$(join "$OUT/t8_link.txt" "$OUT/t8_nolink.txt" 2>/dev/null | awk '$1~/^(esll|eslr|alkoven)$/{d=$2-$3; if(d>m)m=d} END{printf "%.2f", m+0}')
  blk_us=$(awk -v r=$rate 'BEGIN{printf "%.0f", 1024e6/r}')
  awk -v e="$extra" -v b="$blk_us" 'BEGIN{exit !(e < 0.01*b)}' \
    && ok "T8 $rate Hz: extra cost of the link per linked channel and block max ${extra} us (same channel, link vs no link) < 1% of the block time ${blk_us} us" \
    || bad "T8 $rate Hz: extra ${extra} us vs block ${blk_us} us"
done
# call graph of output_timed in the real module build (Makefile flags):
# follow module-internal functions, collect every external call target
objdump -d --no-show-raw-insn "$OUT/protect.bflogic" > "$OUT/protect.dis"
calls_of() { awk -v fn="<$1>:" '$2==fn{f=1;next} f&&/^$/{f=0} f' "$OUT/protect.dis" | grep -oE 'call +[0-9a-f]+ <[^>]+>' | sed -E 's/.*<([^>]+)>.*/\1/' | sed -E 's/@plt$//; s/\+0x[0-9a-f]+$//' | sort -u; }
seen=" output_timed "; todo="output_timed"; ext=""
while [ -n "$todo" ]; do
  set -- $todo; f=$1; shift; todo="$*"
  for c in $(calls_of "$f"); do
    case "$seen" in *" $c "*) continue ;; esac
    seen="$seen$c "
    if grep -q "^[0-9a-f]* <$c>:" "$OUT/protect.dis" && ! grep -q "<$c@plt>:" "$OUT/protect.dis"; then todo="$todo $c"; else ext="$ext $c"; fi
  done
done
bad_calls=$(echo $ext | tr ' ' '\n' | grep -vE '^(log10|pow|__stack_chk_fail)$' | tr '\n' ' ')
[ -z "${bad_calls// /}" ] && ok "T8 objdump protect.bflogic: call graph of output_timed [$(echo $seen)] reaches only external [$(echo $ext)]" \
  || bad "T8 output_timed reaches: $ext"

echo; echo "-- T9 fault injection (NaN, 0, 2.0, wrong block) --"
sub "$H" t9 "$C/t2.txt" 48000

echo; echo "-- T10 lmc link off/on during a burst --"
sub "$H" t10 "$C/t2.txt" 48000 seq
sub "$H" t10 "$C/t2.txt" 48000 thr

echo; echo "-- T11 release ramp without steps --"
sub "$H" t11 "$C/t2.txt" 48000
sub "$H" t11 "$C/t2.txt" 192000

echo; echo "[tests] $pass PASS, $fail FAIL"
[ $fail -eq 0 ]
