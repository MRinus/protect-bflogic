#!/bin/bash
# run_tests.sh - build + test the protect channel link offline (T1-T11).
#
# Runs on Linux (bash, gcc). From Git Bash on Windows it re-executes itself
# explicitly inside WSL (distribution $WSL_DISTRO, default Ubuntu-24.04),
# because the module, its pthread/TSan harness and the target (holographer)
# are Linux. Any other environment (PowerShell launching WSL's
# C:\Windows\System32\bash.exe with a Windows cwd, cmd, macOS, ...) is
# detected and aborted with a message instead of failing half-way.
#
#   bash protect-link/tests/run_tests.sh            # from Git Bash or Linux
#
# Nothing here touches holographer, BruteFIR or any audio device.
set -u
WSL_DISTRO=${WSL_DISTRO:-Ubuntu-24.04}
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

case "$(uname -s)" in
  Linux)
    # one output stream: when started from Git Bash via wsl.exe with
    # "> file 2>&1", separate stdout/stderr handles overwrote each other
    exec 2>&1
    ;;
  MINGW*|MSYS*|CYGWIN*)
    command -v wsl.exe >/dev/null || { echo "[run_tests] wsl.exe not found - need WSL ($WSL_DISTRO) for gcc/pthread/TSan" >&2; exit 2; }
    winpath=$(cygpath -m "$here/run_tests.sh")          # C:/Users/... (no backslashes)
    lxpath=$(MSYS_NO_PATHCONV=1 wsl.exe -d "$WSL_DISTRO" -- wslpath -a "$winpath" | tr -d '\r')
    [ -n "$lxpath" ] || { echo "[run_tests] wslpath failed for $winpath" >&2; exit 2; }
    echo "[run_tests] Git Bash ($(uname -s)) detected -> re-running inside WSL '$WSL_DISTRO': $lxpath"
    MSYS_NO_PATHCONV=1 exec wsl.exe -d "$WSL_DISTRO" -- bash "$lxpath"
    ;;
  *)
    echo "[run_tests] unsupported environment '$(uname -s)': run from Git Bash or a Linux bash" >&2
    exit 2 ;;
esac
command -v gcc >/dev/null || { echo "[run_tests] gcc not found" >&2; exit 2; }

root=$(cd "$here/.." && pwd)
out=${OUT:-/tmp/protect-link-tests}
rm -rf "$out"; mkdir -p "$out"
echo "[run_tests] shell: $BASH $BASH_VERSION, $(uname -sr), $(gcc --version | head -1)"
echo "[run_tests] source: $root, build dir: $out"

CFLAGS_STRICT="-std=c99 -D_POSIX_C_SOURCE=200809L -O2 -fPIC -Wall -Wextra"
INC="-I$root/ref/bf112"
fail=0

warn_count() {  # $1 = source; prints warnings (normalised) to $2
  gcc $CFLAGS_STRICT $INC -shared -o "$out/tmp.so" "$1" -lm 2> "$2.raw"
  local rc=$?
  # strip file:line:col; the multiset of warning texts is compared
  sed -E 's/^[^:]*:[0-9]+:[0-9]+: //' "$2.raw" | grep -E '^(warning|error)' | sed -E "s/[0-9]+ bytes/N bytes/" | sort > "$2"
  return $rc
}

echo; echo "== build: module (-Wall -Wextra, strict C99) =="
warn_count "$root/ref/bflogic_protect.deployed.c" "$out/warn_ref.txt"; rc_ref=$?
warn_count "$root/src/bflogic_protect.c" "$out/warn_new.txt"; rc_new=$?
echo "  reference (deployed) : exit $rc_ref, $(wc -l < "$out/warn_ref.txt") warning(s)"
echo "  new (link)           : exit $rc_new, $(wc -l < "$out/warn_new.txt") warning(s)"
# compare as multisets (same warning text may occur in old code too)
newwarn=$(comm -13 <(sort "$out/warn_ref.txt") <(sort "$out/warn_new.txt"))
if [ $rc_new -ne 0 ] || [ -n "$newwarn" ]; then
  echo "  FAIL new warnings/errors:"; echo "$newwarn" | sed 's/^/    /'; cat "$out/warn_new.txt.raw"; fail=1
else
  echo "  PASS no new warnings"
fi
# the real module build as the Makefile does it (BRUTEFIR_SRC layout)
mkdir -p "$out/bfsrc/src" && cp "$root/ref/bf112/bfmod.h" "$out/bfsrc/src/"
make -s -C "$root" BRUTEFIR_SRC="$out/bfsrc" CFLAGS="-O2 -fPIC -Wall -Wextra" TARGET="$out/protect.bflogic" >/dev/null 2>"$out/make.err" \
  && echo "  PASS make (Makefile) -> protect.bflogic" || { echo "  FAIL make"; cat "$out/make.err"; fail=1; }

echo; echo "== build: harness =="
HFLAGS="-std=gnu99 -O2 -g -Wall -Wextra -pthread $INC"
gcc $HFLAGS -DMODULE_SRC="\"$root/src/bflogic_protect.c\"" -o "$out/harness_new" "$here/harness.c" -lm 2>>"$out/harness_build.log" || fail=1
gcc $HFLAGS -DMODULE_SRC="\"$root/ref/bflogic_protect.deployed.c\"" -DREFERENCE_BUILD -o "$out/harness_ref" "$here/harness.c" -lm 2>>"$out/harness_build.log" || fail=1
gcc -std=gnu99 -O1 -g -fsanitize=thread -pthread $INC -DMODULE_SRC="\"$root/src/bflogic_protect.c\"" -o "$out/harness_tsan" "$here/harness.c" -lm 2>>"$out/harness_build.log" || fail=1
WRAPS="malloc calloc realloc free vfprintf fprintf fwrite fputs fputc write vsnprintf snprintf"
gcc $HFLAGS -U_FORTIFY_SOURCE -D_FORTIFY_SOURCE=0 -DHOTPATH_WRAP -DMODULE_SRC="\"$root/src/bflogic_protect.c\"" \
    $(for s in $WRAPS; do printf -- '-Wl,--wrap=%s ' "$s"; done) -o "$out/harness_t8" "$here/harness.c" -lm 2>>"$out/harness_build.log" || fail=1
[ $fail -eq 0 ] && echo "  PASS harness builds (new, reference, TSan, hot-path wrap); compiler messages: $out/harness_build.log ($(grep -c 'warning:' "$out/harness_build.log") warnings, of which $(grep 'warning:' "$out/harness_build.log" | grep -vc 'unused parameter') are not 'unused parameter'; the module itself is checked above)"               || { echo "  FAIL harness build"; cat "$out/harness_build.log"; }

echo; echo "== tests =="
OUT="$out" ROOT="$root" HERE="$here" bash "$here/tests.sh"
rc=$?
[ $rc -ne 0 ] && fail=1
echo
[ $fail -eq 0 ] && echo "[run_tests] RESULT: ALL PASS" || echo "[run_tests] RESULT: FAILURES"
exit $fail
