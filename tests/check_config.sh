#!/bin/bash
# check_config.sh - offline: does this module accept a BruteFIR config's
# logic "protect" { ... } block, and what does it log at startup?
#
#   bash tests/check_config.sh <brutefir-config-or-protect-file> [more files]
#
# Linux bash + gcc. BruteFIR 1.1.2 headers (bfmod.h ...) from $BF_INC_DIR,
# default <repo>/ref/bf112 (same layout as run_tests.sh). The block is taken
# from the line 'logic: "protect" {' up to the next line that starts with
# '};'. Each file: prints "accepted"/"REFUSED" and the startup log.
# Exit 0 only if every file is accepted. Nothing here starts BruteFIR or
# touches an audio device.
set -u
here=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
root=$(cd "$here/.." && pwd)
inc=${BF_INC_DIR:-$root/ref/bf112}
[ -f "$inc/bfmod.h" ] || { echo "check_config: $inc/bfmod.h not found (set BF_INC_DIR)" >&2; exit 2; }
[ $# -ge 1 ] || { echo "usage: check_config.sh <config> [config ...]" >&2; exit 2; }
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT
gcc -std=gnu99 -O2 -pthread -I"$inc" -DMODULE_SRC="\"$root/src/bflogic_protect.c\"" \
    -o "$work/harness" "$here/harness.c" -lm 2> "$work/build.log" \
  || { echo "check_config: harness build failed" >&2; cat "$work/build.log" >&2; exit 2; }
rc=0
for cfg in "$@"; do
  if grep -q '^logic: "protect" {' "$cfg"; then
    sed 's/\r$//' "$cfg" | awk '/^logic: "protect" \{/{f=1;next} /^};/{f=0} f' > "$work/body.txt"
  else
    sed 's/\r$//' "$cfg" > "$work/body.txt"        # already the block body
  fi
  if "$work/harness" preinit "$work/body.txt" > /dev/null 2> "$work/log.txt"; then
    echo "== $cfg: accepted"
  else
    echo "== $cfg: REFUSED"; rc=1
  fi
  sed 's/^/   /' "$work/log.txt"
done
exit $rc
