# ref/ - reference inputs for tests/run_tests.sh

- `bflogic_protect.deployed.c` (in the repo): the module source that was
  live until 2026-11-22 (module "alk-v1m-la": channel link 54ecb2b +
  lookahead). sha256
  67f5c001cca786f3818a0d6864f159ef4ac557128e403fbf24fdf99aad8a1f45,
  1899 lines, byte-identical to `src/bflogic_protect.c` at commit a6e7122.
  Origin: holographer's own session copied the live source on 2026-11-22;
  the hash of that copy was checked before it was committed here.
  T1b checks that the current module without the new features is
  bit-identical to it (output hashes, `lmc` output, startup log). The
  build step checks that `src/` adds no compiler warnings against it.
  Override: `REF_SRC=<file>`.
  - Earlier reference (commit 9b75907 only): sha256 188a1890..., equal to
    fad1221 (protect v3, no link). It was live on 2026-11-08, not later.
  - After the Cue-020 deploy (module "alk-v1m-la-integ", written live
    2026-11-22), the live source is the integrator module itself. From then
    on this file is the last baseline before the integrator, not the live
    state. Keep it as long as T1b must show that the integrator changes
    nothing in the existing modes. When the next change needs the live
    state as its baseline, replace this file with the live source and
    record its hash, date and origin here. Then rerun the suite.
- `bf112/bfmod.h` (NOT in the repo): BruteFIR v1.1.2 module header, from
  `src/bfmod.h` of the BruteFIR repository at tag v1.1.2. BruteFIR is a
  separate project (ISC license, like this module), so the header is not
  copied here; whether to add it is OPEN (the owner decides). Put the
  file at `ref/bf112/bfmod.h` or set `BF_INC_DIR=<dir>`. Also used by
  `tests/check_config.sh` and `tests/integ/integ_tests.sh`.

From Git Bash, `run_tests.sh` re-runs itself inside WSL. Environment
variables do not pass through `wsl.exe` unless they are listed in
`WSLENV`. So set `BF_INC_DIR`/`REF_SRC` in a Linux shell, or use the
default paths.
