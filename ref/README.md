# ref/ - reference inputs for tests/run_tests.sh

- `bflogic_protect.deployed.c` (in the repo): the module source as deployed
  on the live system, read 2026-11-08 (sha256
  188a1890530b313c98059b47f79c237cab2b4f15065a0fa3964b46686f261390,
  1050 lines; built `protect.bflogic` dated 2026-11-01). T1b checks that
  the current module without link/lookahead/integrator is bit-identical to
  it (output hashes, `lmc` output, startup log); the build step checks that
  `src/` adds no compiler warnings against it. Override: `REF_SRC=<file>`.
  If the live module changes, replace this file with the new deployed source
  and record the new hash here.
- `bf112/bfmod.h` (NOT in the repo): BruteFIR v1.1.2 module header, from
  `src/bfmod.h` of the BruteFIR repository at tag v1.1.2. BruteFIR is a
  separate project under its own license, so it is not copied here. Put the
  file at `ref/bf112/bfmod.h` or set `BF_INC_DIR=<dir>`. Also used by
  `tests/check_config.sh` and `tests/integ/integ_tests.sh`.

From Git Bash, `run_tests.sh` re-runs itself inside WSL; environment
variables do not pass through `wsl.exe` unless listed in `WSLENV`, so set
`BF_INC_DIR`/`REF_SRC` in a Linux shell, or use the default paths.
