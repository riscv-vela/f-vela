#!/usr/bin/env bash
# Run one bare-metal rvv_ptr_test variant on the FPGA (fvela_firesim.sh --baremetal).
#
# usage: ./run_rvv_baremetal.sh <variant> [fvela_firesim.sh run options, e.g. --skip-infrasetup --attach]
#   variant: trap | sv39 | sv39_trap | sv39u_alias | sv39u_alias_trap | plain
#            -> build/baremetal/rvv_ptr_test_<variant>.riscv ("plain" = rvv_ptr_test.riscv)
#
# The program prints its result through HTIF and ends by itself; the uartlog is then copied to
# sims/firesim/deploy/results-workload/<date>-baremetal/baremetal0/uartlog.
# After one full run, another variant starts with --skip-infrasetup (the staged program is refreshed).
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD="${CHIPYARD:-/home/esca/_f_vela/chipyard}"

[[ $# -ge 1 ]] || { sed -n 2,10p "$0"; exit 2; }
variant="$1"; shift
if [[ "$variant" == plain ]]; then name=rvv_ptr_test.riscv; else name="rvv_ptr_test_${variant}.riscv"; fi
bin="$HERE/build/baremetal/$name"
[[ -f "$bin" ]] || { echo "missing $bin (run: make baremetal)" >&2; exit 1; }

exec "$CHIPYARD/generators/_f_vela/script/fvela_firesim.sh" run --baremetal --binary "$bin" "$@"
