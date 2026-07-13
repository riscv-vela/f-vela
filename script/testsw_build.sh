#!/usr/bin/env bash
########################################################################################################################
# testsw_build.sh
#
#   Builds the RISC-V test software under software/test by delegating to its top-level Makefile.
#
#   usage:
#     ./testsw_build.sh                 # build all tests (gemmini + pid + rope + profiler)
#     ./testsw_build.sh gemmini         # build only rv_gemmini_test
#     ./testsw_build.sh pid             # build only rv_pid_test
#     ./testsw_build.sh rope            # build only rv_rope_test
#     ./testsw_build.sh profiler        # build only profiler_test
#     ./testsw_build.sh gemmini pid     # build multiple folders
#     ./testsw_build.sh clean           # clean all
#     ./testsw_build.sh -h | --help     # show this help
########################################################################################################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

TEST_DIR="$CHIPYARD_DIR/generators/_f_vela/software/test"

VALID_TARGETS="gemmini pid rope profiler all clean"

usage() {
    sed -n '3,15p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

if [[ ! -f "$TEST_DIR/Makefile" ]]; then
    echo "ERROR: Makefile not found at $TEST_DIR/Makefile" >&2
    exit 1
fi

# no argument -> build everything
if [[ $# -eq 0 ]]; then
    echo ">> Building ALL tests in $TEST_DIR"
    exec make -C "$TEST_DIR" all
fi

# help
case "$1" in
    -h|--help) usage; exit 0 ;;
esac

# validate every requested target before building
for target in "$@"; do
    if [[ " $VALID_TARGETS " != *" $target "* ]]; then
        echo "ERROR: unknown target '$target' (expected: $VALID_TARGETS)" >&2
        usage
        exit 1
    fi
done

echo ">> Building targets [$*] in $TEST_DIR"
exec make -C "$TEST_DIR" "$@"
