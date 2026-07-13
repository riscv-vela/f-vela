#!/usr/bin/env bash
########################################################################################################################
# run_sim.sh
#
#   Run the Chipyard Verilator simulation for the gemmini / pid / rope tests.
#   Based on verilator_sim.sh, generalized to pick the right build directory per test type.
#
#   usage:
#     ./run_sim.sh <test> [program] [options]
#
#       <test>      gemmini | pid | rope        (required)
#       [program]   binary basename to run (e.g. gemm, vfpid_4d, vfrope_test).
#                   Extension optional; .riscv / .elf are auto-resolved.
#                   Also accepts a full path to a binary. If omitted, a per-test default is used.
#
#   options:
#     -s, --sim <CONFIG>   simulator config (default: FVelaSoCConfigTest).
#                          Accepts either "FVelaSoCConfigTest" or the full
#                          "simulator-chipyard.harness-FVelaSoCConfigTest".
#     -l, --list           list the available built binaries for <test> and exit.
#     -c, --max-cycles N   +max-cycles value (default: 100000000).
#     -v, --verbose        pass +verbose to the simulator (default: off).
#         --no-verbose     do not pass +verbose (explicit; this is the default).
#     -h, --help           show this help.
#
#   examples:
#     ./run_sim.sh gemmini                 # run default gemmini test (ternary_gemm)
#     ./run_sim.sh gemmini gemm            # run rv_gemmini_test/build/gemm.riscv
#     ./run_sim.sh pid vfpid_4d
#     ./run_sim.sh rope vfrope_test
#     ./run_sim.sh gemmini --list
########################################################################################################################

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

SOFTWARE_DIR="$CHIPYARD_DIR/generators/_f_vela/software"
VERILATOR_DIR="$CHIPYARD_DIR/sims/verilator"

# --- defaults ---------------------------------------------------------------------------------------------------------
SIMULATOR="simulator-chipyard.harness-FVelaSoCConfigTest"
MAX_CYCLES=100000000
DO_LIST=false
VERBOSE=false

usage() {
    sed -n '4,31p' "${BASH_SOURCE[0]}" | sed 's/^#\{0,1\}//'
}

# --- positional: test type --------------------------------------------------------------------------------------------
TEST_TYPE="$1"; shift 2>/dev/null

case "$TEST_TYPE" in
    gemmini) BUILD_DIR="$SOFTWARE_DIR/test/rv_gemmini_test/build"; DEFAULT_PROG="ternary_gemm" ;;
    pid)     BUILD_DIR="$SOFTWARE_DIR/test/rv_pid_test/build";     DEFAULT_PROG="vfpid_4d"     ;;
    rope)    BUILD_DIR="$SOFTWARE_DIR/test/rv_rope_test/build";    DEFAULT_PROG="vfrope_test"  ;;
    profiler) BUILD_DIR="$SOFTWARE_DIR/test/profiler_test/build"; DEFAULT_PROG="vfprofiler_test"  ;;
    -h|--help|"") usage; exit 0 ;;
    *)
        echo "Unknown test type: '$TEST_TYPE' (expected gemmini | pid | rope | profiler)"
        usage
        exit 1
        ;;
esac

# --- option / program parsing -----------------------------------------------------------------------------------------
PROG=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        -s|--sim)
            if [[ "$2" == simulator-chipyard.harness-* ]]; then
                SIMULATOR="$2"
            else
                SIMULATOR="simulator-chipyard.harness-$2"
            fi
            shift 2 ;;
        -c|--max-cycles) MAX_CYCLES="$2"; shift 2 ;;
        -l|--list)       DO_LIST=true;    shift ;;
        -v|--verbose)    VERBOSE=true;    shift ;;
        --no-verbose)    VERBOSE=false;   shift ;;
        -h|--help)       usage; exit 0 ;;
        -*)
            echo "Unknown option: $1"; usage; exit 1 ;;
        *)  PROG="$1"; shift ;;
    esac
done

# --- list mode --------------------------------------------------------------------------------------------------------
if $DO_LIST; then
    echo "Available binaries in $BUILD_DIR:"
    if [ -d "$BUILD_DIR" ]; then
        find "$BUILD_DIR" -maxdepth 1 \( -name '*.riscv' -o -name '*.elf' \) -printf '  %f\n' | sort
    else
        echo "  (build directory does not exist — build the test first with 'make')"
    fi
    exit 0
fi

# --- resolve ELF/binary path ------------------------------------------------------------------------------------------
[ -z "$PROG" ] && PROG="$DEFAULT_PROG"

resolve_binary() {
    local p="$1"
    # absolute/relative path given directly
    if [ -f "$p" ]; then echo "$p"; return 0; fi
    # inside the build dir, try as-is then with extensions
    for cand in "$BUILD_DIR/$p" "$BUILD_DIR/$p.riscv" "$BUILD_DIR/$p.elf"; do
        [ -f "$cand" ] && { echo "$cand"; return 0; }
    done
    return 1
}

ELF_FILE="$(resolve_binary "$PROG")"
if [ -z "$ELF_FILE" ]; then
    echo "Binary not found for '$PROG' in $BUILD_DIR"
    echo "Tried: $PROG, $BUILD_DIR/$PROG[.riscv|.elf]"
    echo "Hint: build it first, or run './run_sim.sh $TEST_TYPE --list' to see what's available."
    exit 1
fi
ELF_NAME="$(basename "$ELF_FILE")"

SIM_PATH="$VERILATOR_DIR/$SIMULATOR"
if [ ! -f "$SIM_PATH" ]; then
    echo "Simulator not found: $SIM_PATH"
    echo "Build it first in sims/verilator (e.g. run script/verilator_make.sh)."
    exit 1
fi

# --- rope needs its LUT hex files next to the simulator (loadMemoryFromFileInline) ------------------------------------
if [ "$TEST_TYPE" = "rope" ]; then
    ROPE_SRC_DIR="$SOFTWARE_DIR/test/rv_rope_test"
    if ls "$ROPE_SRC_DIR"/*.hex >/dev/null 2>&1; then
        echo "Copying RoPE LUT hex files to $VERILATOR_DIR ..."
        cp -f "$ROPE_SRC_DIR"/*.hex "$VERILATOR_DIR/"
    fi
fi

LOG_FILE="$BUILD_DIR/${ELF_NAME}.log"

echo "=============================================="
echo "  Simulation Starting..."
echo "  Test type: $TEST_TYPE"
echo "  Simulator: $SIMULATOR"
echo "  Binary:    $ELF_NAME"
echo "  Verbose:   $VERBOSE"
echo "  Log:       $LOG_FILE"
echo "=============================================="

# --- environment ------------------------------------------------------------------------------------------------------
source "$CHIPYARD_DIR/env.sh"
ulimit -s unlimited

# --- assemble simulator plusargs --------------------------------------------------------------------------------------
SIM_ARGS=( +permissive )
$VERBOSE && SIM_ARGS+=( +verbose )
SIM_ARGS+=( +max-cycles="$MAX_CYCLES" +permissive-off )

# --- run (bare-metal) -------------------------------------------------------------------------------------------------
# stdbuf -o0 -e0 "$SIM_PATH" "${SIM_ARGS[@]}" "$ELF_FILE" 2>&1 | tee "$LOG_FILE"
# STATUS=${PIPESTATUS[0]}

# --> # stdbuf -o0 -e0 "$SIM_PATH" \
#     +permissive \
#     +verbose \
#     +max-cycles="$MAX_CYCLES" \
#     +permissive-off \
#     "$ELF_FILE" \
#     2>&1 | tee "$LOG_FILE"

"$SIM_PATH" \
    "${SIM_ARGS[@]}" \
    "$ELF_FILE" \
    2>&1 | tee "$LOG_FILE"
STATUS=${PIPESTATUS[0]}

if [ $STATUS -eq 0 ]; then
    echo "  Simulation Completed Successfully!"
    tail -n 20 "$LOG_FILE" | grep -E "SUCCESS|FAIL|PASSED" || true
else
    echo "  Simulation Encountered an Error (exit $STATUS)"
    exit 1
fi
