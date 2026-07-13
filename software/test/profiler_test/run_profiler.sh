#!/usr/bin/env bash
########################################################################################################################
# run_profiler.sh
#
#   Build (if needed) + run the Gemmini profiler test on Verilator, extract the profile
#   data from the log, and render the load/execute/store bar-graph with profiler/profile.py.
#
#   usage:
#     ./run_profiler.sh [program] [options]
#
#       [program]   test basename to run (default: profiler_test).
#                   e.g. profiler_test, ternary_profiler_test
#
#   options:
#     -s, --sim <CONFIG>   simulator config (default: FVelaSoCConfigTest)
#     -c, --max-cycles N   +max-cycles value (default: 100000000)
#     --no-build           skip 'make' (use an already-built binary)
#     --no-graph           run the sim but skip the python visualization
#     -h, --help           show this help
#
#   outputs (all under ./build/, named after <program>):
#     <program>.riscv.log   full simulation log
#     <program>.csv         extracted "type, start, end" lines
#     <program>.png         rendered bar graph
########################################################################################################################

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"
SOFTWARE_DIR="$CHIPYARD_DIR/generators/_f_vela/software"
VERILATOR_DIR="$CHIPYARD_DIR/sims/verilator"
PROFILE_PY="$SCRIPT_DIR/plot/profile.py"

SIMULATOR="simulator-chipyard.harness-FVelaSoCConfigTest"
MAX_CYCLES=100000000
DO_BUILD=true
DO_GRAPH=true

usage() { sed -n '4,28p' "${BASH_SOURCE[0]}" | sed 's/^#\{0,1\}//'; }

PROG="gemm_profiler_test"
while [[ $# -gt 0 ]]; do
    case "$1" in
        -s|--sim)
            if [[ "$2" == simulator-chipyard.harness-* ]]; then SIMULATOR="$2"; else SIMULATOR="simulator-chipyard.harness-$2"; fi
            shift 2 ;;
        -c|--max-cycles) MAX_CYCLES="$2"; shift 2 ;;
        --no-build)      DO_BUILD=false;  shift ;;
        --no-graph)      DO_GRAPH=false;  shift ;;
        -h|--help)       usage; exit 0 ;;
        -*) echo "Unknown option: $1"; usage; exit 1 ;;
        *)  PROG="${1%.riscv}"; shift ;;   # program basename (strip .riscv if given)
    esac
done

BUILD_DIR="$SCRIPT_DIR/build"
ELF_FILE="$BUILD_DIR/$PROG.riscv"
LOG_FILE="$BUILD_DIR/$PROG.riscv.log"
CSV_FILE="$BUILD_DIR/$PROG.csv"
PNG_FILE="$BUILD_DIR/$PROG.png"

# --- environment (toolchain + conda python with matplotlib) -----------------------------------------------------------
source "$CHIPYARD_DIR/env.sh"
ulimit -s unlimited

# --- build ------------------------------------------------------------------------------------------------------------
if $DO_BUILD; then
    echo ">>> Building profiler test..."
    make -C "$SCRIPT_DIR" || { echo "Build failed"; exit 1; }
fi

if [ ! -f "$ELF_FILE" ]; then
    echo "Binary not found: $ELF_FILE (run without --no-build, or 'make' first)"
    exit 1
fi

SIM_PATH="$VERILATOR_DIR/$SIMULATOR"
if [ ! -f "$SIM_PATH" ]; then
    echo "Simulator not found: $SIM_PATH"
    echo "Build it first in sims/verilator (e.g. run script/verilator_make.sh)."
    exit 1
fi

echo "=============================================="
echo "  Profiler Simulation"
echo "  Simulator: $SIMULATOR"
echo "  Binary:    $(basename "$ELF_FILE")"
echo "  Log:       $LOG_FILE"
echo "=============================================="

# --- run (bare-metal) -------------------------------------------------------------------------------------------------
"$SIM_PATH" \
    +permissive \
    +max-cycles="$MAX_CYCLES" \
    +permissive-off \
    "$ELF_FILE" \
    2>&1 | tee "$LOG_FILE"
STATUS=${PIPESTATUS[0]}
if [ $STATUS -ne 0 ]; then
    echo "  Simulation error (exit $STATUS)"
    exit 1
fi

# --- extract the software-dumped profile block ("type, start, end" CSV) -----------------------------------------------
#   These lines come from the P[] buffer the profiler DMA wrote, bounded by the markers.
awk '/=== PROFILE DUMP BEGIN ===/{f=1;next} /=== PROFILE DUMP END ===/{f=0} f' "$LOG_FILE" \
    | grep -E '^[[:space:]]*[0-9]+[[:space:]]*,[[:space:]]*[0-9]+[[:space:]]*,[[:space:]]*[0-9]+[[:space:]]*$' \
    > "$CSV_FILE"

NLINES=$(wc -l < "$CSV_FILE")
echo "----------------------------------------------"
echo "  Extracted $NLINES profile events -> $CSV_FILE"

if [ "$NLINES" -eq 0 ]; then
    echo "  WARNING: no profile events captured."
    echo "  Check that the simulator was built with the Profiler enabled and that"
    echo "  the workload issued profiled mvin/compute/mvout commands."
    exit 1
fi

# --- visualize --------------------------------------------------------------------------------------------------------
# Pick a python that actually has matplotlib. env.sh's active env may not (the
# conda-env alongside it does), so probe a few candidates.
pick_python() {
    for py in "$PYTHON" python3 \
              "$CHIPYARD_DIR/.conda-env/bin/python3" \
              "$CHIPYARD_DIR/.conda-lock-env/bin/python3"; do
        [ -n "$py" ] || continue
        command -v "$py" >/dev/null 2>&1 || [ -x "$py" ] || continue
        if "$py" -c "import matplotlib" >/dev/null 2>&1; then echo "$py"; return 0; fi
    done
    return 1
}

if $DO_GRAPH; then
    PYBIN="$(pick_python)"
    if [ -z "$PYBIN" ]; then
        echo "  WARNING: no python with matplotlib found; skipping graph."
        echo "  CSV data is ready at $CSV_FILE — render it later with:"
        echo "    <python-with-matplotlib> $PROFILE_PY $CSV_FILE $PNG_FILE"
    else
        echo "  Rendering graph with $PYBIN ..."
        "$PYBIN" "$PROFILE_PY" "$CSV_FILE" "$PNG_FILE"
        echo "  Graph: $PNG_FILE"
    fi
fi
echo "  Done."
