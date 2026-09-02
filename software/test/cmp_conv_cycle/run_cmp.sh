#!/usr/bin/env bash
########################################################################################################################
# run_cmp.sh
#
#   Build (if needed) + run every cycle-count program in this folder on Verilator, and print
#   side-by-side comparison tables (total cycles, plus an LD/EX/ST breakdown so the CPU
#   im2col/matmul/writeback split, Gemmini's active-cycle counters, and the HW profiler's
#   summed event durations are all read off the same three labels).
#
#   Conv2D (3-way):
#     rocket_cpu_conv               -> run on BOTH sims (it's accelerator-free, so the same
#                                       binary gives the CPU-only baseline in each SoC context)
#     vanilla_gemmini_conv          -> run on NoCustomFvelaTest   (Rocket + upstream Gemmini)
#     custom_gemmini_conv_profiler  -> run on FVelaSoCConfigTest  (Rocket + F-Vela custom Gemmini)
#
#   mpgemm / int8 x ternary matmul (2-way -- no vanilla-Gemmini equivalent exists):
#     rocket_cpu_mpgemm               -> run on FVelaSoCConfigTest
#     custom_gemmini_mpgemm_profiler  -> run on FVelaSoCConfigTest
#
#   usage:
#     ./run_cmp.sh [options]
#
#   options:
#     -c, --max-cycles N   +max-cycles value (default: 100000000)
#     --no-build           skip 'make' (use already-built binaries)
#     -h, --help           show this help
#
#   outputs (all under ./build/):
#     <program>.<sim>.riscv.log   full simulation log for each run
#     <program>.csv                "type, start, end" ld/ex/st events -- from the two *_profiler
#                                   programs' HW profiler dump, and from rocket_cpu_conv /
#                                   rocket_cpu_mpgemm's t0..t3 stage timestamps (same format)
#     <program>.png                rendered load/execute/store bar graph (if matplotlib is available)
########################################################################################################################

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"
VERILATOR_DIR="$CHIPYARD_DIR/sims/verilator"
PROFILE_PY="$CHIPYARD_DIR/generators/_f_vela/software/test/profiler_test/plot/profile.py"

VANILLA_SIM="simulator-chipyard.harness-NoCustomFvelaTest"
CUSTOM_SIM="simulator-chipyard.harness-FVelaSoCConfigTest"
MAX_CYCLES=100000000
DO_BUILD=true

usage() { sed -n '4,29p' "${BASH_SOURCE[0]}" | sed 's/^#\{0,1\}//'; }

while [[ $# -gt 0 ]]; do
    case "$1" in
        -c|--max-cycles) MAX_CYCLES="$2"; shift 2 ;;
        --no-build)      DO_BUILD=false;  shift ;;
        -h|--help)       usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
done

BUILD_DIR="$SCRIPT_DIR/build"

source "$CHIPYARD_DIR/env.sh"
ulimit -s unlimited

if $DO_BUILD; then
    echo ">>> Building cmp_conv_cycle tests..."
    make -C "$SCRIPT_DIR" || { echo "Build failed"; exit 1; }
fi

# run_one <elf-basename> <simulator-name> -> writes build/<elf-basename>.<simulator-name>.riscv.log,
# echoes the matched "RESULT ... cycles: N" line's N to stdout.
run_one() {
    local prog="$1" sim="$2"
    local elf="$BUILD_DIR/$prog.riscv"
    local sim_path="$VERILATOR_DIR/$sim"
    local log="$BUILD_DIR/$prog.$sim.riscv.log"

    if [ ! -f "$elf" ]; then
        echo "Binary not found: $elf" >&2
        return 1
    fi
    if [ ! -f "$sim_path" ]; then
        echo "Simulator not found: $sim_path (build it in sims/verilator first)" >&2
        return 1
    fi

    echo "  -> running $prog on $sim ..." >&2
    "$sim_path" +permissive +max-cycles="$MAX_CYCLES" +permissive-off "$elf" \
        2>&1 | tee "$log" >&2
    local status=${PIPESTATUS[0]}
    if [ "$status" -ne 0 ]; then
        echo "  simulation error (exit $status) for $prog on $sim" >&2
        return 1
    fi

    # -a: some runs prepend a few NUL bytes to the UART log during boot warm-up, which would
    # otherwise make grep treat the file as binary and silently skip the match.
    grep -aoE 'RESULT [a-z_]+ cycles: *[0-9]+' "$log" | grep -aoE '[0-9]+$'
}

# extract_field <log> <LD_CYCLES|EX_CYCLES|ST_CYCLES> -> that line's trailing number, whatever
# the parenthesized source annotation says (im2col gather / ternary unpack / RDMA_ACTIVE_CYCLE /
# counter, .../ profiler, ... events).
extract_field() {
    local log="$1" field="$2"
    [ -f "$log" ] || return 1
    grep -aoE "${field}[^:]*: *[0-9]+" "$log" | head -1 | grep -aoE '[0-9]+$'
}

# extract_checksum <log> -> the "RESULT output_checksum: 0x........" hex value from that log.
# All five programs print this (see each file's output_checksum()/FNV-1a comment) so you can
# verify two implementations computed the *same* result, not just that they took a comparable
# number of cycles -- cycle counts alone never tell you that.
extract_checksum() {
    local log="$1"
    [ -f "$log" ] || return 1
    grep -aoE 'output_checksum: 0x[0-9a-f]+' "$log" | head -1 | grep -aoE '0x[0-9a-f]+$'
}

# compare_checksum <label> <checksum-a> <checksum-b>
compare_checksum() {
    local label="$1" a="$2" b="$3"
    if [ -z "$a" ] || [ -z "$b" ]; then
        printf "  %-55s %s\n" "$label" "SKIP (missing checksum)"
    elif [ "$a" = "$b" ]; then
        printf "  %-55s MATCH  (%s)\n" "$label" "$a"
    else
        printf "  %-55s MISMATCH  (%s vs %s)\n" "$label" "$a" "$b"
    fi
}

# extract_profiler_csv <log> <csv-out> <png-out-basename-for-messages> -> extracts the
# PROFILE-DUMP block into a CSV and renders it, same pipeline run_profiler.sh uses.
extract_profiler_csv() {
    local log="$1" csv="$2" png="$3"
    [ -f "$log" ] || return 1
    awk '/=== PROFILE DUMP BEGIN ===/{f=1;next} /=== PROFILE DUMP END ===/{f=0} f' "$log" \
        | grep -aE '^[[:space:]]*[0-9]+[[:space:]]*,[[:space:]]*[0-9]+[[:space:]]*,[[:space:]]*[0-9]+[[:space:]]*$' \
        > "$csv"
    local nlines
    nlines=$(wc -l < "$csv")
    echo "Extracted $nlines profiler events -> $csv"
    if [ "$nlines" -gt 0 ]; then
        local pybin
        pybin="$(pick_python)"
        if [ -n "$pybin" ]; then
            "$pybin" "$PROFILE_PY" "$csv" "$png" && echo "Graph: $png"
        else
            echo "  (no python with matplotlib found; skipping graph -- CSV is ready)"
        fi
    fi
}

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

print_ldexst_row() {
    printf "  %-42s %12s %12s %12s\n" "$1" "$2" "$3" "$4"
}

echo "=============================================="
echo "  Conv2D cycle comparison"
echo "  vanilla sim: $VANILLA_SIM"
echo "  custom  sim: $CUSTOM_SIM"
echo "=============================================="

CPU_ON_VANILLA=$(run_one rocket_cpu_conv "$VANILLA_SIM")
VANILLA_GEMMINI=$(run_one vanilla_gemmini_conv "$VANILLA_SIM")
CPU_ON_CUSTOM=$(run_one rocket_cpu_conv "$CUSTOM_SIM")
CUSTOM_GEMMINI=$(run_one custom_gemmini_conv_profiler "$CUSTOM_SIM")

echo
echo "=============================================="
echo "  mpgemm (int8 x ternary) comparison -- no vanilla-Gemmini equivalent"
echo "  sim: $CUSTOM_SIM"
echo "=============================================="

CPU_MPGEMM=$(run_one rocket_cpu_mpgemm "$CUSTOM_SIM")
CUSTOM_MPGEMM=$(run_one custom_gemmini_mpgemm_profiler "$CUSTOM_SIM")

# --- extract the profiler dumps into CSVs + plots, same as run_profiler.sh -------------------------------------
# custom_gemmini_*_profiler.c use the real HW profiler; rocket_cpu_conv.c / rocket_cpu_mpgemm.c
# emit the same "type, start, end" PROFILE DUMP format from their t0..t3 stage timestamps
# (see dump_profile() in each), so they go through the identical extract+plot pipeline.
echo
extract_profiler_csv \
    "$BUILD_DIR/custom_gemmini_conv_profiler.$CUSTOM_SIM.riscv.log" \
    "$BUILD_DIR/custom_gemmini_conv_profiler.csv" \
    "$BUILD_DIR/custom_gemmini_conv_profiler.png"
extract_profiler_csv \
    "$BUILD_DIR/custom_gemmini_mpgemm_profiler.$CUSTOM_SIM.riscv.log" \
    "$BUILD_DIR/custom_gemmini_mpgemm_profiler.csv" \
    "$BUILD_DIR/custom_gemmini_mpgemm_profiler.png"
extract_profiler_csv \
    "$BUILD_DIR/rocket_cpu_conv.$VANILLA_SIM.riscv.log" \
    "$BUILD_DIR/rocket_cpu_conv.$VANILLA_SIM.csv" \
    "$BUILD_DIR/rocket_cpu_conv.$VANILLA_SIM.png"
extract_profiler_csv \
    "$BUILD_DIR/rocket_cpu_conv.$CUSTOM_SIM.riscv.log" \
    "$BUILD_DIR/rocket_cpu_conv.$CUSTOM_SIM.csv" \
    "$BUILD_DIR/rocket_cpu_conv.$CUSTOM_SIM.png"
extract_profiler_csv \
    "$BUILD_DIR/rocket_cpu_mpgemm.$CUSTOM_SIM.riscv.log" \
    "$BUILD_DIR/rocket_cpu_mpgemm.csv" \
    "$BUILD_DIR/rocket_cpu_mpgemm.png"

# --- conv summary table ----------------------------------------------------------------------------------------------
echo
echo "=============================================="
echo "  CONV RESULTS (cycles)"
echo "=============================================="
printf "  %-42s %15s\n" "Rocket CPU conv (on $VANILLA_SIM)"       "${CPU_ON_VANILLA:-FAIL}"
printf "  %-42s %15s\n" "Vanilla Gemmini conv (tiled_conv_auto)"  "${VANILLA_GEMMINI:-FAIL}"
printf "  %-42s %15s\n" "Rocket CPU conv (on $CUSTOM_SIM)"        "${CPU_ON_CUSTOM:-FAIL}"
printf "  %-42s %15s\n" "Custom Gemmini conv (HW profiler)"       "${CUSTOM_GEMMINI:-FAIL}"
echo "=============================================="
if [ -n "$CPU_ON_VANILLA" ] && [ -n "$VANILLA_GEMMINI" ] && [ "$VANILLA_GEMMINI" -gt 0 ]; then
    awk -v c="$CPU_ON_VANILLA" -v g="$VANILLA_GEMMINI" 'BEGIN{printf "  Vanilla speedup vs CPU: %.2fx\n", c/g}'
fi
if [ -n "$CPU_ON_CUSTOM" ] && [ -n "$CUSTOM_GEMMINI" ] && [ "$CUSTOM_GEMMINI" -gt 0 ]; then
    awk -v c="$CPU_ON_CUSTOM" -v g="$CUSTOM_GEMMINI" 'BEGIN{printf "  Custom  speedup vs CPU: %.2fx\n", c/g}'
fi
if [ -n "$VANILLA_GEMMINI" ] && [ -n "$CUSTOM_GEMMINI" ] && [ "$CUSTOM_GEMMINI" -gt 0 ]; then
    awk -v v="$VANILLA_GEMMINI" -v cu="$CUSTOM_GEMMINI" 'BEGIN{printf "  Custom  speedup vs vanilla Gemmini: %.2fx\n", v/cu}'
fi

# --- conv output correctness (do the three implementations actually agree?) ------------------------------------------
CPU_LOG="$BUILD_DIR/rocket_cpu_conv.$CUSTOM_SIM.riscv.log"
VANILLA_LOG="$BUILD_DIR/vanilla_gemmini_conv.$VANILLA_SIM.riscv.log"
CUSTOM_LOG="$BUILD_DIR/custom_gemmini_conv_profiler.$CUSTOM_SIM.riscv.log"

echo
echo "  CONV output checksum (do the three implementations agree on the actual numbers?):"
compare_checksum "Rocket CPU vs Vanilla Gemmini"   "$(extract_checksum "$CPU_LOG")" "$(extract_checksum "$VANILLA_LOG")"
compare_checksum "Rocket CPU vs Custom Gemmini"    "$(extract_checksum "$CPU_LOG")" "$(extract_checksum "$CUSTOM_LOG")"
compare_checksum "Vanilla Gemmini vs Custom Gemmini" "$(extract_checksum "$VANILLA_LOG")" "$(extract_checksum "$CUSTOM_LOG")"

# --- conv LD/EX/ST breakdown -------------------------------------------------------------------------------------------
echo
echo "=============================================="
echo "  CONV LD / EX / ST BREAKDOWN (cycles) -- DIAGNOSTIC ONLY, NOT A 1:1 COMPARISON"
echo "=============================================="
echo "  These three rows measure three DIFFERENT KINDS of thing, not the same quantity under"
echo "  different names -- do not read \"CPU EX\" vs \"Gemmini EX\" as \"should be equal\":"
echo "    - Rocket CPU:        wall-clock time of a whole C function (loads, shifts, branches,"
echo "                         stores, loop overhead -- everything the scalar core did)"
echo "    - active-cycle counters: HW unit busy time (RDMA/EXE/WDMA), gated by internal FSM"
echo "                         signals, not the same accounting as a software function"
echo "    - profiler-summed events: sum of many small per-instruction durations that overlap"
echo "                         in time across tiles, so their sum is not a wall-clock interval"
echo "  The FAIR, apples-to-apples number is the total in the RESULTS table above (rdcycle"
echo "  end-to-end, with gemmini_fence()/completion wait on the HW side). Use this breakdown"
echo "  only to see where *each implementation's own* time went, not to compare rows against"
echo "  each other. HW rows can also legitimately have LD+EX+ST > total, since ld/ex/st pipeline"
echo "  and overlap across tiles -- CPU cannot do that, its three stages run strictly back-to-back."
echo "=============================================="
print_ldexst_row "" "LD" "EX" "ST"
print_ldexst_row "Rocket CPU conv" \
    "$(extract_field "$CPU_LOG" LD_CYCLES)" "$(extract_field "$CPU_LOG" EX_CYCLES)" "$(extract_field "$CPU_LOG" ST_CYCLES)"
print_ldexst_row "Vanilla Gemmini (active-cycle counters)" \
    "$(extract_field "$VANILLA_LOG" LD_CYCLES)" "$(extract_field "$VANILLA_LOG" EX_CYCLES)" "$(extract_field "$VANILLA_LOG" ST_CYCLES)"
print_ldexst_row "Custom Gemmini (profiler-summed events)" \
    "$(grep -aoE 'LD_CYCLES \(profiler[^:]*: *[0-9]+' "$CUSTOM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')" \
    "$(grep -aoE 'EX_CYCLES \(profiler[^:]*: *[0-9]+' "$CUSTOM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')" \
    "$(grep -aoE 'ST_CYCLES \(profiler[^:]*: *[0-9]+' "$CUSTOM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')"
print_ldexst_row "Custom Gemmini (active-cycle counters)" \
    "$(grep -aoE 'LD_CYCLES \(counter[^:]*: *[0-9]+' "$CUSTOM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')" \
    "$(grep -aoE 'EX_CYCLES \(counter[^:]*: *[0-9]+' "$CUSTOM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')" \
    "$(grep -aoE 'ST_CYCLES \(counter[^:]*: *[0-9]+' "$CUSTOM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')"

# --- mpgemm summary table + breakdown -----------------------------------------------------------------------------
echo
echo "=============================================="
echo "  MPGEMM RESULTS (cycles)"
echo "=============================================="
printf "  %-42s %15s\n" "Rocket CPU mpgemm (on $CUSTOM_SIM)"        "${CPU_MPGEMM:-FAIL}"
printf "  %-42s %15s\n" "Custom Gemmini mpgemm (HW profiler)"       "${CUSTOM_MPGEMM:-FAIL}"
echo "=============================================="
if [ -n "$CPU_MPGEMM" ] && [ -n "$CUSTOM_MPGEMM" ] && [ "$CUSTOM_MPGEMM" -gt 0 ]; then
    awk -v c="$CPU_MPGEMM" -v g="$CUSTOM_MPGEMM" 'BEGIN{printf "  Custom  speedup vs CPU: %.2fx\n", c/g}'
fi

CPU_MPGEMM_LOG="$BUILD_DIR/rocket_cpu_mpgemm.$CUSTOM_SIM.riscv.log"
CUSTOM_MPGEMM_LOG="$BUILD_DIR/custom_gemmini_mpgemm_profiler.$CUSTOM_SIM.riscv.log"

echo
echo "  MPGEMM output checksum (do the two implementations agree on the actual numbers?):"
compare_checksum "Rocket CPU vs Custom Gemmini" "$(extract_checksum "$CPU_MPGEMM_LOG")" "$(extract_checksum "$CUSTOM_MPGEMM_LOG")"

echo
echo "=============================================="
echo "  MPGEMM LD / EX / ST BREAKDOWN (cycles) -- DIAGNOSTIC ONLY, NOT A 1:1 COMPARISON"
echo "  (see the CONV breakdown's note above -- same caveat applies here)"
echo "=============================================="
print_ldexst_row "" "LD" "EX" "ST"
print_ldexst_row "Rocket CPU mpgemm" \
    "$(extract_field "$CPU_MPGEMM_LOG" LD_CYCLES)" "$(extract_field "$CPU_MPGEMM_LOG" EX_CYCLES)" "$(extract_field "$CPU_MPGEMM_LOG" ST_CYCLES)"
print_ldexst_row "Custom Gemmini (profiler-summed events)" \
    "$(grep -aoE 'LD_CYCLES \(profiler[^:]*: *[0-9]+' "$CUSTOM_MPGEMM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')" \
    "$(grep -aoE 'EX_CYCLES \(profiler[^:]*: *[0-9]+' "$CUSTOM_MPGEMM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')" \
    "$(grep -aoE 'ST_CYCLES \(profiler[^:]*: *[0-9]+' "$CUSTOM_MPGEMM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')"
print_ldexst_row "Custom Gemmini (active-cycle counters)" \
    "$(grep -aoE 'LD_CYCLES \(counter[^:]*: *[0-9]+' "$CUSTOM_MPGEMM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')" \
    "$(grep -aoE 'EX_CYCLES \(counter[^:]*: *[0-9]+' "$CUSTOM_MPGEMM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')" \
    "$(grep -aoE 'ST_CYCLES \(counter[^:]*: *[0-9]+' "$CUSTOM_MPGEMM_LOG" 2>/dev/null | grep -aoE '[0-9]+$')"
