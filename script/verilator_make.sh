#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../../" && pwd)"
VERILATOR_DIR="$CHIPYARD_DIR/sims/verilator"

echo "Script directory: $SCRIPT_DIR"
echo "Chipyard root: $CHIPYARD_DIR"
echo "Verilator root: $VERILATOR_DIR"

CONFIG_NAME="FVelaSoCConfigTest" # default config name
# CONFIG_NAME="REFV256D128RocketConfig"
# CONFIG_NAME="FVelaGemminiConfigTest" # default config name

echo ""
echo "=============================================="
echo "  Verilator Build Script"
echo "  Chipyard root: $CHIPYARD_DIR"
echo "  Config Scala file: $CONFIG_NAME"
echo "=============================================="

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo "  -c, --config <name>   Set CONFIG_NAME (default: $CONFIG_NAME)"
    echo "                        Ex: -c MyCustomConfig (package 'chipyard.' is added automatically)"
    echo "  -d, --debug           Set VERILATOR_DEBUG=1 for a debug build (default: release build)."
    echo "                        Also builds the *-debug binary with waveform tracing enabled"
    echo "                        (--trace), so run_vsim.sh -w can dump a VCD waveform."
    echo "  -w, --wave            Alias for -d/--debug (build with waveform tracing enabled)."
    echo "  --fst                 Use FST instead of VCD for the waveform (sets USE_FST=1)."
    echo "                        Only takes effect together with -d/--debug/-w."
    echo "  --clean               Run 'make clean' before building"
    echo "  -h, --help            Display this help message"
    exit 1
}

#argument parsing
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--config)
            CONFIG_NAME="$2"
            shift 2
            ;;
        --clean)
            DO_CLEAN=true
            shift
            ;;
        -d|--debug|-w|--wave)
            export VERILATOR_DEBUG=1
            shift
            ;;
        --fst)
            export USE_FST=1
            shift
            ;;
        -h|--help)
            usage
            echo "Example usage:"
            echo "  $0 -c MyCustomConfig (default: $CONFIG_NAME)"
            echo "  $0 -d (build in debug mode, with waveform tracing enabled)"
            echo "  $0 -d --fst (debug build, dump waveform as FST instead of VCD)"
            echo "  $0 --clean"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

export JAVA_TOOL_OPTIONS="-Xmx12G -Xss16M -XX:+UseG1GC"
echo "Java memory options set to: $JAVA_TOOL_OPTIONS"

# source env.sh to set up environment variables. This is needed to get VERILATOR_ROOT and other variables that the Makefile relies on.
if [ -f "$CHIPYARD_DIR/env.sh" ]; then
    source "$CHIPYARD_DIR/env.sh"
else
    echo "Error: env.sh not found in $CHIPYARD_DIR"
    exit 1
fi

# clean
if [ "$DO_CLEAN" = true ]; then
    echo "Cleaning up previous builds..."
    make -C "$VERILATOR_DIR" clean
    exit 0
fi

if [ -n "$VERILATOR_DEBUG" ]; then
    # build debug version if VERILATOR_DEBUG is set
    echo "Building in debug mode (VERILATOR_DEBUG=1), waveform tracing enabled"
    echo "Waveform format: $([ "$USE_FST" = "1" ] && echo FST || echo VCD) (USE_FST=${USE_FST:-0})"
    echo "Building Verilator simulation..."
    make -C "$VERILATOR_DIR" debug CONFIG="$CONFIG_NAME" -j$(nproc)
else
    # build release version by default
    echo "Building in release mode"
    echo "Building Verilator simulation..."
    make -C "$VERILATOR_DIR" CONFIG="$CONFIG_NAME" -j$(nproc) VERILATOR_THREADS=4
fi

if [ $? -eq 0 ]; then
    echo "@@@ Build Successful! @@@"
else
    echo "@@@ Build Failed! @@@"
    exit 1
fi




