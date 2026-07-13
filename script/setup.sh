#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

echo ""
echo "=============================================="
echo "  F-Vela Setup Script"
echo "  Chipyard root: $CHIPYARD_DIR"
echo "  build.sbt will be modified to include the F-Vela config..."
echo "=============================================="
echo ""

cd $CHIPYARD_DIR

# Check if env.sh exists and source it
if [ -f "$CHIPYARD_DIR/env.sh" ]; then
    echo "Sourcing env.sh to set up environment variables..."
    source "$CHIPYARD_DIR/env.sh"
else
    echo "Error: env.sh not found in $CHIPYARD_DIR. Please run setup.sh from the root of the Chipyard repository."
    exit 1
fi

# Backup build.sbt before modification
if [ -f "$CHIPYARD_DIR/build.sbt" ]; then
    echo "Backing up build.sbt to build.sbt.bak..."
    cp "$CHIPYARD_DIR/build.sbt" "$CHIPYARD_DIR/build.sbt.bak"
else
    echo "Warning: build.sbt not found in $CHIPYARD_DIR. Skipping backup."
fi

# Patch build.sbt to include F-Vela config
BUILD_SBT_PATCH="generators/_f_vela/script/chipyard_vela.patch" # 절대경로사용 X

if [ -f "$BUILD_SBT_PATCH" ]; then
    echo "Applying patch to build.sbt..."
    diff -u build.sbt generators/_f_vela/script/f-vela_build.sbt > generators/_f_vela/script/chipyard_vela.patch
    patch -p0 < "$BUILD_SBT_PATCH"
else
    echo "Patch file not found!"
    diff -u build.sbt generators/_f_vela/script/f-vela_build.sbt > generators/_f_vela/script/chipyard_vela.patch
    patch -p0 < "$BUILD_SBT_PATCH"
fi

# chipyard(verilator, firemarshal, firesim) sims folder linking (optional)
LINK_DIR="$CHIPYARD_DIR/generators/_f_vela/_link"
if [ -d "$LINK_DIR" ]; then
    echo "Linking sims/verilator and firesim from $LINK_DIR to $CHIPYARD_DIR..."
    if [ -L "$LINK_DIR/sims/verilator/" ] || [ -L "$LINK_DIR/sims/firesim/" ]; then
        echo "Link already exists at $LINK_DIR/sims/verilator or $LINK_DIR/sims/firesim. Skipping linking."
    elif [ -e "$LINK_DIR/sims/verilator/" ] || [ -e "$LINK_DIR/sims/firesim/" ]; then
        echo "Error: A file or directory already exists at $LINK_DIR/sims/verilator or $LINK_DIR/sims/firesim. Please remove it before linking."
    else       
        echo "Creating symbolic link for sims/verilator and firesim..."
        ln -s "$CHIPYARD_DIR/sims/verilator" "$LINK_DIR/verilator"
        ln -s "$CHIPYARD_DIR/sims/firesim" "$LINK_DIR/firesim"
    fi
else
    echo "Warning: Link directory not found at $LINK_DIR. Skipping linking."
fi  

echo ""
echo "F-Vela setup completed successfully!"
echo "You can now run the verilator build script to build the F-Vela SoC with the specified configuration."
echo "" 

echo "Next steps:"
echo "1. Run the verilator build script. ex) script/verilator_make.sh [-c FVelaSoCConfigTest]"
echo "2. Run the test code build script. ex) script/testsw_build.sh"
echo "3. Run the verilator simulation.   ex) script/run_vsim.sh gemmini ternary_gemm_test.riscv"

