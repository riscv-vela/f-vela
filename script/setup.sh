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

# ----------------------------------------------------------------------------------------------------------------------
# Apply the rocket-chip vector patch (needed for the Saturn vector unit + custom PID/RoPE insns).
#   The patch adds RocketVectorDecoder.io.vector and an EX-stage vector re-decode so a vector op
#   issued right after a vsetvl is recognized (and stalled) before vconfig settles -- otherwise the
#   first vector load/arith after vsetvl traps as an illegal instruction.
#   The patched sources live under generators/_f_vela/rocket-patch/ mirroring the chipyard tree; each
#   target is backed up as <file>.bak (once, preserving the true original) and then overwritten.
#   To revert: restore each <file>.bak over <file> (or `git -C generators/rocket-chip checkout`).
ROCKET_PATCH_DIR="$CHIPYARD_DIR/generators/_f_vela/rocket-patch"
if [ -d "$ROCKET_PATCH_DIR/generators" ]; then
    echo "Applying rocket-chip vector patch from generators/_f_vela/rocket-patch ..."
    while IFS= read -r -d '' pf; do
        rel="${pf#$ROCKET_PATCH_DIR/}"                       # e.g. generators/rocket-chip/src/main/scala/rocket/RocketCore.scala
        target="$CHIPYARD_DIR/$rel"
        if [ ! -f "$target" ]; then
            echo "  Warning: target not found, skipping: $rel"
            continue
        fi
        if [ ! -f "$target.bak" ]; then
            cp "$target" "$target.bak"
            echo "  Backed up original: $rel -> $rel.bak"
        else
            echo "  Backup already exists (keeping original): $rel.bak"
        fi
        cp "$pf" "$target"
        echo "  Patched: $rel"
    done < <(find "$ROCKET_PATCH_DIR/generators" -type f -name '*.scala' -print0)
else
    echo "Warning: rocket-patch directory not found at $ROCKET_PATCH_DIR. Skipping rocket-chip patch."
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

