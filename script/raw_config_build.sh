#!/usr/bin/env bash
########################################################################################################################
# raw_config_build.sh
#
# Build stock/upstream Chipyard configurations while temporarily disabling the
# F-Vela Rocket vector patch and the _f_vela SBT dependency.
#
# Added modes:
#   --firesim              Build the FireSim recipe selected in deploy/config_build.yaml.
#   --firesim-replace-rtl  Run only the U280 replace-rtl stage for quick validation.
#   --restore              Recover the exact pre-run files after SIGKILL, power loss, etc.
#
# Safety model:
#   - Before setup.sh or any source edit, this script snapshots build.sbt, all
#     four Rocket files and the F-Vela-only FireSim wrapper source into
#     $CHIPYARD_DIR/.raw_config_build_state.
#   - EXIT, Ctrl-C, TERM and HUP restore those exact snapshots.
#   - The state directory doubles as a lock, preventing concurrent raw builds.
#   - SIGKILL/power loss cannot run a shell trap; in that case the state directory
#     remains and `./raw_config_build.sh --restore` performs manual recovery.
#
# Usage:
#   ./raw_config_build.sh [-c CONFIG]... [Verilator options]
#   ./raw_config_build.sh --firesim -c NoCustomFvelaTest
#   ./raw_config_build.sh --firesim-replace-rtl -c NoCustomFvelaTest
#   ./raw_config_build.sh --restore
########################################################################################################################

set -o pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../../" && pwd)"

DEFAULT_CONFIGS=("NoCustomFvelaTest")
CONFIGS=()
SKIP_SETUP=false
KEEP_ROCKET_PATCH=false
BUILD_FIRESIM=false
FIRESIM_REPLACE_RTL_ONLY=false
RESTORE_ONLY=false
EXTRA_ARGS=()

ROCKET_FILES=(
    "generators/rocket-chip/src/main/scala/rocket/Configs.scala"
    "generators/rocket-chip/src/main/scala/rocket/Instructions.scala"
    "generators/rocket-chip/src/main/scala/rocket/RocketCore.scala"
    "generators/rocket-chip/src/main/scala/rocket/VectorUnit.scala"
)

# These FireSim wrappers refer to chipyard.FVela* Config classes supplied by
# the _f_vela SBT project. They must not be compiled while _f_vela is disabled.
F_VELA_ONLY_FIRECHIP_FILES=(
    "generators/firechip/chip/src/main/scala/FVelaTargetConfigs.scala"
)

TEMPORARILY_MODIFIED_FILES=(
    "${ROCKET_FILES[@]}"
    "${F_VELA_ONLY_FIRECHIP_FILES[@]}"
)

F_VELA_LINE_PATTERN='"_f_vela" -> f_vela,'
STATE_DIR="$CHIPYARD_DIR/.raw_config_build_state"
STATE_ACTIVE=false

invalidate_classpath_cache() {
    # Generated Scala/SBT artifacts must not cross the F-Vela/raw boundary.
    # Removing only firechip.jar is insufficient: Zinc may reuse RocketVectorDecoder
    # bytecode compiled from the patched VectorUnit.scala, even after the source is
    # temporarily restored to upstream. That stale class is recognizable because it
    # still contains io.vector, which does not exist in the upstream decoder IO.
    echo ">> Invalidating Scala/SBT caches for the F-Vela/raw source boundary ..."

    rm -rf -- "$CHIPYARD_DIR/.classpath_cache"

    rm -rf -- \
        "$CHIPYARD_DIR/generators/rocket-chip/target" \
        "$CHIPYARD_DIR/generators/saturn/target" \
        "$CHIPYARD_DIR/generators/gemmini/target" \
        "$CHIPYARD_DIR/generators/chipyard/target" \
        "$CHIPYARD_DIR/generators/firechip/chip/target"
}

save_exact_state() {
    if ! mkdir "$STATE_DIR" 2>/dev/null; then
        echo "Error: recovery/lock directory already exists: $STATE_DIR"
        echo "Another raw build may be running, or a previous run was interrupted."
        echo "Check for a running process first; if none exists, run:"
        echo "  $0 --restore"
        return 1
    fi

    mkdir -p "$STATE_DIR/files"

    if ! cp -a -- "$CHIPYARD_DIR/build.sbt" "$STATE_DIR/files/build.sbt"; then
        rm -rf -- "$STATE_DIR"
        return 1
    fi

    local rel
    for rel in "${TEMPORARILY_MODIFIED_FILES[@]}"; do
        if [ ! -f "$CHIPYARD_DIR/$rel" ]; then
            echo "Error: source file not found: $CHIPYARD_DIR/$rel"
            rm -rf -- "$STATE_DIR"
            return 1
        fi
        mkdir -p "$STATE_DIR/files/$(dirname "$rel")"
        if ! cp -a -- "$CHIPYARD_DIR/$rel" "$STATE_DIR/files/$rel"; then
            rm -rf -- "$STATE_DIR"
            return 1
        fi
    done

    {
        echo "pid=$$"
        echo "started=$(date -Is)"
        echo "chipyard=$CHIPYARD_DIR"
    } > "$STATE_DIR/metadata"

    STATE_ACTIVE=true
    echo ">> Saved exact pre-run state in: $STATE_DIR"
}

restore_exact_state() {
    if [ ! -d "$STATE_DIR/files" ]; then
        if $STATE_ACTIVE; then
            echo "Warning: recovery state is missing: $STATE_DIR/files"
            return 1
        fi
        return 0
    fi

    echo ""
    echo ">> Restoring exact pre-run build.sbt and temporarily modified sources ..."

    local failed=false
    local rel

    if ! cp -a -- "$STATE_DIR/files/build.sbt" "$CHIPYARD_DIR/build.sbt"; then
        echo "Error: failed to restore build.sbt"
        failed=true
    fi

    for rel in "${TEMPORARILY_MODIFIED_FILES[@]}"; do
        if ! cp -a -- "$STATE_DIR/files/$rel" "$CHIPYARD_DIR/$rel"; then
            echo "Error: failed to restore $rel"
            failed=true
        else
            echo "   Restored: $rel"
        fi
    done

    if $failed; then
        echo "Error: restoration was incomplete. Recovery files were preserved in:"
        echo "  $STATE_DIR"
        return 1
    fi

    invalidate_classpath_cache
    rm -rf -- "$STATE_DIR"
    STATE_ACTIVE=false
    echo "   Restored: build.sbt"
    echo ">> Exact pre-run state restored successfully."
}

on_exit() {
    local status=$?
    trap - EXIT INT TERM HUP

    if $STATE_ACTIVE || [ -d "$STATE_DIR/files" ]; then
        if ! restore_exact_state; then
            echo "Manual recovery required: $0 --restore"
            exit 2
        fi
    fi

    exit "$status"
}

revert_rocket_patch() {
    echo ">> Temporarily reverting the rocket-chip vector patch to pristine .bak files ..."

    local rel
    for rel in "${ROCKET_FILES[@]}"; do
        local target="$CHIPYARD_DIR/$rel"
        if [ ! -f "$target.bak" ]; then
            echo "Error: required pristine backup not found: $target.bak"
            return 1
        fi
    done

    for rel in "${ROCKET_FILES[@]}"; do
        local target="$CHIPYARD_DIR/$rel"
        cp -- "$target.bak" "$target" || return 1
        echo "   Reverted: $rel"
    done
}

disable_f_vela_module() {
    local build_sbt="$CHIPYARD_DIR/build.sbt"

    if grep -qF "$F_VELA_LINE_PATTERN" "$build_sbt" && \
       ! grep -qF "// $F_VELA_LINE_PATTERN" "$build_sbt"; then
        echo ">> Temporarily un-wiring the _f_vela SBT project ..."
        sed -i "s|\(^[[:space:]]*\)\"_f_vela\" -> f_vela,|\1// \"_f_vela\" -> f_vela,|" "$build_sbt" || return 1
    elif grep -qF "// $F_VELA_LINE_PATTERN" "$build_sbt"; then
        echo ">> _f_vela SBT project is already disabled."
    else
        echo "Error: could not locate the _f_vela optionalModules entry in build.sbt"
        return 1
    fi
}

disable_f_vela_firechip_sources() {
    local rel
    for rel in "${F_VELA_ONLY_FIRECHIP_FILES[@]}"; do
        local target="$CHIPYARD_DIR/$rel"

        if [ ! -f "$target" ]; then
            echo "Error: expected F-Vela FireSim source not found: $target"
            return 1
        fi

        if grep -qE 'class[[:space:]]+FireSimNoCustomFvelaTest([[:space:]]|extends)' "$target"; then
            echo "Error: FireSimNoCustomFvelaTest is defined in the F-Vela-only source:"
            echo "  $target"
            echo "Move FireSimNoCustomFvelaTest to RawTargetConfigs.scala before running a raw build."
            return 1
        fi

        echo ">> Temporarily excluding F-Vela-only FireSim wrappers: $rel"
        rm -f -- "$target" || return 1
    done
}

revert_for_raw_build() {
    revert_rocket_patch || return 1
    disable_f_vela_module || return 1
    disable_f_vela_firechip_sources || return 1
    invalidate_classpath_cache
}

clean_nocustom_generated_rtl() {
    local staging="$CHIPYARD_DIR/sims/firesim-staging/generated-src/firechip.chip.FireSim.FireSimNoCustomFvelaTest"
    local generated="$CHIPYARD_DIR/sims/firesim/sim/generated-src/xilinx_alveo_u280/xilinx_alveo_u280-firesim-FireSim-FireSimNoCustomFvelaTest-BaseXilinxAlveoU280Config"

    # Only generated directories for this exact target are removed. Completed
    # results-build archives and the existing F-Vela target are not touched.
    rm -rf -- "$staging" "$generated"
}

validate_firesim_recipe() {
    local deploy_dir="$CHIPYARD_DIR/sims/firesim/deploy"

    (
        cd "$deploy_dir" || exit 1
        python - <<'PY'
import yaml

with open("config_build.yaml", encoding="utf-8") as f:
    build = yaml.safe_load(f)
with open("config_build_recipes.yaml", encoding="utf-8") as f:
    recipes = yaml.safe_load(f)

selected = build.get("builds_to_run", [])
if selected != ["nocustom_fvela_u280"]:
    raise SystemExit(
        "config_build.yaml must select only: builds_to_run: [nocustom_fvela_u280]; "
        f"current value: {selected!r}"
    )

recipe = recipes.get("nocustom_fvela_u280")
if recipe is None:
    raise SystemExit("Missing recipe: nocustom_fvela_u280")

expected = {
    "PLATFORM": "xilinx_alveo_u280",
    "TARGET_PROJECT": "firesim",
    "DESIGN": "FireSim",
    "TARGET_CONFIG": "FireSimNoCustomFvelaTest",
    "PLATFORM_CONFIG": "BaseXilinxAlveoU280Config",
}
for key, value in expected.items():
    if recipe.get(key) != value:
        raise SystemExit(
            f"nocustom_fvela_u280.{key} must be {value!r}; "
            f"current value: {recipe.get(key)!r}"
        )

print("FireSim recipe validation: PASS")
PY
    )
}

build_firesim() {
    if [ ${#CONFIGS[@]} -ne 1 ] || [ "${CONFIGS[0]}" != "NoCustomFvelaTest" ]; then
        echo "Error: --firesim modes currently support only: -c NoCustomFvelaTest"
        return 1
    fi

    validate_firesim_recipe || return 1
    clean_nocustom_generated_rtl

    if $FIRESIM_REPLACE_RTL_ONLY; then
        echo ">> Running raw FireSim replace-rtl validation ..."
        (
            cd "$CHIPYARD_DIR/sims/firesim" || exit 1
            source sourceme-manager.sh --skip-ssh-setup || exit 1
            cd sim || exit 1

            make \
                PLATFORM=xilinx_alveo_u280 \
                TARGET_PROJECT=firesim \
                TARGET_PROJECT_MAKEFRAG=../../../generators/firechip/chip/src/main/makefrag/firesim \
                DESIGN=FireSim \
                TARGET_CONFIG=FireSimNoCustomFvelaTest \
                PLATFORM_CONFIG=BaseXilinxAlveoU280Config \
                replace-rtl
        )
    else
        echo ">> Building raw FireSim Alveo U280 bitstream ..."
        (
            cd "$CHIPYARD_DIR/sims/firesim" || exit 1
            source sourceme-manager.sh --skip-ssh-setup || exit 1
            cd deploy || exit 1
            firesim buildbitstream
        )
    fi
}

usage() {
    cat <<'EOF'
Usage:
  ./raw_config_build.sh [-c CONFIG]... [--skip-setup] [--keep-rocket-patch]
                        [-d|--debug] [--fst] [--clean]
  ./raw_config_build.sh --firesim -c NoCustomFvelaTest [--skip-setup]
  ./raw_config_build.sh --firesim-replace-rtl -c NoCustomFvelaTest [--skip-setup]
  ./raw_config_build.sh --restore

Options:
  -c, --config NAME       Chipyard Config class. Default: NoCustomFvelaTest
  --skip-setup            Do not run setup.sh before the build
  --keep-rocket-patch     Keep the F-Vela patch; Verilator-only option
  --firesim               Build nocustom_fvela_u280 using firesim buildbitstream
  --firesim-replace-rtl   Run only the raw U280 replace-rtl stage
  --restore               Restore files from an interrupted raw build
  -d, --debug, -w, --wave Pass --debug to verilator_make.sh
  --fst                   Pass --fst to verilator_make.sh
  --clean                 Pass --clean to verilator_make.sh
  -h, --help              Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--config)
            if [ $# -lt 2 ]; then
                echo "Error: $1 requires a Config name"
                exit 1
            fi
            CONFIGS+=("$2")
            shift 2
            ;;
        --skip-setup)
            SKIP_SETUP=true
            shift
            ;;
        --keep-rocket-patch)
            KEEP_ROCKET_PATCH=true
            shift
            ;;
        --firesim)
            BUILD_FIRESIM=true
            shift
            ;;
        --firesim-replace-rtl)
            BUILD_FIRESIM=true
            FIRESIM_REPLACE_RTL_ONLY=true
            shift
            ;;
        --restore)
            RESTORE_ONLY=true
            shift
            ;;
        -d|--debug|-w|--wave)
            EXTRA_ARGS+=("--debug")
            shift
            ;;
        --fst)
            EXTRA_ARGS+=("--fst")
            shift
            ;;
        --clean)
            EXTRA_ARGS+=("--clean")
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

if $RESTORE_ONLY; then
    if [ ${#CONFIGS[@]} -ne 0 ] || $BUILD_FIRESIM || $SKIP_SETUP || $KEEP_ROCKET_PATCH || [ ${#EXTRA_ARGS[@]} -ne 0 ]; then
        echo "Error: --restore cannot be combined with build options"
        exit 1
    fi
    if [ ! -d "$STATE_DIR/files" ]; then
        echo "No recovery state found: $STATE_DIR"
        exit 0
    fi
    STATE_ACTIVE=true
    restore_exact_state
    exit $?
fi

if [ ${#CONFIGS[@]} -eq 0 ]; then
    CONFIGS=("${DEFAULT_CONFIGS[@]}")
fi

if $BUILD_FIRESIM && $KEEP_ROCKET_PATCH; then
    echo "Error: --firesim cannot be combined with --keep-rocket-patch"
    exit 1
fi

if $BUILD_FIRESIM && [ ${#EXTRA_ARGS[@]} -ne 0 ]; then
    echo "Error: --debug/--fst/--clean are Verilator-only options"
    exit 1
fi

echo ""
echo "=============================================="
if $BUILD_FIRESIM; then
    echo "  Raw / Origin FireSim Build"
else
    echo "  Raw / Origin Verilator Build"
fi
echo "  Chipyard root: $CHIPYARD_DIR"
echo "  Configs:       ${CONFIGS[*]}"
echo "=============================================="
echo ""

if [ -f "$CHIPYARD_DIR/env.sh" ]; then
    source "$CHIPYARD_DIR/env.sh"
else
    echo "Error: env.sh not found in $CHIPYARD_DIR"
    exit 1
fi

if ! $KEEP_ROCKET_PATCH; then
    save_exact_state || exit 1
    trap on_exit EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP
fi

if ! $SKIP_SETUP; then
    echo ">> Running setup.sh ..."
    if ! "$SCRIPT_DIR/setup.sh"; then
        echo "Error: setup.sh failed"
        exit 1
    fi
    echo ""
fi

if ! $KEEP_ROCKET_PATCH; then
    if ! revert_for_raw_build; then
        echo "Error: failed to prepare the temporary raw build state"
        exit 1
    fi
    echo ""
fi

if $BUILD_FIRESIM; then
    if ! build_firesim; then
        echo ""
        echo "@@@ Raw FireSim build failed @@@"
        exit 1
    fi

    echo ""
    if $FIRESIM_REPLACE_RTL_ONLY; then
        echo "@@@ Raw FireSim replace-rtl completed successfully @@@"
    else
        echo "@@@ Raw FireSim bitstream build completed successfully @@@"
    fi
else
    FAILED=()
    for cfg in "${CONFIGS[@]}"; do
        echo ""
        echo "=============================================="
        echo "  Building CONFIG=$cfg"
        echo "=============================================="
        if ! "$SCRIPT_DIR/verilator_make.sh" -c "$cfg" "${EXTRA_ARGS[@]}"; then
            FAILED+=("$cfg")
        fi
    done

    echo ""
    echo "=============================================="
    if [ ${#FAILED[@]} -eq 0 ]; then
        echo "  @@@ All raw configs built successfully: ${CONFIGS[*]} @@@"
    else
        echo "  @@@ Failed configs: ${FAILED[*]} @@@"
        exit 1
    fi
    echo "=============================================="
fi
