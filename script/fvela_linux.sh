#!/usr/bin/env bash
set -Eeuo pipefail

# FireSim F-Vela + Alveo U280 Buildroot launcher
# Defaults match the paths in the supplied setup guides. Override them with
# command-line options or the environment variables shown below.

# example usage:
# ./fvela_linux.sh run \
#   --binary /home/esca/_f_vela/chipyard/generators/_f_vela/software/test/cmp_conv_cycle/build/linux/rocket_cpu_mpgemm.linux \
#   --target-name rocket_cpu_mpgemm.linux \
#   --attach

# 새 Linux 바이너리 실행 (bitstream 변경 없음, FPGA 재프로그래밍 생략)
# ./fvela_linux.sh run \
#   --binary /home/esca/_f_vela/chipyard/generators/_f_vela/software/test/profiler_test/build/linux/gemm_profiler_test.linux \
#   --target-name gemm_profiler_test.linux \
#   --skip-infrasetup \
#   --attach
# (--skip-infrasetup 이어도 br-base.img의 스테이징 사본은 자동으로 새로 고쳐집니다.)

# chmod +x fvela-linux.sh
# ./fvela-linux.sh run --attach

# # 다른 Linux 바이너리 실행
# ./fvela-linux.sh run --binary /path/to/my_test.linux --attach

# # 새 bitstream 지정
# ./fvela-linux.sh run \
#   --bitstream /path/to/firesim.tar.gz \
#   --attach

# # 이미지에 넣어둔 바이너리 재사용
# ./fvela-linux.sh run --skip-copy --attach

# # 이미 FPGA가 준비된 경우
# ./fvela-linux.sh run --skip-copy --skip-infrasetup --attach

# # 실행 화면 재접속
# ./fvela-linux.sh attach

# # 상태 확인
# ./fvela-linux.sh status

# # 종료
# ./fvela-linux.sh stop


CHIPYARD="${CHIPYARD:-/home/esca/_f_vela/chipyard}"
HW_CONFIG="${HW_CONFIG:-fvela_u280}"
WORKLOAD="${WORKLOAD:-br-base.json}"
TARGET_NAME="${TARGET_NAME:-vfrope_test2_linux.linux}"
MOUNT_DIR=""
TEST_BINARY=""
BITSTREAM_TAR=""
SKIP_COPY=0
SKIP_INFRASETUP=0
AUTO_ATTACH=0

usage() {
  cat <<'EOF'
Usage:
  ./fvela-linux.sh run [options] [linux-binary]
  ./fvela-linux.sh attach
  ./fvela-linux.sh status
  ./fvela-linux.sh stop

run options:
  --binary PATH          Linux RISC-V executable to copy into br-base.img
  --bitstream PATH       Update fvela_u280.bitstream_tar before running
  --target-name NAME     Filename inside Buildroot /media
  --chipyard PATH        Chipyard root (default: /home/esca/_f_vela/chipyard)
  --skip-copy            Reuse the executable already stored in br-base.img
  --skip-infrasetup      Reuse the currently programmed FPGA (still refreshes
                         the staged rootfs copy so a new binary takes effect)
  --attach               Attach to fsim0 after starting
  -h, --help             Show this help

Examples:
  ./fvela-linux.sh run
  ./fvela-linux.sh run --binary ./build/linux/my_test.linux --attach
  ./fvela-linux.sh run --bitstream /path/to/firesim.tar.gz --skip-copy

Inside Buildroot, log in as root and run:
  /media/vfrope_test2_linux.linux; echo $?
EOF
}

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
note() { printf '\n==> %s\n' "$*"; }
need() { command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"; }

set_paths() {
  FIRESIM="$CHIPYARD/sims/firesim"
  DEPLOY="$FIRESIM/deploy"
  IMAGE="$CHIPYARD/software/firemarshal/images/firechip/br-base/br-base.img"
  DEFAULT_BINARY="$CHIPYARD/generators/_f_vela/software/test/rv_rope_test/build/linux/vfrope_test2_linux.linux"
  TEST_BINARY="${TEST_BINARY:-$DEFAULT_BINARY}"
}

load_firesim() {
  [[ -f "$FIRESIM/sourceme-manager.sh" ]] || die "Missing $FIRESIM/sourceme-manager.sh"
  # sourceme-manager.sh relies on the current directory (relative ./env.sh, $(pwd)).
  cd "$FIRESIM"
  set +u
  source "$FIRESIM/sourceme-manager.sh"
  set -u
  command -v firesim >/dev/null 2>&1 || die "firesim was not added to PATH"
  cd "$DEPLOY"
}

cleanup_mount() {
  if [[ -n "${MOUNT_DIR:-}" ]]; then
    if mountpoint -q "$MOUNT_DIR" 2>/dev/null; then
      sudo umount "$MOUNT_DIR" || true
    fi
    rmdir "$MOUNT_DIR" 2>/dev/null || true
  fi
}
trap cleanup_mount EXIT INT TERM

validate_linux_binary() {
  [[ -f "$TEST_BINARY" ]] || die "Linux executable not found: $TEST_BINARY (build it with 'make linux' or use --binary)"
  local desc
  desc="$(file -b "$TEST_BINARY")"
  [[ "$desc" == *RISC-V* ]] || die "Not a RISC-V executable: $desc"
  if readelf -s "$TEST_BINARY" 2>/dev/null | grep -Eq '(^|[[:space:]])(tohost|fromhost)($|[[:space:]])'; then
    die "This looks like a bare-metal/HTIF binary (tohost/fromhost found), not a Linux userspace executable"
  fi
}

copy_into_image() {
  [[ -f "$IMAGE" ]] || die "Buildroot image not found: $IMAGE"
  validate_linux_binary
  need mountpoint
  MOUNT_DIR="$(mktemp -d /tmp/fvela-br-base.XXXXXX)"
  note "Copying $(basename "$TEST_BINARY") to br-base.img as /media/$TARGET_NAME"
  sudo mount -o loop "$IMAGE" "$MOUNT_DIR"
  sudo mkdir -p "$MOUNT_DIR/media"
  sudo install -m 0755 "$TEST_BINARY" "$MOUNT_DIR/media/$TARGET_NAME"
  sync
  sudo umount "$MOUNT_DIR"
  rmdir "$MOUNT_DIR"
  MOUNT_DIR=""
  if findmnt -rn -S "$IMAGE" | grep -q .; then
    die "Image is still mounted; refusing to continue"
  fi
}

sync_staged_rootfs() {
  # `firesim infrasetup` stages a copy of $IMAGE under
  # run_farm.recipe_arg_overrides.default_simulation_dir/sim_slot_N/ (and
  # rsyncdir/), and `runworkload` boots from that staged copy, never from
  # $IMAGE directly. --skip-infrasetup skips that staging step, so a fresh
  # edit to $IMAGE would otherwise silently boot stale, previously-staged
  # data. Refresh every staged copy we can find instead.
  local cfg="$DEPLOY/config_runtime.yaml"
  local runs_dir
  runs_dir="$(CFG_PATH="$cfg" python - <<'PY'
import os
import yaml
with open(os.environ["CFG_PATH"], encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}
print(
    cfg.get("run_farm", {})
    .get("recipe_arg_overrides", {})
    .get("default_simulation_dir", "")
)
PY
)"
  [[ -n "$runs_dir" && -d "$runs_dir" ]] || {
    note "No existing run directory at '${runs_dir:-<unset>}'; nothing to refresh (infrasetup will stage it on the next non-skipped run)"
    return
  }

  local base found=0 staged
  base="$(basename "$IMAGE")"
  while IFS= read -r -d '' staged; do
    found=1
    note "Refreshing staged rootfs copy: $staged"
    cp -f "$IMAGE" "$staged"
  done < <(find "$runs_dir" -type f -name "*-$base" -print0 2>/dev/null)

  if (( found == 0 )); then
    note "Warning: no staged rootfs copy named *-$base found under $runs_dir; run without --skip-infrasetup at least once first"
  fi
}

configure_runtime() {
  local cfg="$DEPLOY/config_runtime.yaml"
  local hwdb="$DEPLOY/config_hwdb.yaml"
  [[ -f "$cfg" ]] || die "Missing $cfg"
  [[ -f "$hwdb" ]] || die "Missing $hwdb"
  [[ -z "$BITSTREAM_TAR" || -f "$BITSTREAM_TAR" ]] || die "Bitstream archive not found: $BITSTREAM_TAR"

  CFG_PATH="$cfg" HWDB_PATH="$hwdb" SELECTED_HW="$HW_CONFIG" SELECTED_WORKLOAD="$WORKLOAD" NEW_BITSTREAM="$BITSTREAM_TAR" python - <<'PY'
import os, shutil, time
import yaml

cfg_path = os.environ["CFG_PATH"]
hwdb_path = os.environ["HWDB_PATH"]
hw_name = os.environ["SELECTED_HW"]
workload = os.environ["SELECTED_WORKLOAD"]
bitstream = os.environ["NEW_BITSTREAM"]

stamp = time.strftime("%Y%m%d-%H%M%S")
for path in (cfg_path, hwdb_path):
    shutil.copy2(path, path + ".bak." + stamp)

with open(cfg_path, encoding="utf-8") as f:
    cfg = yaml.safe_load(f) or {}
cfg.setdefault("target_config", {})["default_hw_config"] = hw_name
cfg["workload"] = {
    "workload_name": workload,
    "terminate_on_completion": False,
    "suffix_tag": None,
}
with open(cfg_path, "w", encoding="utf-8") as f:
    yaml.safe_dump(cfg, f, sort_keys=False)

with open(hwdb_path, encoding="utf-8") as f:
    hwdb = yaml.safe_load(f) or {}
if hw_name not in hwdb:
    raise SystemExit(f"Hardware config '{hw_name}' is missing from {hwdb_path}")
if bitstream:
    hwdb[hw_name]["bitstream_tar"] = "file://" + os.path.abspath(bitstream)
    with open(hwdb_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(hwdb, f, sort_keys=False)

uri = hwdb[hw_name].get("bitstream_tar", "")
if not uri.startswith("file://"):
    raise SystemExit(f"{hw_name}.bitstream_tar must be a local file:// URL (current: {uri!r})")
path = uri[7:]
if not os.path.isfile(path):
    raise SystemExit(f"Configured bitstream archive does not exist: {path}")
print(f"Hardware : {hw_name}")
print(f"Workload : {workload}")
print(f"Bitstream: {path}")
PY
}

status_cmd() {
  set_paths
  printf 'FireSim screens:\n'
  screen -ls 2>/dev/null || true
  printf '\nFireSim processes:\n'
  pgrep -af 'FireSim-xilinx' || true
  printf '\nXDMA device users (sudo may ask for a password):\n'
  sudo fuser -v /dev/xdma0_* 2>/dev/null || true
}

stop_cmd() {
  set_paths
  load_firesim
  note "Stopping the current FireSim workload"
  firesim kill
}

attach_cmd() {
  need screen
  screen -r fsim0
}

run_cmd() {
  set_paths
  need file
  need readelf
  need python
  need findmnt
  load_firesim

  if pgrep -f 'FireSim-xilinx' >/dev/null 2>&1 || screen -ls 2>/dev/null | grep -q '[.]fsim0'; then
    note "Stopping the previous FireSim workload"
    firesim kill
  fi

  (( SKIP_COPY == 1 )) || copy_into_image
  note "Selecting $HW_CONFIG and $WORKLOAD"
  configure_runtime

  if (( SKIP_INFRASETUP == 0 )); then
    note "Programming/preparing the Alveo U280"
    firesim infrasetup
  else
    note "Skipping FPGA reprogram; refreshing the staged rootfs copy instead"
    sync_staged_rootfs
  fi
  note "Booting Buildroot Linux"
  firesim runworkload

  printf '\nStarted. Attach with: %q attach\n' "$0"
  printf 'Buildroot login: root\n'
  printf 'Run the test: /media/%s; echo $?\n' "$TARGET_NAME"
  (( AUTO_ATTACH == 0 )) || attach_cmd
}

COMMAND="${1:-run}"
[[ $# -eq 0 ]] || shift

while [[ $# -gt 0 ]]; do
  case "$1" in
    --binary) [[ $# -ge 2 ]] || die "--binary needs a path"; TEST_BINARY="$2"; shift 2 ;;
    --bitstream) [[ $# -ge 2 ]] || die "--bitstream needs a path"; BITSTREAM_TAR="$2"; shift 2 ;;
    --target-name) [[ $# -ge 2 ]] || die "--target-name needs a name"; TARGET_NAME="$2"; shift 2 ;;
    --chipyard) [[ $# -ge 2 ]] || die "--chipyard needs a path"; CHIPYARD="$2"; shift 2 ;;
    --skip-copy) SKIP_COPY=1; shift ;;
    --skip-infrasetup) SKIP_INFRASETUP=1; shift ;;
    --attach) AUTO_ATTACH=1; shift ;;
    -h|--help) usage; exit 0 ;;
    --*) die "Unknown option: $1" ;;
    *) [[ -z "$TEST_BINARY" ]] || die "Only one binary may be specified"; TEST_BINARY="$1"; shift ;;
  esac
done

case "$COMMAND" in
  run) run_cmd ;;
  attach) set_paths; attach_cmd ;;
  status) status_cmd ;;
  stop) stop_cmd ;;
  help|-h|--help) usage ;;
  *) usage >&2; die "Unknown command: $COMMAND" ;;
esac
