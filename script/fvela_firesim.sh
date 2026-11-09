#!/usr/bin/env bash
set -Eeuo pipefail

# FireSim F-Vela + Alveo U280 launcher: Buildroot Linux or bare-metal programs.
# (formerly fvela_linux.sh; that name is kept as a symlink to this file)
# Defaults match the paths in the supplied setup guides. Override them with
# command-line options or the environment variables shown below.

# --- Linux (Buildroot). Without --workload the workload is br-base (br-base.img) ---
# 새 Linux 바이너리를 br-base.img /media 에 넣고 실행
# ./fvela_firesim.sh run --binary ../software/test/profiler_test/build/linux/gemm_profiler_test.linux --attach
#
# rootfs/sync_rootfs.sh 로 만든 워크로드 이미지로 실행 (파일은 rootfs/NAME/overlay/)
# ./fvela_firesim.sh run --workload vela_vpu_pid_test --attach
#
# --- Bare-metal (HTIF .riscv, no Linux): the program is the boot binary, it ends by itself ---
# ./fvela_firesim.sh run --baremetal --binary ../software/test/vela_vpu_pid_test/build/vfpid_4f.riscv --attach
# ./fvela_firesim.sh run --baremetal ../software/test/vela_vpu_pid_test/build/vfpid_4d.riscv --skip-infrasetup --attach
#
# --- 공통 ---
# --skip-infrasetup: FPGA 재프로그래밍 생략 (bitstream 변경 없을 때). 그래도 스테이징된 사본
#   (rootfs 이미지 / bare-metal 바이너리)은 자동으로 새로 고쳐집니다. 단, 해당 워크로드를
#   infrasetup 과 함께 한 번은 실행한 적이 있어야 합니다.
# 새 bitstream:      ./fvela_firesim.sh run --bitstream /path/to/firesim.tar.gz --attach
# 실행 화면 재접속:  ./fvela_firesim.sh attach
# 상태 확인 / 종료:  ./fvela_firesim.sh status / ./fvela_firesim.sh stop
# 로그:              rootfs/logs/<날짜>/<시각>-<workload>[-<program>]/ (uartlog, run.info), 최신은 rootfs/logs/latest
#                    ./fvela_firesim.sh log  -> 최신 실행의 uartlog 를 실시간으로 보기


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
WORKLOAD_GIVEN=0
BINARY_GIVEN=0
BAREMETAL=0
# bare-metal programs are staged under one fixed name, so --skip-infrasetup can switch programs
BM_WORKLOAD="baremetal"
BM_STAGED="baremetal.riscv"
# per-run logs: <LOG_ROOT>/<YYYY-MM-DD>/<HHMMSS>-<workload>[-<program>]/, <LOG_ROOT>/latest -> newest run
LOG_ROOT="${FVELA_LOG_ROOT:-}"
RUN_LOG_DIR=""

usage() {
  cat <<'EOF'
Usage:
  ./fvela_firesim.sh run [options] [binary]
  ./fvela_firesim.sh attach
  ./fvela_firesim.sh status
  ./fvela_firesim.sh stop
  ./fvela_firesim.sh log                 Follow the uartlog of the latest run (rootfs/logs/latest)

Logs: every run gets generators/_f_vela/rootfs/logs/<YYYY-MM-DD>/<HHMMSS>-<workload>[-<program>]/
  uartlog    copied live while the simulation runs; complete after stop or when the program ends
  run.info   workload, hw config, bitstream, program, start/end time, command line
  *.csv      FireSim heartbeat / memory stats, copied when the simulation ends

run options:
  --binary PATH          Linux mode: RISC-V Linux executable to copy into the workload's rootfs
                         (/media/<target-name>). Bare-metal mode: the .riscv program to boot.
  --baremetal            Boot --binary as a bare-metal (HTIF) program instead of Linux. Creates the
                         workload "baremetal" (deploy/workloads/baremetal.json, program staged as
                         baremetal/baremetal.riscv). The program ends by itself.
  --workload NAME        FireSim workload (deploy/workloads/NAME.json, default br-base).
                         Its common_rootfs is the image used for --binary/staging.
                         Without --binary, nothing is copied (the image is prepared
                         separately from generators/_f_vela/rootfs/NAME/overlay/ by
                         rootfs/sync_rootfs.sh NAME, or a test's deploy_to_image.sh).
                         A workload without rootfs runs as bare-metal.
  --bitstream PATH       Update <hw-config>.bitstream_tar before running
  --hw-config NAME       config_hwdb.yaml entry to run (default fvela_u280, or $HW_CONFIG).
                         nocustom_gemmini_only_u280 = FireSimNoCustomGemminiOnlyTest (upstream Gemmini, no vector unit)
  --target-name NAME     Linux mode: filename inside Buildroot /media
  --chipyard PATH        Chipyard root (default: /home/esca/_f_vela/chipyard)
  --skip-copy            Linux mode: reuse the executable already stored in the image
  --skip-infrasetup      Reuse the currently programmed FPGA (still refreshes the staged
                         rootfs / bare-metal binary so a new binary takes effect)
  --attach               Attach to fsim0 after starting
  -h, --help             Show this help

Examples:
  ./fvela_firesim.sh run --binary ./build/linux/my_test.linux --attach         # Linux, br-base
  ./fvela_firesim.sh run --workload vela_vpu_pid_test --attach                       # Linux, prepared image
  ./fvela_firesim.sh run --hw-config nocustom_gemmini_only_u280 --workload ncnn_conv_test --attach
  ./fvela_firesim.sh run --baremetal --binary ./build/my_test.riscv --attach   # bare-metal
  ./fvela_firesim.sh run --baremetal ./build/other.riscv --skip-infrasetup     # next bare-metal program
  ./fvela_firesim.sh run --bitstream /path/to/firesim.tar.gz --skip-copy

Inside Buildroot, log in as root and run:
  /media/vfrope_test2_linux.linux; echo $?
EOF
}

die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }
note() { printf '\n==> %s\n' "$*"; }
need() { command -v "$1" >/dev/null 2>&1 || die "Required command not found: $1"; }

set_paths() {
  LOG_ROOT="${LOG_ROOT:-$CHIPYARD/generators/_f_vela/rootfs/logs}"
  FIRESIM="$CHIPYARD/sims/firesim"
  DEPLOY="$FIRESIM/deploy"
  IMAGE="$CHIPYARD/software/firemarshal/images/firechip/br-base/br-base.img"
  # A workload JSON other than br-base names its own rootfs. FireSim resolves common_rootfs
  # relative to workloads/<benchmark_name>/.
  # "common_rootfs": null means a bare-metal workload (the boot binary is the whole program).
  local wl_json="$DEPLOY/workloads/$WORKLOAD"
  BOOTBIN=""
  if [[ "$WORKLOAD" != "br-base.json" ]]; then
    [[ -f "$wl_json" ]] || die "Workload not found: $wl_json"
    local paths
    paths="$(WL_JSON="$wl_json" python - <<'PY'
import json, os
p = os.environ["WL_JSON"]
wl = json.load(open(p))
base = os.path.join(os.path.dirname(p), wl.get("benchmark_name", ""))
res = lambda f: os.path.normpath(os.path.join(base, f)) if f else ""
print(res(wl.get("common_rootfs")))
print(res(wl.get("common_bootbinary")))
PY
)"
    IMAGE="$(sed -n 1p <<<"$paths")"
    BOOTBIN="$(sed -n 2p <<<"$paths")"
    if [[ -n "$IMAGE" && ! -f "$IMAGE" ]]; then
      # workloads whose rootfs is built from generators/_f_vela/rootfs/<name>/overlay/
      local wl_name="${WORKLOAD%.json}"
      if [[ -d "$CHIPYARD/generators/_f_vela/rootfs/$wl_name/overlay" ]]; then
        die "Rootfs of workload $WORKLOAD not found: $IMAGE
       Create it first: $CHIPYARD/generators/_f_vela/rootfs/sync_rootfs.sh $wl_name"
      fi
      die "Rootfs of workload $WORKLOAD not found: $IMAGE"
    fi
    [[ -z "$BOOTBIN" || -f "$BOOTBIN" ]] || die "Boot binary of workload $WORKLOAD not found: $BOOTBIN"
  fi
  DEFAULT_BINARY="$CHIPYARD/generators/_f_vela/software/test/vela_vpu_rope_test/build/linux/vfrope_test2_linux.linux"
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
    die "This looks like a bare-metal/HTIF binary (tohost/fromhost found), not a Linux userspace executable (run it with --baremetal)"
  fi
}

validate_baremetal_binary() {
  [[ -f "$TEST_BINARY" ]] || die "Bare-metal program not found: $TEST_BINARY"
  local desc
  desc="$(file -b "$TEST_BINARY")"
  [[ "$desc" == *RISC-V* ]] || die "Not a RISC-V executable: $desc"
  if ! readelf -s "$TEST_BINARY" 2>/dev/null | grep -Eq '(^|[[:space:]])tohost($|[[:space:]])'; then
    die "No tohost symbol: not a bare-metal/HTIF program (a Linux executable runs without --baremetal)"
  fi
}

prepare_baremetal() {
  (( BINARY_GIVEN == 1 )) || die "--baremetal needs the program: --binary PATH (or a PATH argument)"
  (( WORKLOAD_GIVEN == 0 )) || die "--baremetal makes its own workload ($BM_WORKLOAD); do not combine it with --workload"
  need file
  need readelf
  validate_baremetal_binary
  local wl_dir="$CHIPYARD/sims/firesim/deploy/workloads"
  note "Bare-metal program: $TEST_BINARY (staged as workloads/$BM_WORKLOAD/$BM_STAGED)"
  mkdir -p "$wl_dir/$BM_WORKLOAD"
  cp -f "$TEST_BINARY" "$wl_dir/$BM_WORKLOAD/$BM_STAGED"
  readlink -f "$TEST_BINARY" > "$wl_dir/$BM_WORKLOAD/SOURCE"
  cat > "$wl_dir/$BM_WORKLOAD.json" <<EOF
{
	"benchmark_name": "$BM_WORKLOAD",
	"common_bootbinary": "$BM_STAGED",
	"common_rootfs": null,
	"common_simulation_outputs": [
		"uartlog"
	]
}
EOF
  WORKLOAD="$BM_WORKLOAD.json"
  WORKLOAD_GIVEN=1
  BINARY_GIVEN=0   # the program is the boot binary; nothing is copied into a rootfs
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

runs_dir() {
  # run_farm.recipe_arg_overrides.default_simulation_dir of config_runtime.yaml (sim_slot_N/ live there)
  CFG_PATH="$DEPLOY/config_runtime.yaml" python - <<'PY'
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
}

sync_staged_rootfs() {
  # `firesim infrasetup` stages a copy of $IMAGE under
  # run_farm.recipe_arg_overrides.default_simulation_dir/sim_slot_N/ (and
  # rsyncdir/), and `runworkload` boots from that staged copy, never from
  # $IMAGE directly. --skip-infrasetup skips that staging step, so a fresh
  # edit to $IMAGE would otherwise silently boot stale, previously-staged
  # data. Refresh every staged copy we can find instead.
  local runs_dir
  runs_dir="$(runs_dir)"
  [[ -n "$runs_dir" && -d "$runs_dir" ]] || {
    note "No existing run directory at '${runs_dir:-<unset>}'; nothing to refresh (infrasetup will stage it on the next non-skipped run)"
    return
  }

  # rootfs for Linux workloads, the boot binary itself for bare-metal ones
  local src
  for src in "$IMAGE" "${BOOTBIN_IF_BAREMETAL:-}"; do
    [[ -n "$src" ]] || continue
    local base found=0 staged
    base="$(basename "$src")"
    while IFS= read -r -d '' staged; do
      found=1
      note "Refreshing staged copy: $staged"
      cp -f "$src" "$staged"
    done < <(find "$runs_dir" -type f -name "*-$base" -print0 2>/dev/null)

    if (( found == 0 )); then
      # a bare-metal run without its staged binary only fails at start ("could not open ..."), so stop here
      [[ "$src" != "${BOOTBIN_IF_BAREMETAL:-}" ]] || die "No staged copy of $base under $runs_dir; run once without --skip-infrasetup"
      note "Warning: no staged copy named *-$base found under $runs_dir; run without --skip-infrasetup at least once first"
    fi
  done
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

start_run_log() {
  # $1 = output of configure_runtime (Hardware/Workload/Bitstream lines)
  local prog="" name
  if (( BAREMETAL == 1 || BINARY_GIVEN == 1 )); then prog="$(readlink -f "$TEST_BINARY")"; fi
  name="$(date +%H%M%S)-${WORKLOAD%.json}"
  [[ -z "$prog" ]] || name+="-$(basename "$prog" | sed -E 's/\.(riscv|linux|elf)$//')"
  RUN_LOG_DIR="$LOG_ROOT/$(date +%F)/$name"
  mkdir -p "$RUN_LOG_DIR"
  {
    printf 'started:    %s\n' "$(date '+%F %T')"
    printf 'workload:   %s\n' "${WORKLOAD%.json}"
    printf 'mode:       %s\n' "$([[ -z "$IMAGE" ]] && echo bare-metal || echo linux)"
    [[ -z "$prog" ]] || printf 'program:    %s\n' "$prog"
    [[ -z "$IMAGE" ]] || printf 'rootfs:     %s\n' "$IMAGE"
    printf 'infrasetup: %s\n' "$( (( SKIP_INFRASETUP )) && echo skipped || echo yes)"
    printf 'command:    %s\n' "$CMDLINE"
    printf '%s\n' "$1"
  } > "$RUN_LOG_DIR/run.info"
  ln -sfn "$(realpath --relative-to="$LOG_ROOT" "$RUN_LOG_DIR")" "$LOG_ROOT/latest"
}

start_log_follower() {
  # Detached, so it survives Ctrl-C of `firesim runworkload` and the end of this script.
  local slot
  slot="$(runs_dir)/sim_slot_0"
  setsid nohup "$SELF" _follow-log "$slot" "$RUN_LOG_DIR" "$(date +%s)" >/dev/null 2>&1 < /dev/null &
}

follow_log_cmd() {
  # internal: copy <slot>/uartlog into <dir>/uartlog for the lifetime of the next simulator
  local slot="$1" dir="$2" t0="$3" pat="${FVELA_SIM_PGREP:-^SCREEN -S fsim0 }" pid="" i f
  for (( i = 0; i < 1800; i++ )); do
    pid="$(pgrep -f "$pat" | head -1 || true)"
    [[ -n "$pid" ]] && break
    sleep 1
  done
  [[ -n "$pid" ]] || { printf 'ended:      simulator did not start (log follower gave up)\n' >> "$dir/run.info"; return 0; }
  # script(1) recreates uartlog when the simulator starts; do not pick up the previous run's file
  for (( i = 0; i < 30; i++ )); do
    [[ -f "$slot/uartlog" ]] && (( $(stat -c %Y "$slot/uartlog") >= t0 )) && break
    sleep 1
  done
  tail -n +1 -F --pid="$pid" "$slot/uartlog" > "$dir/uartlog" 2>/dev/null || true
  for f in heartbeat.csv memory_stats0.csv latency_histogram.csv; do
    [[ -r "$slot/$f" ]] && cp -f "$slot/$f" "$dir/" 2>/dev/null || true
  done
  printf 'ended:      %s\n' "$(date '+%F %T')" >> "$dir/run.info"
}

log_cmd() {
  set_paths
  [[ -e "$LOG_ROOT/latest" ]] || die "No run log yet under $LOG_ROOT"
  local dir
  dir="$(readlink -f "$LOG_ROOT/latest")"
  printf '==> %s\n' "$dir"
  sed 's/^/    /' "$dir/run.info"
  printf '==> uartlog (Ctrl-C to stop following)\n'
  tail -n +1 -F "$dir/uartlog"
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
  [[ -e "$LOG_ROOT/latest" ]] && printf '\nLog: %s\n' "$(readlink -f "$LOG_ROOT/latest")"
  return 0
}

attach_cmd() {
  need screen
  screen -r fsim0
}

run_cmd() {
  (( BAREMETAL == 0 )) || prepare_baremetal
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

  # a dedicated workload image already holds its test files unless a --binary is given
  if (( WORKLOAD_GIVEN == 1 && BINARY_GIVEN == 0 )); then
    SKIP_COPY=1
  fi
  BOOTBIN_IF_BAREMETAL=""
  if [[ -z "$IMAGE" ]]; then
    (( BINARY_GIVEN == 0 )) || die "--binary needs a workload with a rootfs; $WORKLOAD is bare-metal"
    SKIP_COPY=1
    BOOTBIN_IF_BAREMETAL="$BOOTBIN"
    note "Bare-metal workload, boot binary: $BOOTBIN"
  else
    note "Rootfs image: $IMAGE"
  fi
  (( SKIP_COPY == 1 )) || copy_into_image
  note "Selecting $HW_CONFIG and $WORKLOAD"
  local cfg_out
  cfg_out="$(configure_runtime)"
  printf '%s\n' "$cfg_out"

  if (( SKIP_INFRASETUP == 0 )); then
    note "Programming/preparing the Alveo U280"
    firesim infrasetup
  else
    note "Skipping FPGA reprogram; refreshing the staged rootfs copy instead"
    sync_staged_rootfs
  fi
  start_run_log "$cfg_out"
  start_log_follower
  note "Starting the simulation ($([[ -z "$IMAGE" ]] && echo bare-metal || echo Buildroot Linux)); log: $RUN_LOG_DIR"
  firesim runworkload

  printf '\nStarted. Attach with: %q attach\n' "$0"
  if [[ -z "$IMAGE" ]]; then
    printf 'Bare-metal program: it ends by itself (*** PASSED *** / FAILED in the log)\n'
  else
    printf 'Buildroot login: root\n'
    if (( WORKLOAD_GIVEN == 1 && BINARY_GIVEN == 0 )); then
      printf 'Test files: ls -l /media\n'
    else
      printf 'Run the test: /media/%s; echo $?\n' "$TARGET_NAME"
    fi
  fi
  printf 'Log: %s/uartlog  (follow: %q log)\n' "$RUN_LOG_DIR" "$0"
  (( AUTO_ATTACH == 0 )) || attach_cmd
}

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
CMDLINE="$0 $*"
if [[ "${1:-}" == _follow-log ]]; then shift; follow_log_cmd "$@"; exit 0; fi
COMMAND="${1:-run}"
[[ $# -eq 0 ]] || shift

while [[ $# -gt 0 ]]; do
  case "$1" in
    --binary) [[ $# -ge 2 ]] || die "--binary needs a path"; TEST_BINARY="$2"; BINARY_GIVEN=1; shift 2 ;;
    --workload) [[ $# -ge 2 ]] || die "--workload needs a name"; WORKLOAD="${2%.json}.json"; WORKLOAD_GIVEN=1; shift 2 ;;
    --bitstream) [[ $# -ge 2 ]] || die "--bitstream needs a path"; BITSTREAM_TAR="$2"; shift 2 ;;
    --hw-config) [[ $# -ge 2 ]] || die "--hw-config needs a name"; HW_CONFIG="$2"; shift 2 ;;
    --target-name) [[ $# -ge 2 ]] || die "--target-name needs a name"; TARGET_NAME="$2"; shift 2 ;;
    --chipyard) [[ $# -ge 2 ]] || die "--chipyard needs a path"; CHIPYARD="$2"; shift 2 ;;
    --baremetal) BAREMETAL=1; shift ;;
    --skip-copy) SKIP_COPY=1; shift ;;
    --skip-infrasetup) SKIP_INFRASETUP=1; shift ;;
    --attach) AUTO_ATTACH=1; shift ;;
    -h|--help) usage; exit 0 ;;
    --*) die "Unknown option: $1" ;;
    *) [[ -z "$TEST_BINARY" ]] || die "Only one binary may be specified"; TEST_BINARY="$1"; BINARY_GIVEN=1; shift ;;
  esac
done
# load_firesim cd's into sims/firesim, so a relative --binary would later resolve against the wrong
# directory (readlink -f then fails and set -e ends the run silently right after infrasetup).
[[ -z "$TEST_BINARY" ]] || TEST_BINARY="$(realpath -m -- "$TEST_BINARY")"

case "$COMMAND" in
  run) run_cmd ;;
  attach) set_paths; attach_cmd ;;
  status) status_cmd ;;
  stop) stop_cmd ;;
  log|logs) log_cmd ;;
  help|-h|--help) usage ;;
  *) usage >&2; die "Unknown command: $COMMAND" ;;
esac
