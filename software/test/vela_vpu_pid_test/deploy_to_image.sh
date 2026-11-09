#!/usr/bin/env bash
# Build the Linux PID tests and install them into the rootfs of the FireSim workload "vela_vpu_pid_test"
# (sims/firesim/deploy/workloads/vela_vpu_pid_test.json).
#
# The files that go into the rootfs are managed in generators/_f_vela/rootfs/vela_vpu_pid_test/overlay/
# (symlinks to build/linux/*.linux -> /media/); this script runs `make linux`, links any new
# build/linux/*.linux into that overlay, and calls rootfs/sync_rootfs.sh.
#
# Only Linux binaries go into the rootfs. build/*.riscv are bare-metal (htif) programs and do not
# run under Linux; they are run as a bare-metal workload instead (no rootfs).
#
# usage: ./deploy_to_image.sh [--workload NAME] [--no-build] [--size 512M] [--image PATH] [--clean] [--dry-run]
#   --workload  install into the rootfs of FireSim workload NAME (default vela_vpu_pid_test)
#   --image     install into this existing image file instead (path, not a workload name)
#   --no-build  do not run `make linux` first
#   --size      grow the image to this size (default: keep the base image size)
#   --clean     recreate the image from br-base first
# then:  ../../../script/fvela_firesim.sh run --workload vela_vpu_pid_test --attach
#        on target: cd /media && ./vfpid_4f_linux.linux
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOTFS="$(cd "$HERE/../../../rootfs" && pwd)"
OVERLAY_MEDIA="$ROOTFS/vela_vpu_pid_test/overlay/media"
BUILD=1
ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build) BUILD=0; shift ;;
    --workload|--image|--size) ARGS+=("$1" "$2"); shift 2 ;;
    --clean|--dry-run|-n) ARGS+=("$1"); shift ;;
    -h|--help) sed -n 2,19p "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if (( BUILD )); then
  echo "[deploy] make linux"
  make -C "$HERE" linux
fi

# link new build outputs into the overlay (relative links, so the tree can move)
mkdir -p "$OVERLAY_MEDIA"
shopt -s nullglob
for b in "$HERE"/build/linux/*.linux; do
  link="$OVERLAY_MEDIA/$(basename "$b")"
  [[ -L "$link" ]] && continue
  echo "[deploy] new overlay link: media/$(basename "$b")"
  ln -s "$(realpath --relative-to="$OVERLAY_MEDIA" "$b")" "$link"
done

exec "$ROOTFS/sync_rootfs.sh" "${ARGS[@]}" vela_vpu_pid_test
