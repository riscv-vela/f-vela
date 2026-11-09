#!/usr/bin/env bash
# Build the BitNet llama-cli (Linux, rv64gc, static) and install it with the model into the rootfs of
# the FireSim workload "bitnet_test" (created on first use by rootfs/sync_rootfs.sh).
#
# Overlay: generators/_f_vela/rootfs/bitnet_test/overlay/media/ (relative symlinks, made here)
#   llama-cli.linux  vela_selftest.linux  mp_transpose_test.linux  run_bitnet.sh
#   i2s_bitnet.gguf, i2s_original.gguf (1.2 GB each)
# Image size: rootfs/bitnet_test/config (SIZE=3G).
#
# usage: ./deploy_to_image.sh [--no-build] [--size 3G] [--clean] [--dry-run]
# then:  ../../../script/fvela_firesim.sh run --workload bitnet_test --attach
#        on target: cd /media && ./run_bitnet.sh -s -m npu -c 4 -n 4
set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOTFS="$(cd "$HERE/../../../rootfs" && pwd)"
WL=bitnet_test
OVERLAY_MEDIA="$ROOTFS/$WL/overlay/media"
BUILD=1
ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --no-build) BUILD=0; shift ;;
    --size) ARGS+=("$1" "$2"); shift 2 ;;
    --clean|--dry-run|-n) ARGS+=("$1"); shift ;;
    -h|--help) sed -n 2,12p "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if (( BUILD )); then
  echo "[deploy] make riscv model"
  make -C "$HERE" riscv model
fi

mkdir -p "$OVERLAY_MEDIA"
[[ -f "$ROOTFS/$WL/config" ]] || printf '# Settings for sync_rootfs.sh %s\n# two 1.2G models (i2s_bitnet, i2s_original); the base image (br-base.img.clean) is ~300M.\nSIZE=3G\n' "$WL" > "$ROOTFS/$WL/config"

chmod +x "$HERE/run_bitnet.sh"
for f in build/linux/llama-cli.linux build/linux/vela_selftest.linux build/linux/mp_transpose_test.linux \
         run_bitnet.sh model/i2s_bitnet.gguf model/i2s_original.gguf; do
  [[ -e "$HERE/$f" ]] || { echo "[deploy] missing $f (run without --no-build)" >&2; exit 1; }
  link="$OVERLAY_MEDIA/$(basename "$f")"
  [[ -L "$link" ]] && continue
  echo "[deploy] overlay link: media/$(basename "$f")"
  ln -s "$(realpath --relative-to="$OVERLAY_MEDIA" "$HERE/$f")" "$link"
done

exec "$ROOTFS/sync_rootfs.sh" "${ARGS[@]}" "$WL"
