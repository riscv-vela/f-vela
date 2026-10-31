#!/usr/bin/env bash
# Install the NCNN test files into the rootfs of the FireSim workload "ncnn_conv_test"
# (sims/firesim/deploy/workloads/ncnn_conv_test.json).
#
# The files that go into the rootfs are managed in generators/_f_vela/rootfs/ncnn_conv_test/overlay/
# (symlinks into build/linux/ and models/); this script calls rootfs/sync_rootfs.sh:
#
#   /media/cpu_lenet.riscv  /media/cpu_vgg16.riscv  /media/cpu_resnet18.riscv   NPU backend disabled
#   /media/npu_lenet.riscv  /media/npu_vgg16.riscv  /media/npu_resnet18.riscv   NPU backend enabled
#   /media/npu_lenet2.riscv npu_vgg16_2.riscv npu_resnet18_2.riscv
#                         velaNPU, cycles from the Gemmini perf counters (same method as gemmini_*)
#   /media/gemmini_lenet.riscv /media/gemmini_vgg16.riscv /media/gemmini_resnet18.riscv
#                         NPU backend on the upstream Gemmini (boot with --hw-config nocustom_gemmini_only_u280)
#   /media/ncnn_infer_scalar.linux /media/ncnn_infer_rvv.linux /media/rvv_ptr_test.linux  (debug tools)
#   /media/ncnn_models/<lenet|vgg16|resnet18>/
#       <m>.ncnn.param <m>.ncnn.bin   the same model files for CPU and NPU
#       ref_seed1234.bin              PyTorch output for INPUT_SEED 1234
#       <m>.npu_calib                 int8 output scales for the NPU backend (velaNPU and stock Gemmini)
#       input.bin ref.bin             (for ncnn_infer_*)
#
# Build first (make testbins, make models / make calib); a missing file stops the sync.
#
# usage: ./deploy_to_image.sh [--size SIZE] [--models "lenet resnet18 vgg16"] [--image PATH] [--clean] [--dry-run]
#   --size     grow the image to SIZE (default: SIZE= in rootfs/ncnn_conv_test/config, 1G)
#   --models   only these models' folders (the others are left as they are in the image)
#   --clean    recreate the image from br-base first
# then:  ../../../script/fvela_firesim.sh run --workload ncnn_conv_test --attach                 (velaNPU SoC)
#        ../../../script/fvela_firesim.sh run --hw-config nocustom_gemmini_only_u280 --workload ncnn_conv_test --attach
#                                                                                             (stock Gemmini SoC)

set -Eeuo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOTFS="$(cd "$HERE/../../../rootfs" && pwd)"
SIZE=""  # NCNN 은 models 도 넣어야 해서 1G 필요 -> 기본값은 rootfs/ncnn_conv_test/config (SIZE=1G)
MODELS=""
ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --size) SIZE="$2"; shift 2 ;;
    --models) MODELS="$2"; shift 2 ;;
    --image) ARGS+=("$1" "$2"); shift 2 ;;
    --clean|--dry-run|-n) ARGS+=("$1"); shift ;;
    -h|--help) sed -n 2,29p "$0"; exit 0 ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

if [[ -n "$MODELS" ]]; then
  for d in "$ROOTFS"/ncnn_conv_test/overlay/media/ncnn_models/*/; do
    m="$(basename "$d")"
    [[ " $MODELS " == *" $m "* ]] || ARGS+=(--exclude "/media/ncnn_models/$m/")
  done
fi

[[ -z "$SIZE" ]] || ARGS+=(--size "$SIZE")
"$ROOTFS/sync_rootfs.sh" "${ARGS[@]}" ncnn_conv_test
echo "[deploy] on target: cd /media && ./cpu_lenet.riscv && ./npu_lenet.riscv   (same for vgg16, resnet18)"
echo "[deploy] stock Gemmini: fvela_firesim.sh run --hw-config nocustom_gemmini_only_u280 --workload ncnn_conv_test --attach"
