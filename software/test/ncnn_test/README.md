# ncnn model export (.param /.bin)

```
pip install pnnx
```

onnx to ncnn
```
pnnx model.onnx inputshape=[1,3,224,224]
```
pytorch weight to ncnn
```
import timm, torch, pnnx
model = timm.create_model("hf_hub:timm/resnet18.a1_in1k", pretrained=True).eval()
x = torch.rand(1, 3, 224, 224)
pnnx.export(model, "resnet18.pt", (x,))   # resnet18.ncnn.param/.bin 생성
```


# Test procedure: NCNN convolution on CPU vs velaNPU

Executables (`make testbins`, source `src/ncnn_conv_test.cpp`, linked with the scalar ncnn):

| NPU backend disabled | NPU backend enabled |
|---|---|
| `cpu_lenet.riscv` `cpu_vgg16.riscv` `cpu_resnet18.riscv` | `npu_lenet.riscv` `npu_vgg16.riscv` `npu_resnet18.riscv` |

CPU and NPU executables load the same `<m>.ncnn.param` / `<m>.ncnn.bin` and build the input from
the same `INPUT_SEED` (default 1234, `--seed N`; generator shared with `models/export_models.py`).
Each prints `TEST_MODEL`, `BACKEND`, `PARAM_FILE`, `BIN_FILE`, `INPUT_TYPE`, `INPUT_SHAPE`,
`INPUT_SEED`, `CLOCK_FREQ_HZ`, then per convolution layer `LAYER[i].NAME/OPS/MEDIAN_CYCLE/TIME_US/
UTILIZATION/GOPS`, totals, the comparison with PyTorch, and `STATUS: PASS|FAIL`, `ERROR: n`.

- CPU executables compute the convolutions in **int8** by default, with exactly the NPU's int8 scheme
  (same quantization, int32 accumulate, bias, output scale, ReLU, clip) as scalar integer code on the
  Rocket core. CPU and NPU therefore do the same work and give the same output: compare
  `OUTPUT.CHECKSUM` of `cpu_<m>.riscv` and `npu_<m>.riscv` from the same platform (the non-conv layers
  run in fp32 on the CPU, and x86 vs RISC-V FMA rounding makes checksums differ across platforms).
  `--fp32` runs ncnn's own fp32 Convolution instead.
- OPS = 2 x MAC; MEDIAN_CYCLE = median over `-r` runs (default 5 LeNet, 3 VGG16/ResNet18; `-w 1` warmup)
- CPU cycles: `rdcycle` before/after the int8 conv kernel (fp32: around ncnn's Convolution).
  NPU cycles: hardware profiler, conv start -> completion. Quantize/dequantize are reported separately
  (`CPU_QUANT_CYCLE`, `CPU_DEQUANT_CYCLE`) for both.
- TIME = cycles / CLOCK_FREQ_HZ (1 GHz = target clock of FireSimFVelaSoCConfig, `--freq-hz`;
  the U280 emulates the SoC at 60 MHz, which is not the SoC's clock)
- UTILIZATION = OPS / (cycles x peak). Peak OPS/cycle:
  NPU 512 (16x16 int8 array); CPU int8 1 (one int8 MAC per 2 cycles: the Rocket integer multiplier is
  iterative, mulUnroll 8, minimum 2 cycles with early-out, 8 cycles for a negative multiplier);
  CPU fp32 2 (one FMA per cycle, pipelined FPU)
- PASS: top-1 equals PyTorch and cos >= 0.98 (int8, CPU and NPU) / 0.999 (CPU `--fp32`)

Run:

```bash
make testbins && ./deploy_to_image.sh            # builds rootfs of the FireSim workload ncnn_conv_test
../../../script/fvela_firesim.sh run --workload ncnn_conv_test --attach
# buildroot login: root
ls -l /media
cd /media && ./cpu_lenet.riscv && ./npu_lenet.riscv   # same for vgg16 / resnet18
```

UART log: written live to `<default_simulation_dir>/sim_slot_0/uartlog`
(`/home/esca/FIRESIM_RUNS_DIR/sim_slot_0/uartlog`) and overwritten by the next run, so copy it out
after each test. FireSim also copies it to `sims/firesim/deploy/results-workload/<date>-<workload>/`
only when the simulation ends by itself (`poweroff` in Linux) while `firesim runworkload` is still
waiting; after `firesim kill` / `fvela_firesim.sh stop` it is not copied.

## Same test on the stock (unmodified) Gemmini

`gemmini_lenet.riscv` `gemmini_vgg16.riscv` `gemmini_resnet18.riscv` (also built by `make testbins`,
or alone with `make gemmini-testbins`) run the same test with the NPU backend on upstream Gemmini.
Only the driver layer differs: `src/gemmini_stock_hal.c` (instead of `src/vela_npu_hal.c`) uses
`generators/gemmini/software/gemmini-rocc-tests/include/gemmini.h`, with no F-Vela additions.
The ncnn layer, int8 scheme, `<m>.npu_calib`, model files and seeded input are the same as `npu_*`.

| | `npu_*` (velaNPU) | `gemmini_*` (stock Gemmini) |
|---|---|---|
| FireSim target | `FireSimFVelaSoCConfig` (hwdb `fvela_u280`) | `FireSimNoCustomGemminiOnlyTest` (recipe/hwdb `nocustom_gemmini_only_u280`): `gemmini.DefaultGemminiConfig` on the same Rocket HugeCore / 128-bit sbus / 8 GiB, **no vector unit** (these tests do not use one) |
| conv call | `tiled_conv_auto(..., WS)`; same `gemmini_params.h` (DIM 16, 256 KiB sp, 64 KiB acc) -> same tiling, same command stream | same |
| `MEDIAN_CYCLE` | profiler: first event start -> last event end | Gemmini perf counters (= `NPU_COUNTER_BUSY_CYCLE`) |
| `NPU_COUNTER_BUSY_CYCLE` | Gemmini perf counters: cycles with the load, execute or store controller busy (sum of `MAIN_*_CYCLES`, `src/gemmini_perf_counters.h`) | same code, same counters |
| `NPU_CALL_CYCLE` | rdcycle around call + fence | same |
| `NPU_{LD,EX,ST}_BUSY_CYCLE`, `NPU_TLB_MISS_CYCLE` | perf counters | same |

**Compare velaNPU and stock Gemmini on `NPU_COUNTER_BUSY_CYCLE`** (and `NPU_CALL_CYCLE`): both are
measured by identical code on both chips. `MEDIAN_CYCLE` is not like-for-like: the velaNPU profiler
span also counts idle gaps inside a convolution, the counters do not. On velaNPU the profiler is
still on while the counters run (its DMA writes of 8 B per event share the memory system).

Hardware differences that are part of velaNPU (velaNPU `baselineInferenceConfig` vs stock
`defaultConfig`): `acc_banks` 4 vs 2, `sp_singleported` false vs true, `has_training_convs` false vs
true, 2-bit weight support; plus the velaNPU RTL changes themselves.

Why no vector unit on the stock side: the first stock bitstream (`FireSimNoCustomFvelaTest`, upstream
Gemmini + upstream Saturn, hwdb `nocustom_fvela_u280`, Aug 24) has the upstream Rocket core without the
F-Vela rocket-chip patch (vector-unit integration fixes, e.g. the trap PC selection `csr.io.pc`). Under
Linux it panics in the kernel (`mlockall` -> illegal instruction: the PC lands in the middle of a 4-byte
instruction) before any Gemmini command. With the patch, upstream Saturn no longer elaborates
(`io.vector` not driven). Without a vector unit none of that glue exists, so the stock target builds
from the current tree: Gemmini unmodified, Rocket core = the same F-Vela-patched core as the velaNPU SoC.
Build it once:

```bash
# sims/firesim/deploy/config_build.yaml: builds_to_run: [nocustom_gemmini_only_u280]
firesim buildbitstream
# then add the hwdb entry FireSim prints (name nocustom_gemmini_only_u280) to config_hwdb.yaml
```

```bash
make testbins && ./deploy_to_image.sh     # same image, now also contains gemmini_*.riscv
../../../script/fvela_firesim.sh run --hw-config nocustom_gemmini_only_u280 --workload ncnn_conv_test --attach
# buildroot login: root
cd /media && ./gemmini_lenet.riscv && ./gemmini_resnet18.riscv && ./gemmini_vgg16.riscv
./cpu_lenet.riscv                          # cpu_* also run on this bitstream (no RoCC use)
```

Do not run `npu_*` on the stock bitstream (the profiler command does not exist there), and do not run
`gemmini_*` on the velaNPU bitstream. The launcher switches `config_runtime.yaml` to the given
hw-config; go back with `--hw-config fvela_u280` (the default).

Functional check on Spike (Gemmini functional model; Spike's cycle numbers mean nothing). `-DVNPU_SPIKE`
skips `mlockall`, which pk does not have:

```bash
RT=$CHIPYARD/generators/gemmini/software/gemmini-rocc-tests; N=$CHIPYARD/software/ncnn/build-riscv-scalar/install
riscv64-unknown-linux-gnu-gcc -O2 -march=rv64gc -DVNPU_SPIKE -I$RT -c src/gemmini_stock_hal.c -o /tmp/hal.o
riscv64-unknown-linux-gnu-g++ -O2 -std=c++11 -march=rv64gc -DVELA_NPU=1 -DTEST_MODEL_ID=0 -DTEST_BACKEND_NPU=1 \
  -DTEST_NPU_STOCK_GEMMINI=1 -Isrc -I$N/include/ncnn src/ncnn_conv_test.cpp src/vela_npu_convolution.cpp /tmp/hal.o \
  -o /tmp/gemmini_lenet_spike.riscv $N/lib/libncnn.a -static -lpthread
strace -f -o /dev/null spike -m4096 --extension=gemmini $RISCV/riscv64-unknown-elf/bin/pk /tmp/gemmini_lenet_spike.riscv -d models -r 1 -w 0
```

# ncnn inference test (LeNet-5 / VGG16 / ResNet18)

Runs ncnn on the F-Vela SoC (`FireSimFVelaSoCConfig`: Rocket HugeCore + Saturn VLEN=512 + Gemmini)
under FireSim Buildroot Linux, and checks the output against PyTorch.

| model    | input       | weights                          | ncnn.bin (fp16 stored) |
|----------|-------------|----------------------------------|------------------------|
| lenet    | 1x1x32x32   | random, seed 0                   | 0.1 MB                 |
| resnet18 | 1x3x224x224 | torchvision ImageNet (official)  | 23 MB                  |
| vgg16    | 1x3x224x224 | torchvision ImageNet (official)  | 277 MB                 |

Each model directory has a fixed-seed `input.bin` and the PyTorch output `ref.bin`.
The runner prints latency, top-5, cosine similarity vs PyTorch, and PASS/FAIL (top-1 match and cos > 0.999).

## Binaries

| file                                | compiler                         | ncnn kernels                         |
|-------------------------------------|----------------------------------|--------------------------------------|
| `build/linux/ncnn_infer_rvv.linux`    | clang 18.1.8 (riscv-gnu-toolchain 2024.09.03 LLVM) | rv64gc + RVV/zfh/zvfh, picked at runtime |
| `build/linux/ncnn_infer_scalar.linux` | chipyard riscv64-unknown-linux-gnu-gcc 13.2 | rv64gc only (baseline)            |

Both are static Linux executables. The chipyard GCC 13.2 cannot build ncnn's RVV code
(no RVV tuple types such as `vfloat32m1x2_t`), so the RVV build uses clang from
`/home/esca/_f_vela/toolchains/riscv64-llvm`.

## Rebuild

```bash
source /home/esca/_f_vela/chipyard/env.sh
NCNN=/home/esca/_f_vela/chipyard/software/ncnn
COMMON="-DCMAKE_BUILD_TYPE=Release -DNCNN_RUNTIME_CPU=ON -DNCNN_OPENMP=OFF -DNCNN_SIMPLEOCV=ON \
  -DNCNN_VULKAN=OFF -DNCNN_BUILD_TOOLS=OFF -DNCNN_BUILD_EXAMPLES=OFF -DNCNN_BUILD_BENCHMARK=ON \
  -DCMAKE_EXE_LINKER_FLAGS=-static"

# ncnn, RVV (clang)
cmake -S $NCNN -B $NCNN/build-riscv-rvv $COMMON -DNCNN_RVV=ON -DNCNN_XTHEADVECTOR=OFF \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/cmake/riscv64-linux-clang.toolchain.cmake \
  -DCMAKE_C_FLAGS="-march=rv64gc -mabi=lp64d -DRUAPU_RISCV_NO_VENDOR_PROBE=1" \
  -DCMAKE_CXX_FLAGS="-march=rv64gc -mabi=lp64d -DRUAPU_RISCV_NO_VENDOR_PROBE=1" \
  -DCMAKE_INSTALL_PREFIX=$NCNN/build-riscv-rvv/install
cmake --build $NCNN/build-riscv-rvv -j && cmake --install $NCNN/build-riscv-rvv

# ncnn, scalar (chipyard gcc)
RISCV_ROOT_PATH=$RISCV cmake -S $NCNN -B $NCNN/build-riscv-scalar $COMMON \
  -DCMAKE_TOOLCHAIN_FILE=$NCNN/toolchains/riscv64-unknown-linux-gnu.toolchain.cmake \
  -DCMAKE_C_FLAGS="-march=rv64gc -DRUAPU_RISCV_NO_VENDOR_PROBE=1" \
  -DCMAKE_CXX_FLAGS="-march=rv64gc -DRUAPU_RISCV_NO_VENDOR_PROBE=1" \
  -DCMAKE_INSTALL_PREFIX=$NCNN/build-riscv-scalar/install
cmake --build $NCNN/build-riscv-scalar -j && cmake --install $NCNN/build-riscv-scalar

make models   # pnnx export (downloads VGG16/ResNet18 weights once)
make          # both RISC-V runners
make host     # optional x86 check: ./build/host/ncnn_infer all -d models
```

**`RUAPU_RISCV_NO_VENDOR_PROBE` is required for this SoC.** At startup ncnn (`ruapu`) executes
probe instructions and treats SIGILL as "not supported". On Rocket with a RoCC accelerator,
every custom-0..3 opcode is decoded as a RoCC command; Gemmini only takes custom-3, so the
T-Head (custom-0) / SpacemiT (custom-1) probes never complete and the core hangs with no output.
The macro (patch in `software/ncnn/src/ruapu.h`) drops those vendor probes; standard ones
(zfh, zvfh, zvl*, ...) are still probed.

## Check on Spike (before FPGA)

ncnn probes ISA extensions by executing instructions and catching SIGILL (`ruapu`), which pk
cannot handle. ncnn skips the probe when it sees a debugger, so run spike under `strace`:

```bash
PK=$RISCV/riscv64-unknown-elf/bin/pk
strace -o /dev/null spike -m4096 --isa=rv64gcv_zvl512b $PK build/linux/ncnn_infer_rvv.linux lenet -d models
```

Under pk, `/proc/self/auxv` is the host's, so the `rvv=` line on Spike is not meaningful
(on an x86 host it happens to report 1). On FireSim Linux it comes from the real kernel.

## Run on FireSim

```bash
./deploy_to_image.sh          # copies rootfs/ncnn_conv_test/overlay/ (binaries + models) into ncnn_conv_test.img (sudo)
../../../script/fvela_firesim.sh run --workload ncnn_conv_test --attach
```

On the target:

```bash
/media/ncnn_infer_rvv.linux all              # lenet, vgg16, resnet18
/media/ncnn_infer_rvv.linux resnet18 -w 1 -n 3
/media/ncnn_infer_scalar.linux lenet         # scalar baseline
/media/ncnn_infer_rvv.linux resnet18 --fp16  # allow fp16 storage/arithmetic if zfh/zvfh detected
```

Options: `-d model_root` (default `/media/ncnn_models`), `-n loops`, `-w warmup`, `-t threads`, `--fp16`.
The first line shows what ncnn detected: `rvv=1 vlenb=64` is expected for VLEN=512.

**Known issue (hardware):** on the current bitstream, RVV code gives wrong results / crashes under
Linux when a strided vector load crosses 4 KiB pages (`rvv_ptr_test.linux`; `contig` mode passes).
Use `ncnn_infer_scalar.linux` until that is fixed.

## velaNPU convolution backend

`--backend` swaps every `Convolution` layer of the Net for `VelaNpuConvolution`
(`src/vela_npu_convolution.cpp`, registered with `net.register_custom_layer("Convolution", ...)`;
ncnn's own Convolution is not modified).

| option | convolution runs on | per-layer cycles |
|---|---|---|
| `--backend cpu` (NPU backend off) | Rocket core, ncnn's stock Convolution | `rdcycle` before/after the convolution |
| `--backend npu` (NPU backend on) | velaNPU, int8 `tiled_conv_auto` (WS) | NPU hardware profiler: first event start -> last event end |

Time = cycles / `--freq-mhz` (default 1000, the FireSim target clock; the U280 emulates it at 60 MHz).
For NPU layers the table also shows `call` (rdcycle around `tiled_conv_auto` + fence, i.e. including
driver/tiling on the core) and the CPU-side int8 quantize (`pre`) and dequantize (`post`) cycles.
Layers the NPU path cannot take (non-square kernel/stride, dilation, asymmetric padding,
activation other than none/ReLU) run on the CPU and are marked in the table.

int8 scheme: per-tensor symmetric weights, per-tensor input scale from each input's |max|,
int32 bias, output scale from a calibration table `<model>.npu_calib` (per-layer output |max|
measured on `calib_input.bin`, a different seed than the test input). Made on the host:

```bash
make calib     # writes models/<m>/calib_input.bin and models/<m>/<m>.npu_calib, deploy copies them
```

The host build runs `--backend npu` through a software model of the same int8 math (no cycles),
which gives cos vs PyTorch 0.99997 (LeNet), 0.993 (ResNet18), 0.987 (VGG16), top-1 matching.
PASS for `--backend npu` is therefore top-1 match and cos > 0.98.

On the target:

```bash
/media/ncnn_infer_scalar.linux all --backend cpu      # NPU off
/media/ncnn_infer_scalar.linux all --backend npu      # NPU on
/media/ncnn_infer_scalar.linux vgg16 --backend npu --npu-prof-mb 256   # bigger profile buffer
```
