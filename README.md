# f-vela: RISC-V-based NPU/VPU FPGA Test Platform

**f-vela** runs a Chipyard-based RISC-V SoC (Rocket + **velaVPU** vector unit (Saturn-based) + **velaNPU** (custom Gemmini))
on an FPGA (FireSim, Xilinx Alveo U280) and in Verilator, to test the NPU/VPU hardware and the
custom PID / RoPE vector instructions with real programs, from bare-metal tests up to Linux apps
(ncnn, BitNet).

- [Install](#install)
- [Folder layout](#folder-layout)
- [Test programs](#test-programs)
- [Verilator simulation](#verilator-simulation)
- [FPGA (FireSim + Alveo U280)](#fpga-firesim--alveo-u280)

---

## Install

### Chipyard 1.13.0

Follow the official Chipyard documentation. f-vela's patches are written against **1.13.0**.

```sh
# grant kernel Read Permission (optional)
sudo chmod 0644 /boot/vmlinuz

# Requirement Package Install
sudo apt-get update
sudo apt-get install libguestfs-tools
libguestfs-test-tool # check 'TEST FINISHED OK'

# Chipyard Install
conda activate base
git clone https://github.com/ucb-bar/chipyard.git
cd chipyard
git checkout 1.13.0
./build-setup.sh riscv-tools
```
<details>
<summary>build-setup.sh script fails on conflict issue</summary>

If the build-setup.sh script fails on conflict issues sometimes it helps to run `conda update -n base --all`

> example) ./build-setup.sh riscv-tools -s 1 -s 2
- -s 1 skips initializing Conda environment
- -s 2 skips initializing Chipyard submodules
- -s 3 skips initializing toolchain collateral (Spike, PK, tests, libgloss)
- -s 4 skips initializing ctags
- -s 5 skips pre-compiling Chipyard Scala sources
- -s 6 skips initializing FireSim
- -s 7 skips pre-compiling FireSim sources
- -s 8 skips initializing FireMarshal
- -s 9 skips pre-compiling FireMarshal default buildroot Linux sources
- -s 10 skips installing CIRCT
- -s 11 skips running repository clean-up
</details>

### f-vela

```sh
cd generators
git clone --branch woojin/integration https://github.com/riscv-vela/f-vela.git _f_vela
cd _f_vela
bash ./script/setup.sh
```

`script/setup.sh` does:
1. Backs up `build.sbt` to `build.sbt.bak` and patches it with `script/f-vela_build.sbt`
   (adds the F-Vela project).
2. **rocket-chip patch** (only on Chipyard 1.13.x): copies `patch/rocket-patch/generators/...` over
   the same paths in chipyard. Each target is backed up once as `<file>.bak`.
3. **firechip / chipyard patch** (only on Chipyard 1.13.x): `patch/<name>-patch/...` is copied into
   `generators/<name>/...`. New files are added, existing ones are backed up as `.bak` and overwritten,
   files already up to date are skipped.
   - `firechip-patch`: `FVelaTargetConfigs.scala` (`FireSimFVelaSoCConfig`, ...), `RawTargetConfigs.scala`
     (`FireSimNoCustomFvelaTest`, `FireSimNoCustomGemminiOnlyTest`, ...), `TargetConfigs.scala` (overwrite)
   - `chipyard-patch`: `config/NoCustomFvelaTest.scala` (stock `NoCustomFvelaTest`, `NoCustomGemminiOnlyTest`,
     `OnlyRocketCoreTest`: upstream Gemmini/Saturn, used by `raw_config_build.sh` and the stock bitstreams)
4. Symlinks `sims/verilator` and `sims/firesim` into `_link/`.

<details>
<summary>rocket-chip patch (patch/rocket-patch)</summary>

| File | Change |
|---|---|
| `rocket/VectorUnit.scala` | adds `RocketVectorDecoder.io.vector` ("is a vector op", independent of legality/`vconfig`) |
| `rocket/RocketCore.scala` | drives `id_ctrl.vec` from `io.vector` (stalls on the `vconfig` hazard) and re-decodes in EX against the settled `vconfig` (`ex_vec_valid`), so the first vector op after `vsetvl` does not trap |
| `rocket/RocketCore.scala` (MEM) | `vec_kill_all` also covers an interrupt slot, so an interrupt during a vector `block_all` gets the right `epc` (fixed data corruption in ncnn RVV) |
| `rocket/Instructions.scala` | bit patterns of the custom `VFPID64B_VF` / `VFPID32B_VF` instructions |
| `rocket/Configs.scala` | `WithRationalCDCs` takes a `direction` argument |
| `tile/FPU.scala` | FP `.vf` vector ops get the right scalar operand |

Details (Korean): `patch/rocket-patch/README.md`.

```bash
# revert: restore each <file>.bak over <file>, e.g.
cp generators/rocket-chip/src/main/scala/rocket/RocketCore.scala.bak \
   generators/rocket-chip/src/main/scala/rocket/RocketCore.scala
# ...or, since rocket-chip is a submodule:
git -C generators/rocket-chip checkout src/main/scala/
```
</details>

To build the **stock** configs (no F-Vela patch, e.g. `NoCustomFvelaTest`), use
`script/raw_config_build.sh`: it removes the patch for the build and restores it afterwards
(`--restore` recovers after an interrupted run).

---

## Folder layout

```
chipyard/generators/_f_vela/
├── src/main/scala/
│   ├── velaNPU/            custom Gemmini (NPU) + SoC/Gemmini configs
│   └── velaVPU/            velaVPU vector unit (Saturn-based) incl. custom PID / RoPE instructions
├── chipyard/               SoC configs: FVelaSoCConfigTest, FVelaGemminiOnlyConfig, FVelaSaturnOnlyConfig, ...
├── patch/                  rocket-patch/, firechip-patch/, chipyard-patch/ (applied by setup.sh)
├── script/
│   ├── setup.sh            install patches (see above)
│   ├── verilator_make.sh   build the Verilator simulator
│   ├── testsw_build.sh     build test programs
│   ├── run_vsim.sh         run a test on the Verilator simulator
│   ├── raw_config_build.sh build stock configs (Verilator or FireSim) without the F-Vela patch
│   └── fvela_firesim.sh    run on the FPGA (FireSim): Linux or bare-metal
├── software/
│   ├── include/, utils/    shared headers (Gemmini, profiler, ...)
│   └── test/               test programs (see below)
├── rootfs/                 files for the FireSim Linux images + run logs
├── firesim_build_dir/      Vivado workspace of `firesim buildbitstream` (not needed to run)
├── docs/                   
└── _link/                  symlinks to sims/verilator, sims/firesim
```

---

## Test programs

`software/test/`. Each folder builds bare-metal programs into `build/*.riscv` (Verilator, FPGA
bare-metal) and, with `make linux`, Linux programs into `build/linux/` (FPGA Linux).

| Folder | What | FPGA Linux workload |
|---|---|---|
| `vela_npu_test/` | velaNPU (custom Gemmini): `gemm`, `matmul`, `ternary_gemm`, `mpgemm`, `fp16_matmul`, ... | `vela_npu_test` |
| `vela_vpu_pid_test/` | velaVPU PID controller: scalar vs vector, custom `vfpid` instructions | `vela_vpu_pid_test` |
| `vela_vpu_rope_test/` | velaVPU custom RoPE instruction `vfrope` (LUT: `make_lut.py`) | `vela_vpu_rope_test` |
| `profiler_test/` | Gemmini profiler; `run_profiler.sh` makes csv/png plots | – |
<!-- | `integ_test/` | npu (Gemmini) + pid + rope + profiler in one program (single / multi-task) | – | -->
<!-- | `cmp_conv_cycle/` | conv / mpgemm cycles: Rocket CPU vs stock Gemmini vs custom Gemmini (`run_cmp.sh`) | – | -->
<!-- | `ncnn_test/` | ncnn LeNet / VGG16 / ResNet18 on CPU, velaNPU, stock Gemmini | `ncnn_conv_test` | -->
<!-- | `bitnet_test/` | BitNet b1.58 2B (`llama-cli`) on Rocket vs velaNPU | `bitnet_test` | -->
<!-- | `tvela_compiler_test/` | - | – | -->

```bash
source ../../env.sh            # from chipyard/generators/_f_vela
./script/testsw_build.sh       # all: npu pid rope profiler integ cmp
./script/testsw_build.sh npu pid   # 'gemmini' still works as an alias of npu
./script/testsw_build.sh linux # Linux versions of npu, pid, rope, profiler, integ, cmp (= make -C software/test linux)
```
`ncnn_test` and `bitnet_test` have their own build steps; see their `README.md`.

---

## Verilator simulation

```bash
# build the simulator (default config FVelaSoCConfigTest) into sims/verilator/
bash script/verilator_make.sh [-c FVelaSoCConfigTest] [-d|-w for a waveform-capable build] [--fst]

# run a bare-metal test: <test> = npu | pid | rope | profiler | integ | conv   (gemmini = alias of npu)
bash script/run_vsim.sh npu ternary_gemm
bash script/run_vsim.sh pid vfpid_4d
bash script/run_vsim.sh rope vfrope_test
bash script/run_vsim.sh profiler ternary_profiler_test
bash script/run_vsim.sh npu --list               # list built programs
bash script/run_vsim.sh npu gemm -w              # + waveform (needs verilator_make.sh -d)
bash script/run_vsim.sh npu gemm -a              # + disassembled commit trace
#   -s CONFIG  simulator config, -c N  max cycles, -g  open in gtkwave

# profiler plot (Verilator run + csv/png into software/test/profiler_test/build/)
bash software/test/profiler_test/run_profiler.sh
```

---

## FPGA (FireSim + Alveo U280)

### FireSim initial setup (once)

1. **Host / board check**: Vivado 2021.1, U280 on PCIe with the `xdma` driver, JTAG visible.
   ```bash
   source /tools/Xilinx/Vivado/2021.1/settings64.sh
   lspci -nn | grep -i xilinx        # 02:00.0 ... [10ee:903f], "Kernel driver in use: xdma"
   ls /dev/xdma0_*                   # control, user, h2c_0, c2h_0, ...
   # Vivado Hardware Manager -> Open Target -> Auto Connect: xcu280_u55c_0
   ```
2. **Passwordless SSH to localhost** (the FireSim manager uses it as build and run host):
   `ssh -o BatchMode=yes localhost hostname` must work without a password.
3. **Manager init**:
   ```bash
   cd chipyard/sims/firesim && source sourceme-manager.sh    # every new shell (fvela_firesim.sh does it itself)
   cd deploy && firesim managerinit --platform xilinx_alveo_u280
   ```
4. **Config files** in `sims/firesim/deploy/`:

   | File | Set |
   |---|---|
   | `config_build.yaml` | `default_build_dir: .../_f_vela/firesim_build_dir`, `build_farm_hosts: [localhost]`, `builds_to_run: [fvela_u280]` |
   | `config_build_recipes.yaml` | `fvela_u280`: `TARGET_CONFIG: FireSimFVelaSoCConfig`, `PLATFORM_CONFIG: BaseXilinxAlveoU280Config`, `fpga_frequency: 60`, `build_strategy: TIMING` |
   | `config_runtime.yaml` | run farm `externally_provisioned`, `default_platform: XilinxAlveoU280InstanceDeployManager`, `default_simulation_dir: /home/esca/FIRESIM_RUNS_DIR`, `default_fpga_db: /opt/firesim-db.json`, `localhost: one_fpgas_spec` |
   | `config_hwdb.yaml` | one entry per bitstream (see below); the `invalid/address` placeholder makes every command fail |
   | `/opt/firesim-db.json` | `{"uid": "Xilinx/21760204S004A", "device": "xcu280_u55c_0", "bdf": "02:00.0"}` |

   `fvela_firesim.sh` sets `default_hw_config` and `workload` in `config_runtime.yaml` on each run
   (it keeps a `.bak.<time>` copy), so those two need no manual edits.
5. **Local fixes** (details in the guide): rsync options in `deploy/runtools/runtime_config.py`
   (rsync 3.3.0 vs 3.1.3 `-z` error), a dummy for the AWS metadata lookup in `deploy/awstools`, and
   `/usr/local/bin/firesim-remove-xdma-module` (`rmmod xdma`; fails while something holds `/dev/xdma0_*`).
6. **Buildroot base image** (FireMarshal), then a clean copy for `rootfs/`:
   ```bash
   cd chipyard/software/firemarshal
   ./marshal -v build br-base.json && ./marshal -v install br-base.json
   # base image of every rootfs/ workload: br-base with an empty /media
   cp --sparse=always images/firechip/br-base/br-base.img images/firechip/br-base/br-base.img.clean
   ```
7. **Bitstream**: build it as described below and add it to `config_hwdb.yaml`.

### Hardware configs

Bitstreams are listed in `sims/firesim/deploy/config_hwdb.yaml` and chosen with `--hw-config`:

| `--hw-config` | Target config | |
|---|---|---|
| `fvela_u280` (default) | `FireSimFVelaSoCConfig` | F-Vela SoC: Rocket + velaVPU + velaNPU (fp16 enabled) |

New bitstream: add the recipe to `builds_to_run` in `sims/firesim/deploy/config_build.yaml` and run
`firesim buildbitstream` (stock configs: `script/raw_config_build.sh --firesim -c ...`). The result goes
to `sims/firesim/deploy/results-build/<date>-<name>/.../firesim.tar.gz`; put that path into
`config_hwdb.yaml` as `bitstream_tar`. Vivado works in `firesim_build_dir/`, which can be deleted
after the build.

### Running: `script/fvela_firesim.sh`

```bash
cd script
# Linux (Buildroot). Without --workload the workload is br-base.
./fvela_firesim.sh run --binary ../software/test/xxx/build/linux/my_test.linux --attach   # copies it to /media of br-base.img
./fvela_firesim.sh run --workload vela_vpu_pid_test --attach                                    # prepared image (rootfs/)
./fvela_firesim.sh run --hw-config nocustom_gemmini_only_u280 --workload ncnn_conv_test --attach

# bare-metal (.riscv with HTIF): the program is the boot binary and ends by itself
./fvela_firesim.sh run --baremetal --binary ../software/test/vela_vpu_pid_test/build/vfpid_4f.riscv --attach

./fvela_firesim.sh attach    # back to the console (screen fsim0; detach: Ctrl-a d)
./fvela_firesim.sh log       # follow the uartlog of the latest run
./fvela_firesim.sh status
./fvela_firesim.sh stop
```
In Buildroot, log in as `root`; test files are in `/media`.

`--skip-infrasetup` keeps the programmed FPGA (same bitstream) and only refreshes the staged image /
bare-metal program, so the next run starts faster. The workload must have had one full run before.

### Linux images: `rootfs/`

Each Linux workload has a folder `rootfs/<workload>/overlay/` that mirrors the target's `/`
(symlinks to build outputs). `rootfs/sync_rootfs.sh <workload>` copies it into
`software/firemarshal/images/firechip/<workload>/<workload>.img` (sudo; unchanged files are skipped).
A new workload (json + image from `br-base.img.clean`) is created on its first sync.

#### Putting test files on the rootfs: `deploy_to_image.sh`

Each test folder has a `deploy_to_image.sh`: it builds the Linux programs, links them into
`rootfs/<workload>/overlay/media/` and runs `sync_rootfs.sh`. Then boot that workload.

```bash
cd software/test/vela_vpu_pid_test
./deploy_to_image.sh                 # make linux -> link new build/linux/*.linux -> sync into vela_vpu_pid_test.img
./deploy_to_image.sh --no-build      # sync only
./deploy_to_image.sh -n              # dry run: list what would be copied
./deploy_to_image.sh --clean         # rebuild the image from br-base.img.clean (drops files no longer in the overlay)
../../../script/fvela_firesim.sh run --workload vela_vpu_pid_test --attach
# on the target: cd /media && ls
```

| Test folder | Workload | Goes to `/media` | Notes |
|---|---|---|---|
| `vela_npu_test/` | `vela_npu_test` | `build/linux/*.linux` | |
| `vela_vpu_pid_test/` | `vela_vpu_pid_test` | `build/linux/*.linux` | |
| `vela_vpu_rope_test/` | `vela_vpu_rope_test` | `build/linux/*.linux` | |
| `profiler_test/` | `profiler_test` | `build/linux/*.linux` | workload created on first run |
<!-- | `ncnn_test/` | `ncnn_conv_test` | cpu/npu/gemmini binaries + `ncnn_models/` | build first (`make testbins`, models); `--models "lenet resnet18"`; 1G image | -->
<!-- | `bitnet_test/` | `bitnet_test` | `llama-cli.linux`, `run_bitnet.sh`, 2 `.gguf` models, self tests | 3G image | -->

The generic options (`--no-build`, `--size`, `--clean`, `-n`/`--dry-run`, `--workload`, `--image`)
are passed to `sync_rootfs.sh`; `./deploy_to_image.sh -h` lists what each script takes.
Files that are not build outputs (scripts, data): add a symlink in the overlay yourself and run
`rootfs/sync_rootfs.sh <workload>`.

```bash
rootfs/sync_rootfs.sh vela_vpu_pid_test          # copy changed files
rootfs/sync_rootfs.sh -n vela_vpu_pid_test       # dry run
rootfs/sync_rootfs.sh --clean vela_vpu_pid_test  # rebuild the image from the clean base, then copy
```
Image sizes (`rootfs/<workload>/config`) and rules: [rootfs/README.md](rootfs/README.md).

### Logs

Every run writes `rootfs/logs/<YYYY-MM-DD>/<HHMMSS>-<workload>[-<program>]/`: `uartlog` (copied live),
`run.info` (workload, hw config, bitstream, program, start/end time, command) and FireSim's csv
stats. `rootfs/logs/latest` points to the newest run. (FireSim's own
`sims/firesim/deploy/results-workload/` only gets the uartlog when the simulation ends by itself, and
its folder names are in UTC.)
