# f-vela: RISC-V-based NPU/VPU FPGA Test Platform

**f-vela** is a project for running a Chipyard-based RISC-V SoC on FPGA boards
and testing NPU (Neural Processing Unit) and VPU (Vector Processing Unit)
hardware in a realistic environment.

This repository provides:

- NPU and VPU RTL integrated into a RISC-V SoC
- A software stack and example programs for NPU/VPU
- Scripts and instructions for running on VCU118 and Alveo U280 (FireSim)

---

## Project Overview

The goals of f-vela are:

- To provide a testbed where a RISC-V SoC runs on FPGA and NPU/VPU can be validated on real hardware
- To leverage Chipyard so NPU/VPU hardware blocks can be integrated, swapped, and extended easily
- To provide a software stack supporting:
  - bare-metal tests
  - RoCC-based tests
  - simple AI / transformer workloads
- To support two FPGA environments:
  - Xilinx VCU118
  - Xilinx Alveo U280 via FireSim

---

## Supported FPGA Boards

Currently, f-vela targets the following FPGA environments:

| FPGA Board                   | Usage                                   | Documentation |
|-----------------------------|-----------------------------------------|---------------|
| Xilinx VCU118               | Direct Chipyard SoC prototyping on FPGA | https://chipyard.readthedocs.io/en/1.13.0/Prototyping/VCU118.html |
| Xilinx Alveo U280 (FireSim) | FireSim-based FPGA simulation           | https://docs.fires.im/en/1.17.0/Getting-Started-Guides/On-Premises-FPGA-Getting-Started/Xilinx-Alveo-U280-FPGAs.html |

- **VCU118**: Follow Chipyard’s VCU118 prototyping flow to generate a bitstream and bring up the SoC on the board.
- **Alveo U280**: Use FireSim’s on-premises U280 flow to deploy and run the SoC + NPU/VPU in a simulated environment on the FPGA.

---

## Required Toolchain and Versions

The following environment is assumed:

### Required

- Chipyard **1.13.0**
- FireSim **1.17.x** (required only when using Alveo U280)
- RISC-V GNU Toolchain  
  (provided after installing Chipyard and sourcing `env.sh`)
- Scala / SBT
- FIRRTL / Verilator
- Xilinx Vivado 2021.1 (for Alveo U280, VCU118 bitstream generation)

## Directory Layout

The top-level layout of this repository is:

```text
f-vela/
├── NPU
│   ├── software
│   │   ├── bareMetalC
│   │   ├── build
│   │   ├── imagenet
│   │   ├── include
│   │   ├── riscv-tests
│   │   ├── rocc-software
│   │   └── transformers
│   └── src                  # NPU RTL (to replace Chipyard's gemmini sources)
│
└── VPU
    ├── software
    └── src                  # VPU RTL (to replace Chipyard's saturn-vector sources)
```

---


## Chipyard Integration and Build Flow

### 1. Install Chipyard and Set Up the Environment

Follow the official Chipyard documentation to install Chipyard.  
Then, set up the environment:

```bash
cd chipyard
source env.sh
```

You must source `env.sh` to get access to the RISC-V toolchain, such as  
`riscv64-unknown-elf-gcc`, `spike`, and other utilities.

---

### 2. Integrate the NPU RTL into Chipyard

The NPU is integrated by replacing the Gemmini module in Chipyard.

1. Locate the Gemmini directory in Chipyard:

   ```text
   chipyard/generators/gemmini/src/main/scala/gemmini/
   ```

2. Remove the existing Gemmini Scala sources in that directory.

3. Copy the sources from `f-vela/NPU/src/` into the Gemmini directory:

   ```bash
   cd chipyard/generators/gemmini/src/main/scala/gemmini
   rm -f *.scala
   cp /path/to/f-vela/NPU/src/*.scala .
   ```

4. Select a Chipyard SoC configuration that uses this NPU/Gemmini setup (or define your own config).

---

## NPU Software & Testbench

The NPU software directory serves two purposes:

1. **It provides the software test suite**, including bare-metal, RoCC-based, and transformer workloads.
2. **It produces all binaries used as workloads** for Verilator simulation, VCU118 FPGA execution, and FireSim (U280).

## Building NPU Software

The NPU software is organized as:

```text
NPU/software/
├── bareMetalC
├── build
├── imagenet
├── include
├── riscv-tests
├── rocc-software
└── transformers
```

All NPU test programs can be built in one shot using the provided `build.sh` script.

1. Make sure the Chipyard environment is already sourced:

   ```bash
   cd chipyard
   source env.sh
   ```

2. Move to the NPU software directory in this repository:

   ```bash
   cd /path/to/f-vela/NPU/software
   ```

3. Run the build script:

   ```bash
   ./build.sh
   ```

The script will:

- Use the RISC-V toolchain from Chipyard (e.g., `riscv64-unknown-elf-gcc`)
- Build all NPU test programs in the subdirectories
- Place the resulting binaries into the `build/` directory:

```text
NPU/software/build/
    ├── ...
    └── (NPU test executables)
```

You can then take the binaries in `build/` and:

- Load them in **Verilator** simulation as the test program, or  
- Run them on the FPGA (VCU118 or U280/FireSim) as workloads to evaluate and experiment with the NPU.

## Testbench and Software Stack

Software tests live under `NPU/software/`:

- `bareMetalC`  
  Simple arithmetic / memory access / micro-benchmark tests
  Includes `gemv_single` / `gemv_double` for INT8×INT8 GEMV acceleration (`gemv_auto`),  
  and `mpgemm.c` for ternary tiled GEMM acceleration (`tiled_mpgemm_auto`).

- `riscv-tests`  
  Extended RISC-V tests that exercise NPU instructions and interfaces

- `rocc-software`  
  Programs that issue RoCC instructions to the NPU and validate results

- `transformers`  
  Small transformer or NN inference workloads that stress GEMM / vector paths in the NPU

All of these can be built via `./build.sh` and then used as workloads in simulation or on FPGA.

---

## VPU Software & PID Test Workflow

This section describes how to build and simulate the software for the RISC-V VPU(Saturn).

> ## ⚠️ REQUIRED BEFORE RUNNING vfpid — DO NOT SKIP
>
> The vfpid workload is built and validated **only** for the following Saturn vector
> configuration. You **must** use a config with these exact parameters in **both**
> Verilator simulation and FireSim, otherwise vfpid will produce wrong results or fail
> to elaborate:
>
> | Parameter            | Required Value |
> |----------------------|----------------|
> | **VLEN** (`vLen`)    | **512 bits**   |
> | **DLEN** (`dLen`)    | **256 bits**   |
>
> - **Verilator**: use the `REFV512D256RocketConfig` config (`V512` = VLEN 512, `D256` = DLEN 256).
> - **FireSim**: use a target config whose Saturn `VectorParams` set `vLen = 512` and `dLen = 256`
>   (i.e. the FireSim equivalent of `REFV512D256RocketConfig`). See
>   [3. FireSim (Alveo U280) Settings for vfpid](#3-firesim-alveo-u280-settings-for-vfpid) below.
>
> If you change VLEN/DLEN, you must rebuild the vfpid binary and regenerate the design.

### 1. Configure CMakeLists.txt

Before building, you need to register the source files and enable the Vector extension in `CMakeLists.txt`.

1. **Add Build Targets**:
   Open `CMakeLists.txt` and add the following lines to register the executable and dump target:
   ```cmake
   # In the Build section
   add_executable(vfpid_4d vfpid_4d.c)
   
   # In the Disassembly section
   add_dump_target(vfpid_4d)
   ```

2. **Enable Vector Extension**:
Important: Ensure that the CPU Architecture flags include the vector extension (v). You must append v to the architecture string in CMakeLists.txt (e.g., -march=rv64gcv).


### 2. Build the PID Binary
Navigate to the test directory and build the targets. This will generate *.riscv (binary) and *.dump (disassembly).
```cmake
cmake --build ./build/ --target all
```

### 3. Run Verilator Simulation
You can run the generated binary on the Chipyard Verilator simulator using specific Rocket Chip configurations.

> **Required config:** vfpid must run on `REFV512D256RocketConfig` (**VLEN = 512 bit, DLEN = 256 bit**).
> Any other VLEN/DLEN combination is unsupported for vfpid.

Navigate to the simulator directory:
```bash
cd chipyard/sims/verilator
```

Run with the ex-512bit VLEN / 256bit DLEN config (note the mandatory `CONFIG=REFV512D256RocketConfig`):
```bash
make run-binary-debug CONFIG=REFV512D256RocketConfig VERILATOR_THREADS=8 -j$(nproc) BINARY=../../tests/build/vfpid_4d.riscv
```

### 4. Verify Output
After the simulation completes, you can inspect the waveform to verify the PID operation. Check the generated VCD file in the output directory:
```bash
/sims/verilator/output/chipyard.TestHarness.<CONFIG>/PID.vcd
```

### 5. FireSim (Alveo U280) Settings for vfpid

When running vfpid under FireSim, the **VLEN = 512 / DLEN = 256** requirement still applies — the
FPGA design must be built from a target config with these exact Saturn vector parameters.

1. **Select / define a FireSim target config with VLEN=512, DLEN=256.**
   In your FireSim target project (Chipyard `firesim` area), make sure the `DESIGN`/`TARGET_CONFIG`
   you build uses the Saturn `VectorParams` equivalent of `REFV512D256RocketConfig`:

   ```scala
   // VectorParams (Saturn) must set:
   //   vLen = 512   // VLEN = 512 bit
   //   dLen = 256   // DLEN = 256 bit
   ```

   Set this as the FireSim `TARGET_CONFIG` (e.g. in `config_build_recipes.yaml` /
   `config_runtime.yaml`) before building the bitstream.

2. **Build the FireSim bitstream / AFI** from that VLEN=512/DLEN=256 target config:

   ```bash
   firesim buildbitstream
   ```

3. **Register the vfpid binary as the workload.**
   Use the same `vfpid_4d.riscv` produced in step 2 (Build the PID Binary) as the FireSim workload,
   then deploy and run:

   ```bash
   firesim infrasetup
   firesim runworkload
   ```

> ⚠️ If the FireSim bitstream was built with a different VLEN/DLEN, you must rebuild it with
> VLEN=512 / DLEN=256 before running vfpid — switching the workload alone is not enough.


---

## Simulation and FPGA Execution

### 1. Verilator Simulation (Chipyard)

You can test the NPU/VPU using Chipyard’s Verilator-based simulator.

Example flow:

```bash
cd chipyard/sims/verilator

# Run the simulator with your NPU test binary
make CONFIG=YourNPUCONFIG run-binary-debug BINARY=/path_to_f-vela/NPU/software/build/bareMetalC/mpgemm-baremetal
```

- The exact `CONFIG` name depends on your Chipyard configuration that includes the f-vela NPU/VPU.
- Typically, the NPU binary built under `NPU/software/build/` is loaded as the test program (via Chipyard’s standard simulation flow).

---

### 2. FPGA Execution on VCU118

To run on a VCU118 board, follow Chipyard’s VCU118 prototyping guide:

- Documentation:  
  https://chipyard.readthedocs.io/en/1.13.0/Prototyping/VCU118.html

The typical flow is:

1. Generate a VCU118 bitstream for the SoC configuration that includes the NPU.
2. Program the VCU118 board with the generated bitstream.
3. Load a binary from `NPU/software/build/` as the workload (e.g., via boot ROM, UART bootloader, or the standard Chipyard flow).
4. Run the program and verify NPU/VPU behavior through UART logs, memory dumps, or performance counters.

---

### 3. FPGA Execution on Alveo U280 (FireSim)

On Alveo U280, the SoC is executed under FireSim:

- Documentation:  
  https://docs.fires.im/en/1.17.0/Getting-Started-Guides/On-Premises-FPGA-Getting-Started/Xilinx-Alveo-U280-FPGAs.html

Typical flow:

1. Configure FireSim to use the SoC design that integrates the f-vela NPU.
2. Build the FireSim target and manager configuration.
3. Use the binaries from `NPU/software/build/` as workloads in the FireSim run farm (e.g., by specifying them in the FireSim workload configuration).
4. Deploy and run the design on the U280, and collect results for NPU/VPU tests.

---

## Maintenance and Acknowledgments

**f-vela** is primarily developed and maintained by the **Embedded Systems and Computer Architecture Laboratory (ESCA) at Korea University**.
The profiler component of f-vela was contributed by **ASO at Yonsei University**.

For more information on the research related to f-vela, please refer to:

- [Three Birds, One Stone: Fast, Accurate-aware and Cost-Efficient Accelerator for Ternary LLM](https://doi.org/10.1145/3797905.3807877)

If you publish research using or building upon f-vela, please acknowledge the **f-vela project** in your publication.

## Contributions

Contributions to **f-vela** are always welcome.

### How to Contribute

- If you find a bug, have a feature request, or need documentation updates,
  please open an Issue.
- For code contributions, including RTL, software, tests, scripts, and
  documentation, please submit a Pull Request.
- When submitting a Pull Request, reference any related issues
  (e.g., `Closes #5`).

### Contact

If you have questions about using the vela NPU/VPU, building software, or running on FPGA/FireSim, feel free to contact:

- Current maintainer: **woojean292@gmail.com**
- Previous maintainer: **bgiant6097@gmail.com**

We encourage users to open GitHub Issues first so discussions remain visible to the community.

<!-- ---

## FPGA Resource Usage and Performance

This section is intended to summarize resource usage and performance for the target FPGA boards.


```text
[Example] ALVEO U280 Synth/Implementation
- LUT   : 
- FF    : 
- BRAM  : 
- URAM  : 0
- DSP   : 
- Fmax  : 60 MHz

[Example] Alveo U280 (FireSim) Performance
- Target Freq        : TBD MHz
- NPU Throughput     : TBD GOPS
- End-to-end Latency : TBD ms
``` -->
