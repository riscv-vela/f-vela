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

## Building NPU Software (build.sh)

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

## Testbench and Software Stack

The f-vela testing environment consists of:

### 1. RTL-Level Testbenches

- Integrated into the full SoC via Chipyard and FireSim.
- You can run full-system tests using Verilator or FPGA (VCU118 / U280).
- Module-level RTL testbenches (if any) can be added under each `src` directory and run with the standard Verilog/Chisel simulation flow.

### 2. Software-Level Tests

Software tests live under `NPU/software/`:

- `bareMetalC`  
  Simple arithmetic / memory access / micro-benchmark tests

- `riscv-tests`  
  Extended RISC-V tests that exercise NPU instructions and interfaces

- `rocc-software`  
  Programs that issue RoCC instructions to the NPU and validate results

- `transformers`  
  Small transformer or NN inference workloads that stress GEMM / vector paths in the NPU

All of these can be built via `./build.sh` and then used as workloads in simulation or on FPGA.

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
