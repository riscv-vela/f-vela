# f-vela: RISC-V-based NPU/VPU FPGA Test Platform

**f-vela** is a project for running a Chipyard-based RISC-V SoC on FPGA boards
and testing NPU (Neural Processing Unit) and VPU (Vector Processing Unit)
hardware in a realistic environment.

## chipyard install

Follow the official Chipyard documentation to install Chipyard.

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
./build-setup.sh riscv-tools
```
<details>
<summary>build-setup.sh script fails on conflict issue</summary>

If the build-setup.sh script fails on conflict issues sometimes it helps to run conda update -n base --all

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

## f_vela install (integrated)
```sh
cd generators
git clone --branch woojin/integration https://github.com/riscv-vela/f-vela.git _f_vela
cd _f_vela
sh ./script/setup.sh # patch build_sbt, linking_verilator folder
```

#### folder layout
```sh
chipyard
│   ├── conda-reqs
│   ├── dockerfiles
│   ├── docs
│   ├── fpga
│   ├── generators *
│   │   ├── _f_vela  <-- here
│   │   ├── ara
│   │   ...
│   │   └── vexiiriscv
│   ├── project
│   ├── scripts
│   ├── sims
│   ├── software
│   ├── target
│   ├── tests
│   ├── toolchains
│   ├── tools
│   ├── vlsi
│   │   ... 
└── └── build-setup.sh -> scripts/build-setup.sh
```
---
## Test Program & Simulation(Verilator)


#### Chipyard Configs with FVela SoC
```bash
# create Verilator Simulator Binary
# - this script create verilator_simulator in ../../sims/verilator/
bash script/verilator_make.sh [-c "FVelaSoCConfigTest"]
```


#### Test Code Build example
PID, Custom RoPE, Custom Gemmini, Profiler Test Program Build 
```bash
source ../../env.sh

# Build Test Program
cd software/test/
make

# or use script
cd script/
./testsw_build.sh [gemmini | pid | rope | profiler]

# this script create executable files in software/test/[test folder]/build/
```

#### gemmini
**Verilator Simulation**
```bash
# run simulation (Baremetal Hardware Simulation)
bash script/run_vsim.sh gemmini ternary_gemm.riscv [-s "FVelaSoCConfigTest"]
```


#### PID
**Verilator Simulation**
```bash
# run simulation (Baremetal Hardware Simulation)
bash script/run_vsim.sh pid vfpid_4d.riscv [-s "FVelaSoCConfigTest"]
```


#### RoPE
<!-- ```bash
# usage) spike  risc-v_bin --isa=rv64gcv --log-commits
spike software/test/rv_rope_test/build/vfrope_test.riscv --isa=rv64gcv --log-commits
``` -->
**Verilator Simulation**
```bash
# run simulation (Baremetal Hardware Simulation)
bash script/run_vsim.sh rope vfrope_test.riscv [-s "FVelaSoCConfigTest"]
```


#### Profiler
**Verilator Simulation**
```bash
# run simulation (Baremetal Hardware Simulation)
bash script/run_vsim.sh profiler ternary_profile_test.riscv [-s "FVelaSoCConfigTest"]
```
**Profiler visualize (Verilator Sim + Plot)**
```bash
bash software/test/profiler_test/run_profiler.sh
# this script create gemmini profilng data(csv, png) in software/test/profiler_test/build directory
```


