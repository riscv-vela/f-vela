# f-vela: RISC-V-based NPU/VPU FPGA Test Platform

**f-vela** is a project for running a Chipyard-based RISC-V SoC on FPGA boards
and testing NPU (Neural Processing Unit) and VPU (Vector Processing Unit)
hardware in a realistic environment.

**This repository provides**:
- NPU and VPU RTL integrated into a RISC-V SoC
- A software stack and example programs for NPU/VPU
- Scripts and instructions for running on VCU118 and Alveo U280 (FireSim)

**Tested Environment**
- Operating System : Ubuntu 24.04.4 LTS
- Kernel : Linux 6.17.0-19-Generic
- Architecture : x86-64

## chipyard Install
```sh
# Requirement Package Install 
sudo apt-get update
sudo apt-get instll libguestfs-tools
libguestfs-test-tool

# grant kernel Read Permission
sudo chmod 0644 /boot/vmlinuz

# Chipyard Install
conda activate base 
git clone https https://github.com/ucb-bar
chipyard.git
cd chipyard
./build-setup.sh riscv-tools
```
<details>
<summary>build-setup.sh script fails on conflict issue</summary>

If the build-setup.sh script fails on conflict issues sometimes it helps to run conda update -n base --all

> example) ./build-setup.sh riscv-tools -s 1 -s 2 -s3
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

## f_vela install
```sh
cd generators
git clone --branch woojin https://github.com/riscv-vela/f-vela.git _f_vela
cd _f_vela
sh ./setup.sh
```

## folder layout
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

## Spike Simulation 
#### gemmini (Custom gemmini)

#### PID (Custom Saturn - PID Unit )

#### RoPE (Custom Saturn - RoVER)


## Verilator Build 

> 