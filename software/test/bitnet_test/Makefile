########################################################################################################################
# bitnet_test / Makefile — BitNet b1.58 2B llama-cli on the F-Vela SoC, Rocket core only or with the velaNPU
#
#   make            # = make riscv: build/linux/llama-cli.linux + vela_selftest.linux + mp_transpose_test.linux
#   make baremetal  # build/baremetal/mp_transpose_test.riscv (Verilator / bare-metal FireSim)
#   make src        # ./setup_bitnet.sh $(BITNET_VERSION): clone BitNet from GitHub into ./BitNet-<version>, apply patches/
#   make model      # model/i2s_bitnet.gguf from model/i2s_original.gguf (I2_S layout conversion)
#   make host       # x86 build with the NPU emulated in C: build/<version>/host/bin/, build/host/vela_selftest
#   make check-host # host: self-test + llama-cli in every VELA_MODE, outputs must match cpu
#   make deploy     # make riscv model, then install into the rootfs of FireSim workload bitnet_test
#   make patches    # write ./BitNet-<version> edits back into patches/<version>/ (after changing BitNet sources)
#
# BITNET_VERSION=latest (default): microsoft/BitNet 0b341e5 + llama.cpp 390c30775, patches/latest
#                pinned          : microsoft/BitNet 404980e + llama.cpp 40ed0f29, patches/pinned (first version used)
# Each version has its own sources and build/<version>/ directory; build/linux/ holds the last one built.
#   make clean      # remove build/ (keeps ./BitNet and the models)
#
# Run-time mode selection (env, see vela/ggml-vela.c, or run_bitnet.sh -m MODE on the target):
#   VELA_MODE=cpu | mp | mpt | int8 | fp16 | npu | npu-mpt | npu-int8
#   VELA_ROPE=off | on | force   (RoPE on the velaVPU RoPE unit; host: emulated, ROM base VELA_EMU_ROPE_BASE)
########################################################################################################################

SELF_DIR   := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
SOFTWARE   := $(abspath $(SELF_DIR)/../..)
CHIPYARD   := $(abspath $(SELF_DIR)/../../../../..)
RISCV      ?= $(CHIPYARD)/.conda-env/riscv-tools
GEMMINI_INC := $(SOFTWARE)/include

# BitNet sources (see setup_bitnet.sh for the commits and what the patches change)
BITNET_VERSION ?= latest
BITNET     := $(SELF_DIR)/BitNet-$(BITNET_VERSION)
BUILD      := build/$(BITNET_VERSION)
# the latest llama.cpp's llama-cli is chat-only; llama-completion is the one-shot prompt tool (old llama-cli)
LLAMA_TARGET := $(if $(filter pinned,$(BITNET_VERSION)),llama-cli,llama-completion)
VELA       := $(SELF_DIR)/vela
PYTHON     ?= /home/esca/miniforge3/envs/bitnet-cpp/bin/python

MODEL_SRC  := model/i2s_original.gguf
MODEL      := model/i2s_bitnet.gguf

# ISA of the RISC-V build. rv64gc = Rocket scalar; rv64gcv lets GCC auto-vectorize for the vector unit.
RISCV_MARCH ?= rv64gc
# the latest ggml adds its own -march=rv64gcv_zfh_zvfh_... on riscv64 unless these are off (keep RISCV_MARCH)
RISCV_GGML_FLAGS := $(if $(filter pinned,$(BITNET_VERSION)),,-DGGML_RVV=OFF -DGGML_RV_ZFH=OFF -DGGML_RV_ZVFH=OFF \
                    -DGGML_RV_ZICBOP=OFF -DGGML_RV_ZIHINTPAUSE=OFF)

CMAKE_COMMON := -G Ninja -DCMAKE_BUILD_TYPE=Release -DGGML_OPENMP=OFF -DLLAMA_CURL=OFF -DLLAMA_BUILD_TOOLS=ON -DLLAMA_BUILD_COMMON=ON \
                -DGGML_VELA=ON -DGGML_VELA_DIR=$(VELA)

.PHONY: all riscv baremetal host src model check-host deploy patches clean
all: riscv

# ---- sources ---------------------------------------------------------------------------------------------------------
src: $(BITNET)/.vela-patched

$(BITNET)/.vela-patched: $(wildcard patches/$(BITNET_VERSION)/*.patch) setup_bitnet.sh
	./setup_bitnet.sh $(BITNET_VERSION) --dir $(BITNET)
	@touch $@

patches:
	./setup_bitnet.sh $(BITNET_VERSION) --dir $(BITNET) --save

# ---- model -----------------------------------------------------------------------------------------------------------
model: $(MODEL)

$(MODEL): $(MODEL_SRC) tools/convert_i2s_layout.py
	$(PYTHON) -I tools/convert_i2s_layout.py $< $@

# ---- RISC-V (FireSim Buildroot Linux) --------------------------------------------------------------------------------
riscv: build/linux/llama-cli.linux build/linux/vela_selftest.linux build/linux/mp_transpose_test.linux

$(BUILD)/riscv/build.ninja: $(BITNET)/.vela-patched cmake/riscv64-linux-gnu.cmake
	cmake -S $(BITNET) -B $(BUILD)/riscv $(CMAKE_COMMON) \
	  -DCMAKE_TOOLCHAIN_FILE=$(SELF_DIR)/cmake/riscv64-linux-gnu.cmake -DRISCV_TOOLS=$(RISCV) \
	  -DCMAKE_C_FLAGS="-march=$(RISCV_MARCH) -mabi=lp64d" -DCMAKE_CXX_FLAGS="-march=$(RISCV_MARCH) -mabi=lp64d" \
	  -DCMAKE_EXE_LINKER_FLAGS=-static -DBUILD_SHARED_LIBS=OFF \
	  -DGGML_NATIVE=OFF -DGGML_LLAMAFILE=OFF -DGGML_VELA_GEMMINI_INC=$(GEMMINI_INC) $(RISCV_GGML_FLAGS)

build/linux/llama-cli.linux: $(BUILD)/riscv/build.ninja $(wildcard $(VELA)/*.[ch]) FORCE
	ninja -C $(BUILD)/riscv $(LLAMA_TARGET)
	@mkdir -p build/linux
	$(RISCV)/bin/riscv64-unknown-linux-gnu-strip -o $@ $(BUILD)/riscv/bin/$(LLAMA_TARGET)
	@echo $(BITNET_VERSION) > build/linux/bitnet_version

build/linux/vela_selftest.linux: test/vela_selftest.c $(VELA)/vela_npu.c $(VELA)/vela_npu.h $(VELA)/vela_rope.c $(VELA)/vela_rope.h
	@mkdir -p build/linux
	$(RISCV)/bin/riscv64-unknown-linux-gnu-gcc -O2 -static -march=$(RISCV_MARCH) -mabi=lp64d \
	  -I$(VELA) -I$(GEMMINI_INC) $< $(VELA)/vela_npu.c $(VELA)/vela_rope.c -o $@ -lm

# mpgemm + b_transpose probe (gemmini.h only)
build/linux/mp_transpose_test.linux: test/mp_transpose_test.c
	@mkdir -p build/linux
	$(RISCV)/bin/riscv64-unknown-linux-gnu-gcc -O2 -static -march=$(RISCV_MARCH) -mabi=lp64d -I$(GEMMINI_INC) $< -o $@ -lm

# Verilator (from chipyard/sims/verilator):
#   ./simulator-chipyard.harness-FVelaSoCConfigTest-debug +permissive +loadmem=ELF +max-cycles=100000000 +permissive-off ELF
# with ELF = build/baremetal/mp_transpose_test_sim.riscv (4 small cases, ~10 min each in simulation)
baremetal: build/baremetal/mp_transpose_test.riscv build/baremetal/mp_transpose_test_sim.riscv
build/baremetal/mp_transpose_test_sim.riscv: test/mp_transpose_test.c
	@mkdir -p build/baremetal
	$(RISCV)/bin/riscv64-unknown-elf-gcc -I$(GEMMINI_INC) -O2 -fno-common -fno-builtin-printf -specs=htif_nano.specs \
	  -march=rv64gc -mabi=lp64d -DBAREMETAL -DSIM_SMALL $< -o $@ -lm
build/baremetal/mp_transpose_test.riscv: test/mp_transpose_test.c
	@mkdir -p build/baremetal
	$(RISCV)/bin/riscv64-unknown-elf-gcc -I$(GEMMINI_INC) -O2 -fno-common -fno-builtin-printf -specs=htif_nano.specs \
	  -march=rv64gc -mabi=lp64d -DBAREMETAL $< -o $@ -lm

# ---- host (x86, NPU emulated) ----------------------------------------------------------------------------------------
HOST_LLAMA := $(BUILD)/host/bin/$(LLAMA_TARGET)
# the latest llama-completion would start a chat when the model has a chat template
HOST_ARGS  := $(if $(filter pinned,$(BITNET_VERSION)),,-no-cnv)
host: $(HOST_LLAMA) build/host/vela_selftest

$(BUILD)/host/build.ninja: $(BITNET)/.vela-patched
	cmake -S $(BITNET) -B $(BUILD)/host $(CMAKE_COMMON) -DGGML_VELA_EMULATE=ON \
	  -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++

$(HOST_LLAMA): $(BUILD)/host/build.ninja $(wildcard $(VELA)/*.[ch]) FORCE
	ninja -C $(BUILD)/host $(LLAMA_TARGET)

build/host/vela_selftest: test/vela_selftest.c $(VELA)/vela_npu.c $(VELA)/vela_npu.h $(VELA)/vela_rope.c $(VELA)/vela_rope.h
	gcc -O2 -Wall -Wextra -DVELA_EMULATE -I$(VELA) $< $(VELA)/vela_npu.c $(VELA)/vela_rope.c -o $@ -lm

# Same check as on the target, on x86: identical text for every mode (greedy), checks pass.
CHECK_PROMPT ?= The capital of France is
check-host: host $(MODEL)
	build/host/vela_selftest -q
	@set -e; for m in cpu mp mpt int8 npu; do \
	  VELA_MODE=$$m VELA_CHECK=2 $(HOST_LLAMA) -m $(MODEL) -p "$(CHECK_PROMPT)" -n 8 --temp 0 -t 8 \
	    --no-warmup $(HOST_ARGS) < /dev/null > $(BUILD)/host/check-$$m.out 2> $(BUILD)/host/check-$$m.err; \
	  printf '%-5s %s\n' $$m "$$(cat $(BUILD)/host/check-$$m.out | tr '\n' ' ')"; \
	  ! grep -q "CHECK FAILED" $(BUILD)/host/check-$$m.err || { echo "$$m: NPU check failed ($(BUILD)/host/check-$$m.err)"; exit 1; }; \
	done; \
	for m in mp mpt int8; do cmp -s $(BUILD)/host/check-cpu.out $(BUILD)/host/check-$$m.out || { echo "$$m output differs from cpu"; exit 1; }; done; \
	echo "check-host ($(BITNET_VERSION)): PASS"

# ---- FireSim rootfs --------------------------------------------------------------------------------------------------
deploy: riscv $(MODEL)
	./deploy_to_image.sh --no-build

clean:
	rm -rf build

FORCE:
