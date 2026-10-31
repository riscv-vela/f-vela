# ncnn inference test (LeNet-5 / VGG16 / ResNet18) for the F-Vela SoC on FireSim (Buildroot Linux).
#
#   make            -> build/linux/ncnn_infer_rvv.linux     (clang 18, ncnn RVV/zfh/zvfh kernels, runtime-dispatched)
#                      build/linux/ncnn_infer_scalar.linux  (chipyard GCC 13, rv64gc only)
#   make host       -> build/host/ncnn_infer               (x86 sanity check against the PyTorch reference)
#   make models     -> models/<name>/*.ncnn.{param,bin}, input.bin, ref.bin  (needs `source chipyard/env.sh`)
#
# The ncnn libraries are built separately (see README.md):
#   software/ncnn/build-riscv-rvv     clang toolchain
#   software/ncnn/build-riscv-scalar  chipyard riscv64-unknown-linux-gnu-gcc
#   software/ncnn/build               host build

CHIPYARD   ?= /home/esca/_f_vela/chipyard
NCNN_DIR   ?= $(CHIPYARD)/software/ncnn
RISCV      ?= $(CHIPYARD)/.conda-env/riscv-tools
RISCV_LLVM_ROOT ?= /home/esca/_f_vela/toolchains/riscv64-llvm

RVV_NCNN    := $(NCNN_DIR)/build-riscv-rvv/install
SCALAR_NCNN := $(NCNN_DIR)/build-riscv-scalar/install
HOST_NCNN   := $(NCNN_DIR)/build
# system compiler: the host ncnn build was made with it (conda g++ from env.sh lacks libmvec)
HOST_CXX    ?= /usr/bin/g++

SRC := src/ncnn_infer.cpp
BUILD := build

CXXFLAGS_COMMON := -O2 -std=c++11

RVV_CXX    := $(RISCV_LLVM_ROOT)/bin/clang++ --target=riscv64-unknown-linux-gnu --sysroot=$(RISCV_LLVM_ROOT)/sysroot
SCALAR_CXX := $(RISCV)/bin/riscv64-unknown-linux-gnu-g++
RISCV_CXXFLAGS := $(CXXFLAGS_COMMON) -march=rv64gc -mabi=lp64d
RISCV_LDFLAGS  := -static -lpthread

all: $(BUILD)/linux/ncnn_infer_rvv.linux $(BUILD)/linux/ncnn_infer_scalar.linux $(BUILD)/linux/rvv_ptr_test.linux testbins

# ---- test-procedure executables: {cpu,npu,gemmini}_{lenet,vgg16,resnet18}.riscv ------------------
# One source (src/ncnn_conv_test.cpp), model and backend fixed at compile time, linked against the
# scalar ncnn (the RVV path is affected by the page-crossing vector bug on the current bitstream).
#   cpu_*      NPU backend off (runs on either bitstream)
#   npu_*      velaNPU (custom Gemmini, pf_gemmini.h + profiler)  -> FireSimFVelaSoCConfig (fvela_u280)
#   npu_<m>2   velaNPU, profiler off, cycles from the Gemmini perf counters (same method as gemmini_*)
#              -> FireSimFVelaSoCConfig (fvela_u280)   e.g. npu_lenet2, npu_vgg16_2, npu_resnet18_2
#   gemmini_*  upstream Gemmini (gemmini-rocc-tests gemmini.h + perf counters)
#                                                                  -> FireSimNoCustomGemminiOnlyTest (nocustom_gemmini_only_u280)
TEST_MODELS := lenet vgg16 resnet18
model_id = $(if $(filter lenet,$(1)),0,$(if $(filter vgg16,$(1)),1,2))
# executable name of backend $(1), model $(2): npuctr -> npu_<m>2 (npu_<m>_2 if <m> ends in a digit)
test_name = $(if $(filter npuctr,$(1)),npu_$(2)$(if $(filter %0 %1 %2 %3 %4 %5 %6 %7 %8 %9,$(2)),_2,2),$(1)_$(2))
TEST_BINS   := $(foreach b,cpu npu npuctr gemmini,$(foreach m,$(TEST_MODELS),$(BUILD)/linux/$(call test_name,$(b),$(m)).riscv))
NPU2_TEST_BINS := $(foreach m,$(TEST_MODELS),$(BUILD)/linux/$(call test_name,npuctr,$(m)).riscv)
GEMMINI_TEST_BINS := $(foreach m,$(TEST_MODELS),$(BUILD)/linux/gemmini_$(m).riscv)
# backend -> HAL object / extra defines
test_hal  = $(BUILD)/obj/$(if $(filter gemmini,$(1)),gemmini_stock_hal,$(if $(filter npuctr,$(1)),vela_npu_hal_ctr,vela_npu_hal)).gcc.o
test_defs = -DTEST_BACKEND_NPU=$(if $(filter cpu,$(1)),0,1) -DTEST_NPU_STOCK_GEMMINI=$(if $(filter gemmini,$(1)),1,0) \
            -DTEST_NPU_CYCLE_COUNTER=$(if $(filter npuctr,$(1)),1,0)
HOST_TEST_BINS := $(foreach b,cpu npu,$(foreach m,$(TEST_MODELS),$(BUILD)/host/$(b)_$(m)))
TEST_SRC    := src/ncnn_conv_test.cpp

testbins: $(TEST_BINS)
gemmini-testbins: $(GEMMINI_TEST_BINS)
npu2-testbins: $(NPU2_TEST_BINS)
host-testbins: $(HOST_TEST_BINS)

define test_bin_rules
$(BUILD)/linux/$(call test_name,$(1),$(2)).riscv: $(TEST_SRC) $(LAYER_SRC) $(LAYER_HDR) $(call test_hal,$(1)) $(SCALAR_NCNN)/lib/libncnn.a
	@mkdir -p $$(dir $$@)
	$(SCALAR_CXX) $(RISCV_CXXFLAGS) -DVELA_NPU=1 -DTEST_MODEL_ID=$(call model_id,$(2)) $(call test_defs,$(1)) \
		-Isrc -I$(SCALAR_NCNN)/include/ncnn $(TEST_SRC) $(LAYER_SRC) $(call test_hal,$(1)) \
		-o $$@ $(SCALAR_NCNN)/lib/libncnn.a $(RISCV_LDFLAGS)
endef

# host build: CPU path, and the NPU path through the int8 software model (no NPU cycles)
define host_test_bin_rules
$(BUILD)/host/$(1)_$(2): $(TEST_SRC) $(LAYER_SRC) $(LAYER_HDR) $(HOST_NCNN)/src/libncnn.a
	@mkdir -p $$(dir $$@)
	$(HOST_CXX) $(CXXFLAGS_COMMON) -DTEST_MODEL_ID=$(call model_id,$(2)) -DTEST_BACKEND_NPU=$(if $(filter npu,$(1)),1,0) \
		-Isrc -I$(NCNN_DIR)/src -I$(HOST_NCNN)/src $(TEST_SRC) $(LAYER_SRC) -o $$@ $(HOST_NCNN)/src/libncnn.a \
		-Wl,--start-group $(wildcard $(HOST_NCNN)/glslang/*/*.a $(HOST_NCNN)/glslang/*/*/*/*.a) -Wl,--end-group \
		-fopenmp -lpthread -ldl
endef

# Hardware check for the RVV strided-copy loop where ncnn crashed (see src/rvv_ptr_test.c).
$(BUILD)/linux/rvv_ptr_test.linux: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(RVV_CXX:clang++=clang) -O2 -march=rv64gcv -mabi=lp64d -fomit-frame-pointer $< -o $@ -static

# Same check, bare-metal (htif) for Verilator: script/run_vsim.sh style, no Linux/interrupts.
# rvv_ptr_test_trap.riscv: rdtime after every vector store traps to an M-mode handler that checks mepc.
# rvv_ptr_test_sv39*.riscv: same, but in S-mode under Sv39 with 4 KiB pages (what Linux user code sees),
# 512 rows -> 256 buffer pages vs a 32-entry DTLB, so page-crossing strided loads keep missing.
# rvv_ptr_test_sv39u_alias*.riscv: U-mode (PTE.U) and the buffers through an alias VA != PA, like
# Linux user code; 128 rows -> 64 buffer pages vs the 32-entry DTLB, shorter than the 512-row runs.
baremetal: $(BUILD)/baremetal/rvv_ptr_test.riscv $(BUILD)/baremetal/rvv_ptr_test_trap.riscv \
           $(BUILD)/baremetal/rvv_ptr_test_sv39.riscv $(BUILD)/baremetal/rvv_ptr_test_sv39_trap.riscv \
           $(BUILD)/baremetal/rvv_ptr_test_sv39u_alias.riscv $(BUILD)/baremetal/rvv_ptr_test_sv39u_alias_trap.riscv \
           $(BUILD)/baremetal/rvv_ptr_test_sv39u_alias_timer.riscv $(BUILD)/baremetal/rvv_ptr_test_timer.riscv \
           $(BUILD)/baremetal/rvv_ptr_test_sv39u_alias_timer_small.riscv $(BUILD)/baremetal/rvv_ptr_test_sv39u_alias_timer_check.riscv

# *_timer.riscv: asynchronous machine timer interrupts every mtime tick (~1000 cycles), like the
# Linux timer tick but much more often; 20 iterations (for the FPGA, where this takes seconds).
$(BUILD)/baremetal/rvv_ptr_test_sv39u_alias_timer.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_SV39 -DBM_UMODE -DBM_ALIAS -DBM_TIMER -DBM_ROWS=128 -DBM_ITERS=20 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

# same + in-loop check that records the cycle of the first divergence (to place the waveform window)
$(BUILD)/baremetal/rvv_ptr_test_sv39u_alias_timer_check.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_SV39 -DBM_UMODE -DBM_ALIAS -DBM_TIMER -DBM_CHECK -DBM_ROWS=16 -DBM_ITERS=1 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

# smallest repro for RTL simulation: one 16-row block (1 copy_loop call), 1 iteration
$(BUILD)/baremetal/rvv_ptr_test_sv39u_alias_timer_small.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_SV39 -DBM_UMODE -DBM_ALIAS -DBM_TIMER -DBM_ROWS=16 -DBM_ITERS=1 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

$(BUILD)/baremetal/rvv_ptr_test_timer.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_TIMER -DBM_ROWS=64 -DBM_ITERS=20 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

$(BUILD)/baremetal/rvv_ptr_test_sv39u_alias.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_SV39 -DBM_UMODE -DBM_ALIAS -DBM_ROWS=128 -DBM_ITERS=2 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

$(BUILD)/baremetal/rvv_ptr_test_sv39u_alias_trap.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_SV39 -DBM_UMODE -DBM_ALIAS -DBM_TRAP -DBM_ROWS=128 -DBM_ITERS=2 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

$(BUILD)/baremetal/rvv_ptr_test_sv39.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_SV39 -DBM_ROWS=512 -DBM_ITERS=2 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

$(BUILD)/baremetal/rvv_ptr_test_sv39_trap.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_SV39 -DBM_TRAP -DBM_ROWS=512 -DBM_ITERS=2 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

BM_CC     := $(RISCV)/bin/riscv64-unknown-elf-gcc
BM_CFLAGS = -DBM_NAME='"$(notdir $@)"' -DBAREMETAL -O2 -march=rv64gcv -mabi=lp64d -mcmodel=medany -fomit-frame-pointer \
             -fno-common -fno-builtin-printf -static -specs=htif_nano.specs

$(BUILD)/baremetal/rvv_ptr_test.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

$(BUILD)/baremetal/rvv_ptr_test_trap.riscv: src/rvv_ptr_test.c
	@mkdir -p $(dir $@)
	$(BM_CC) $(BM_CFLAGS) -DBM_TRAP -DBM_ITERS=1 $< -o $@
	$(RISCV)/bin/riscv64-unknown-elf-objdump -d $@ > $@.dump

# ---- velaNPU convolution backend (src/vela_npu_convolution.cpp + src/vela_npu_hal.c) -------------
# RISC-V builds get VELA_NPU=1 (Gemmini RoCC driver from software/include/pf_gemmini.h);
# the host build only has the CPU / calibration paths.
FVELA_SW   := $(CHIPYARD)/generators/_f_vela/software
ROCC_TESTS := $(CHIPYARD)/generators/gemmini/software/gemmini-rocc-tests
NPU_INC    := -I$(FVELA_SW) -I$(FVELA_SW)/include -I$(ROCC_TESTS)
# pf_gemmini.h assigns sp_tiled_matmul_os to a WS-typed pointer (unused here: we only use WS)
NPU_CFLAGS := -O2 -march=rv64gc -mabi=lp64d -fno-common $(NPU_INC)
LAYER_SRC  := src/vela_npu_convolution.cpp
LAYER_HDR  := src/vela_npu_convolution.h src/vela_npu_hal.h

$(BUILD)/obj/vela_npu_hal.gcc.o: src/vela_npu_hal.c src/vela_npu_hal.h src/gemmini_perf_counters.h
	@mkdir -p $(dir $@)
	$(RISCV)/bin/riscv64-unknown-linux-gnu-gcc $(NPU_CFLAGS) -Wno-incompatible-pointer-types -c $< -o $@

$(BUILD)/obj/vela_npu_hal_ctr.gcc.o: src/vela_npu_hal.c src/vela_npu_hal.h src/gemmini_perf_counters.h
	@mkdir -p $(dir $@)
	$(RISCV)/bin/riscv64-unknown-linux-gnu-gcc $(NPU_CFLAGS) -DVNPU_COUNTER_ONLY=1 -Wno-incompatible-pointer-types -c $< -o $@

# Upstream Gemmini driver: only gemmini-rocc-tests (its gemmini.h / gemmini_params.h, regenerated
# when the stock Gemmini is elaborated), nothing from software/include.
STOCK_CFLAGS := -O2 -march=rv64gc -mabi=lp64d -fno-common -I$(ROCC_TESTS)

$(BUILD)/obj/gemmini_stock_hal.gcc.o: src/gemmini_stock_hal.c src/vela_npu_hal.h src/gemmini_perf_counters.h
	@mkdir -p $(dir $@)
	$(RISCV)/bin/riscv64-unknown-linux-gnu-gcc $(STOCK_CFLAGS) -c $< -o $@

$(BUILD)/obj/vela_npu_hal.clang.o: src/vela_npu_hal.c src/vela_npu_hal.h src/gemmini_perf_counters.h
	@mkdir -p $(dir $@)
	$(RVV_CXX:clang++=clang) $(NPU_CFLAGS) -Wno-incompatible-function-pointer-types -c $< -o $@

$(BUILD)/linux/ncnn_infer_rvv.linux: $(SRC) $(LAYER_SRC) $(LAYER_HDR) $(BUILD)/obj/vela_npu_hal.clang.o $(RVV_NCNN)/lib/libncnn.a
	@mkdir -p $(dir $@)
	$(RVV_CXX) $(RISCV_CXXFLAGS) -DVELA_NPU=1 -Isrc -I$(RVV_NCNN)/include/ncnn $(SRC) $(LAYER_SRC) \
		$(BUILD)/obj/vela_npu_hal.clang.o -o $@ $(RVV_NCNN)/lib/libncnn.a $(RISCV_LDFLAGS)

$(BUILD)/linux/ncnn_infer_scalar.linux: $(SRC) $(LAYER_SRC) $(LAYER_HDR) $(BUILD)/obj/vela_npu_hal.gcc.o $(SCALAR_NCNN)/lib/libncnn.a
	@mkdir -p $(dir $@)
	$(SCALAR_CXX) $(RISCV_CXXFLAGS) -DVELA_NPU=1 -Isrc -I$(SCALAR_NCNN)/include/ncnn $(SRC) $(LAYER_SRC) \
		$(BUILD)/obj/vela_npu_hal.gcc.o -o $@ $(SCALAR_NCNN)/lib/libncnn.a $(RISCV_LDFLAGS)

host: $(BUILD)/host/ncnn_infer

# also writes models/<m>/<m>.npu_calib (needed by --backend npu on the target)
calib: $(BUILD)/host/ncnn_infer
	cd models && python export_models.py --refs-only
	$(BUILD)/host/ncnn_infer all -d models --npu-calib

$(BUILD)/host/ncnn_infer: $(SRC) $(LAYER_SRC) $(LAYER_HDR) $(HOST_NCNN)/src/libncnn.a
	@mkdir -p $(dir $@)
	$(HOST_CXX) $(CXXFLAGS_COMMON) -Isrc -I$(NCNN_DIR)/src -I$(HOST_NCNN)/src $(SRC) $(LAYER_SRC) -o $@ $(HOST_NCNN)/src/libncnn.a \
		-Wl,--start-group $(wildcard $(HOST_NCNN)/glslang/*/*.a $(HOST_NCNN)/glslang/*/*/*/*.a) -Wl,--end-group \
		-fopenmp -lpthread -ldl

models:
	cd models && python export_models.py

clean:
	rm -rf $(BUILD)

.PHONY: all host models calib baremetal testbins gemmini-testbins npu2-testbins host-testbins clean

# generated here, after every variable the rules use is defined
$(foreach b,cpu npu npuctr gemmini,$(foreach m,$(TEST_MODELS),$(eval $(call test_bin_rules,$(b),$(m)))))
$(foreach b,cpu npu,$(foreach m,$(TEST_MODELS),$(eval $(call host_test_bin_rules,$(b),$(m)))))
