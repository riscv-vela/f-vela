########################################################################################################################
# cmp_conv_cycle / Makefile
#
#   Builds the three Conv2D cycle-count comparisons into ./build/*.riscv (+ *.dump):
#
#     rocket_cpu_conv.c             - pure Rocket-core software conv, no accelerator at all.
#                                      Header-free, so this exact binary is also what you run on
#                                      both simulators below to get a matched CPU-only baseline.
#     vanilla_gemmini_conv.c        - upstream Gemmini conv (tiled_conv_auto), rdcycle+fence timed.
#                                      Uses gemmini-rocc-tests' own headers (NOT the F-Vela ones).
#                                      Run on: simulator-chipyard.harness-NoCustomFvelaTest
#     custom_gemmini_conv_profiler.c- F-Vela custom Gemmini conv, timed with the real HW profiler
#                                      (pf_gemmini.h / gemmini_profiler), same P[] dump format as
#                                      profiler_test/gemm_profiler_test.c.
#                                      Run on: simulator-chipyard.harness-FVelaSoCConfigTest
#
#   ...and two more for the mpgemm (int8 x ternary matmul) comparison -- 2-way only, since
#   ternary_gemm_auto is a custom/F-Vela-only feature with no vanilla-Gemmini equivalent:
#
#     rocket_cpu_mpgemm.c            - pure Rocket-core mpgemm reference. Same header-free /
#                                       run-on-either-sim story as rocket_cpu_conv.c.
#     custom_gemmini_mpgemm_profiler.c - F-Vela custom Gemmini mpgemm (ternary_gemm_auto),
#                                       profiler-timed, based on profiler_test/ternary_profiler_test.c.
#                                       Run on: simulator-chipyard.harness-FVelaSoCConfigTest
#
#   usage:
#     make                # build all five (bare-metal)
#     make rocket_cpu_conv / vanilla_gemmini_conv / custom_gemmini_conv_profiler
#     make rocket_cpu_mpgemm / custom_gemmini_mpgemm_profiler
#     make linux           # build the Linux (firemarshal/busybox) userspace variants
#     make clean
#
#   See run_cmp.sh to actually run all three against their respective Verilator sims and
#   get a side-by-side cycle-count table.
#
#   The RISC-V toolchain is added to PATH automatically from Chipyard's conda env.
########################################################################################################################

# ---- Paths -----------------------------------------------------------------------------------------------------------
SELF_DIR   := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
SOFTWARE   := $(abspath $(SELF_DIR)/../..)
CHIPYARD   := $(abspath $(SELF_DIR)/../../../../..)
RISCV_BIN  := $(CHIPYARD)/.conda-env/riscv-tools/bin
export PATH := $(RISCV_BIN):$(PATH)

# gemmini-rocc-tests provides:
#   - include/gemmini_testutils.h, include/gemmini.h  (the upstream/vanilla Gemmini API)
#   - rocc-software/src/xcustom.h                     (RoCC instruction macros both APIs need)
ROCC_TESTS := $(CHIPYARD)/generators/gemmini/software/gemmini-rocc-tests

BUILD_DIR  := build

# ---- Toolchain -------------------------------------------------------------------------------------------------------
RISCV_PREFIX ?= riscv64-unknown-elf-
CC            := $(RISCV_PREFIX)gcc
OBJDUMP       := $(RISCV_PREFIX)objdump

# ---- Flags -----------------------------------------------------------------------------------------------------------
COMMON_CFLAGS := -O2 -fno-common -fno-builtin-printf \
                  -specs=htif_nano.specs -march=rv64gc -mabi=lp64d \
                  -DBAREMETAL
LDLIBS        := -lm

# rocket_cpu_conv.c is header-free (no gemmini include at all) beyond the local conv_dims.h.
CPU_CFLAGS      := $(COMMON_CFLAGS)
# vanilla_gemmini_conv.c pulls in gemmini-rocc-tests' own "include/gemmini_testutils.h".
VANILLA_CFLAGS  := $(COMMON_CFLAGS) -I$(ROCC_TESTS)
# custom_gemmini_conv_profiler.c pulls in pf_gemmini.h (F-Vela) + rocc-software/src/xcustom.h.
CUSTOM_CFLAGS   := $(COMMON_CFLAGS) -I$(SOFTWARE) -I$(SOFTWARE)/include -I$(ROCC_TESTS)

PROGS := rocket_cpu_conv vanilla_gemmini_conv custom_gemmini_conv_profiler \
         rocket_cpu_mpgemm custom_gemmini_mpgemm_profiler
TARGETS := $(addprefix $(BUILD_DIR)/,$(addsuffix .riscv,$(PROGS)))
DUMPS   := $(addprefix $(BUILD_DIR)/,$(addsuffix .dump,$(PROGS)))

.PHONY: all clean linux $(PROGS)
all: $(TARGETS) $(DUMPS)

# convenience: `make vanilla_gemmini_conv` -> build vanilla_gemmini_conv.riscv + .dump
$(PROGS): %: $(BUILD_DIR)/%.riscv $(BUILD_DIR)/%.dump

$(BUILD_DIR)/rocket_cpu_conv.riscv: rocket_cpu_conv.c conv_dims.h | $(BUILD_DIR)
	$(CC) $(CPU_CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD_DIR)/vanilla_gemmini_conv.riscv: vanilla_gemmini_conv.c conv_dims.h | $(BUILD_DIR)
	$(CC) $(VANILLA_CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD_DIR)/custom_gemmini_conv_profiler.riscv: custom_gemmini_conv_profiler.c conv_dims.h | $(BUILD_DIR)
	$(CC) $(CUSTOM_CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD_DIR)/rocket_cpu_mpgemm.riscv: rocket_cpu_mpgemm.c mpgemm_dims.h | $(BUILD_DIR)
	$(CC) $(CPU_CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD_DIR)/custom_gemmini_mpgemm_profiler.riscv: custom_gemmini_mpgemm_profiler.c mpgemm_dims.h | $(BUILD_DIR)
	$(CC) $(CUSTOM_CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD_DIR)/%.dump: $(BUILD_DIR)/%.riscv
	$(OBJDUMP) -D $< > $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---- Linux userspace build (for firemarshal/busybox images) ----------------------------------------------------------
#   All three sources are dual-mode (see `#ifndef BAREMETAL`), so they're reused here, just
#   compiled with the riscv64-unknown-linux-gnu toolchain and without -DBAREMETAL / htif specs.
#   Output goes to build/linux/*.linux (+ *.dump).
LINUX_PREFIX  ?= riscv64-unknown-linux-gnu-
LINUX_CC      := $(LINUX_PREFIX)gcc
LINUX_OBJDUMP := $(LINUX_PREFIX)objdump

LINUX_COMMON_CFLAGS := -O2 -fno-common -fno-builtin-printf -march=rv64gc -mabi=lp64d -static
LINUX_CPU_CFLAGS     := $(LINUX_COMMON_CFLAGS)
LINUX_VANILLA_CFLAGS := $(LINUX_COMMON_CFLAGS) -I$(ROCC_TESTS)
LINUX_CUSTOM_CFLAGS  := $(LINUX_COMMON_CFLAGS) -I$(SOFTWARE) -I$(SOFTWARE)/include -I$(ROCC_TESTS)

LINUX_BUILD_DIR := build/linux
LINUX_TARGETS   := $(addprefix $(LINUX_BUILD_DIR)/,$(addsuffix .linux,$(PROGS)))
LINUX_DUMPS     := $(addprefix $(LINUX_BUILD_DIR)/,$(addsuffix .dump,$(PROGS)))

linux: $(LINUX_TARGETS) $(LINUX_DUMPS)

$(LINUX_BUILD_DIR)/rocket_cpu_conv.linux: rocket_cpu_conv.c conv_dims.h | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CPU_CFLAGS) $< -o $@ $(LDLIBS)

$(LINUX_BUILD_DIR)/vanilla_gemmini_conv.linux: vanilla_gemmini_conv.c conv_dims.h | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_VANILLA_CFLAGS) $< -o $@ $(LDLIBS)

$(LINUX_BUILD_DIR)/custom_gemmini_conv_profiler.linux: custom_gemmini_conv_profiler.c conv_dims.h | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CUSTOM_CFLAGS) $< -o $@ $(LDLIBS)

$(LINUX_BUILD_DIR)/rocket_cpu_mpgemm.linux: rocket_cpu_mpgemm.c mpgemm_dims.h | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CPU_CFLAGS) $< -o $@ $(LDLIBS)

$(LINUX_BUILD_DIR)/custom_gemmini_mpgemm_profiler.linux: custom_gemmini_mpgemm_profiler.c mpgemm_dims.h | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CUSTOM_CFLAGS) $< -o $@ $(LDLIBS)

$(LINUX_BUILD_DIR)/%.dump: $(LINUX_BUILD_DIR)/%.linux
	$(LINUX_OBJDUMP) -D $< > $@

$(LINUX_BUILD_DIR):
	mkdir -p $(LINUX_BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
