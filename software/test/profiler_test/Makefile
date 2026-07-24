########################################################################################################################
# profiler_test / Makefile
#
#   Builds the Gemmini profiler test into ./build/*.riscv (+ *.dump).
#
#   usage:
#     make            # build every test (bare-metal)
#     make linux      # build gemm_profiler_test + ternary_profiler_test for Linux (firemarshal/busybox)
#     make clean
#
#   Unlike the plain gemmini tests, this test includes pf_gemmini.h, which pulls in
#   headers via "include/..." and "rocc-software/src/..." paths. Hence the extra -I roots.
#
#   The RISC-V toolchain is added to PATH automatically from Chipyard's conda env.
########################################################################################################################

# ---- Paths -----------------------------------------------------------------------------------------------------------
SELF_DIR    := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
SOFTWARE    := $(abspath $(SELF_DIR)/../..)
CHIPYARD    := $(abspath $(SELF_DIR)/../../../../..)
RISCV_BIN   := $(CHIPYARD)/.conda-env/riscv-tools/bin
# gemmini-rocc-tests provides rocc-software/src/xcustom.h that pf_gemmini.h expects
ROCC_TESTS  := $(CHIPYARD)/generators/gemmini/software/gemmini-rocc-tests
export PATH := $(RISCV_BIN):$(PATH)

BUILD_DIR  := build

# ---- Toolchain -------------------------------------------------------------------------------------------------------
RISCV_PREFIX ?= riscv64-unknown-elf-
CC           := $(RISCV_PREFIX)gcc
OBJDUMP      := $(RISCV_PREFIX)objdump

# ---- Flags -----------------------------------------------------------------------------------------------------------
#   -I$(SOFTWARE)         -> resolves "include/gemmini_params.h", "include/gemmini_counter.h"
#   -I$(SOFTWARE)/include -> resolves "pf_gemmini.h" and the plain-named headers
#   -I$(ROCC_TESTS)       -> resolves "rocc-software/src/xcustom.h"
CFLAGS := -I$(SOFTWARE) -I$(SOFTWARE)/include -I$(ROCC_TESTS) \
          -O2 -fno-common -fno-builtin-printf \
          -specs=htif_nano.specs -march=rv64gc -mabi=lp64d \
          -DBAREMETAL
LDLIBS := -lm

# ---- Sources / targets -----------------------------------------------------------------------------------------------
SRCS    := $(wildcard *.c)
PROGS   := $(basename $(SRCS))
TARGETS := $(addprefix $(BUILD_DIR)/,$(addsuffix .riscv,$(PROGS)))
DUMPS   := $(addprefix $(BUILD_DIR)/,$(addsuffix .dump,$(PROGS)))

.PHONY: all clean linux $(PROGS)
all: $(TARGETS) $(DUMPS)

$(PROGS): %: $(BUILD_DIR)/%.riscv $(BUILD_DIR)/%.dump

$(BUILD_DIR)/%.riscv: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD_DIR)/%.dump: $(BUILD_DIR)/%.riscv
	$(OBJDUMP) -D $< > $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---- Linux userspace build (for firemarshal/busybox images) ----------------------------------------------------------
#   Both profiler tests are already dual-mode (see `#ifndef BAREMETAL` in each file), so the same
#   sources are reused here, just compiled with the riscv64-unknown-linux-gnu toolchain and without
#   -DBAREMETAL / htif specs. Output goes to build/linux/*.linux (+ *.dump).
LINUX_PREFIX  ?= riscv64-unknown-linux-gnu-
LINUX_CC      := $(LINUX_PREFIX)gcc
LINUX_OBJDUMP := $(LINUX_PREFIX)objdump

LINUX_BUILD_DIR := build/linux
LINUX_CFLAGS     := -I$(SOFTWARE) -I$(SOFTWARE)/include -I$(ROCC_TESTS) \
                     -O2 -fno-common -fno-builtin-printf \
                     -march=rv64gc -mabi=lp64d -static
LINUX_LDLIBS     := -lm

LINUX_PROGS   := gemm_profiler_test ternary_profiler_test
LINUX_TARGETS := $(addprefix $(LINUX_BUILD_DIR)/,$(addsuffix .linux,$(LINUX_PROGS)))
LINUX_DUMPS   := $(addprefix $(LINUX_BUILD_DIR)/,$(addsuffix .dump,$(LINUX_PROGS)))

linux: $(LINUX_TARGETS) $(LINUX_DUMPS)

$(LINUX_BUILD_DIR)/%.linux: %.c | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CFLAGS) $< -o $@ $(LINUX_LDLIBS)

$(LINUX_BUILD_DIR)/%.dump: $(LINUX_BUILD_DIR)/%.linux
	$(LINUX_OBJDUMP) -D $< > $@

$(LINUX_BUILD_DIR):
	mkdir -p $(LINUX_BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
