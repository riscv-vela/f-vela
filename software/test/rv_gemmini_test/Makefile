########################################################################################################################
# rv_gemmini_test / Makefile
#
#   Builds the Gemmini bare-metal tests into ./build/*.riscv (+ *.dump).
#
#   usage:
#     make            # build every test (bare-metal)
#     make gemm       # build a single test (target == source basename)
#     make linux      # build gemm + ternary_gemm for Linux (firemarshal/busybox) userspace
#     make clean
#
#   The RISC-V toolchain is added to PATH automatically from Chipyard's conda env,
#   so sourcing env.sh first is optional.
########################################################################################################################

# ---- Paths -----------------------------------------------------------------------------------------------------------
SELF_DIR   := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
SOFTWARE   := $(abspath $(SELF_DIR)/../..)
CHIPYARD   := $(abspath $(SELF_DIR)/../../../../..)
RISCV_BIN  := $(CHIPYARD)/.conda-env/riscv-tools/bin
export PATH := $(RISCV_BIN):$(PATH)

BUILD_DIR  := build

# ---- Toolchain -------------------------------------------------------------------------------------------------------
RISCV_PREFIX ?= riscv64-unknown-elf-
CC           := $(RISCV_PREFIX)gcc
OBJDUMP      := $(RISCV_PREFIX)objdump

# ---- Flags -----------------------------------------------------------------------------------------------------------
CFLAGS := -I$(SOFTWARE)/include -I$(SOFTWARE)/libgemmini \
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

# convenience: `make gemm` -> build gemm.riscv + gemm.dump
$(PROGS): %: $(BUILD_DIR)/%.riscv $(BUILD_DIR)/%.dump

$(BUILD_DIR)/%.riscv: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD_DIR)/%.dump: $(BUILD_DIR)/%.riscv
	$(OBJDUMP) -D $< > $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---- Linux userspace build (for firemarshal/busybox images) ----------------------------------------------------------
#   gemm.c / ternary_gemm.c are already dual-mode (see `#ifndef BAREMETAL` in each file), so the
#   same sources are reused here, just compiled with the riscv64-unknown-linux-gnu toolchain and
#   without -DBAREMETAL / htif specs. Output goes to build/linux/*.linux (+ *.dump).
LINUX_PREFIX  ?= riscv64-unknown-linux-gnu-
LINUX_CC      := $(LINUX_PREFIX)gcc
LINUX_OBJDUMP := $(LINUX_PREFIX)objdump

LINUX_BUILD_DIR := build/linux
LINUX_CFLAGS     := -I$(SOFTWARE)/include -I$(SOFTWARE)/libgemmini \
                     -O2 -fno-common -fno-builtin-printf \
                     -march=rv64gc -mabi=lp64d -static
LINUX_LDLIBS     := -lm

LINUX_PROGS   := gemm ternary_gemm
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
