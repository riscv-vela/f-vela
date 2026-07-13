########################################################################################################################
# rv_pid_test / Makefile
#
#   Builds the PID bare-metal tests into ./build/*.riscv (+ *.dump).
#   Mirrors the flags in this folder's CMakeLists.txt (rv64imafdv / lp64d, htif specs).
#
#   usage:
#     make            # build every test
#     make vfpid_4f   # build a single test (target == source basename)
#     make clean
#
#   The RISC-V toolchain is added to PATH automatically from Chipyard's conda env.
########################################################################################################################

# ---- Paths -----------------------------------------------------------------------------------------------------------
SELF_DIR   := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
CHIPYARD   := $(abspath $(SELF_DIR)/../../../../..)
RISCV_BIN  := $(CHIPYARD)/.conda-env/riscv-tools/bin
export PATH := $(RISCV_BIN):$(PATH)

BUILD_DIR  := build

# ---- Toolchain -------------------------------------------------------------------------------------------------------
RISCV_PREFIX ?= riscv64-unknown-elf-
CC           := $(RISCV_PREFIX)gcc
OBJDUMP      := $(RISCV_PREFIX)objdump

# ---- Flags -----------------------------------------------------------------------------------------------------------
ARCH   := rv64imafdv
ABI    := lp64d
CMODEL := medany

CFLAGS  := -std=gnu99 -O2 -Wall -Wextra \
           -fno-common -fno-builtin-printf \
           -march=$(ARCH) -mabi=$(ABI) -mcmodel=$(CMODEL) \
           -specs=htif_nano.specs
# NOTE: htif_nano.specs is already in CFLAGS; adding it again here breaks the
# single-step compile+link ("spec 'link' already defined"), so it is not repeated.
LDFLAGS := -static -Wl,-u,_printf_float

# ---- Sources / targets -----------------------------------------------------------------------------------------------
SRCS    := $(wildcard *.c)
PROGS   := $(basename $(SRCS))
TARGETS := $(addprefix $(BUILD_DIR)/,$(addsuffix .riscv,$(PROGS)))
DUMPS   := $(addprefix $(BUILD_DIR)/,$(addsuffix .dump,$(PROGS)))

.PHONY: all clean $(PROGS)
all: $(TARGETS) $(DUMPS)

$(PROGS): %: $(BUILD_DIR)/%.riscv $(BUILD_DIR)/%.dump

$(BUILD_DIR)/%.riscv: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $< -o $@

$(BUILD_DIR)/%.dump: $(BUILD_DIR)/%.riscv
	$(OBJDUMP) -D $< > $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
