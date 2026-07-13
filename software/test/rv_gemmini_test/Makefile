########################################################################################################################
# rv_gemmini_test / Makefile
#
#   Builds the Gemmini bare-metal tests into ./build/*.riscv (+ *.dump).
#
#   usage:
#     make            # build every test
#     make gemm       # build a single test (target == source basename)
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

.PHONY: all clean $(PROGS)
all: $(TARGETS) $(DUMPS)

# convenience: `make gemm` -> build gemm.riscv + gemm.dump
$(PROGS): %: $(BUILD_DIR)/%.riscv $(BUILD_DIR)/%.dump

$(BUILD_DIR)/%.riscv: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@ $(LDLIBS)

$(BUILD_DIR)/%.dump: $(BUILD_DIR)/%.riscv
	$(OBJDUMP) -D $< > $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
