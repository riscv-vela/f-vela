########################################################################################################################
# rv_rope_test / Makefile
#
#   Builds the RoPE tests into ./build/:
#     - vfrope_test.riscv       : C test (htif bare-metal, rv64gcv)   <- from vfrope_test.c
#     - vfrope_asm.riscv        : hand-written asm test               <- from vfrope_test.S + test2.ld
#   plus matching *.dump disassemblies.
#
#   usage:
#     make            # build C + asm tests (and LUTs if make_lut.py exists)
#     make lut        # (re)generate lutA.hex / lutB.hex via make_lut.py
#     make linux      # build *_linux.c sources for Linux (firemarshal/busybox) userspace
#     make clean
#
#   The RISC-V toolchain is added to PATH automatically from Chipyard's conda env.
########################################################################################################################

# ---- Paths -----------------------------------------------------------------------------------------------------------
SELF_DIR      := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
CHIPYARD      := $(abspath $(SELF_DIR)/../../../../..)
RISCV_BIN     := $(CHIPYARD)/.conda-env/riscv-tools/bin
VERILATOR_DIR := $(CHIPYARD)/sims/verilator
export PATH   := $(RISCV_BIN):$(PATH)

BUILD_DIR  := build

# ---- Toolchain -------------------------------------------------------------------------------------------------------
RISCV_PREFIX ?= riscv64-unknown-elf-
CC           := $(RISCV_PREFIX)gcc
OBJDUMP      := $(RISCV_PREFIX)objdump
PYTHON       ?= python3

# ---- Flags -----------------------------------------------------------------------------------------------------------
# C test: full bare-metal via htif specs
CFLAGS   := -march=rv64gcv -mabi=lp64d -O2 -static -mcmodel=medany \
            -fno-common -fno-builtin-printf -specs=htif_nano.specs
CLDLIBS  := -lm

# Asm test: standalone _start + custom linker script, no libc/startfiles
ASMLD    := test2.ld
ASFLAGS  := -march=rv64gcv -mabi=lp64d -nostdlib -nostartfiles -T $(ASMLD)

# ---- Sources / targets -----------------------------------------------------------------------------------------------
# *_linux.c sources are built separately by `make linux` below, so they're excluded here.
SRCS       := $(filter-out %_linux.c,$(wildcard *.c))
PROGS      := $(basename $(SRCS))
ASM_TARGET := $(BUILD_DIR)/vfrope_asm.riscv
TARGETS    := $(addprefix $(BUILD_DIR)/,$(addsuffix .riscv,$(PROGS)))
DUMPS      := $(addprefix $(BUILD_DIR)/,$(addsuffix .dump,$(PROGS)))

.PHONY: all lut copy-lut clean linux $(PROGS)
all: lut copy-lut $(TARGETS) $(DUMPS)

# LUT generation is best-effort: skip cleanly if the script/interpreter is absent.
lut:
	@if [ -f make_lut.py ]; then \
		echo "Generating LUTs via make_lut.py"; \
		$(PYTHON) make_lut.py || echo "warning: make_lut.py failed, continuing"; \
	fi

# Copy the LUT hex files into the Verilator sim directory so the hardware's
# loadMemoryFromFileInline can find them (mirrors run.sh).
copy-lut:
	@if ls *.hex >/dev/null 2>&1; then \
		if [ -d "$(VERILATOR_DIR)" ]; then \
			echo "Copying LUT hex files to $(VERILATOR_DIR)"; \
			cp -f *.hex "$(VERILATOR_DIR)/"; \
		else \
			echo "warning: Verilator dir not found ($(VERILATOR_DIR)), skipping LUT copy"; \
		fi; \
	fi

$(PROGS): %: $(BUILD_DIR)/%.riscv $(BUILD_DIR)/%.dump

$(BUILD_DIR)/%.riscv: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $< -o $@

$(ASM_TARGET): vfrope_test.S $(ASMLD) | $(BUILD_DIR)
	$(CC) $(ASFLAGS) $< -o $@

$(BUILD_DIR)/%.dump: $(BUILD_DIR)/%.riscv
	$(OBJDUMP) -D $< > $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---- Linux userspace build (for firemarshal/busybox images) ----------------------------------------------------------
#   Builds *_linux.c sources against the riscv64-unknown-linux-gnu toolchain (Linux ABI, no htif
#   specs) into ./build/linux/*.linux (+ *.dump). vfrope_test2_linux.c drops the explicit
#   `csrs mstatus` vector-enable (M-mode only) since Linux enables vector state lazily per process.
LINUX_PREFIX  ?= riscv64-unknown-linux-gnu-
LINUX_CC      := $(LINUX_PREFIX)gcc
LINUX_OBJDUMP := $(LINUX_PREFIX)objdump

LINUX_BUILD_DIR := build/linux
LINUX_CFLAGS     := -march=rv64gcv -mabi=lp64d -O2 -static

LINUX_SRCS    := $(wildcard *_linux.c)
LINUX_PROGS   := $(basename $(LINUX_SRCS))
LINUX_TARGETS := $(addprefix $(LINUX_BUILD_DIR)/,$(addsuffix .linux,$(LINUX_PROGS)))
LINUX_DUMPS   := $(addprefix $(LINUX_BUILD_DIR)/,$(addsuffix .dump,$(LINUX_PROGS)))

linux: $(LINUX_TARGETS) $(LINUX_DUMPS)

$(LINUX_BUILD_DIR)/%.linux: %.c | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CFLAGS) $< -o $@

$(LINUX_BUILD_DIR)/%.dump: $(LINUX_BUILD_DIR)/%.linux
	$(LINUX_OBJDUMP) -D $< > $@

$(LINUX_BUILD_DIR):
	mkdir -p $(LINUX_BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
