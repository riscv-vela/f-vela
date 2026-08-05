########################################################################################################################
# integ_test / Makefile
#
#   Integrates the gemmini, pid, rope and profiler tests from the sibling test folders into two combined test
#   programs, each built for both bare-metal (HTIF) and Linux/busybox (firemarshal) userspace:
#
#     integ_single  - single-thread menu: press 1/2/3/4 on the console (HTIF console under bare-metal /
#                      Verilator+FireSim sim; UART/tty console under FireSim+Alveo busybox Linux) to run
#                      gemmini / pid / rope / profiler in turn, one at a time, looping back to the menu
#                      afterwards. 'q' quits.
#
#     integ_multi   - runs gemmini + pid + rope "at once":
#                        * bare-metal build (integ_multi.c): cooperative fiber scheduler (fiber.c/fiber_asm.S),
#                          since this SoC config is single-hart and bare metal has no OS scheduler at all.
#                        * `make linux` build (integ_multi_linux.c): real pthreads, preemptively time-sliced
#                          by the kernel on that same single hart.
#
#   The four accelerator workloads themselves are ported unchanged (see task_gemmini.c / task_profiler.c /
#   task_pid.c / task_rope.c -- each cites the original file it came from) into callable `*_task_run()`
#   functions so both integ_single and integ_multi can call into them.
#
#   usage:
#     make            # build integ_single.riscv + integ_multi.riscv (bare-metal)
#     make single     # build only integ_single.riscv
#     make multi      # build only integ_multi.riscv
#     make linux      # build integ_single.linux + integ_multi_linux.linux (Linux/busybox userspace)
#     make clean
#
#   The RISC-V toolchain is added to PATH automatically from Chipyard's conda env.
########################################################################################################################

# ---- Paths -----------------------------------------------------------------------------------------------------------
SELF_DIR   := $(patsubst %/,%,$(dir $(abspath $(lastword $(MAKEFILE_LIST)))))
SOFTWARE   := $(abspath $(SELF_DIR)/../..)
CHIPYARD   := $(abspath $(SELF_DIR)/../../../../..)
RISCV_BIN  := $(CHIPYARD)/.conda-env/riscv-tools/bin
# gemmini-rocc-tests provides rocc-software/src/xcustom.h that task_gemmini.c / task_profiler.c expect
ROCC_TESTS := $(CHIPYARD)/generators/gemmini/software/gemmini-rocc-tests
export PATH := $(RISCV_BIN):$(PATH)

BUILD_DIR  := build

# ---- Toolchain -------------------------------------------------------------------------------------------------------
RISCV_PREFIX ?= riscv64-unknown-elf-
CC           := $(RISCV_PREFIX)gcc
OBJDUMP      := $(RISCV_PREFIX)objdump

# ---- Flags -----------------------------------------------------------------------------------------------------------
#   -I.                        -> task_*.h / fiber.h
#   -I$(SOFTWARE)               -> resolves "include/gemmini_params.h" etc. (pf_gemmini.h, see task_profiler.c)
#   -I$(SOFTWARE)/include       -> resolves gemmini_testutils.h, pf_gemmini.h, fv_gemmini.h
#   -I$(SOFTWARE)/libgemmini, -I$(ROCC_TESTS) -> resolve xcustom.h, mirrors rv_gemmini_test / profiler_test Makefiles
#   rv64gcv/lp64d/medany        -> superset covering gemmini (rv64gc), pid (rv64imafdv) and rope (rv64gcv) sources
INCLUDES := -I. -I$(SOFTWARE) -I$(SOFTWARE)/include -I$(SOFTWARE)/libgemmini -I$(ROCC_TESTS)
CFLAGS   := -march=rv64gcv -mabi=lp64d -mcmodel=medany -O2 \
            -fno-common -fno-builtin-printf \
            -specs=htif_nano.specs -DBAREMETAL \
            $(INCLUDES)
# -Wl,-u,_printf_float: force-link nano-libc's float printf support (task_pid.c prints %.2f floats)
# -Wl,--allow-multiple-definition: task_gemmini.c (via gemmini_testutils.h -> fv_gemmini.h) and
#   task_profiler.c (via pf_gemmini.h) both pull in a non-static `int ceil_divide_int(...)` -- identical
#   bodies in both headers, just duplicated rather than shared, so it's safe to let the linker pick either.
LDFLAGS := -static -Wl,-u,_printf_float -Wl,--allow-multiple-definition
LDLIBS  := -lm

# ---- Sources -----------------------------------------------------------------------------------------------------------
TASK_SRCS       := task_gemmini.c task_profiler.c task_pid.c task_rope.c
SINGLE_SRCS     := integ_single.c $(TASK_SRCS)
MULTI_TASK_SRCS := task_gemmini.c task_pid.c task_rope.c    # multi-thread test: gemmini + pid + rope only
MULTI_C_SRCS    := integ_multi.c fiber.c $(MULTI_TASK_SRCS)
MULTI_ASM_SRCS  := fiber_asm.S

SINGLE_OBJS := $(addprefix $(BUILD_DIR)/,$(SINGLE_SRCS:.c=.o))
MULTI_OBJS  := $(addprefix $(BUILD_DIR)/,$(MULTI_C_SRCS:.c=.o)) $(addprefix $(BUILD_DIR)/,$(MULTI_ASM_SRCS:.S=.o))

.PHONY: all single multi clean linux
all: single multi

single: $(BUILD_DIR)/integ_single.riscv $(BUILD_DIR)/integ_single.dump
multi:  $(BUILD_DIR)/integ_multi.riscv  $(BUILD_DIR)/integ_multi.dump

$(BUILD_DIR)/integ_single.riscv: $(SINGLE_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD_DIR)/integ_multi.riscv: $(MULTI_OBJS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(BUILD_DIR)/%.o: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.o: %.S | $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD_DIR)/%.dump: $(BUILD_DIR)/%.riscv
	$(OBJDUMP) -D $< > $@

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---- Linux userspace build (for firemarshal/busybox images) ----------------------------------------------------------
#   Reuses the exact same task_*.c / integ_single.c sources (they're dual-mode, like rv_gemmini_test/gemm.c),
#   just compiled with the riscv64-unknown-linux-gnu toolchain and without -DBAREMETAL / htif specs.
#   integ_multi_linux.c (pthreads) replaces integ_multi.c (fibers) + fiber.c/fiber_asm.S for this build.
LINUX_PREFIX  ?= riscv64-unknown-linux-gnu-
LINUX_CC      := $(LINUX_PREFIX)gcc
LINUX_OBJDUMP := $(LINUX_PREFIX)objdump

LINUX_BUILD_DIR := build/linux
LINUX_CFLAGS     := -march=rv64gcv -mabi=lp64d -O2 -static \
                     -fno-common -fno-builtin-printf \
                     $(INCLUDES)
LINUX_LDFLAGS    := -static -pthread -Wl,--allow-multiple-definition
LINUX_LDLIBS     := -lm

LINUX_SINGLE_SRCS := integ_single.c $(TASK_SRCS)
LINUX_MULTI_SRCS  := integ_multi_linux.c $(MULTI_TASK_SRCS)

LINUX_SINGLE_OBJS := $(addprefix $(LINUX_BUILD_DIR)/,$(LINUX_SINGLE_SRCS:.c=.o))
LINUX_MULTI_OBJS  := $(addprefix $(LINUX_BUILD_DIR)/,$(LINUX_MULTI_SRCS:.c=.o))

linux: $(LINUX_BUILD_DIR)/integ_single.linux $(LINUX_BUILD_DIR)/integ_single.dump \
       $(LINUX_BUILD_DIR)/integ_multi_linux.linux $(LINUX_BUILD_DIR)/integ_multi_linux.dump

$(LINUX_BUILD_DIR)/integ_single.linux: $(LINUX_SINGLE_OBJS) | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CFLAGS) $(LINUX_LDFLAGS) $^ -o $@ $(LINUX_LDLIBS)

$(LINUX_BUILD_DIR)/integ_multi_linux.linux: $(LINUX_MULTI_OBJS) | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CFLAGS) $(LINUX_LDFLAGS) $^ -o $@ $(LINUX_LDLIBS)

$(LINUX_BUILD_DIR)/%.o: %.c | $(LINUX_BUILD_DIR)
	$(LINUX_CC) $(LINUX_CFLAGS) -c $< -o $@

$(LINUX_BUILD_DIR)/%.dump: $(LINUX_BUILD_DIR)/%.linux
	$(LINUX_OBJDUMP) -D $< > $@

$(LINUX_BUILD_DIR):
	mkdir -p $(LINUX_BUILD_DIR)

clean:
	rm -rf $(BUILD_DIR)
