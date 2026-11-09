########################################################################################################################
# software/test / Makefile  (top-level dispatcher)
#
#   Builds the npu (vela_npu_test), pid, rope, profiler and integ_test test folders by delegating to each folder's own Makefile.
#
#   usage:
#     make            # build all (npu + pid + rope + profiler + integ + cmp), bare-metal
#     make npu        # build only vela_npu_test (velaNPU = custom Gemmini); 'make gemmini' is an alias
#     make pid        # build only vela_vpu_pid_test
#     make rope       # build only vela_vpu_rope_test
#     make profiler   # build only profiler_test
#     make integ      # build only integ_test (combined single-thread menu + multi-thread tests)
#     make cmp        # build only cmp_conv_cycle (Rocket CPU vs vanilla vs custom Gemmini conv)
#     make linux      # build the Linux (firemarshal/busybox) userspace variants of all tests
#     make clean      # clean all
########################################################################################################################

.PHONY: all npu gemmini pid rope profiler integ cmp linux clean

all: npu pid rope profiler integ cmp

npu gemmini:
	$(MAKE) -C vela_npu_test

pid:
	$(MAKE) -C vela_vpu_pid_test

rope:
	$(MAKE) -C vela_vpu_rope_test

profiler:
	$(MAKE) -C profiler_test

integ:
	$(MAKE) -C integ_test

cmp:
	$(MAKE) -C cmp_conv_cycle

linux:
	$(MAKE) -C vela_npu_test   linux
	$(MAKE) -C vela_vpu_pid_test     linux
	$(MAKE) -C vela_vpu_rope_test    linux
	$(MAKE) -C profiler_test   linux
	$(MAKE) -C integ_test      linux
	$(MAKE) -C cmp_conv_cycle  linux

clean:
	$(MAKE) -C vela_npu_test   clean
	$(MAKE) -C vela_vpu_pid_test     clean
	$(MAKE) -C vela_vpu_rope_test    clean
	$(MAKE) -C profiler_test   clean
	$(MAKE) -C integ_test      clean
	$(MAKE) -C cmp_conv_cycle  clean
