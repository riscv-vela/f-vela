########################################################################################################################
# software/test / Makefile  (top-level dispatcher)
#
#   Builds the gemmini, pid, rope, profiler and integ_test test folders by delegating to each folder's own Makefile.
#
#   usage:
#     make            # build all (gemmini + pid + rope + profiler + integ), bare-metal
#     make gemmini    # build only rv_gemmini_test
#     make pid        # build only rv_pid_test
#     make rope       # build only rv_rope_test
#     make profiler   # build only profiler_test
#     make integ      # build only integ_test (combined single-thread menu + multi-thread tests)
#     make linux      # build the Linux (firemarshal/busybox) userspace variants of all tests
#     make clean      # clean all
########################################################################################################################

.PHONY: all gemmini pid rope profiler integ linux clean

all: gemmini pid rope profiler integ

gemmini:
	$(MAKE) -C rv_gemmini_test

pid:
	$(MAKE) -C rv_pid_test

rope:
	$(MAKE) -C rv_rope_test

profiler:
	$(MAKE) -C profiler_test

integ:
	$(MAKE) -C integ_test

linux:
	$(MAKE) -C rv_gemmini_test linux
	$(MAKE) -C rv_pid_test     linux
	$(MAKE) -C rv_rope_test    linux
	$(MAKE) -C profiler_test   linux
	$(MAKE) -C integ_test      linux

clean:
	$(MAKE) -C rv_gemmini_test clean
	$(MAKE) -C rv_pid_test     clean
	$(MAKE) -C rv_rope_test    clean
	$(MAKE) -C profiler_test   clean
	$(MAKE) -C integ_test      clean
