## integ_test

Combines the `rv_gemmini_test`, `rv_pid_test`, `rv_rope_test` and `profiler_test` workloads from the
sibling test folders into two integration test programs, each built for both bare-metal (HTIF) and
Linux/busybox (firemarshal, FireSim+Alveo) userspace.

### integ_single -- single-thread menu test

Prints a menu on the console and waits for a keypress to run one test at a time, looping back to the menu
afterwards:

```
1) Gemmini tiled matmul test
2) PID vector custom-instruction test
3) RoPE vector custom-instruction test
4) Gemmini profiler test
q) Quit
```

"Console" here is the HTIF console under bare-metal / Verilator+FireSim sim, and the busybox tty
(UART-backed on the FireSim+Alveo target) under Linux.

### integ_multi -- multi-thread test (gemmini + pid + rope, 3 iterations each)

- **Bare-metal** (`integ_multi.c`): the `FireSimFVelaSoCConfig` / `FVelaGemminiOnlyConfig` SoC configs this
  repo builds for are single-hart (`WithNHugeCores(1)`), and bare metal has no OS/scheduler at all -- so
  there's no way to run real concurrent threads. Instead, `fiber.c` / `fiber_asm.S` implement a minimal
  cooperative fiber (coroutine) scheduler: each workload gets its own stack and voluntarily yields between
  iterations (never mid-RoCC-command or mid-vector-op), and a hand-written register-only context switch
  (`fiber_switch`) resumes the next one. This is concurrency by interleaving, not hardware parallelism --
  see the comment block in `fiber.h` for the full reasoning, including why it's safe for gemmini (only one
  fiber ever touches the accelerator) and pid/rope (vector state never needs saving across a switch, since
  switches only happen once a vector sequence has fully retired).
- **Linux/busybox** (`integ_multi_linux.c`, built via `make linux`): real `pthread_create`/`pthread_join`.
  The kernel preemptively time-slices the three threads on that same single hart, and does save/restore
  vector register state across context switches, so pid and rope can safely preempt each other
  mid-sequence. gemmini's RoCC scratchpad/accumulator/config state is still only touched by the gemmini
  thread, so it's unaffected either way.

### Layout

- `task_gemmini.c/.h`, `task_profiler.c/.h`, `task_pid.c/.h`, `task_rope.c/.h` -- the four workloads,
  ported unchanged from their original standalone test files into callable `*_task_run()` functions (see
  each file's header comment for which original `.c` it came from). Dual-mode like
  `rv_gemmini_test/gemm.c`: the same source builds for both bare-metal and Linux.
- `fiber.h/.c`, `fiber_asm.S` -- the bare-metal-only cooperative scheduler used by `integ_multi.c`.
- `integ_single.c` -- dual-mode single-thread menu.
- `integ_multi.c` / `integ_multi_linux.c` -- bare-metal (fibers) / Linux (pthreads) multi-thread mains.

### Usage

```
make            # build integ_single.riscv + integ_multi.riscv (bare-metal)
make single     # build only integ_single.riscv
make multi      # build only integ_multi.riscv
make linux      # build integ_single.linux + integ_multi_linux.linux (Linux/busybox userspace)
make clean
```

Or from `software/test/`: `make integ` (bare-metal) / `make linux` (includes this folder's Linux build too),
alongside the existing `gemmini` / `pid` / `rope` / `profiler` targets.
