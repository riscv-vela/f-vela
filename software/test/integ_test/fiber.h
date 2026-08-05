// See LICENSE for license details.
#ifndef INTEG_TEST_FIBER_H
#define INTEG_TEST_FIBER_H

#include <stdint.h>

// Minimal cooperative fiber (coroutine) scheduler -- bare-metal only.
//
// WHY THIS EXISTS: the F-Vela SoC configs used for the FireSim/Alveo target
// (FireSimFVelaSoCConfig -> chipyard.FVelaSoCConfigTest) and for the plain
// bare-metal HTIF sim are both single-hart
// (freechips.rocketchip.rocket.WithNHugeCores(1)). Bare metal additionally
// has no OS, so there is no scheduler available to time-slice real threads
// across even that one hart. To still exercise gemmini + pid + rope
// "concurrently" in the bare-metal multi-thread test (integ_multi.c), this
// file implements cooperative fibers: each workload gets its own stack and
// voluntarily yields at well-defined points (between iterations, never
// mid-RoCC-command or mid-vector-op), and fiber_switch() hand-swaps the
// integer callee-saved register set to resume the next one. This is
// concurrency by convention (interleaving), not parallelism -- there is
// still only one hart. Under Linux (integ_multi_linux.c), real pthreads are
// used instead, since the kernel *can* preemptively time-slice threads on a
// single hart.
typedef struct {
    uint64_t ra, sp;
    uint64_t s[12];
} fiber_regs_t;

typedef void (*fiber_entry_t)(void);

typedef struct {
    fiber_regs_t regs;
    fiber_entry_t entry;
    int done;
} fiber_t;

// Implemented in fiber_asm.S. Saves the caller's integer callee-saved register
// state (ra, sp, s0-s11) into *from, then restores the same set from *to and
// resumes execution there. A plain leaf-style register swap -- it does not
// touch FP/vector state, so callers must never switch away in the middle of
// a float/vector instruction sequence (only between complete task
// iterations; see integ_multi.c).
void fiber_switch(fiber_regs_t *from, fiber_regs_t *to);

// Prepare `f` to start running `entry` the first time it is switched to.
// `stack_top` must point one-past-the-end of a 16-byte-aligned stack region
// (i.e. `stack_array + array_len`, growing down from there).
void fiber_init(fiber_t *f, void *stack_top, fiber_entry_t entry);

// Round-robin-run every fiber in `fibers[0..n)` to completion, returning
// once all of them have returned from their entry function. Must be called
// from the main ("boot") context, not from within a fiber.
void fiber_run_all(fiber_t *fibers[], int n);

// Called from within a running fiber to hand control to the next runnable
// fiber (round-robin), or back to fiber_run_all()'s caller once every fiber
// has finished. A no-op if this fiber is the only one still runnable.
void fiber_yield(void);

#endif
