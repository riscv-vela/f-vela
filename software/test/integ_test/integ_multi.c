// See LICENSE for license details.
//
// integ_test multi-thread test -- BARE METAL variant.
//
// Runs gemmini + pid + rope "concurrently" via the cooperative fiber
// scheduler in fiber.c/fiber_asm.S. See fiber.h for why fibers (not real
// threads) are used here: this SoC config is single-hart and bare metal has
// no OS/scheduler at all. Each workload gets its own stack and yields
// between iterations (never mid-RoCC-command / mid-vector-op), so the three
// interleave on the one hart without corrupting each other's state:
//   - gemmini's scratchpad/accumulator/config state is only ever touched by
//     the gemmini fiber, so interleaving other fibers around it is safe.
//   - pid and rope both use the vector register file, but a cooperative
//     switch only ever happens at an iteration boundary, after each task's
//     vector sequence has fully retired -- never mid-sequence -- so there's
//     no need to save/restore vector state across fiber_switch() at all.
//
// For the Linux/busybox counterpart (real pthreads, preemptive), see
// integ_multi_linux.c.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "fiber.h"
#include "task_gemmini.h"
#include "task_pid.h"
#include "task_rope.h"

#define ITERATIONS 3
#define FIBER_STACK_BYTES 8192

static uint8_t gemmini_stack[FIBER_STACK_BYTES] __attribute__((aligned(16)));
static uint8_t pid_stack[FIBER_STACK_BYTES]     __attribute__((aligned(16)));
static uint8_t rope_stack[FIBER_STACK_BYTES]    __attribute__((aligned(16)));

static void gemmini_fiber_main(void) {
    for (int i = 0; i < ITERATIONS; i++) {
        printf("\n[fiber:gemmini] --- iteration %d/%d ---\n", i + 1, ITERATIONS);
        gemmini_task_run();
        fiber_yield();
    }
    printf("[fiber:gemmini] done.\n");
}

static void pid_fiber_main(void) {
    for (int i = 0; i < ITERATIONS; i++) {
        printf("\n[fiber:pid] --- iteration %d/%d ---\n", i + 1, ITERATIONS);
        pid_task_run();
        fiber_yield();
    }
    printf("[fiber:pid] done.\n");
}

static void rope_fiber_main(void) {
    for (int i = 0; i < ITERATIONS; i++) {
        printf("\n[fiber:rope] --- iteration %d/%d ---\n", i + 1, ITERATIONS);
        rope_task_run();
        fiber_yield();
    }
    printf("[fiber:rope] done.\n");
}

int main(void) {
    printf("==================================================\n");
    printf(" F-Vela integ_test: multi-thread (bare-metal, cooperative fibers)\n");
    printf(" gemmini + pid + rope, %d iterations each, single hart\n", ITERATIONS);
    printf("==================================================\n");

    static fiber_t f_gemmini, f_pid, f_rope;
    fiber_init(&f_gemmini, gemmini_stack + sizeof(gemmini_stack), gemmini_fiber_main);
    fiber_init(&f_pid,     pid_stack     + sizeof(pid_stack),     pid_fiber_main);
    fiber_init(&f_rope,    rope_stack    + sizeof(rope_stack),    rope_fiber_main);

    fiber_t *all[3] = { &f_gemmini, &f_pid, &f_rope };
    fiber_run_all(all, 3);

    printf("\nAll fibers finished. integ_multi (bare-metal) PASSED.\n");
    exit(0);
}
