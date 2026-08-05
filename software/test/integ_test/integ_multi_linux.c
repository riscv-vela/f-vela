// See LICENSE for license details.
//
// integ_test multi-thread test -- LINUX/BUSYBOX variant.
//
// Runs gemmini + pid + rope concurrently as real pthreads. Unlike the
// bare-metal variant (integ_multi.c, cooperative fibers -- see fiber.h for
// why), this SoC still only backs one hart (WithNHugeCores(1)), but under
// Linux that's enough for genuine concurrency: the kernel preemptively
// time-slices the three threads, and (unlike the coprocessor-state hazard
// bare-metal fibers have to avoid by construction) it also saves/restores
// each thread's vector register file across context switches, so pid and
// rope can safely be preempted mid-vector-sequence by each other. gemmini's
// RoCC scratchpad/accumulator state is still only ever touched by the
// gemmini thread, so it's unaffected by preemption from the other two.
//
// Build with the riscv64-unknown-linux-gnu toolchain + -pthread (see
// Makefile `linux` target).

#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include "task_gemmini.h"
#include "task_pid.h"
#include "task_rope.h"

#define ITERATIONS 3

static void *gemmini_thread_main(void *arg) {
    (void)arg;
    for (int i = 0; i < ITERATIONS; i++) {
        printf("\n[thread:gemmini] --- iteration %d/%d ---\n", i + 1, ITERATIONS);
        gemmini_task_run();
    }
    printf("[thread:gemmini] done.\n");
    return NULL;
}

static void *pid_thread_main(void *arg) {
    (void)arg;
    for (int i = 0; i < ITERATIONS; i++) {
        printf("\n[thread:pid] --- iteration %d/%d ---\n", i + 1, ITERATIONS);
        pid_task_run();
    }
    printf("[thread:pid] done.\n");
    return NULL;
}

static void *rope_thread_main(void *arg) {
    (void)arg;
    for (int i = 0; i < ITERATIONS; i++) {
        printf("\n[thread:rope] --- iteration %d/%d ---\n", i + 1, ITERATIONS);
        rope_task_run();
    }
    printf("[thread:rope] done.\n");
    return NULL;
}

int main(void) {
    printf("==================================================\n");
    printf(" F-Vela integ_test: multi-thread (Linux/busybox, pthreads)\n");
    printf(" gemmini + pid + rope, %d iterations each\n", ITERATIONS);
    printf(" NOTE: this SoC config is single-hart (WithNHugeCores(1)), so the\n");
    printf(" kernel time-slices these threads rather than running them in true\n");
    printf(" hardware parallel; the point is to exercise the kernel's preemption\n");
    printf(" + vector-context-switch path across three ISA-extension workloads\n");
    printf(" at once, not to measure speedup.\n");
    printf("==================================================\n");

    pthread_t t_gemmini, t_pid, t_rope;
    int rc;

    rc = pthread_create(&t_gemmini, NULL, gemmini_thread_main, NULL);
    if (rc != 0) {
        fprintf(stderr, "pthread_create(gemmini) failed: %d\n", rc);
        exit(1);
    }

    rc = pthread_create(&t_pid, NULL, pid_thread_main, NULL);
    if (rc != 0) {
        fprintf(stderr, "pthread_create(pid) failed: %d\n", rc);
        exit(1);
    }

    rc = pthread_create(&t_rope, NULL, rope_thread_main, NULL);
    if (rc != 0) {
        fprintf(stderr, "pthread_create(rope) failed: %d\n", rc);
        exit(1);
    }

    pthread_join(t_gemmini, NULL);
    pthread_join(t_pid, NULL);
    pthread_join(t_rope, NULL);

    printf("\nAll threads finished. integ_multi_linux PASSED.\n");
    return 0;
}
