// See LICENSE for license details.
//
// Gemmini profiler workload, ported from profiler_test/gemm_profiler_test.c
// into a callable task (`profiler_task_run`). Behavior is unchanged; only
// entry/exit points differ (no `main`/`exit`).
//
// `full_printMatrix` / `init_mats` / `read_cycles` / `type_name` /
// `dump_profile` are `static` here so they don't collide with the
// same-named helpers in task_gemmini.c when both are linked into the same
// integ_test binary.
//
// NOTE: pf_gemmini.h (unlike gemmini_testutils.h, which task_gemmini.c uses)
// does not shadow libc's rand(), so init_mats() below normally draws from
// libc's rand(). If this file is linked together with task_gemmini.c (as it
// is in every integ_test binary), task_gemmini.c's gemmini_testutils.h pulls
// in a global (non-static) deterministic rand() replacement that wins at
// link time for the whole binary -- harmless here since both callers only
// want "some" pseudo-random elem_t values, but worth knowing if you go
// looking for libc's rand() and don't find it in the disassembly.

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "pf_gemmini.h"

#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "task_profiler.h"

// ---- Workload dimensions (single small matmul) ----------------------------------------
#define PRINT 1

#define ACTIVATION NO_ACTIVATION

#define NO_BIAS 1
#define REPEATING_BIAS 1

#define A_TRANSPOSE 0
#define B_TRANSPOSE 0

#define MAT_DIM_I 1
#define MAT_DIM_K 64
#define MAT_DIM_J 64

#define A_STRIDE MAT_DIM_K //The number of A's column
#define B_STRIDE MAT_DIM_J //The number of B's column
#define D_STRIDE MAT_DIM_J //The number of D's column
#define C_STRIDE MAT_DIM_J //The number of C's column

static uint64_t read_cycles(void) {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// ---- Profile buffer ---------------------------------------------------------------------------------------------------
// Max number of profile events we can capture. Each entry is 8 bytes. Zero-initialised
// (in .bss), so a value of 0 marks an unused slot.
#define PROFILE_MAX_ENTRIES 4096
static volatile uint64_t P[PROFILE_MAX_ENTRIES] row_align(1);

#define PROFILE_TYPE(e)  ((int)     (((e) >> 62) & 0x3ULL))
#define PROFILE_START(e) ((uint32_t)(((e) >> 31) & 0x7FFFFFFFULL))
#define PROFILE_END(e)   ((uint32_t)( (e)        & 0x7FFFFFFFULL))

static const char *type_name(int t) {
    switch (t) {
        case 0:  return "ld";
        case 1:  return "ex";
        case 2:  return "st";
        default: return "??";
    }
}

// Dump captured profile events. Everything between the two markers is the
// data written by the profiler DMA into P[].
static void dump_profile(void) {
    printf("=== PROFILE DUMP BEGIN ===\n");
    int count = 0;
    for (int i = 0; i < PROFILE_MAX_ENTRIES; i++) {
        uint64_t e = P[i];
        if (e == 0) break;   // first empty slot => end of records
        printf("%d, %u, %u\n", PROFILE_TYPE(e), PROFILE_START(e), PROFILE_END(e));
        count++;
    }
    printf("=== PROFILE DUMP END ===\n");
    printf("Captured %d profile events (0=ld,1=ex,2=st).\n", count);
}

static void full_printMatrix(elem_t m[MAT_DIM_I][MAT_DIM_J]) {
    for (size_t i = 0; i < MAT_DIM_I; ++i) {
        for (size_t j = 0; j < MAT_DIM_J; ++j)
            printf("%d ", m[i][j]);
        printf("\n");
    }
}

static void init_mats(elem_t A[MAT_DIM_I][MAT_DIM_K], elem_t B[MAT_DIM_K][MAT_DIM_J], acc_t D[MAT_DIM_I][MAT_DIM_J]) {
    for (size_t i = 0; i < MAT_DIM_I; ++i)
        for (size_t k = 0; k < MAT_DIM_K; ++k)
            A[i][k] = rand() % 2; // 0 또는 1

    for (size_t k = 0; k < MAT_DIM_K; ++k) {
        for (size_t j = 0; j < MAT_DIM_J; ++j) {
            B[k][j] = rand() % 3 - 1;
        }
    }

    for (size_t i = 0; i < MAT_DIM_I; ++i)
        for (size_t j = 0; j < MAT_DIM_J; ++j)
            D[i][j] = NO_BIAS ? 0 : rand(); // 0 또는 1
}

void profiler_task_run(void) {
#ifndef BAREMETAL
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall failed");
        return;
    }
#endif

    printf("==============================================\n");
    printf("  Gemmini Profiler Test\n");
    printf("  Matmul: I=%d J=%d K=%d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
    printf("  P buffer @ %p (%d entries)\n", (void *)P, PROFILE_MAX_ENTRIES);
    printf("==============================================\n");

    gemmini_flush(0);

    // Operands
    static elem_t full_A[MAT_DIM_I][MAT_DIM_K] row_align(1);
    static elem_t full_B[MAT_DIM_K][MAT_DIM_J] row_align(1);
    static elem_t full_C[MAT_DIM_I][MAT_DIM_J] row_align(1);
    static acc_t full_D[MAT_DIM_I][MAT_DIM_J] row_align_acc(1);

    init_mats(full_A, full_B, full_D);

    counter_configure(0, RDMA_BYTES_REC);
    counter_configure(1, WDMA_BYTES_SENT);
    counter_reset();

    // Register the profile buffer with the accelerator.
    printf("Setting profiler address...\n");
    printf("Starting gemmini matmul\n");
    printf("I: %d, J: %d, K: %d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
    printf("NO_BIAS: %d, REPEATING_BIAS: %d\n", NO_BIAS, REPEATING_BIAS);
    printf("A_TRANSPOSE: %d, B_TRANSPOSE: %d\n", A_TRANSPOSE, B_TRANSPOSE);

    gemmini_profiler(P);

    uint64_t start = read_cycles();

    tiled_matmul_auto(MAT_DIM_I, MAT_DIM_J, MAT_DIM_K,
            (elem_t*)full_A, (elem_t*)full_B, NO_BIAS ? NULL : &full_D[0][0], full_C,
            A_STRIDE, B_STRIDE, D_STRIDE, C_STRIDE,
            MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
            ACTIVATION, 1.0, 0, REPEATING_BIAS,
            false, B_TRANSPOSE,
            false, false, 0, WS);                   // full_C, low_D

    gemmini_fence();

    uint64_t end = read_cycles();
    printf("Cycles taken: %llu\n", end-start);

    const uint64_t total_macs = MAT_DIM_I * MAT_DIM_J * MAT_DIM_K;
    const uint64_t ideal_cycles = total_macs / (DIM * DIM);
    const uint64_t utilization = 100 * ideal_cycles / (end-start);
    printf("Total macs: %llu\n", total_macs);
    printf("Ideal cycles: %llu\n", ideal_cycles);
    printf("Utilization: %llu%%\n", utilization);

    printf("RDMA_BYTES_REC: %u\n", counter_read(0));
    printf("WDMA_BYTES_SENT: %u\n", counter_read(1));

    // Dump the profiler-captured events (consumed by plot/profile.py).
    dump_profile();

#ifdef PRINT
    printf("\n");
    printf("C:\n");
    full_printMatrix(full_C);
    printf("\n");
    printf("A:\n");
    full_printMatrix(full_A);
    printf("\n");
    printf("B:\n");
    full_printMatrix(full_B);
    printf("\n");
    printf("D:\n");
    full_printMatrix(full_D);
#endif

    printf("PROFILER TEST DONE\n");
}
