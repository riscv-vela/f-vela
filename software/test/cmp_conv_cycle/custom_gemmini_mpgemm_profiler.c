// Custom (F-Vela) Gemmini mpgemm (int8 x ternary matmul), timed with the real HW profiler.
//
// Based on profiler_test/ternary_profiler_test.c (which is itself the profiler-instrumented
// version of rv_gemmini_test/ternary_gemm.c), but reworked to match the RESULT/LD/EX/ST output
// conventions used across this folder (rocket_cpu_mpgemm.c, vanilla_gemmini_conv.c,
// custom_gemmini_conv_profiler.c) so run_cmp.sh can pull all of them into one table.
//
// There is no vanilla-Gemmini counterpart for this workload: ternary_gemm_auto only exists in
// the F-Vela headers (fv_gemmini.h / pf_gemmini.h) -- upstream gemmini.h has no ternary/mixed-
// precision matmul instruction. So mpgemm is a 2-way comparison (Rocket CPU vs. custom
// Gemmini), unlike conv's 3-way one.
//
// Also configures the same counter_configure() active-cycle counters vanilla_gemmini_conv.c
// uses, printed alongside the profiler-summed LD/EX/ST -- see custom_gemmini_conv_profiler.c's
// header for why that's worth checking on the ternary-capable systolic array.
//
// Run this on the FVelaSoCConfigTest sim (Rocket + f_vela_gemmini.GemminiCustomConfig).

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "pf_gemmini.h"

#include "mpgemm_dims.h"

#define NO_BIAS 1

// Same FNV-1a checksum rocket_cpu_mpgemm.c prints -- see that file's comment.
static uint32_t output_checksum(const void * buf, size_t len) {
    const uint8_t * p = (const uint8_t *)buf;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}
#define REPEATING_BIAS 1
#define A_STRIDE MAT_DIM_K
#define B_STRIDE MAT_DIM_J
#define D_STRIDE MAT_DIM_J
#define C_STRIDE MAT_DIM_J

static uint64_t read_cycles(void) {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// ---- Profile buffer (see profiler_test/gemm_profiler_test.c for the format) ------------
#define PROFILE_MAX_ENTRIES 4096
static volatile uint64_t P[PROFILE_MAX_ENTRIES] row_align(1);

#define PROFILE_TYPE(e)  ((int)     (((e) >> 62) & 0x3ULL))
#define PROFILE_START(e) ((uint32_t)(((e) >> 31) & 0x7FFFFFFFULL))
#define PROFILE_END(e)   ((uint32_t)( (e)        & 0x7FFFFFFFULL))

static void dump_profile(void) {
    printf("=== PROFILE DUMP BEGIN ===\n");
    int count = 0;
    uint64_t type_cycles[3] = {0, 0, 0};
    int type_count[3] = {0, 0, 0};
    for (int i = 0; i < PROFILE_MAX_ENTRIES; i++) {
        uint64_t e = P[i];
        if (e == 0) break;   // first empty slot => end of records
        int type = PROFILE_TYPE(e);
        printf("%d, %u, %u\n", type, PROFILE_START(e), PROFILE_END(e));
        if (type >= 0 && type <= 2) {
            type_cycles[type] += PROFILE_END(e) - PROFILE_START(e);
            type_count[type]++;
        }
        count++;
    }
    printf("=== PROFILE DUMP END ===\n");
    printf("Captured %d profile events (0=ld,1=ex,2=st).\n", count);
    printf("  LD_CYCLES (profiler, %d events): %lu\n", type_count[0], type_cycles[0]);
    printf("  EX_CYCLES (profiler, %d events): %lu\n", type_count[1], type_cycles[1]);
    printf("  ST_CYCLES (profiler, %d events): %lu\n", type_count[2], type_cycles[2]);
}

static void init_mats_packed(elem_t A[MAT_DIM_I][MAT_DIM_K], elem_t B[MAT_DIM_K][MAT_DIM_J/4]) {
    for (int i = 0; i < MAT_DIM_I; i++)
        for (int k = 0; k < MAT_DIM_K; k++)
            A[i][k] = rand() % 2;

    for (int k = 0; k < MAT_DIM_K; k++) {
        for (int jp = 0; jp < MAT_DIM_J / 4; jp++) {
            uint8_t packed_val = 0;
            for (int i = 0; i < 4; i++) {
                int ternary_val = rand() % 3 - 1;
                uint8_t two_bit_val = 0;
                if (ternary_val == 1) two_bit_val = 1;
                else if (ternary_val == -1) two_bit_val = 3;
                packed_val |= (two_bit_val & 0x03) << (i * 2);
            }
            B[k][jp] = packed_val;
        }
    }
}

int main() {
#ifndef BAREMETAL
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall failed");
        exit(1);
    }
#endif

    printf("==============================================\n");
    printf("  Custom (F-Vela) Gemmini mpgemm Profiler Test\n");
    printf("  I=%d J=%d K=%d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
    printf("  P buffer @ %p (%d entries)\n", (void *)P, PROFILE_MAX_ENTRIES);
    printf("==============================================\n");

    gemmini_flush(0);

    static elem_t full_A[MAT_DIM_I][MAT_DIM_K] row_align(1);
    static elem_t full_B[MAT_DIM_K][MAT_DIM_J/4] row_align(1);
    static elem_t full_C[MAT_DIM_I][MAT_DIM_J] row_align(1);
    static acc_t full_D[MAT_DIM_I][MAT_DIM_J] row_align_acc(1);

    init_mats_packed(full_A, full_B);

    counter_configure(0, LOOP_MATMUL_ACTIVE_CYCLES);
    counter_configure(1, EXE_ACTIVE_CYCLE);
    counter_configure(2, RDMA_ACTIVE_CYCLE);
    counter_configure(3, WDMA_ACTIVE_CYCLE);
    counter_reset();

    gemmini_profiler(P);

    printf("Gemmini mpgemm...\n");
    uint64_t start = read_cycles();

    ternary_gemm_auto(MAT_DIM_I, MAT_DIM_J, MAT_DIM_K,
            (elem_t*)full_A, (elem_t*)full_B, NO_BIAS ? NULL : &full_D[0][0], (elem_t*)full_C,
            A_STRIDE, B_STRIDE, D_STRIDE, C_STRIDE,
            MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
            NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, REPEATING_BIAS,
            false, false);

    gemmini_fence();

    uint64_t end = read_cycles();

    printf("RESULT custom_gemmini_mpgemm cycles: %lu\n", end - start);
    printf("  LD_CYCLES (counter, RDMA_ACTIVE_CYCLE): %u\n", counter_read(2));
    printf("  EX_CYCLES (counter, EXE_ACTIVE_CYCLE):  %u\n", counter_read(1));
    printf("  ST_CYCLES (counter, WDMA_ACTIVE_CYCLE): %u\n", counter_read(3));
    printf("  LOOP_MATMUL_ACTIVE_CYCLES (bonus):      %u\n", counter_read(0));
    printf("RESULT output_checksum: 0x%08x\n", output_checksum(&full_C[0][0], sizeof(full_C)));

    dump_profile();

    printf("PROFILER TEST DONE\n");
    return 0;
}
