// See LICENSE for license details.
//
// Profiler-instrumented version of rv_gemmini_test/ternary_gemm.c.
//
// Same ternary matmul workload, but with the Gemmini profiler enabled:
//   - allocate a P[] buffer and register it with gemmini_profiler(P)
//   - run ternary_gemm_auto()
//   - gemmini_fence()
//   - decode P[] and print each event as "type, start, end" between the
//     PROFILE-DUMP markers, ready for plot/profile.py.
//
// Each P[] entry is packed by the hardware as Cat(q[2b], start_time[31b], end_cycle[31b]):
//     type  = (entry >> 62) & 0x3      // 0 = ld, 1 = ex, 2 = st
//     start = (entry >> 31) & 0x7FFFFFFF
//     end   =  entry        & 0x7FFFFFFF

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "pf_gemmini.h"   // profiler + ternary_gemm_auto + counters (superset of fv_gemmini.h)

#define PRINT 1

#define ACTIVATION NO_ACTIVATION

#define NO_BIAS 1
#define REPEATING_BIAS 1

#define A_TRANSPOSE 0
#define B_TRANSPOSE 0

#define MAT_DIM_I 1
#define MAT_DIM_K 64
#define MAT_DIM_J 64
// #define MAT_DIM_I 1
// #define MAT_DIM_K 512
// #define MAT_DIM_J 512


#define A_STRIDE MAT_DIM_K //The number of A's column
#define B_STRIDE MAT_DIM_J //The number of B's column
#define D_STRIDE MAT_DIM_J //The number of D's column
#define C_STRIDE MAT_DIM_J //The number of C's column

// read_cycles lives in gemmini_testutils.h, which would clash with pf_gemmini.h
// (both are full gemmini headers), so define a local copy here.
static uint64_t read_cycles() {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// ---- Profile buffer ---------------------------------------------------------------------------------------------------
// data(instruction_type: 1bit, start_cycle: 31bits, end_cycle:31bits)
#define PROFILE_MAX_ENTRIES 4096
static volatile uint64_t P[PROFILE_MAX_ENTRIES] row_align(1);

#define PROFILE_TYPE(e)  ((int)     (((e) >> 62) & 0x3ULL)) 		// q_type(p) (p >> 62)
#define PROFILE_START(e) ((uint32_t)(((e) >> 31) & 0x7FFFFFFFULL)) 	// start(p) ((p >> 31) & ((1UL << 31) - 1))
#define PROFILE_END(e)   ((uint32_t)( (e)        & 0x7FFFFFFFULL)) 	// end(p) (p & ((1UL << 31) - 1))

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

void full_printMatrix(elem_t m[MAT_DIM_I][MAT_DIM_J]) {
	for (size_t i = 0; i < MAT_DIM_I; ++i) {
		for (size_t j = 0; j < MAT_DIM_J; ++j)
			printf("%d ", m[i][j]);
		printf("\n");
	}
}

static void init_mats_packed(elem_t A[MAT_DIM_I][MAT_DIM_K], elem_t B[MAT_DIM_K][MAT_DIM_J/4]) {
  	for (size_t i = 0; i < MAT_DIM_I; ++i)
		for (size_t k = 0; k < MAT_DIM_K; ++k)
			A[i][k] = rand() % 2; // 0 또는 1

  	for (size_t k = 0; k < MAT_DIM_K; ++k) {
		for (size_t j_packed = 0; j_packed < MAT_DIM_J / 4; ++j_packed) {
			uint8_t packed_val = 0;

			// 4개의 2-bit 값을 생성하여 하나의 uint8_t로 묶습니다.
			for (int i = 0; i < 4; ++i) {
				// -1, 0, 1 중 하나를 랜덤하게 생성
				int ternary_val =  rand() % 3 - 1; 
				// printf("%d ", ternary_val);

				// 2-bit로 인코딩 (0b11: -1, 0b00: 0, 0b01: 1)
				uint8_t two_bit_val = 0;
				if (ternary_val == 1) {
					two_bit_val = 1; // 0b01
				} else if (ternary_val == -1) {
					two_bit_val = 3; // 0b11
				}
				// packed_val에 2비트씩 채워넣기
				packed_val |= (two_bit_val & 0x03) << (i * 2);
			}
			B[k][j_packed] = packed_val;
			// printf("\n");
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

		gemmini_flush(0);

		static elem_t full_A[MAT_DIM_I][MAT_DIM_K] row_align(1);
		static elem_t full_B[MAT_DIM_K][MAT_DIM_J/4] row_align(1);
		static elem_t full_C[MAT_DIM_I][MAT_DIM_J] row_align(1);
		static acc_t full_D[MAT_DIM_I][MAT_DIM_J] row_align_acc(1);

		init_mats_packed(full_A, full_B);

		counter_configure(0, RDMA_BYTES_REC);
		counter_configure(1, WDMA_BYTES_SENT);
		counter_reset();

		printf("Starting gemmini ternary matmul (profiled)\n");
		printf("I: %d, J: %d, K: %d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
		printf("NO_BIAS: %d, REPEATING_BIAS: %d\n", NO_BIAS, REPEATING_BIAS);
		printf("A_TRANSPOSE: %d, B_TRANSPOSE: %d\n", A_TRANSPOSE, B_TRANSPOSE);
		printf("P buffer @ %p (%d entries)\n", (void *)P, PROFILE_MAX_ENTRIES);

		// Register the profile buffer with the accelerator before the workload.
		gemmini_profiler(P);

		uint64_t start = read_cycles();

		ternary_gemm_auto(MAT_DIM_I, MAT_DIM_J, MAT_DIM_K,
						(elem_t*)full_A, (elem_t*)full_B, NO_BIAS ? NULL : &full_D[0][0], (elem_t*)full_C,
						A_STRIDE, B_STRIDE, D_STRIDE, C_STRIDE,
						MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
						ACTIVATION, ACC_SCALE_IDENTITY, 0, REPEATING_BIAS,
						false, false);                   // full_C, low_D

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
	exit(0);
}
