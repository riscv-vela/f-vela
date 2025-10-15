// See LICENSE for license details.

#include <stdint.h>
#include <stddef.h>
#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "include/gemmini_testutils.h"

#define PRINT 0

#define ACTIVATION NO_ACTIVATION

#define NO_BIAS 1
#define REPEATING_BIAS 0

//A_TRANSPOSE must be 0
#define A_TRANSPOSE 0
#define B_TRANSPOSE 0

//MAT_DIM_I must be 1
#define MAT_DIM_I 1
#define MAT_DIM_K 256
#define MAT_DIM_J 128

#define JB ((MAT_DIM_J + DIM - 1) / DIM)
#define KB ((MAT_DIM_K + DIM - 1) / DIM)

#define A_STRIDE MAT_DIM_K // The number of A's column

// The number of B's column
#if B_TRANSPOSE==0         
#define B_STRIDE (KB*DIM) 
#else
#define B_STRIDE (JB*DIM)
#endif

#define D_STRIDE MAT_DIM_J // The number of D's column
#define C_STRIDE MAT_DIM_J // The number of C's column

void full_printMatrix(elem_t m[MAT_DIM_I][MAT_DIM_J]) {
  for (size_t i = 0; i < MAT_DIM_I; ++i) {
    for (size_t j = 0; j < MAT_DIM_J; ++j)
      printf("%d ", m[i][j]);
    printf("\n");
  }
}

static void init_mats(elem_t A[MAT_DIM_I][MAT_DIM_K], elem_t B[MAT_DIM_K][MAT_DIM_J]) {

  for (size_t i = 0; i < MAT_DIM_I; ++i)
    for (size_t k = 0; k < MAT_DIM_K; ++k)
      A[i][k] = rand() % 2;

  for (size_t k = 0; k < MAT_DIM_K; ++k)
    for (size_t j = 0; j < MAT_DIM_J; ++j)
      B[k][j] = rand() % 3 -1;
}

static void reorder_K(elem_t *src, elem_t *dst)
{
  size_t out = 0;

  for (size_t jb = 0; jb < JB; ++jb)          // j 블록
    for (size_t k = 0; k < DIM; ++k)          // 블록 내부 j
      for (size_t kb = 0; kb < KB; ++kb)        // k 블록
        for (size_t j = 0; j < DIM; ++j){
          dst[out] =  src[(kb*DIM + k)*MAT_DIM_J+ jb*DIM + j]; 
          out ++;
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
#if B_TRANSPOSE==0
    static elem_t full_B[MAT_DIM_K][MAT_DIM_J] row_align(1);
    static elem_t full_B_reordered[JB*DIM][KB*DIM] row_align(1);
#else
    static elem_t full_B[MAT_DIM_J][MAT_DIM_K] row_align(1);
    static elem_t full_B_reordered[KB*DIM][JB*DIM] row_align(1);
#endif
    static elem_t full_C[MAT_DIM_I][MAT_DIM_J] row_align(1);
    static acc_t full_D[MAT_DIM_I][MAT_DIM_J] row_align_acc(1);
    
    init_mats(full_A, full_B);
    reorder_K((elem_t *)full_B, (elem_t *)full_B_reordered);

    counter_configure(0, RDMA_BYTES_REC);
    counter_configure(1, WDMA_BYTES_SENT);
    counter_reset();

    printf("Starting gemmini matmul\n");
    printf("I: %d, J: %d, K: %d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
    printf("NO_BIAS: %d, REPEATING_BIAS: %d\n", NO_BIAS, REPEATING_BIAS);
    printf("A_TRANSPOSE: %d, B_TRANSPOSE: %d\n", A_TRANSPOSE, B_TRANSPOSE);
    uint64_t start = read_cycles();

    gemv_auto(MAT_DIM_I, MAT_DIM_J, MAT_DIM_K,
            (elem_t*)full_A, (elem_t*)full_B_reordered, NO_BIAS ? NULL : &full_D[0][0], (elem_t*)full_C,
            A_STRIDE, B_STRIDE, D_STRIDE, C_STRIDE,
            MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
            ACTIVATION, ACC_SCALE_IDENTITY, 0, REPEATING_BIAS,
            A_TRANSPOSE, B_TRANSPOSE,
            false, false,        /* full_C, low_D */
            0, 0, 0);            /* a_spad_id, b_spad_id, c_spad_id */

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

#ifdef PRINT
    printf("C:\n");
    full_printMatrix(full_C);
#endif

  exit(0);
}

