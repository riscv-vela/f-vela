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

// A_TRANSPOSE must be 0
// To make code simple, B1_TRANSPOSE is fixed to 0
#define A_TRANSPOSE 0
#define B1_TRANSPOSE 0
#define B2_TRANSPOSE 0

// MAT_DIM_I must be 1
#define MAT_DIM_I 1
#define MAT_DIM_K 64
#define MAT_DIM_J 128
#define MAT_DIM_V 64

#define VB ((MAT_DIM_J + DIM - 1) / DIM)
#define JB ((MAT_DIM_J + DIM - 1) / DIM)
#define KB ((MAT_DIM_K + DIM - 1) / DIM)

// The number of B's column
#if B2_TRANSPOSE==0         
#define B2_STRIDE (JB*DIM) 
#else
#define B1_STRIDE (VB*DIM)
#endif
 
#define A1_STRIDE MAT_DIM_K // The number of A's column
#define B1_STRIDE (KB*DIM)
#define D1_STRIDE MAT_DIM_J // The number of D's column
#define C1_STRIDE MAT_DIM_J // The number of C's column

#define A2_STRIDE MAT_DIM_J // The number of A's column
#define D2_STRIDE MAT_DIM_V // The number of D's column
#define C2_STRIDE MAT_DIM_V // The number of C's column


void full_printMatrix(elem_t m[MAT_DIM_I][MAT_DIM_J]) {
  for (size_t i = 0; i < MAT_DIM_I; ++i) {
    for (size_t j = 0; j < MAT_DIM_J; ++j)
      printf("%d ", m[i][j]);
    printf("\n");
  }
}

static void init_mats(elem_t A[MAT_DIM_I][MAT_DIM_K], elem_t B[MAT_DIM_K][MAT_DIM_J], elem_t B2[MAT_DIM_J][MAT_DIM_V]) {

  for (size_t i = 0; i < MAT_DIM_I; ++i)
    for (size_t k = 0; k < MAT_DIM_K; ++k)
      A[i][k] = rand() % 2;

  for (size_t k = 0; k < MAT_DIM_K; ++k)
    for (size_t j = 0; j < MAT_DIM_J; ++j)
      B[k][j] = rand() % 3 -1;

  for (size_t j = 0; j < MAT_DIM_V; ++j)
    for (size_t v = 0; v < MAT_DIM_V; ++v)
      B2[j][v] = rand() % 3 -1;
}

static void reorder_K(elem_t *src, elem_t *dst, int stride, int row_block, int col_block)
{
  size_t out = 0;

  for (size_t jb = 0; jb < col_block; ++jb){            // j 블록
    for (size_t k = 0; k < DIM; ++k){           // 블록 내부 k
      for (size_t kb = 0; kb < row_block; ++kb){       // k 블록
        for (size_t j = 0; j < DIM; ++j){       // 블록 내부 j
          dst[out] =  src[(kb*DIM + k)*stride+ jb*DIM + j]; 
          out ++;
        }
      }
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
    static elem_t full_B1[MAT_DIM_K][MAT_DIM_J] row_align(1);
    static elem_t full_B1_reordered[JB*DIM][KB*DIM] row_align(1);
#if B2_TRANSPOSE==0
    static elem_t full_B2[MAT_DIM_J][MAT_DIM_V] row_align(1);
    static elem_t full_B2_reordered[VB*DIM][JB*DIM] row_align(1);
#else
    static elem_t full_B2[MAT_DIM_V][MAT_DIM_J] row_align(1);
    static elem_t full_B2_reordered[JB*DIM][VB*DIM] row_align(1);
#endif
    static elem_t full_C1[MAT_DIM_I][MAT_DIM_J] row_align(1);
    static acc_t full_D1[MAT_DIM_I][MAT_DIM_J] row_align_acc(1);
    static elem_t full_C2[MAT_DIM_I][MAT_DIM_V] row_align(1);
    static acc_t full_D2[MAT_DIM_I][MAT_DIM_V] row_align_acc(1);
    
    init_mats(full_A, full_B1, full_B2);
    reorder_K((elem_t *)full_B1, (elem_t *)full_B1_reordered, MAT_DIM_J, KB, JB);
    reorder_K((elem_t *)full_B2, (elem_t *)full_B2_reordered, MAT_DIM_V, JB, VB);

    printf("Starting 2 gemmini matmul\n");
    printf("MAT 1: I: %d, J: %d, K: %d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
    printf("MAT 2: I: %d, V: %d, J: %d\n", MAT_DIM_I, MAT_DIM_V, MAT_DIM_J);

    uint64_t start = read_cycles();

    gemv_auto(MAT_DIM_I, MAT_DIM_J, MAT_DIM_K,
            (elem_t*)full_A, (elem_t*)full_B1_reordered, NO_BIAS ? NULL : &full_D1[0][0], (elem_t*)full_C1,
            A1_STRIDE, B1_STRIDE, D1_STRIDE, C1_STRIDE,
            MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
            ACTIVATION, ACC_SCALE_IDENTITY, 0, REPEATING_BIAS,
            A_TRANSPOSE, B1_TRANSPOSE,
            false, false,        /* full_C, low_D */
            1, 0, 3);            /* a_spad_id, b_spad_id, c_spad_id */

    gemv_auto(MAT_DIM_I, MAT_DIM_V, MAT_DIM_J,
            NULL, (elem_t*)full_B2_reordered, NO_BIAS ? NULL : &full_D2[0][0], (elem_t*)full_C2,
            A2_STRIDE, B2_STRIDE, D2_STRIDE, C2_STRIDE,
            MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
            ACTIVATION, ACC_SCALE_IDENTITY, 0, REPEATING_BIAS,
            A_TRANSPOSE, B2_TRANSPOSE,
            false, false,        /* full_C, low_D */
            3, 0, 1);            /* a_spad_id, b_spad_id, c_spad_id */

    gemmini_fence();

    uint64_t end = read_cycles();
    printf("Cycles taken: %llu\n", end-start);

#ifdef PRINT
    printf("C:\n");
    full_printMatrix(full_C2);
#endif

  exit(0);
}

