#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "include/gemmini_testutils.h"

#define DIM 16 
#define N 1
#define Bank_size 12

#define SP_ADDR(bank,row)  (((bank)<<12) | (row))    // bank 위·row 아래
#define ACC_ADDR(bank,row) ((1u<<31) | SP_ADDR(bank,row))

int main() {
#ifndef BAREMETAL
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    exit(1);
  }
#endif

  gemmini_flush(0);
  gemmini_config_ld(DIM * sizeof(elem_t));
  gemmini_config_ex(WEIGHT_STATIONARY, 0, 0);
  gemmini_extended_config_st(DIM * sizeof(elem_t), RELU, 1);

  // 행렬 선언 및 초기화
  static elem_t A[DIM][DIM], B1[DIM][DIM], B2[DIM][DIM], D[DIM][DIM];
  static elem_t C_caseA[DIM][DIM], C_caseB[DIM][DIM];
  static elem_t gold_caseA[DIM][DIM], gold_caseB[DIM][DIM];

  for (size_t i = 0; i < DIM; ++i) {
    for (size_t j = 0; j < DIM; ++j) {
      A[i][j] = i/2;
      B1[i][j]  = 1;
      B2[i][j]  = 2;
      D[i][j]  = 1;
    }
  }

  // SPM 주소 지정
/* Mat 하나는 16행, bank/row 안겹치게 */
  uint32_t A_addr  = SP_ADDR(0, 0);            // bank0, row0
  uint32_t B_addr  = SP_ADDR(1, 64);           // bank1, row16
  uint32_t B2_addr = SP_ADDR(1, 80);           // bank1, row32
  uint32_t D_addr  = SP_ADDR(2, 128);  
  uint32_t C_addr_A = (1 << (ADDR_LEN-1));       // Accumulator 주소 1
  uint32_t C_addr_B = (1 << (ADDR_LEN-1)) + DIM; // Accumulator 주소 2

  // 메모리 로딩
  gemmini_mvin(A, A_addr);
  gemmini_mvin(B1,  B_addr);
  gemmini_mvin(B2, B2_addr);
  gemmini_mvin(D,  D_addr);

  gemmini_fence();
  // -------------------------
  // CASE A: reuse weight
  // -------------------------
  gemmini_preload(B_addr, C_addr_A);
  gemmini_compute_preloaded(A_addr, D_addr);

  gemmini_preload_zeros(C_addr_A);
  gemmini_compute_accumulated(A_addr, D_addr);

  gemmini_mvout(C_caseA, C_addr_A);
  gemmini_fence();

  // -------------------------
  // CASE B: new weight again
  // -------------------------
  gemmini_preload(B_addr, C_addr_B);
  gemmini_compute_preloaded(A_addr, D_addr);

  gemmini_preload(B2_addr, C_addr_B);
  gemmini_compute_preloaded(A_addr, D_addr);

  gemmini_mvout(C_caseB, C_addr_B);
  gemmini_fence();

  // -------------------------
  // 소프트웨어 정답 계산
  // -------------------------
  static full_t tmp1[DIM][DIM];
  matmul(A, B1, D, gold_caseA);

  matmul(A, B1, D, tmp1);
  matmul(A, B2, D, gold_caseB);     // no accumulate

  // -------------------------
  // 비교 및 출력
  // -------------------------
  printf("===== CASE A: Reuse B =====\n");
  printMatrix(C_caseA);
  printf("Gold:\n");
  printMatrix(gold_caseA);
  if (!is_equal(C_caseA, gold_caseA)) {
    printf("CASE A Mismatch!\n");
  }

  printf("===== CASE B: Reload B =====\n");
  printMatrix(C_caseB);
  printf("Gold:\n");
  printMatrix(gold_caseB);
  if (!is_equal(C_caseB, gold_caseB)) {
    printf("CASE B Mismatch!\n");
  }

  return 0;
}
