//----------------------------------------------------------------------
// gemmini_cycle_demo.c
//----------------------------------------------------------------------
//  * 각 구간에서 0번 카운터를 원하는 이벤트로 재설정하며 재사용
//  * LOAD_ACTIVE_CYCLE == DMA 가 실제 busy 인 사이클 → spike/RTL 모두 잘 증가
//----------------------------------------------------------------------

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#include <sys/mman.h>
#endif
#include "include/gemmini_testutils.h"
#include "include/gemmini_counter.h"   

#define GARBAGE_ADDR ((uint32_t)(-1))
#define DIM 16
#define N   1
#define SP_ADDR(bank,row)  (((bank)<<12) | (row))    // bank 위·row 아래
#define ACC_ADDR(bank,row) ((1u<<31) | SP_ADDR(bank,row))

/* ---------- 계측 매크로 ---------- */
#define MEASURE(idx, event, msg, block)                \
    do {                                               \
        counter_reset();                               \
        counter_configure((idx), (event));             \
        counter_snapshot_reset();                      \
                                                       \
        block                                          \
                                                       \
        gemmini_fence();                               \
        counter_snapshot_take();                       \
        uint32_t cyc = counter_read((idx));            \
        printf("%-32s : %u cycles\n", (msg), cyc);     \
    } while (0)

#define MEASURE2(idx1, ev1, idx2, ev2, msg, CODE)         \
  do {                                                    \
    counter_reset();                                      \
    counter_configure(idx1, ev1);                         \
    counter_configure(idx2, ev2);                         \
    counter_snapshot_reset();                             \
    CODE                                                  \
    gemmini_fence();                                      \
    counter_snapshot_take();                              \
    uint32_t v1 = counter_read(idx1);                     \
    uint32_t v2 = counter_read(idx2);                     \
    printf(msg " : %u (active) , %u (preload-haz) cycles\n",\
           v1, v2);                                       \
  } while (0)
      
int main() {
#ifndef BAREMETAL
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    return 1;
  }
#endif

  /* ------------------------------------------------------------------ */
  /* 0. Gemmini 초기화                                                  */
  /* ------------------------------------------------------------------ */
  gemmini_flush(0);
  gemmini_config_ld(DIM * sizeof(elem_t));
  gemmini_config_ex(WEIGHT_STATIONARY, 0, 0);
  gemmini_extended_config_st(DIM * sizeof(elem_t), RELU, 1);

  /* ------------------------------------------------------------------ */
  /* 1. 데이터 준비                                                     */
  /* ------------------------------------------------------------------ */
  static elem_t A[DIM][DIM], B1[DIM][DIM], B2[DIM][DIM], D[DIM][DIM], A1[DIM];
  static elem_t C_caseA[DIM][DIM], C_caseB[DIM][DIM], C_caseC[DIM];

  for (size_t i = 0; i < DIM; ++i) {
    for (size_t j = 0; j < DIM; ++j) {
      A[i][j]  = 1;
      B1[i][j] = 1;
      B2[i][j] = 2;
      D[i][j]  = 3;
    }
    A1[i] = 5;
  }

  /* SPM 주소 */
  uint32_t A_addr  = SP_ADDR(0, 0);            // bank0, row0
  uint32_t B_addr  = SP_ADDR(1, 64);           // bank1, row16
  uint32_t B2_addr = SP_ADDR(1, 80);           // bank1, row32
  uint32_t D_addr  = SP_ADDR(2, 128); 
  uint32_t C_addr_A = (1u << (ADDR_LEN-1));          // Acc 0
  uint32_t C_addr_B = (1u << (ADDR_LEN-1)) + DIM;    // Acc 1

  /* ------------------------------------------------------------------ */
  /* 2. MVIN A                                                          */
  /* ------------------------------------------------------------------ */
  MEASURE(0, LOAD_ACTIVE_CYCLE, "mvin A", {
      gemmini_mvin(A, A_addr);
  });

  /* 3. MVIN B1+B2+D                                                    */
  MEASURE(0, LOAD_ACTIVE_CYCLE, "mvin B1+B2+D", {
      gemmini_mvin(B1, B_addr);
      gemmini_mvin(B2, B2_addr);
      gemmini_mvin(D,  D_addr);
  });

  /* ------------------------------------------------------------------ */
  /* 4. CASE-A  (Preload-Compute-Accumulate)                            */
  /* ------------------------------------------------------------------ */

    gemmini_preload(B_addr, C_addr_A);
  MEASURE(0, EXE_ACTIVE_CYCLE,
        "CASE-A compute-preloaded ", {
    gemmini_compute_preloaded(A_addr, D_addr);
  });

  MEASURE(0, EXE_ACTIVE_CYCLE, "CASE-A compute-accumulated", {
      gemmini_preload_zeros(C_addr_A);
      gemmini_compute_accumulated(A_addr, D_addr);
  });

  MEASURE(0, STORE_ACTIVE_CYCLE, "CASE-A mvout", {
      gemmini_mvout(C_caseA, C_addr_A);
  });

  /* ------------------------------------------------------------------ */
  /* 5. CASE-B : B 두 번 preload                                        */
  /* ------------------------------------------------------------------ */
  MEASURE(0, EXE_ACTIVE_CYCLE, "CASE-B 2×compute_preloaded", {
      gemmini_preload(B_addr,       C_addr_B);
      gemmini_compute_preloaded(A_addr, D_addr);
      gemmini_preload(B2_addr, C_addr_B);
      gemmini_compute_preloaded(A_addr, D_addr);
  });

  MEASURE(0, STORE_ACTIVE_CYCLE, "CASE-B mvout", {
      gemmini_mvout(C_caseB, C_addr_B);
  });

  /* ------------------------------------------------------------------ */
  /* 6. GEMV                                                            */
  /* ------------------------------------------------------------------ */
  MEASURE(0, LOAD_ACTIVE_CYCLE, "GEMV mvin A1", {
      gemmini_extended_mvin(A1, A_addr, DIM, 1);
  });
  MEASURE(0, EXE_ACTIVE_CYCLE,  "GEMV compute_preloaded", {
      gemmini_extended_preload(B_addr, C_addr_B, DIM, DIM, DIM, 1);
      gemmini_extended_compute_preloaded(A_addr, D_addr, DIM, 1, DIM, 1);
  });
  MEASURE(0, EXE_ACTIVE_CYCLE,  "GEMV compute_accumulated", {
      gemmini_extended_preload(GARBAGE_ADDR, C_addr_B, DIM, DIM, DIM, 1);
      gemmini_extended_compute_accumulated(A_addr, D_addr, DIM, 1, DIM, 1);
  });

  MEASURE(0, STORE_ACTIVE_CYCLE, "GEMV mvout", {
      gemmini_extended_mvout(C_caseC, C_addr_B, DIM, 1);
  });

  return 0;
}
