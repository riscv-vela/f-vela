// Test 항목
// 1. 단일 Tile 행렬 곱 (Weight Stationary)
//
// 동작 흐름:
//   flush → config_ld/st/ex → mvin(A,B) → preload(B,C_acc) → compute(A,GARBAGE) → mvout(C) → fence → 검증
//
// OS vs WS 인자 구분
//   OS (Output Stationary):
//     preload(D_bias, C_output)     - D가 초기 누산값(바이어스)
//     compute_preloaded(A, B)       - A와 B가 행렬 입력
//
//   WS (Weight Stationary):
//     preload(B_weights, C_output)  - B가 PE에 고정(stationary)
//     compute_preloaded(A, D_bias)  - A가 스트리밍, D는 바이어스(없으면 GARBAGE_ADDR)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fv_gemmini.h"
#include "gemmini_testutils.h"

// 행렬 크기 = 하드웨어 DIM (16×16)
#define MAT_DIM DIM

int main() {

    static elem_t A[MAT_DIM][MAT_DIM] row_align(1);
    static elem_t B[MAT_DIM][MAT_DIM] row_align(1);
    static elem_t C_cpu[MAT_DIM][MAT_DIM] row_align(1);
    static elem_t C_gemmini[MAT_DIM][MAT_DIM] row_align(1);

    // ----------------------------------------------------------------
    // 1. 행렬 초기화
    //    주의: 이 가속기의 weightType = SInt(2.W) (삼진 가중치)
    //    일반(non-MpGEMM) 모드에서도 B 값이 하드웨어 PE에서 2비트로 잘립니다.
    //    정확한 검증을 위해 B 값을 삼진 범위 {-1, 0, 1} 으로 제한합니다.
    // ----------------------------------------------------------------
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            A[i][j] = (elem_t)((i + j) % 4 - 1);       // 범위: -1 ~ 2
            B[i][j] = (elem_t)(((i - j) % 3 + 3) % 3 - 1); // 삼진 범위: -1, 0, 1
            C_cpu[i][j] = 0;
            C_gemmini[i][j] = 0;
        }
    }

    // ----------------------------------------------------------------
    // 2. CPU 계산 (검증용)
    // ----------------------------------------------------------------
    printf("CPU: Calculating golden reference...\n");
    for (int i = 0; i < MAT_DIM; i++)
        for (int j = 0; j < MAT_DIM; j++)
            for (int k = 0; k < MAT_DIM; k++)
                C_cpu[i][j] += A[i][k] * B[k][j];

    // ----------------------------------------------------------------
    // 3. Gemmini 가속기 초기화
    // ----------------------------------------------------------------
    printf("Gemmini: Starting Hardware MatMul (Weight Stationary)...\n");

    gemmini_flush(0);                          // TLB flush
    gemmini_config_ld(DIM * sizeof(elem_t));   // mvin 스트라이드 설정
    gemmini_config_st(DIM * sizeof(elem_t));   // mvout 스트라이드 설정
    gemmini_config_ex(WEIGHT_STATIONARY, NO_ACTIVATION, 0); // WS 방식, activation 없음

    // ----------------------------------------------------------------
    // 4. 스크래치패드 주소 정의
    //    스크래치패드 (일반 데이터): bit[31] = 0
    //    누산기 (acc 메모리):        bit[31] = 1  반드시 설정
    //    누산기 overwrite 모드:      bit[30] = 0 (새로 씀)
    //    누산기 accumulate 모드:     bit[30] = 1 (누적)
    // ----------------------------------------------------------------
    const uint32_t A_sp  = 0;
    const uint32_t B_sp  = DIM;
    const uint32_t C_acc = (1U << 31);  // 누산기 0번 주소 (overwrite)

    // ----------------------------------------------------------------
    // 5. mvin: DRAM → 스크래치패드
    // ----------------------------------------------------------------
    gemmini_mvin(A, A_sp);  // A 행렬을 spad[0..DIM-1]에 로드
    gemmini_mvin(B, B_sp);  // B 행렬을 spad[DIM..2*DIM-1]에 로드

    // ----------------------------------------------------------------
    // 6. Preload: B 행렬을 PE 배열에 로드하고, 출력 주소(누산기) 지정
    //    gemmini_preload(BD_spad_addr, C_acc_addr)
    //    - BD_spad_addr: 스크래치패드에서 B를 가져올 주소
    //    - C_acc_addr:   결과를 쓸 누산기 주소
    // ----------------------------------------------------------------
    gemmini_preload(B_sp, C_acc);

    // ----------------------------------------------------------------
    // 7. Compute: A × B 실행 (Weight Stationary)
    //    gemmini_compute_preloaded(A_spad_addr, D_spad_addr)
    //    - A_spad_addr: A를 읽을 스크래치패드 주소
    //    - GARBAGE_ADDR: D(bias)를 사용하지 않음 (WS 모드에서 bias는 compute의 2번째 인자)
    // ----------------------------------------------------------------
    gemmini_compute_preloaded(A_sp, GARBAGE_ADDR);

    // ----------------------------------------------------------------
    // 8. mvout: 누산기 → DRAM
    //    gemmini_mvout(dram_addr, acc_addr)
    // ----------------------------------------------------------------
    gemmini_mvout(C_gemmini, C_acc);

    // ----------------------------------------------------------------
    // 9. Fence: 모든 명령이 완료될 때까지 CPU 대기
    // ----------------------------------------------------------------
    gemmini_fence();

    // ----------------------------------------------------------------
    // 10. 결과 검증
    // ----------------------------------------------------------------
    bool success = true;
    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            if (C_cpu[i][j] != C_gemmini[i][j]) {
                printf("Error at [%d][%d]: CPU=%d, Gemmini=%d\n",
                       i, j, C_cpu[i][j], C_gemmini[i][j]);
                success = false;
            }
        }
    }

    printMatrix(C_cpu);
    printf("\n");
    printMatrix(C_gemmini);

    if (success) {
        printf("--- [SUCCESS] Gemmini MatMul Test Passed! ---\n");
    } else {
        printf("--- [FAIL] Gemmini MatMul Test Failed ---\n");
    }

    return 0;
}