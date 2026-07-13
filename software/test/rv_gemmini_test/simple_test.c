#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "fv_gemmini.h"
#include "gemmini_testutils.h"

#define MAT_DIM DIM

int main() {

    static elem_t A[MAT_DIM][MAT_DIM] row_align(1);
    static elem_t B[MAT_DIM][MAT_DIM] row_align(1);
    static elem_t C_cpu[MAT_DIM][MAT_DIM] row_align(1);
    static elem_t C_gemmini[MAT_DIM][MAT_DIM] row_align(1);

    for (int i = 0; i < MAT_DIM; i++) {
        for (int j = 0; j < MAT_DIM; j++) {
            A[i][j] = (elem_t)((i + j) % 4 - 1);       
            B[i][j] = (elem_t)(((i - j) % 3 + 3) % 3 - 1);
            C_cpu[i][j] = 0;
            C_gemmini[i][j] = 0;
        }
    }

    // CPU 계산 (검증용)
    printf("CPU: Calculating reference...\n");
    for (int i = 0; i < MAT_DIM; i++)
        for (int j = 0; j < MAT_DIM; j++)
            for (int k = 0; k < MAT_DIM; k++)
                C_cpu[i][j] += A[i][k] * B[k][j];

    printf("Gemmini: Starting Hardware MatMul (Weight Stationary)...\n");

    // Gemmini 계산
    gemmini_flush(0);                          
    gemmini_config_ld(DIM * sizeof(elem_t));   
    gemmini_config_st(DIM * sizeof(elem_t));   
    gemmini_config_ex(WEIGHT_STATIONARY, NO_ACTIVATION, 0); 

    const uint32_t A_sp  = 0;
    const uint32_t B_sp  = DIM;
    const uint32_t C_acc = (1U << 31);
    gemmini_mvin(A, A_sp); 
    gemmini_mvin(B, B_sp); 
    gemmini_preload(B_sp, C_acc);
    gemmini_compute_preloaded(A_sp, GARBAGE_ADDR);
    gemmini_mvout(C_gemmini, C_acc);

    gemmini_fence();
    
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

    printf("CPU Result (C_cpu):\n");
    printMatrix(C_cpu);
    printf("\n");
    
    printf("Gemmini Result (C_gemmini):\n");
    printMatrix(C_gemmini);

    if (success) {
        printf("--- [SUCCESS] Gemmini MatMul Test Passed! ---\n");
    } else {
        printf("--- [FAIL] Gemmini MatMul Test Failed ---\n");
    }

    return 0;
}