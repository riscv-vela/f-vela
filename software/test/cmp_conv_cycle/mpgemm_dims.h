// Shared mpgemm (mixed-precision GEMM: int8 activations x ternary weights) problem size for
// rocket_cpu_mpgemm.c and custom_gemmini_mpgemm_profiler.c. Matches profiler_test/
// ternary_profiler_test.c's dimensions (small enough to actually finish in Verilator).
#ifndef CMP_MPGEMM_DIMS_H
#define CMP_MPGEMM_DIMS_H

#define MAT_DIM_I 1
#define MAT_DIM_K 64
#define MAT_DIM_J 64

#endif // CMP_MPGEMM_DIMS_H
