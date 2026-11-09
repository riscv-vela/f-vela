// gemm.c — plain int8 GEMM on Gemmini (tiled_matmul_auto, WS), checked against the CPU.
//
//   C[i][j] = sat8( A[i][:] . B[:][j] + D[i][j] )      (scale 1, no activation)
//
// This is the ordinary int8 path (is_mpgemm = 0). The 2-bit ternary weight path
// (BitNet) is tested separately in ternary_gemm.c (correctness) and ternary_gemm_perf.c (performance).
//
// Cases: GEMV (I=1), one tile, multi-tile, odd sizes (hardware padding),
//        full bias D, repeating bias (every row adds D[0][:]).
// Value ranges keep most outputs inside int8 so a wrong sum is not hidden by
// saturation, while some outputs still saturate.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "gemmini_testutils.h"

#define MAXM 64   // all matrices are MAXM x MAXM buffers; every stride is MAXM

static uint32_t rng = 0x1234ABCDu;
static inline uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng; }
static inline int rr(int lo, int hi) { return lo + (int)((rnd() >> 8) % (uint32_t)(hi - lo + 1)); }

static elem_t A[MAXM][MAXM] row_align(1);
static elem_t B[MAXM][MAXM] row_align(1);
static elem_t C[MAXM][MAXM] row_align(1);
static acc_t  D[MAXM][MAXM] row_align_acc(1);

static int total_err = 0, total_runs = 0, total_fail = 0;

static void run(const char *name, int I, int J, int K, bool bias, bool repeating_bias) {
  for (int i = 0; i < MAXM; i++)
    for (int j = 0; j < MAXM; j++) { A[i][j] = B[i][j] = C[i][j] = 0; D[i][j] = 0; }
  for (int i = 0; i < I; i++) for (int k = 0; k < K; k++) A[i][k] = (elem_t)rr(-8, 7);
  for (int k = 0; k < K; k++) for (int j = 0; j < J; j++) B[k][j] = (elem_t)rr(-3, 3);
  if (bias) for (int i = 0; i < I; i++) for (int j = 0; j < J; j++) D[i][j] = rr(-64, 64);

  const uint64_t start = read_cycles();
  tiled_matmul_auto(I, J, K, &A[0][0], &B[0][0], bias ? &D[0][0] : NULL, &C[0][0],
      MAXM, MAXM, MAXM, MAXM,
      MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
      NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, repeating_bias,
      false, false,        // transpose_A, transpose_B
      false, false,        // full_C, low_D
      0, WS);
  gemmini_fence();
  const uint64_t cycles = read_cycles() - start;

  int err = 0;
  for (int i = 0; i < I; i++) for (int j = 0; j < J; j++) {
    acc_t acc = bias ? D[repeating_bias ? 0 : i][j] : 0;
    for (int k = 0; k < K; k++) acc += (acc_t)A[i][k] * (acc_t)B[k][j];
    const elem_t ref = acc > elem_t_max ? elem_t_max : (acc < elem_t_min ? elem_t_min : (elem_t)acc);
    if (C[i][j] != ref) {
      if (err < 4) printf("    [%d][%d] got=%d ref=%d\n", i, j, (int)C[i][j], (int)ref);
      err++;
    }
  }

  total_err += err; total_runs++; if (err) total_fail++;
  printf("[%s] %-8s I=%2d J=%2d K=%2d bias=%d rep=%d : %d mismatches, %lu cycles\n",
         err ? "FAIL" : "PASS", name, I, J, K, bias, repeating_bias, err,
         (unsigned long)cycles);
}

int main(void) {
#ifndef BAREMETAL
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) { perror("mlockall"); exit(1); }
#endif
  printf("=== int8 gemm (tiled_matmul_auto, WS, DIM=%d) ===\n", DIM);
  gemmini_flush(0);

  run("gemv",    1, 64, 64, false, false);
  run("d16",    16, 16, 16, false, false);
  run("d64",    64, 64, 64, false, false);
  run("odd",    37, 50, 29, false, false);
  run("bias",   32, 32, 32, true,  false);
  run("repbias",32, 48, 32, true,  true);

  printf("=== int8 gemm: %d/%d PASS, %d mismatches ===\n",
         total_runs - total_fail, total_runs, total_err);
  exit(total_err == 0 ? 0 : 1);
}
