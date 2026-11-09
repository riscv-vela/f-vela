// ternary_gemm.c — functional test of the 2-bit ternary weight GEMM (BitNet-style, is_mpgemm = 1).
//
//   C[i][j] = sat8( A[i][:] . W[:][j] + D[i][j] ),   A: int8,  W: {-1, 0, +1}
//
// W is passed packed: 4 weights per byte, 2 bits each, column j = 4*jp + n sits in
// bits [2n+1:2n] of byte Bp[k][jp]. Encoding (MpPE TernaryMulUnit: bit0 = enable,
// bit1 = sign): 0b01 = +1, 0b11 = -1, 0b00 = 0. 0b10 is never generated.
//
// Small sizes, many shapes, CPU reference. For the large performance run see ternary_gemm_perf.c.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "gemmini_testutils.h"

#define MAXI 32
#define MAXJ 128   // logical (unpacked) columns; must be a multiple of 4
#define MAXK 64

static uint32_t rng = 0x5EED7E57u;
static inline uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng; }
static inline int rr(int lo, int hi) { return lo + (int)((rnd() >> 8) % (uint32_t)(hi - lo + 1)); }

static elem_t A[MAXI][MAXK] row_align(1);
static elem_t Bp[MAXK][MAXJ / 4] row_align(1);  // packed weights (what Gemmini reads)
static elem_t W[MAXK][MAXJ];                    // same weights unpacked (CPU reference)
static elem_t C[MAXI][MAXJ] row_align(1);
static acc_t  D[MAXI][MAXJ] row_align_acc(1);

static int total_err = 0, total_runs = 0, total_fail = 0;

static inline uint8_t ternary_encode(int w) { return w == 1 ? 0x1 : (w == -1 ? 0x3 : 0x0); }

static void run(const char *name, int I, int J, int K, bool bias, bool repeating_bias) {
  for (int i = 0; i < MAXI; i++) for (int j = 0; j < MAXJ; j++) { C[i][j] = 0; D[i][j] = 0; }
  for (int i = 0; i < MAXI; i++) for (int k = 0; k < MAXK; k++) A[i][k] = 0;
  for (int k = 0; k < MAXK; k++) {
    for (int j = 0; j < MAXJ; j++) W[k][j] = 0;
    for (int jp = 0; jp < MAXJ / 4; jp++) Bp[k][jp] = 0;
  }

  for (int i = 0; i < I; i++) for (int k = 0; k < K; k++) A[i][k] = (elem_t)rr(-8, 7);
  for (int k = 0; k < K; k++)
    for (int j = 0; j < J; j++) {
      const int w = rr(-1, 1);
      W[k][j] = (elem_t)w;
      Bp[k][j / 4] = (elem_t)((uint8_t)Bp[k][j / 4] | (ternary_encode(w) << (2 * (j % 4))));
    }
  if (bias) for (int i = 0; i < I; i++) for (int j = 0; j < J; j++) D[i][j] = rr(-64, 64);

  const uint64_t start = read_cycles();
  ternary_gemm_auto(I, J, K, &A[0][0], &Bp[0][0], bias ? &D[0][0] : NULL, &C[0][0],
      MAXK, MAXJ, MAXJ, MAXJ,          // stride_A, stride_B (logical, divided by 4 inside), stride_D, stride_C
      MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
      NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, repeating_bias,
      false, false);                   // full_C, low_D
  gemmini_fence();
  const uint64_t cycles = read_cycles() - start;

  int err = 0;
  for (int i = 0; i < I; i++) for (int j = 0; j < J; j++) {
    acc_t acc = bias ? D[repeating_bias ? 0 : i][j] : 0;
    for (int k = 0; k < K; k++) acc += (acc_t)A[i][k] * (acc_t)W[k][j];
    const elem_t ref = acc > elem_t_max ? elem_t_max : (acc < elem_t_min ? elem_t_min : (elem_t)acc);
    if (C[i][j] != ref) {
      if (err < 4) printf("    [%d][%d] got=%d ref=%d\n", i, j, (int)C[i][j], (int)ref);
      err++;
    }
  }

  total_err += err; total_runs++; if (err) total_fail++;
  printf("[%s] %-8s I=%2d J=%3d K=%2d bias=%d rep=%d : %d mismatches, %lu cycles\n",
         err ? "FAIL" : "PASS", name, I, J, K, bias, repeating_bias, err,
         (unsigned long)cycles);
}

int main(void) {
#ifndef BAREMETAL
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) { perror("mlockall"); exit(1); }
#endif
  printf("=== ternary gemm (is_mpgemm, DIM=%d) ===\n", DIM);
  gemmini_flush(0);

  run("gemv",     1,  64, 64, false, false);
  run("t16",     16,  64, 16, false, false);
  run("multiJ",  16, 128, 64, false, false);
  run("multiIK", 32,  64, 64, false, false);
  run("oddIK",    5,  64, 40, false, false);
  run("partJ",   16,  32, 16, false, false);
  run("bias",    16,  64, 32, true,  false);
  run("repbias", 16,  64, 32, true,  true);

  printf("=== ternary gemm: %d/%d PASS, %d mismatches ===\n",
         total_runs - total_fail, total_runs, total_err);
  exit(total_err == 0 ? 0 : 1);
}
