// fp16_matmul.c — fp16 matmul (runtime is_fp mode) on a support_fp Gemmini.
//
//   C[i][j] = ((D[i][j] or 0) + fp16(A) @ fp16(B)) * scale
//
// The hardware multiplies in fp16 and accumulates in fp32, so the reference does the
// same: each product is re-quantized to fp16, then summed in fp32. Inputs are small
// integers (or 0.5-steps) so every step is exact and the compare is bit-exact.
//
// Cases:
//   known  : A = B = 1 -> C = K (full and partial J)
//   dims   : random data, full / partial / 1.5-tile I, J, K (fp32 output)
//   bias   : fp32 bias D, single and 2 K-tiles
//   xpose  : transpose_B with odd partial dims
//   int8   : output_as_float = 0 -> C = sat8(rne(acc * scale)), incl. ties and saturation
//   i8mm   : plain int8 matmul on the int unit, interleaved between the fp16 cases. The fp
//            unit is slower than the int unit (MpWontolic delays the int output to match),
//            so every int<->fp switch checks that no rows are dropped or misaddressed.
//
// Requires Fp16MatmulGemminiConfig (support_fp). Needs no interrupts: tiled_matmul
// fences at the end of every call.

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "include/gemmini.h"
#include "include/gemmini_fp16.h"

#define MAXM 32

static uint32_t rng = 0xBEEF1234u;
static inline uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng; }
static inline int rr(int lo, int hi) { return lo + (int)((rnd() >> 8) % (uint32_t)(hi - lo + 1)); }

static fp16_t A[MAXM][MAXM], B[MAXM][MAXM];
static fp32_t D[MAXM][MAXM], C[MAXM][MAXM];
static elem_t C8[MAXM][MAXM];
static elem_t A8[MAXM][MAXM], B8[MAXM][MAXM];

static int total_err = 0, total_runs = 0, total_fail = 0;

// fp16 multiply, fp32 accumulate (what the hardware computes)
static inline float fp16_mul(fp16_t a, fp16_t b) {
  return fp16_to_float(fp16_from_float(fp16_to_float(a) * fp16_to_float(b)));
}

static void clear_all(void) {
  for (int i = 0; i < MAXM; i++)
    for (int j = 0; j < MAXM; j++) {
      A[i][j] = B[i][j] = fp16_from_float(0.0f);
      D[i][j] = C[i][j] = 0.0f;
      C8[i][j] = 0;
    }
}

static void report(const char *name, int I, int J, int K, int err) {
  total_err += err; total_runs++; if (err) total_fail++;
  printf("[%s] %-8s I=%d J=%d K=%d : %d mismatches\n", err ? "FAIL" : "PASS", name, I, J, K, err);
}

// B is [K][J] normally, [J][K] when transpose_B. All strides are MAXM.
static void run_fp32(const char *name, int I, int J, int K, int bias, int transpose_B, int known) {
  clear_all();
  for (int i = 0; i < I; i++) for (int k = 0; k < K; k++)
    A[i][k] = fp16_from_float(known ? 1.0f : (float)rr(-4, 4));
  for (int k = 0; k < K; k++) for (int j = 0; j < J; j++) {
    fp16_t w = fp16_from_float(known ? 1.0f : (float)rr(-4, 4));
    if (transpose_B) B[j][k] = w; else B[k][j] = w;
  }
  if (bias)
    for (int i = 0; i < I; i++) for (int j = 0; j < J; j++) D[i][j] = (float)rr(-32, 32);

  tiled_matmul_auto_fp(I, J, K, &A[0][0], &B[0][0], bias ? &D[0][0] : NULL, &C[0][0],
      MAXM, MAXM, MAXM, MAXM, transpose_B, 1 /*output_as_float*/, ACC_SCALE_IDENTITY);

  int err = 0;
  for (int i = 0; i < I; i++) for (int j = 0; j < J; j++) {
    float ref = bias ? D[i][j] : 0.0f;   // acc starts at D, then accumulates
    for (int k = 0; k < K; k++)
      ref += fp16_mul(A[i][k], transpose_B ? B[j][k] : B[k][j]);
    union { float f; uint32_t u; } g = { C[i][j] }, r = { ref };
    if (g.u != r.u) {
      if (err < 4) printf("    [%d][%d] got=0x%08x ref=0x%08x\n", i, j, (unsigned)g.u, (unsigned)r.u);
      err++;
    }
  }
  report(name, I, J, K, err);
}

static void run_int8(const char *name, int I, int J, int K, float scale) {
  clear_all();
  // 0.5-steps in [-2,2]: fp16 products and the fp32 sum are exact, so the only
  // inexact step is the final round-to-int (ties exercise round-near-even).
  for (int i = 0; i < I; i++) for (int k = 0; k < K; k++) A[i][k] = fp16_from_float(0.5f * (float)rr(-4, 4));
  for (int k = 0; k < K; k++) for (int j = 0; j < J; j++) B[k][j] = fp16_from_float(0.5f * (float)rr(-4, 4));

  tiled_matmul_auto_fp(I, J, K, &A[0][0], &B[0][0], NULL, (fp32_t *)&C8[0][0],
      MAXM, MAXM, 0, MAXM /*stride_C in int8 elements*/, 0, 0 /*output_as_float*/, scale);

  int err = 0;
  for (int i = 0; i < I; i++) for (int j = 0; j < J; j++) {
    float acc = 0.0f;
    for (int k = 0; k < K; k++) acc += fp16_mul(A[i][k], B[k][j]);
    const elem_t ref = (elem_t)ACC_SCALE(acc, scale);   // sat8(rne(acc*scale))
    if (C8[i][j] != ref) {
      if (err < 4) printf("    [%d][%d] got=%d ref=%d (acc=%d/4)\n", i, j, (int)C8[i][j], (int)ref, (int)(acc * 4));
      err++;
    }
  }
  report(name, I, J, K, err);
}

static void run_i8mm(const char *name, int I, int J, int K) {
  clear_all();
  for (int i = 0; i < MAXM; i++) for (int j = 0; j < MAXM; j++) A8[i][j] = B8[i][j] = 0;
  for (int i = 0; i < I; i++) for (int k = 0; k < K; k++) A8[i][k] = (elem_t)rr(-4, 4);
  for (int k = 0; k < K; k++) for (int j = 0; j < J; j++) B8[k][j] = (elem_t)rr(-1, 1);

  tiled_matmul_auto(I, J, K, &A8[0][0], &B8[0][0], NULL, &C8[0][0],
      MAXM, MAXM, MAXM, MAXM,
      MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
      NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, false,
      false, false, false, false, 0, WS);

  int err = 0;
  for (int i = 0; i < I; i++) for (int j = 0; j < J; j++) {
    int32_t acc = 0;
    for (int k = 0; k < K; k++) acc += (int32_t)A8[i][k] * (int32_t)B8[k][j];
    const elem_t ref = acc > elem_t_max ? elem_t_max : (acc < elem_t_min ? elem_t_min : (elem_t)acc);
    if (C8[i][j] != ref) {
      if (err < 4) printf("    [%d][%d] got=%d ref=%d\n", i, j, (int)C8[i][j], (int)ref);
      err++;
    }
  }
  report(name, I, J, K, err);
}

int main(void) {
#ifndef BAREMETAL
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) { perror("mlockall"); exit(1); }
#endif
  printf("=== fp16 matmul (DIM=%d, WS) ===\n", DIM);
  gemmini_flush(0);

  // int8 before any fp op
  run_i8mm("i8mm",    16, 16, 16);

  // known values: C = K everywhere
  run_fp32("known",   16, 16, 16, 0, 0, 1);
  run_i8mm("i8mm",    16, 16, 16);   // fp -> int switch
  run_fp32("knownJ8", 16,  8, 16, 0, 0, 1);

  // partial and multi-tile dims
  run_fp32("d16",  16, 16, 16, 0, 0, 0);
  run_fp32("I8",    8, 16, 16, 0, 0, 0);
  run_fp32("J8",   16,  8, 16, 0, 0, 0);
  run_fp32("K8",   16, 16,  8, 0, 0, 0);
  run_fp32("I24",  24, 16, 16, 0, 0, 0);
  run_fp32("J24",  16, 24, 16, 0, 0, 0);
  run_fp32("K24",  16, 16, 24, 0, 0, 0);
  run_i8mm("i8mmJ24", 16, 24, 16);

  // fp32 bias D
  run_fp32("bias",   16, 16, 16, 1, 0, 0);
  run_fp32("biasK32", 16, 16, 32, 1, 0, 0);
  run_i8mm("i8mmK24", 16, 16, 24);

  // transpose_B, odd partial dims
  run_fp32("xd16",  16, 16, 16, 0, 1, 0);
  run_fp32("xI1J5",  1,  5, 16, 0, 1, 0);
  run_fp32("xJ13",  16, 13, 16, 0, 1, 0);
  run_fp32("xK13",  16, 16, 13, 0, 1, 0);

  // int8 output
  run_int8("int8x1", 16, 16, 16, 1.0f);
  run_int8("int8x.5", 16, 16, 16, 0.5f);
  run_int8("int8x8", 16, 16, 16, 8.0f);

  // int8 after all fp ops
  run_i8mm("i8mm",    16, 16, 16);

  printf("=== fp16 matmul: %d/%d PASS, %d mismatches ===\n",
         total_runs - total_fail, total_runs, total_err);
  exit(total_err == 0 ? 0 : 1);
}
