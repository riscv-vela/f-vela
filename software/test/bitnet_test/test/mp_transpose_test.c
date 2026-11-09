// mp_transpose_test.c — mpgemm with b_transpose (LoopMatmul "mpgemm_transpose"), never tested before.
//
// Weights W[N][K] in {-1,0,+1} come in the "original" model packing (model/i2s_original.gguf):
//   orig[n][k/4], weight k at bits [2(k%4)+1 : 2(k%4)], 00 = 0, 01 = +1, 11 = -1
// Hypothesis from the RTL (MpExeUnit in_b_transpose path: column c of a 64-column block takes spad
// row c/4, 32-bit group c%4 = 16 k-weights; LdB with transpose loads 16-byte k-chunks of B^T rows):
//   BT[N/4][K] bytes, row n4, bytes [16*kb + 4*c, +4) = orig[4*n4 + c][4*kb .. 4*kb+3]
// i.e. the original rows interleaved in 4-byte words, no bit changes. Called as a normal
// tiled_matmul with transpose_B = 1, is_mpgemm = 1, dim_J = N/4, stride_B = K (bytes).
//
// Bare-metal (Verilator / FPGA) and Linux. Checks int8 (saturated) and int32 (full_C) output.
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "gemmini.h"

static inline uint64_t read_cycles(void) { uint64_t c; asm volatile("rdcycle %0" : "=r"(c)); return c; }

#define MAXI 32
#define MAXN 256
#define MAXK 128

static uint32_t rng = 0x7E57B17Eu;
static inline uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng; }
static inline int rr(int lo, int hi) { return lo + (int)((rnd() >> 8) % (uint32_t)(hi - lo + 1)); }

static elem_t A[MAXI][MAXK] row_align(1);
static int8_t W[MAXN][MAXK];                         // reference weights
static uint8_t ORIG[MAXN][MAXK / 4];                 // original model packing
static uint8_t BT[MAXN / 4][MAXK] row_align(1);      // hypothesised transpose layout
static elem_t C8[MAXI][MAXN] row_align(1);
static acc_t C32[MAXI][MAXN] row_align_acc(1);

static int total_fail = 0, total_runs = 0;

static void run(const char *name, int I, int N, int K, int tile_I, int tile_K, bool full_C) {
  memset(A, 0, sizeof A); memset(W, 0, sizeof W); memset(ORIG, 0, sizeof ORIG); memset(BT, 0, sizeof BT);
  memset(C8, 0x55, sizeof C8); memset(C32, 0x55, sizeof C32);

  for (int i = 0; i < I; i++) for (int k = 0; k < K; k++) A[i][k] = (elem_t)rr(-8, 7);
  for (int n = 0; n < N; n++)
    for (int k = 0; k < K; k++) {
      const int w = rr(-1, 1);
      W[n][k] = (int8_t)w;
      const uint8_t code = w == 1 ? 0x1 : (w == -1 ? 0x3 : 0x0);
      ORIG[n][k / 4] |= (uint8_t)(code << (2 * (k % 4)));
    }
  for (int n4 = 0; n4 < N / 4; n4++)
    for (int kb = 0; kb < K / 16; kb++)
      for (int c = 0; c < 4; c++)
        memcpy(&BT[n4][16 * kb + 4 * c], &ORIG[4 * n4 + c][4 * kb], 4);

  // tile_J = 1 block of packed columns (64 outputs); I/K tiles as given
  const uint64_t start = read_cycles();
  tiled_matmul(I, N / 4, K, &A[0][0], (const elem_t *)&BT[0][0], NULL, full_C ? (void *)&C32[0][0] : (void *)&C8[0][0],
      MAXK, MAXK, 0, MAXN,
      MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
      NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, false,
      tile_I, 1, tile_K,
      false, true,                // transpose_A, transpose_B
      full_C, false, 0,
      WS, true, false);           // is_mpgemm
  gemmini_fence();
  const uint64_t cycles = read_cycles() - start;

  int err = 0;
  for (int i = 0; i < I; i++)
    for (int n = 0; n < N; n++) {
      acc_t acc = 0;
      for (int k = 0; k < K; k++) acc += (acc_t)A[i][k] * W[n][k];
      const acc_t ref = full_C ? acc : (acc > elem_t_max ? elem_t_max : (acc < elem_t_min ? elem_t_min : acc));
      const acc_t got = full_C ? C32[i][n] : C8[i][n];
      if (got != ref) {
        if (err < 6) printf("    [%d][%d] got=%d ref=%d\n", i, n, (int)got, (int)ref);
        err++;
      }
    }
  total_runs++;
  if (err) total_fail++;
  printf("[%s] %-8s I=%2d N=%3d K=%3d tile_I=%d tile_K=%d %s: %d mismatches, %lu cycles\n",
         err ? "FAIL" : "PASS", name, I, N, K, tile_I, tile_K, full_C ? "int32" : "int8 ", err, (unsigned long)cycles);
}

int main(void) {
#ifndef BAREMETAL
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) { perror("mlockall"); exit(1); }
#endif
  printf("=== mpgemm b_transpose (original weight packing), DIM=%d ===\n", DIM);
  gemmini_flush(0);

  run("one",     16,  64,  16, 1, 1, false);   // one block each
#ifdef SIM_SMALL   // Verilator: a few cases, small CPU reference
  run("k2",      16,  64,  32, 1, 2, false);
  run("jblocks", 16, 128,  32, 1, 2, false);
  run("int32",    5,  64,  32, 1, 2, true);
  printf("=== mp transpose: %d/%d PASS ===\n", total_runs - total_fail, total_runs);
  exit(total_fail == 0 ? 0 : 1);
#endif
  run("k4",      16,  64,  64, 1, 4, false);   // 4 k-blocks in one tile
  run("ktiles",  16,  64,  64, 1, 2, false);   // k split over 2 tiles (accumulate)
  run("jblocks", 16, 192,  64, 1, 4, false);   // 3 column tiles
  run("ioddk",    5, 128, 128, 1, 8, false);   // padded I
  run("itiles",  32,  64,  32, 1, 2, false);   // 2 row tiles
  run("int32",   16, 128, 128, 1, 8, true);    // full_C (what BitNet needs)
  run("int32i5",  5,  64,  64, 1, 4, true);

  printf("=== mp transpose: %d/%d PASS ===\n", total_runs - total_fail, total_runs);
  exit(total_fail == 0 ? 0 : 1);
}
