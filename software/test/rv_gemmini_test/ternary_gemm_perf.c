// ternary_gemm_perf.c — large ternary-weight GEMM (mpgemm mode, is_mpgemm = 1): performance run + CPU check.
//
//   C[i][j] = sat8( A[i][:] . W[:][j] + D[i][j] ),   A: int8,  W: {-1, 0, +1}
//
// Same operation as ternary_gemm.c (same packing: 4 weights/byte, 0b01 = +1,
// 0b11 = -1, 0b00 = 0, column j = 4*jp + n in bits [2n+1:2n]) at a large size, to
// measure cycles / DMA traffic. The result is still compared with a CPU reference.
//
// Size defaults to 256 x 256 x 256; override for quick simulation, e.g.
//   -DMAT_DIM_I=64 -DMAT_DIM_J=64 -DMAT_DIM_K=64

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "gemmini_testutils.h"

#ifndef MAT_DIM_I
#define MAT_DIM_I 256
#endif
#ifndef MAT_DIM_K
#define MAT_DIM_K 256
#endif
#ifndef MAT_DIM_J
#define MAT_DIM_J 256   // logical (unpacked) columns; must be a multiple of 4
#endif

// Bias is off by default: in mpgemm mode the hardware D loader (LoopMatmul ldD) loads
// only the first 1/4 of the D columns (it is missing the max_j * 4 that stC has), so
// with bias the other 3/4 of the columns are wrong. Build with -DNO_BIAS=0 to check
// that path.
#ifndef NO_BIAS
#define NO_BIAS 1
#endif
#ifndef REPEATING_BIAS
#define REPEATING_BIAS 0
#endif
#define PRINT_CORNER 8  // print the top-left PRINT_CORNER x PRINT_CORNER of C (0 = off)

#define A_STRIDE MAT_DIM_K
#define B_STRIDE MAT_DIM_J  // logical stride; ternary_gemm_auto divides it by 4
#define D_STRIDE MAT_DIM_J
#define C_STRIDE MAT_DIM_J

static uint32_t rng = 0xB17E7000u;
static inline uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng; }
static inline int rr(int lo, int hi) { return lo + (int)((rnd() >> 8) % (uint32_t)(hi - lo + 1)); }

static elem_t full_A[MAT_DIM_I][MAT_DIM_K] row_align(1);
static elem_t full_B[MAT_DIM_K][MAT_DIM_J / 4] row_align(1);  // packed weights
static elem_t full_C[MAT_DIM_I][MAT_DIM_J] row_align(1);
static acc_t  full_D[MAT_DIM_I][MAT_DIM_J] row_align_acc(1);

static inline int ternary_decode(uint8_t packed, int n) {
  const int v = (packed >> (2 * n)) & 0x3;
  return v == 0x1 ? 1 : (v == 0x3 ? -1 : 0);
}

static void init_mats(void) {
  for (size_t i = 0; i < MAT_DIM_I; ++i)
    for (size_t k = 0; k < MAT_DIM_K; ++k)
      full_A[i][k] = (elem_t)rr(-8, 7);

  for (size_t k = 0; k < MAT_DIM_K; ++k)
    for (size_t jp = 0; jp < MAT_DIM_J / 4; ++jp) {
      uint8_t packed = 0;
      for (int n = 0; n < 4; ++n) {
        const int w = rr(-1, 1);
        const uint8_t code = w == 1 ? 0x1 : (w == -1 ? 0x3 : 0x0);
        packed |= code << (2 * n);
      }
      full_B[k][jp] = (elem_t)packed;
    }

  for (size_t i = 0; i < MAT_DIM_I; ++i)
    for (size_t j = 0; j < MAT_DIM_J; ++j)
      full_D[i][j] = NO_BIAS ? 0 : rr(-64, 64);
}

static int check(void) {
  int err = 0;
  for (size_t i = 0; i < MAT_DIM_I; ++i)
    for (size_t j = 0; j < MAT_DIM_J; ++j) {
      acc_t acc = NO_BIAS ? 0 : full_D[REPEATING_BIAS ? 0 : i][j];
      for (size_t k = 0; k < MAT_DIM_K; ++k)
        acc += (acc_t)full_A[i][k] * ternary_decode((uint8_t)full_B[k][j / 4], j % 4);
      const elem_t ref = acc > elem_t_max ? elem_t_max : (acc < elem_t_min ? elem_t_min : (elem_t)acc);
      if (full_C[i][j] != ref) {
        if (err < 8) printf("    [%u][%u] got=%d ref=%d\n", (unsigned)i, (unsigned)j, (int)full_C[i][j], (int)ref);
        err++;
      }
    }
  return err;
}

int main(void) {
#ifndef BAREMETAL
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    perror("mlockall failed");
    exit(1);
  }
#endif

  gemmini_flush(0);
  init_mats();

  counter_configure(0, RDMA_BYTES_REC);
  counter_configure(1, WDMA_BYTES_SENT);
  counter_reset();

  printf("=== ternary gemm perf (mpgemm mode) ===\n");
  printf("I: %d, J: %d, K: %d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
  printf("NO_BIAS: %d, REPEATING_BIAS: %d\n", NO_BIAS, REPEATING_BIAS);

  const uint64_t start = read_cycles();
  ternary_gemm_auto(MAT_DIM_I, MAT_DIM_J, MAT_DIM_K,
      &full_A[0][0], &full_B[0][0], NO_BIAS ? NULL : &full_D[0][0], &full_C[0][0],
      A_STRIDE, B_STRIDE, D_STRIDE, C_STRIDE,
      MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
      NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, REPEATING_BIAS,
      false, false);                   // full_C, low_D
  gemmini_fence();
  const uint64_t end = read_cycles();

  const uint64_t total_macs = (uint64_t)MAT_DIM_I * MAT_DIM_J * MAT_DIM_K;
  const uint64_t ideal_cycles = total_macs / (DIM * DIM);  // int8 DIM x DIM array as the baseline
  printf("Cycles taken: %lu\n", (unsigned long)(end - start));
  printf("Total macs: %lu\n", (unsigned long)total_macs);
  printf("Ideal cycles (int8 DIMxDIM): %lu\n", (unsigned long)ideal_cycles);
  printf("Utilization (vs int8): %lu%%\n", (unsigned long)(100 * ideal_cycles / (end - start)));
  printf("RDMA_BYTES_REC: %u\n", (unsigned)counter_read(0));
  printf("WDMA_BYTES_SENT: %u\n", (unsigned)counter_read(1));

#if PRINT_CORNER > 0
  printf("C[0:%d][0:%d]:\n", PRINT_CORNER, PRINT_CORNER);
  for (int i = 0; i < PRINT_CORNER && i < MAT_DIM_I; ++i) {
    for (int j = 0; j < PRINT_CORNER && j < MAT_DIM_J; ++j) printf("%4d ", (int)full_C[i][j]);
    printf("\n");
  }
#endif

  const int err = check();
  printf("=== ternary gemm perf: %s, %d mismatches ===\n", err ? "FAIL" : "PASS", err);
  exit(err == 0 ? 0 : 1);
}
