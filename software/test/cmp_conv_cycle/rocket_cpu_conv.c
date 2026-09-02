// Rocket Core (RISC-V scalar, no accelerator) reference Conv2D, timed with rdcycle.
//
// This program calls no RoCC / Gemmini instruction at all -- it is pure C running
// on the Rocket core. Because of that it is header-free (no gemmini_testutils.h)
// and the exact same binary can be run on *any* SoC config: NoCustomFvelaTest
// (vanilla Gemmini) and FVelaSoCConfigTest (custom Gemmini) alike. Run it once per
// simulator to get the "CPU-only" baseline cycle count for each comparison.
//
// ---- ld / ex / st breakdown -------------------------------------------------------------
// Both Gemmini profilers (see vanilla_gemmini_conv.c's counter_configure() and
// custom_gemmini_conv_profiler.c's gemmini_profiler() dump) report their cycles split into
// three stages: ld (move data into the systolic array), ex (systolic MAC execution), st
// (move results back out). For the comparison to be apples-to-apples, this file computes
// Conv the same way tiled_conv_auto() does internally -- im2col, then a plain matmul -- and
// times the same three stages explicitly:
//   ld: gather the zero-padded receptive fields out of `input` into an im2col patch matrix
//       (the software equivalent of Gemmini's mvin: getting operands into a compute-ready
//       layout)
//   ex: the patch-matrix x weight-matrix MAC loop (the actual convolution arithmetic --
//       equivalent of Gemmini's systolic execute)
//   st: reshape/copy the flat result matrix into the caller-visible output tensor layout
//       (equivalent of Gemmini's mvout)
// On a scalar core these three stages run strictly back-to-back with no overlap, so
// ld+ex+st == the total -- unlike the accelerators, where DMA and compute can pipeline, so
// their stage totals can each be a large fraction of a *smaller* overall wall-clock number.
// That difference is itself one of the interesting things this comparison should surface.

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif

#include "conv_dims.h"

typedef int8_t elem_t;
typedef int32_t acc_t;
#define ELEM_T_MIN INT8_MIN
#define ELEM_T_MAX INT8_MAX

#define PATCH_SIZE (KERNEL_DIM * KERNEL_DIM * IN_CHANNELS)
#define N_PATCHES (BATCH_SIZE * OUT_ROW_DIM * OUT_COL_DIM)

static uint64_t read_cycles(void) {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// FNV-1a over the raw output bytes. All three conv programs seed identically (none call
// srand(), so libc's default seed applies) and call rand() in the same order/count, so their
// inputs are bit-identical; this checksum is how you actually verify the three implementations
// computed the same Conv, as opposed to just comparing how long each one took.
static uint32_t output_checksum(const void * buf, size_t len) {
    const uint8_t * p = (const uint8_t *)buf;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

// Emit the ld/ex/st stage timestamps as a "type, start, end" PROFILE DUMP block, same format
// the HW profilers (custom_gemmini_conv_profiler.c, ternary_profiler_test.c) use -- so
// run_cmp.sh / profiler_test/plot/profile.py can extract and plot this CPU run the same way
// (0=ld, 1=ex, 2=st). On CPU the three stages run back-to-back, so this is just t0..t3.
static void dump_profile(uint64_t t0, uint64_t t1, uint64_t t2, uint64_t t3) {
    printf("=== PROFILE DUMP BEGIN ===\n");
    printf("0, %lu, %lu\n", t0, t1);
    printf("1, %lu, %lu\n", t1, t2);
    printf("2, %lu, %lu\n", t2, t3);
    printf("=== PROFILE DUMP END ===\n");
    printf("Captured 3 profile events (0=ld,1=ex,2=st).\n");
}

static void init_random(elem_t * buf, int len) {
    for (int i = 0; i < len; i++)
        buf[i] = (rand() % 5) - 2;
}

static void init_random_acc(acc_t * buf, int len) {
    for (int i = 0; i < len; i++)
        buf[i] = (rand() % 5) - 2;
}

// weights[OUT_CHANNELS][KERNEL_DIM][KERNEL_DIM][IN_CHANNELS] -> weights_mat[PATCH_SIZE][OUT_CHANNELS]
// Not timed (same as vanilla/custom: laying weights out for the compute engine is a one-time
// data-layout step, done identically -- and outside the timed region -- in all three programs).
static void flatten_weights(
        elem_t weights[OUT_CHANNELS][KERNEL_DIM][KERNEL_DIM][IN_CHANNELS],
        elem_t weights_mat[PATCH_SIZE][OUT_CHANNELS]) {

    for (int outc = 0; outc < OUT_CHANNELS; outc++) {
        for (int krow = 0; krow < KERNEL_DIM; krow++) {
            for (int kcol = 0; kcol < KERNEL_DIM; kcol++) {
                for (int inc = 0; inc < IN_CHANNELS; inc++) {
                    int wmatrow = krow * KERNEL_DIM * IN_CHANNELS + kcol * IN_CHANNELS + inc;
                    weights_mat[wmatrow][outc] = weights[outc][krow][kcol][inc];
                }
            }
        }
    }
}

// ld: input[BATCH][IN_ROW][IN_COL][IN_CHANNELS] -> im2col_buf[N_PATCHES][PATCH_SIZE]
static void im2col_gather(
        elem_t input[BATCH_SIZE][IN_ROW_DIM][IN_COL_DIM][IN_CHANNELS],
        elem_t im2col_buf[N_PATCHES][PATCH_SIZE]) {

    for (int b = 0; b < BATCH_SIZE; b++) {
        for (int orow = 0; orow < OUT_ROW_DIM; orow++) {
            for (int ocol = 0; ocol < OUT_COL_DIM; ocol++) {
                int patch_row = (b * OUT_ROW_DIM + orow) * OUT_COL_DIM + ocol;

                for (int krow = 0; krow < KERNEL_DIM; krow++) {
                    for (int kcol = 0; kcol < KERNEL_DIM; kcol++) {
                        int irow = orow * STRIDE + krow - PADDING;
                        int icol = ocol * STRIDE + kcol - PADDING;
                        int patch_col_base = (krow * KERNEL_DIM + kcol) * IN_CHANNELS;

                        for (int kch = 0; kch < IN_CHANNELS; kch++) {
                            elem_t pixel = irow < 0 || irow >= IN_ROW_DIM ||
                                icol < 0 || icol >= IN_COL_DIM ?
                                0 : input[b][irow][icol][kch];
                            im2col_buf[patch_row][patch_col_base + kch] = pixel;
                        }
                    }
                }
            }
        }
    }
}

// ex: output_mat[N_PATCHES][OUT_CHANNELS] = im2col_buf @ weights_mat + bias, clipped.
static void matmul_ex(
        elem_t im2col_buf[N_PATCHES][PATCH_SIZE],
        elem_t weights_mat[PATCH_SIZE][OUT_CHANNELS],
        acc_t bias[OUT_CHANNELS],
        elem_t output_mat[N_PATCHES][OUT_CHANNELS]) {

    for (int i = 0; i < N_PATCHES; i++) {
        for (int j = 0; j < OUT_CHANNELS; j++) {
            acc_t result = bias[j];
            for (int k = 0; k < PATCH_SIZE; k++)
                result += im2col_buf[i][k] * weights_mat[k][j];

            result = result > ELEM_T_MAX ? ELEM_T_MAX : (result < ELEM_T_MIN ? ELEM_T_MIN : result);
            output_mat[i][j] = result;
        }
    }
}

// st: output_mat[N_PATCHES][OUT_CHANNELS] -> output[BATCH][OUT_ROW][OUT_COL][OUT_CHANNELS].
// N_PATCHES == BATCH_SIZE*OUT_ROW_DIM*OUT_COL_DIM in the same (b,orow,ocol) order im2col_gather
// used, so this is a straight reshape/copy -- the software equivalent of Gemmini's mvout
// writing the accumulator's tile-order results into the caller's tensor layout.
static void writeback_st(
        elem_t output_mat[N_PATCHES][OUT_CHANNELS],
        elem_t output[BATCH_SIZE][OUT_ROW_DIM][OUT_COL_DIM][OUT_CHANNELS]) {

    elem_t * dst = &output[0][0][0][0];
    elem_t * src = &output_mat[0][0];
    for (int i = 0; i < N_PATCHES * OUT_CHANNELS; i++)
        dst[i] = src[i];
}

int main() {
#ifndef BAREMETAL
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall failed");
        exit(1);
    }
#endif

    printf("==============================================\n");
    printf("  Rocket CPU Conv (no accelerator, im2col+matmul)\n");
    printf("  BATCH=%d IN=%dx%dx%d OUT_CH=%d K=%d PAD=%d STRIDE=%d -> OUT=%dx%dx%d\n",
           BATCH_SIZE, IN_ROW_DIM, IN_COL_DIM, IN_CHANNELS, OUT_CHANNELS, KERNEL_DIM,
           PADDING, STRIDE, OUT_ROW_DIM, OUT_COL_DIM, OUT_CHANNELS);
    printf("==============================================\n");

    static elem_t input[BATCH_SIZE][IN_ROW_DIM][IN_COL_DIM][IN_CHANNELS];
    static elem_t weights[OUT_CHANNELS][KERNEL_DIM][KERNEL_DIM][IN_CHANNELS];
    static acc_t bias[OUT_CHANNELS];
    static elem_t weights_mat[PATCH_SIZE][OUT_CHANNELS];
    static elem_t im2col_buf[N_PATCHES][PATCH_SIZE];
    static elem_t output_mat[N_PATCHES][OUT_CHANNELS];
    static elem_t output[BATCH_SIZE][OUT_ROW_DIM][OUT_COL_DIM][OUT_CHANNELS];

    init_random(&input[0][0][0][0], sizeof(input) / sizeof(elem_t));
    init_random(&weights[0][0][0][0], sizeof(weights) / sizeof(elem_t));
    init_random_acc(bias, OUT_CHANNELS);
    flatten_weights(weights, weights_mat);

    printf("Rocket CPU conv...\n");
    uint64_t t0 = read_cycles();
    im2col_gather(input, im2col_buf);
    uint64_t t1 = read_cycles();
    matmul_ex(im2col_buf, weights_mat, bias, output_mat);
    uint64_t t2 = read_cycles();
    writeback_st(output_mat, output);
    uint64_t t3 = read_cycles();

    uint64_t ld_cycles = t1 - t0;
    uint64_t ex_cycles = t2 - t1;
    uint64_t st_cycles = t3 - t2;

    printf("RESULT rocket_cpu_conv cycles: %lu\n", t3 - t0);
    printf("  LD_CYCLES (im2col gather): %lu\n", ld_cycles);
    printf("  EX_CYCLES (MAC/matmul):    %lu\n", ex_cycles);
    printf("  ST_CYCLES (writeback):     %lu\n", st_cycles);
    printf("RESULT output_checksum: 0x%08x\n", output_checksum(&output[0][0][0][0], sizeof(output)));

    dump_profile(t0, t1, t2, t3);

    printf("DONE\n");
    return 0;
}
