// Upstream / vanilla Gemmini Conv2D, timed WITHOUT a hardware profiler.
//
// Vanilla Gemmini has no Profiler.scala-style trace buffer, but it doesn't need one to get
// an accurate cycle count:
//
//   1. gemmini_fence() (include/gemmini.h) lowers to a plain RISC-V `fence` instruction.
//      On the RoCC interface, `fence` stalls Rocket's pipeline until the accelerator's
//      busy signal deasserts -- i.e. until every outstanding mvin/mvout/compute command
//      (and its DMA traffic) has actually completed, not just been issued.
//   2. rdcycle is a core CSR readable from user mode. Because Rocket only executes
//      Gemmini's RoCC instructions synchronously with backpressure (the command queue
//      blocks the core when full) and gemmini_fence() blocks until completion, a
//      rdcycle...rdcycle delta wrapped around { tiled_conv_auto(...); gemmini_fence(); }
//      is an exact, hardware-verified count of how many core cycles the whole Conv took --
//      DMA + systolic-array execution included. This is exactly the pattern already used
//      by gemmini-rocc-tests/bareMetalC/conv.c and conv_perf.c.
//
// As a bonus, vanilla Gemmini also exposes internal per-stage active-cycle counters
// (gemmini_counter.h) through counter_configure()/counter_read() -- no external profiler
// needed for a breakdown either. We read LOOP_MATMUL_ACTIVE_CYCLES/EXE_ACTIVE_CYCLE/
// RDMA_ACTIVE_CYCLE/WDMA_ACTIVE_CYCLE alongside the rdcycle total.
//
// Run this on the NoCustomFvelaTest sim (Rocket + upstream gemmini.DefaultGemminiConfig).

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "include/gemmini_testutils.h"

#include "conv_dims.h"

#define NO_BIAS false
#define PATCH_SIZE (KERNEL_DIM * KERNEL_DIM * IN_CHANNELS)
#define N_PATCHES (BATCH_SIZE * OUT_ROW_DIM * OUT_COL_DIM)

// Same FNV-1a checksum rocket_cpu_conv.c and custom_gemmini_conv_profiler.c print -- since all
// three seed identically (no srand() call anywhere) and call rand() in the same order/count,
// matching checksums are how you verify the three implementations computed the same Conv, not
// just that they took a comparable number of cycles.
static uint32_t output_checksum(const void * buf, size_t len) {
    const uint8_t * p = (const uint8_t *)buf;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

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

static void init_random(elem_t * buf, int len) {
    for (int i = 0; i < len; i++)
        buf[i] = (rand() % 5) - 2;
}

static void init_random_acc(acc_t * buf, int len) {
    for (int i = 0; i < len; i++)
        buf[i] = (rand() % 5) - 2;
}

int main() {
#ifndef BAREMETAL
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall failed");
        exit(1);
    }
#endif

    printf("==============================================\n");
    printf("  Vanilla Gemmini Conv (tiled_conv_auto)\n");
    printf("  BATCH=%d IN=%dx%dx%d OUT_CH=%d K=%d PAD=%d STRIDE=%d -> OUT=%dx%dx%d\n",
           BATCH_SIZE, IN_ROW_DIM, IN_COL_DIM, IN_CHANNELS, OUT_CHANNELS, KERNEL_DIM,
           PADDING, STRIDE, OUT_ROW_DIM, OUT_COL_DIM, OUT_CHANNELS);
    printf("==============================================\n");

    gemmini_flush(0);

    static elem_t input[BATCH_SIZE][IN_ROW_DIM][IN_COL_DIM][IN_CHANNELS];
    static elem_t weights[OUT_CHANNELS][KERNEL_DIM][KERNEL_DIM][IN_CHANNELS];
    static acc_t bias[OUT_CHANNELS];
    static elem_t weights_mat[PATCH_SIZE][OUT_CHANNELS];
    static elem_t output_mat[N_PATCHES][OUT_CHANNELS];

    init_random(&input[0][0][0][0], sizeof(input) / sizeof(elem_t));
    init_random(&weights[0][0][0][0], sizeof(weights) / sizeof(elem_t));
    init_random_acc(bias, OUT_CHANNELS);
    flatten_weights(weights, weights_mat);

    // Per-stage active-cycle counters (no external profiler required).
    counter_configure(0, LOOP_MATMUL_ACTIVE_CYCLES);
    counter_configure(1, EXE_ACTIVE_CYCLE);
    counter_configure(2, RDMA_ACTIVE_CYCLE);
    counter_configure(3, WDMA_ACTIVE_CYCLE);
    counter_reset();

    printf("Gemmini conv...\n");
    uint64_t start = read_cycles();

    tiled_conv_auto(
        BATCH_SIZE, IN_ROW_DIM, IN_COL_DIM, IN_CHANNELS,
        OUT_CHANNELS, OUT_ROW_DIM, OUT_COL_DIM,
        STRIDE, 1, 1, PADDING, KERNEL_DIM,
        false, false, false, false, false,

        (elem_t*)input,
        (elem_t*)weights_mat,
        NO_BIAS ? NULL : (acc_t*)bias,
        (elem_t*)output_mat,

        NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, 0, 0,

        WS);

    // Belt-and-suspenders: tiled_conv_auto already ends on a fence internally, but an
    // explicit fence here is what actually guarantees the rdcycle below only fires once
    // every DMA write Gemmini issued has landed (see file header).
    gemmini_fence();

    uint64_t end = read_cycles();

    // LD/EX/ST aliases line up with rocket_cpu_conv.c's and custom_gemmini_conv_profiler.c's
    // labels: RDMA (loading operands in) ~ ld, EXE (systolic MACs) ~ ex, WDMA (writing
    // results out) ~ st. Unlike the CPU, these can overlap in HW, so LD+EX+ST may exceed the
    // total wall-clock cycles above -- that gap *is* the DMA/compute pipelining.
    printf("RESULT vanilla_gemmini_conv cycles: %lu\n", end - start);
    printf("  LD_CYCLES (RDMA_ACTIVE_CYCLE):        %u\n", counter_read(2));
    printf("  EX_CYCLES (EXE_ACTIVE_CYCLE):         %u\n", counter_read(1));
    printf("  ST_CYCLES (WDMA_ACTIVE_CYCLE):        %u\n", counter_read(3));
    printf("  LOOP_MATMUL_ACTIVE_CYCLES (bonus):    %u\n", counter_read(0));
    printf("RESULT output_checksum: 0x%08x\n", output_checksum(&output_mat[0][0], sizeof(output_mat)));

    printf("DONE\n");
    return 0;
}
