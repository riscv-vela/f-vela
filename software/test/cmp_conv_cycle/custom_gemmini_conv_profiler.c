// Custom (F-Vela) Gemmini Conv2D, timed with the real HW profiler.
//
// Same flow as profiler_test/gemm_profiler_test.c, but the workload is tiled_conv_auto()
// instead of a matmul:
//   1. Allocate a P[] array the accelerator fills with profile data.
//   2. Hand its address to the accelerator with gemmini_profiler(P).
//   3. Run the Conv (tiled_conv_auto), so the profiler records load/execute/store
//      events into P[].
//   4. gemmini_fence() so every DMA write to P[] has landed.
//   5. Decode P[] and print each event as "type, start, end" between PROFILE-BEGIN/END
//      markers -- same format run_profiler.sh / profiler/profile.py already parse.
//
// Each P[] entry is packed by hardware as Cat(q[2b], start_time[31b], end_cycle[31b]):
//     type  = (entry >> 62) & 0x3      // 0 = ld, 1 = ex, 2 = st
//     start = (entry >> 31) & 0x7FFFFFFF
//     end   =  entry        & 0x7FFFFFFF
//
// Also prints the plain rdcycle wall-clock total (start/end around the fenced call), so this
// single top-line number is directly comparable to rocket_cpu_conv's and vanilla_gemmini_conv's.
// The per-event durations are additionally summed by type into LD_CYCLES/EX_CYCLES/ST_CYCLES
// totals, using the exact same labels rocket_cpu_conv.c and vanilla_gemmini_conv.c use.
//
// Does custom Gemmini *need* the profiler for a correct number, or would the plain
// counter_configure()/counter_read() counters (the ones vanilla_gemmini_conv.c uses) work
// here too, even though the systolic array's PE datapath was extended for ternary matmul?
// CounterFile.scala (f_vela_gemmini) is byte-for-byte identical to upstream gemmini's, and
// ExecuteController.scala wires EXE_ACTIVE_CYCLE to `firing || matmul_in_progress` -- signals
// that are generic to the control FSM, not specific to the int8 vs. ternary PE datapath. So
// the counters should still be valid. This file configures them too (LOOP_MATMUL/EXE/RDMA/
// WDMA_ACTIVE_CYCLE, printed as LD/EX/ST like vanilla_gemmini_conv.c) purely to check that
// empirically: if they land in the same ballpark as the profiler-summed LD/EX/ST above, the
// counters work fine and the profiler is only needed when you want the fine-grained
// per-instruction timeline (e.g. for the load/execute/store bar graph), not for a basic
// cycle count.
//
// Run this on the FVelaSoCConfigTest sim (Rocket + f_vela_gemmini.GemminiCustomConfig).

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif
#include "pf_gemmini.h"

#include "conv_dims.h"

#define NO_BIAS false
#define PATCH_SIZE (KERNEL_DIM * KERNEL_DIM * IN_CHANNELS)
#define N_PATCHES (BATCH_SIZE * OUT_ROW_DIM * OUT_COL_DIM)

// Same FNV-1a checksum rocket_cpu_conv.c and vanilla_gemmini_conv.c print -- since all three
// seed identically (no srand() call anywhere) and call rand() in the same order/count, matching
// checksums are how you verify the three implementations computed the same Conv, not just that
// they took a comparable number of cycles.
static uint32_t output_checksum(const void * buf, size_t len) {
    const uint8_t * p = (const uint8_t *)buf;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static uint64_t read_cycles(void) {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// ---- Profile buffer (see profiler_test/gemm_profiler_test.c for the format) ------------
#define PROFILE_MAX_ENTRIES 4096
static volatile uint64_t P[PROFILE_MAX_ENTRIES] row_align(1);

#define PROFILE_TYPE(e)  ((int)     (((e) >> 62) & 0x3ULL))
#define PROFILE_START(e) ((uint32_t)(((e) >> 31) & 0x7FFFFFFFULL))
#define PROFILE_END(e)   ((uint32_t)( (e)        & 0x7FFFFFFFULL))

// Dump every captured event, and also sum each type's (end-start) durations into
// LD_CYCLES/EX_CYCLES/ST_CYCLES totals -- the same labels rocket_cpu_conv.c and
// vanilla_gemmini_conv.c use, so the three are directly comparable. Because HW pipelines
// ld/ex/st across many small tiled instructions, these per-type events overlap each other in
// time; summing their individual durations (rather than looking at min-start/max-end) is what
// makes this number comparable to the *active*-cycle counters vanilla_gemmini_conv.c reads,
// not to the overall wall-clock total.
static void dump_profile(void) {
    printf("=== PROFILE DUMP BEGIN ===\n");
    int count = 0;
    uint64_t type_cycles[3] = {0, 0, 0};
    int type_count[3] = {0, 0, 0};
    for (int i = 0; i < PROFILE_MAX_ENTRIES; i++) {
        uint64_t e = P[i];
        if (e == 0) break;   // first empty slot => end of records
        int type = PROFILE_TYPE(e);
        printf("%d, %u, %u\n", type, PROFILE_START(e), PROFILE_END(e));
        if (type >= 0 && type <= 2) {
            type_cycles[type] += PROFILE_END(e) - PROFILE_START(e);
            type_count[type]++;
        }
        count++;
    }
    printf("=== PROFILE DUMP END ===\n");
    printf("Captured %d profile events (0=ld,1=ex,2=st).\n", count);
    printf("  LD_CYCLES (profiler, %d events): %lu\n", type_count[0], type_cycles[0]);
    printf("  EX_CYCLES (profiler, %d events): %lu\n", type_count[1], type_cycles[1]);
    printf("  ST_CYCLES (profiler, %d events): %lu\n", type_count[2], type_cycles[2]);
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
    printf("  Custom (F-Vela) Gemmini Conv Profiler Test\n");
    printf("  BATCH=%d IN=%dx%dx%d OUT_CH=%d K=%d PAD=%d STRIDE=%d -> OUT=%dx%dx%d\n",
           BATCH_SIZE, IN_ROW_DIM, IN_COL_DIM, IN_CHANNELS, OUT_CHANNELS, KERNEL_DIM,
           PADDING, STRIDE, OUT_ROW_DIM, OUT_COL_DIM, OUT_CHANNELS);
    printf("  P buffer @ %p (%d entries)\n", (void *)P, PROFILE_MAX_ENTRIES);
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

    // Same active-cycle counters vanilla_gemmini_conv.c uses -- see file header: this is here
    // to check whether they still work on the custom (ternary-capable) systolic array.
    counter_configure(0, LOOP_MATMUL_ACTIVE_CYCLES);
    counter_configure(1, EXE_ACTIVE_CYCLE);
    counter_configure(2, RDMA_ACTIVE_CYCLE);
    counter_configure(3, WDMA_ACTIVE_CYCLE);
    counter_reset();

    // Register the profile buffer with the accelerator.
    gemmini_profiler(P);

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

    gemmini_fence();

    uint64_t end = read_cycles();

    printf("RESULT custom_gemmini_conv cycles: %lu\n", end - start);

    // If these land near the profiler's summed LD/EX/ST below, the plain HW counters are
    // still valid on the custom systolic array and the profiler isn't required just to get a
    // cycle count -- see the file header note.
    printf("  LD_CYCLES (counter, RDMA_ACTIVE_CYCLE): %u\n", counter_read(2));
    printf("  EX_CYCLES (counter, EXE_ACTIVE_CYCLE):  %u\n", counter_read(1));
    printf("  ST_CYCLES (counter, WDMA_ACTIVE_CYCLE): %u\n", counter_read(3));
    printf("  LOOP_MATMUL_ACTIVE_CYCLES (bonus):      %u\n", counter_read(0));
    printf("RESULT output_checksum: 0x%08x\n", output_checksum(&output_mat[0][0], sizeof(output_mat)));

    dump_profile();

    printf("PROFILER TEST DONE\n");
    return 0;
}
