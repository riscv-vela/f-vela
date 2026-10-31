// Stock (upstream, unmodified) Gemmini backend for the ncnn VelaNpuConvolution layer.
// Implements the same interface as vela_npu_hal.c (vela_npu_hal.h), so the ncnn layer, the int8
// scheme and the calibration table are shared; only the driver header and the cycle source differ.
//
// Target: FireSimNoCustomGemminiOnlyTest (chipyard.NoCustomGemminiOnlyTest = gemmini.DefaultGemminiConfig
// on a Rocket HugeCore without vector unit; DIM 16 int8, WS/OS, 256 KiB scratchpad, 64 KiB accumulator,
// 8 perf counters).
// Driver: generators/gemmini/software/gemmini-rocc-tests/include/gemmini.h (no F-Vela additions).
//
// Stock Gemmini has no hardware profiler (SET_PROFILER_PADDR is an F-Vela addition). Cycles come
// from Gemmini's performance counters (gemmini_perf_counters.h, read the same way on velaNPU):
//   span_cycles = counter_busy = cycles with the load, execute or store controller busy.
// call_cycles = rdcycle around tiled_conv_auto + fence, as in vela_npu_hal.c.
#include "vela_npu_hal.h"

#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "include/gemmini.h" // -I generators/gemmini/software/gemmini-rocc-tests

#include "gemmini_perf_counters.h"

static inline uint64_t rdcycle(void)
{
    uint64_t c;
    asm volatile("rdcycle %0" : "=r"(c));
    return c;
}

int vnpu_peak_macs_per_cycle(void)
{
    return DIM * DIM; // systolic array: DIM x DIM int8 MACs per cycle
}

// buffer_mb is unused (no profiler buffer on stock Gemmini).
int vnpu_init(size_t buffer_mb)
{
    (void)buffer_mb;
    // Gemmini walks the page table itself (virtual addresses); a page fault on its side is not
    // handled by Linux, so keep every buffer resident. (-DVNPU_SPIKE: functional check under
    // spike --extension=gemmini + pk, which has no mlockall.)
#ifndef VNPU_SPIKE
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
        perror("vnpu_init: mlockall");
        return -1;
    }
#endif
    gemmini_flush(0);
    gpc_init();
    return 0;
}

void vnpu_conv_int8(int in_h, int in_w, int in_c, int out_c, int out_h, int out_w,
                    int stride, int padding, int kernel,
                    const int8_t* input, const int8_t* weights, const int32_t* bias, int8_t* output,
                    int relu, float scale, vnpu_profile_t* prof)
{
    memset(prof, 0, sizeof(*prof));

    gemmini_fence(); // nothing of a previous call still counting
    gpc_start();

    uint64_t c0 = rdcycle();
    tiled_conv_auto(
        1, in_h, in_w, in_c,
        out_c, out_h, out_w,
        stride, 1, 1, padding, kernel,
        false, false, false, false, false,
        (const elem_t*)input, (const elem_t*)weights, (const acc_t*)bias, (elem_t*)output,
        relu ? RELU : NO_ACTIVATION, (acc_scale_t)scale, 0, 0, 0,
        WS);
    gemmini_fence();
    uint64_t c1 = rdcycle();
    prof->call_cycles = c1 - c0;

    gpc_result_t r;
    gpc_stop(&r);
    prof->counter_busy = r.busy;
    for (int t = 0; t < 3; t++) prof->counter_unit_busy[t] = prof->busy_cycles[t] = r.unit[t];
    prof->tlb_miss_cycles = r.tlb_miss;
    prof->span_cycles = r.busy;
}
