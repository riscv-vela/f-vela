// Thin C interface to the velaNPU (custom Gemmini) for the ncnn VelaNpuConvolution layer.
// Implemented in vela_npu_hal.c (plain C, because pf_gemmini.h is a C header), and for the
// upstream, unmodified Gemmini in gemmini_stock_hal.c (cycles from Gemmini's perf counters).
#ifndef VELA_NPU_HAL_H
#define VELA_NPU_HAL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Profiler result for one NPU call.
typedef struct
{
    uint64_t span_cycles;    // first event start -> last event end (NPU clock = core clock)
    uint64_t call_cycles;    // rdcycle around tiled_conv_auto + fence (includes driver/tiling overhead)
    uint64_t busy_cycles[3]; // sum of event durations per type: 0=ld(mvin) 1=ex(compute) 2=st(mvout)
    uint64_t events[3];      // event count per type
    uint64_t total_events;
    uint64_t stale_events;   // entries outside this call's time window (late writes), ignored
    int overflow;            // 1 if the profile buffer filled up (span is then incomplete)
    // Gemmini performance counters (gemmini_perf_counters.h), read identically on velaNPU and stock
    // Gemmini -> the like-for-like metric between the two
    uint64_t counter_busy;         // cycles with the load, execute or store controller busy
    uint64_t counter_unit_busy[3]; // 0 = ld, 1 = ex, 2 = st busy
    uint64_t tlb_miss_cycles;      // DMA_TLB_MISS_CYCLE
} vnpu_profile_t;

// Peak int8 MACs per cycle of the systolic array (DIM*DIM), for utilization.
int vnpu_peak_macs_per_cycle(void);

// One-time setup: flush the accelerator TLB and register a profile buffer of `buffer_mb` MiB.
// buffer_mb == 0: no hardware profiler; span_cycles / busy_cycles then come from the Gemmini perf
// counters (ld|ex|st controller busy), the same method as gemmini_stock_hal.c. Returns 0 on success.
int vnpu_init(size_t buffer_mb);

// int8 convolution on the NPU (tiled_conv_auto, weight-stationary), batch 1, square kernel/stride,
// symmetric padding. Layouts: input [in_h][in_w][in_c], weights [k*k*in_c][out_c]
// (row = (kh*k + kw)*in_c + ic), bias [out_c] int32 (may be NULL), output [out_h][out_w][out_c].
// out = clip_int8(round((conv + bias) * scale)), ReLU applied if relu != 0.
// Waits for completion (gemmini_fence) and fills *prof from the hardware profiler.
void vnpu_conv_int8(int in_h, int in_w, int in_c, int out_c, int out_h, int out_w,
                    int stride, int padding, int kernel,
                    const int8_t* input, const int8_t* weights, const int32_t* bias, int8_t* output,
                    int relu, float scale, vnpu_profile_t* prof);

// fp16 matmul on the NPU's FpExeUnit (gemmini_fp16.h tiled_matmul_auto_fp, WS): fp16 multiply,
// fp32 accumulate. Only in the HAL built with -DVNPU_FP=1 (needs a support_fp bitstream).
//   C[M][N] = D[M][N] + A[M][K] @ B[K][N]
//   A, B: IEEE half bit patterns, row strides lda / ldb in elements
//   D: fp32 bias (may be NULL), row stride ldd; ldd == 0 = one row [N] repeated for every row
//      (Gemmini repeating_bias: the NPU loads N floats instead of an M x N matrix)
//   C: fp32 (row stride ldc)
// Waits for completion (gemmini_fence) and fills *prof like vnpu_conv_int8.
void vnpu_matmul_fp16(int M, int N, int K,
                      const uint16_t* A, int lda, const uint16_t* B, int ldb,
                      const float* D, int ldd, float* C, int ldc, vnpu_profile_t* prof);

#ifdef __cplusplus
}
#endif

#endif // VELA_NPU_HAL_H
