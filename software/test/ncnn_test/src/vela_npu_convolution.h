// VelaNpuConvolution: ncnn Convolution replacement with a velaNPU backend.
//
// Registered per Net with net.register_custom_layer("Convolution", ...), so ncnn's own
// Convolution layer is untouched and still used for everything else.
//
//   Backend::Cpu   NPU backend disabled. Runs ncnn's stock Convolution (Rocket core) and records
//                  the rdcycle difference around each convolution.
//   Backend::Npu   NPU backend enabled. Quantizes to int8 and runs the convolution on the velaNPU
//                  (tiled_conv_auto). Cycles come from the NPU hardware profiler (first event start
//                  -> last event end). Layers the NPU path does not support fall back to Cpu.
//   Backend::CpuInt8 NPU backend disabled, int8 on the Rocket core: exactly the NPU's int8 scheme
//                  (same quantization, layouts, int32 accumulate, bias, output scale, ReLU, clip),
//                  but the convolution kernel runs as scalar integer code. Cycles: rdcycle around
//                  the int8 kernel (quantize/dequantize reported separately, as for the NPU).
//                  The output is bit-identical to the NPU's if the NPU is correct.
//   Backend::Calib Runs like Cpu and records each layer's output |max| into a calibration table,
//                  which Npu and CpuInt8 need for the int8 output scale.
//   Backend::NpuFp16 NPU backend enabled, fp16 on the NPU's FpExeUnit (fp16 multiply, fp32
//                  accumulate; needs a support_fp bitstream and the HAL built with -DVNPU_FP=1).
//                  The fp mode has no hardware conv loop, so the CPU does im2col (fp32 NCHW ->
//                  fp16 [OH*OW][K*K*IC]) and the NPU runs the matmul with the bias as fp32 D; ReLU
//                  and fp32 NHWC -> NCHW are done on the CPU. No calibration table needed.
#ifndef VELA_NPU_CONVOLUTION_H
#define VELA_NPU_CONVOLUTION_H

#include <stdint.h>

#include <map>
#include <string>
#include <vector>

#include "layer.h"

namespace vela {

enum class Backend
{
    Cpu,
    Npu,
    Calib,
    CpuInt8,
    NpuFp16
};

struct ConvStat
{
    std::string name;
    int in_c, in_h, in_w, out_c, out_h, out_w, kernel, stride, pad;
    uint64_t macs;
    bool on_npu;          // false: ran on the CPU (Cpu/Calib/CpuInt8 backend, or Npu fallback)
    bool int8;            // int8 path (NPU, or CpuInt8 on the CPU); false: ncnn fp32 Convolution
    bool fp16;            // NpuFp16 path (fp16 matmul on the NPU)
    std::string fallback; // why the NPU path was not used (empty if on_npu)
    uint64_t cpu_cycles;  // CPU conv: rdcycle around ncnn's Convolution::forward (fp32) or the int8 kernel
    uint64_t pre_cycles;  // int8 path: CPU-side int8 quantize + NCHW->NHWC; fp16: alloc + fp16 convert + im2col (rdcycle)
    uint64_t call_cycles; // NPU: rdcycle around tiled_conv_auto / tiled_matmul_auto_fp + fence
    uint64_t wrap_cycles; // NPU: rdcycle around the whole HAL call = call_cycles + perf counter and
                          // profiler setup/readout (measurement overhead, not part of the conv)
    uint64_t npu_cycles;  // NPU: profiler span, conv start -> completion
    uint64_t post_cycles; // int8 path: CPU-side dequantize + NHWC->NCHW; fp16: ReLU + NHWC->NCHW (rdcycle)
    // fp16 path, parts of pre_cycles: buffers are allocated once per layer (first call, i.e. warmup)
    uint64_t alloc_cycles;  // im2col / output buffers + bias row (0 after the first call)
    uint64_t cvt_cycles;    // input fp32 -> fp16 (once per input element, before im2col)
    uint64_t im2col_cycles; // fp16 im2col gather
    uint64_t npu_events;
    uint64_t npu_stale_events; // profiler entries outside this call's window (ignored)
    bool profile_overflow;
    // NPU: Gemmini perf counters, measured the same way on velaNPU and stock Gemmini
    uint64_t npu_counter_busy;       // ld, ex or st controller busy
    uint64_t npu_counter_unit[3];    // ld / ex / st busy
    uint64_t npu_tlb_miss_cycles;    // DMA TLB miss cycles
};

struct Config
{
    Backend backend = Backend::Cpu;
    std::map<std::string, float> calib; // layer name -> output |max| (fp32)
    std::vector<ConvStat> stats;        // filled during forward, in execution order
};

Config& config();

// Calibration table file: one "layer_name max_abs" pair per line.
bool load_calib(const std::string& path);
bool save_calib(const std::string& path);

// Must be called once before the first NPU convolution (no-op without VELA_NPU).
int npu_init(size_t profile_buffer_mb);
bool npu_available();
bool npu_is_emulated(); // host build: software model of the NPU int8 conv, no cycle numbers
int npu_peak_macs_per_cycle();

ncnn::Layer* VelaNpuConvolution_layer_creator(void* userdata);

} // namespace vela

#endif // VELA_NPU_CONVOLUTION_H
