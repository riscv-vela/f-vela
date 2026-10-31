// See vela_npu_convolution.h.
#include "vela_npu_convolution.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <algorithm>

#include "modelbin.h"
#include "paramdict.h"

#include "vela_npu_hal.h"

// int8 convolution with exactly the NPU's arithmetic (tiled_conv_auto): int32 accumulate, + bias,
// * scale, round to nearest even, ReLU, clip to int8. Layouts: input [H][W][IC], weights
// [(kh*K + kw)*IC + ic][OC], output [OH][OW][OC]. Used as the CpuInt8 backend on the Rocket core
// and, in the host build, as the software model of the NPU.
static void cpu_conv_int8_kernel(int in_h, int in_w, int in_c, int out_c, int out_h, int out_w, int stride, int padding,
                                 int kernel, const int8_t* input, const int8_t* weights, const int32_t* bias,
                                 int8_t* output, int relu, float scale, int32_t* acc /* out_c entries */)
{
    for (int oy = 0; oy < out_h; oy++)
        for (int ox = 0; ox < out_w; ox++)
        {
            for (int oc = 0; oc < out_c; oc++) acc[oc] = bias ? bias[oc] : 0;
            for (int ky = 0; ky < kernel; ky++)
            {
                int iy = oy * stride + ky - padding;
                if (iy < 0 || iy >= in_h) continue;
                for (int kx = 0; kx < kernel; kx++)
                {
                    int ix = ox * stride + kx - padding;
                    if (ix < 0 || ix >= in_w) continue;
                    const int8_t* ip = input + ((size_t)iy * in_w + ix) * in_c;
                    const int8_t* wp = weights + (size_t)(ky * kernel + kx) * in_c * out_c;
                    for (int ic = 0; ic < in_c; ic++)
                    {
                        const int a = ip[ic];
                        const int8_t* wr = wp + (size_t)ic * out_c;
                        for (int oc = 0; oc < out_c; oc++) acc[oc] += a * wr[oc];
                    }
                }
            }
            int8_t* op = output + ((size_t)oy * out_w + ox) * out_c;
            for (int oc = 0; oc < out_c; oc++)
            {
                float v = nearbyintf(acc[oc] * scale);
                if (relu && v < 0.f) v = 0.f;
                op[oc] = (int8_t)std::min(127.f, std::max(-128.f, v));
            }
        }
}

#if !VELA_NPU
// Host build: the NPU is replaced by the same int8 kernel (software model, no cycle numbers), so the
// int8 scheme, layouts and calibration can be checked on x86 before running on the FPGA.
#define VELA_NPU_EMU 1
extern "C" int vnpu_init(size_t)
{
    return 0;
}
extern "C" int vnpu_peak_macs_per_cycle(void)
{
    return 16 * 16; // DIM 16 in software/include/gemmini_params.h
}
extern "C" void vnpu_conv_int8(int in_h, int in_w, int in_c, int out_c, int out_h, int out_w, int stride, int padding,
                               int kernel, const int8_t* input, const int8_t* weights, const int32_t* bias,
                               int8_t* output, int relu, float scale, vnpu_profile_t* prof)
{
    memset(prof, 0, sizeof(*prof));
    std::vector<int32_t> acc(out_c);
    cpu_conv_int8_kernel(in_h, in_w, in_c, out_c, out_h, out_w, stride, padding, kernel, input, weights, bias, output,
                         relu, scale, acc.data());
}
#endif

namespace vela {

Config& config()
{
    static Config c;
    return c;
}

// rdcycle on RISC-V; on the host (calibration / sanity runs) nanoseconds instead.
static inline uint64_t read_cycles()
{
#if defined(__riscv)
    uint64_t c;
    asm volatile("rdcycle %0" : "=r"(c));
    return c;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + ts.tv_nsec;
#endif
}

bool load_calib(const std::string& path)
{
    FILE* fp = fopen(path.c_str(), "r");
    if (!fp) return false;
    char name[256];
    float v;
    config().calib.clear();
    while (fscanf(fp, "%255s %f", name, &v) == 2) config().calib[name] = v;
    fclose(fp);
    return !config().calib.empty();
}

bool save_calib(const std::string& path)
{
    FILE* fp = fopen(path.c_str(), "w");
    if (!fp) return false;
    for (const auto& kv : config().calib) fprintf(fp, "%s %.8g\n", kv.first.c_str(), kv.second);
    fclose(fp);
    return true;
}

static bool g_npu_ready = false;

int npu_init(size_t profile_buffer_mb)
{
    if (g_npu_ready) return 0;
    if (vnpu_init(profile_buffer_mb) != 0) return -1;
    g_npu_ready = true;
    return 0;
}

// true on RISC-V (real velaNPU) and on the host (software model, no cycle numbers)
bool npu_available()
{
    return true;
}

int npu_peak_macs_per_cycle()
{
    return vnpu_peak_macs_per_cycle();
}

bool npu_is_emulated()
{
#if VELA_NPU
    return false;
#else
    return true;
#endif
}

static inline int8_t quant(float v, float inv_scale)
{
    int q = (int)lrintf(v * inv_scale);
    return (int8_t)std::min(127, std::max(-128, q));
}

class VelaNpuConvolution : public ncnn::Layer
{
public:
    VelaNpuConvolution()
    {
        one_blob_only = true;
        support_inplace = false;
    }

    ~VelaNpuConvolution() override
    {
        delete cpu_conv;
    }

    int load_param(const ncnn::ParamDict& pd) override
    {
        num_output = pd.get(0, 0);
        kernel_w = pd.get(1, 0);
        kernel_h = pd.get(11, kernel_w);
        dilation_w = pd.get(2, 1);
        dilation_h = pd.get(12, dilation_w);
        stride_w = pd.get(3, 1);
        stride_h = pd.get(13, stride_w);
        pad_left = pd.get(4, 0);
        pad_right = pd.get(15, pad_left);
        pad_top = pd.get(14, pad_left);
        pad_bottom = pd.get(16, pad_top);
        pad_value = pd.get(18, 0.f);
        bias_term = pd.get(5, 0);
        weight_data_size = pd.get(6, 0);
        int8_scale_term = pd.get(8, 0);
        activation_type = pd.get(9, 0);
        dynamic_weight = pd.get(19, 0);

        // ncnn's own (arch-optimized) Convolution, used for the CPU backend and for fallbacks.
        cpu_conv = ncnn::create_layer_cpu("Convolution");
        if (!cpu_conv) return -1;
        cpu_conv->name = name;
        cpu_conv->type = "Convolution";
        return cpu_conv->load_param(pd);
    }

    int load_model(const ncnn::ModelBin& mb) override
    {
        if (dynamic_weight || int8_scale_term)
            return -1; // not produced by pnnx for these models; keep it simple

        weight_data = mb.load(weight_data_size, 0);
        if (weight_data.empty()) return -100;
        if (bias_term)
        {
            bias_data = mb.load(num_output, 1);
            if (bias_data.empty()) return -100;
        }

        ncnn::Mat weights[2] = {weight_data, bias_data};
        int ret = cpu_conv->load_model(ncnn::ModelBinFromMatArray(weights));
        if (ret != 0) return ret;

        in_channels = weight_data_size / num_output / (kernel_w * kernel_h);
        fallback = npu_unsupported_reason();
        if (fallback.empty()) quantize_weights();
        return 0;
    }

    int create_pipeline(const ncnn::Option& opt) override
    {
        int ret = cpu_conv->create_pipeline(opt);
        if (ret != 0) return ret;
        if (use_int8())
        {
            // int8 path takes plain fp32 NCHW (elempack 1) input
            support_packing = false;
            support_any_packing = false;
            support_fp16_storage = false;
            support_bf16_storage = false;
            support_int8_storage = false;
        }
        else
        {
            support_packing = cpu_conv->support_packing;
            support_any_packing = cpu_conv->support_any_packing;
            support_fp16_storage = cpu_conv->support_fp16_storage;
            support_bf16_storage = cpu_conv->support_bf16_storage;
            support_int8_storage = cpu_conv->support_int8_storage;
        }
        return 0;
    }

    int destroy_pipeline(const ncnn::Option& opt) override
    {
        return cpu_conv->destroy_pipeline(opt);
    }

    int forward(const ncnn::Mat& bottom_blob, ncnn::Mat& top_blob, const ncnn::Option& opt) const override
    {
        ConvStat st;
        st.name = name;
        st.in_c = bottom_blob.c * bottom_blob.elempack;
        st.in_h = bottom_blob.h;
        st.in_w = bottom_blob.w;
        st.kernel = kernel_w;
        st.stride = stride_w;
        st.pad = pad_left;
        st.out_c = num_output;
        st.out_h = (bottom_blob.h + pad_top + pad_bottom - (dilation_h * (kernel_h - 1) + 1)) / stride_h + 1;
        st.out_w = (bottom_blob.w + pad_left + pad_right - (dilation_w * (kernel_w - 1) + 1)) / stride_w + 1;
        st.macs = (uint64_t)st.out_h * st.out_w * num_output * in_channels * kernel_w * kernel_h;
        st.cpu_cycles = st.pre_cycles = st.call_cycles = st.npu_cycles = st.post_cycles = st.npu_events = 0;
        st.npu_stale_events = 0;
        st.profile_overflow = false;
        st.npu_counter_busy = 0;
        st.npu_counter_unit[0] = st.npu_counter_unit[1] = st.npu_counter_unit[2] = 0;
        st.npu_tlb_miss_cycles = 0;
        st.int8 = false;

        int ret = 0;
        if (use_int8())
        {
            st.on_npu = use_npu();
            st.int8 = true;
            ret = forward_int8(bottom_blob, top_blob, opt, st);
        }
        else
        {
            st.on_npu = false;
            st.fallback = (config().backend == Backend::Npu || config().backend == Backend::CpuInt8) ? fallback : std::string();
            uint64_t c0 = read_cycles();
            ret = cpu_conv->forward(bottom_blob, top_blob, opt);
            st.cpu_cycles = read_cycles() - c0;
            if (config().backend == Backend::Calib) record_calib(top_blob);
        }
        config().stats.push_back(st);
        return ret;
    }

private:
    bool use_npu() const
    {
        return config().backend == Backend::Npu && fallback.empty();
    }

    // int8 path: on the NPU (Npu) or with the same int8 math on the CPU (CpuInt8)
    bool use_int8() const
    {
        return (config().backend == Backend::Npu || config().backend == Backend::CpuInt8) && fallback.empty();
    }

    std::string npu_unsupported_reason() const
    {
        if (kernel_w != kernel_h) return "non-square kernel";
        if (stride_w != stride_h) return "non-square stride";
        if (dilation_w != 1 || dilation_h != 1) return "dilation";
        if (pad_left < 0 || pad_left != pad_right || pad_left != pad_top || pad_left != pad_bottom) return "asymmetric/auto padding";
        if (pad_value != 0.f) return "pad_value";
        if (activation_type != 0 && activation_type != 1) return "activation type";
        if ((config().backend == Backend::Npu || config().backend == Backend::CpuInt8) &&
            config().calib.find(name) == config().calib.end())
            return "no calibration entry";
        return std::string();
    }

    // Per-tensor symmetric int8 weights, laid out for tiled_conv_auto: [(kh*K + kw)*IC + ic][oc].
    void quantize_weights()
    {
        const float* w = weight_data;
        float maxabs = 0.f;
        for (int i = 0; i < weight_data_size; i++) maxabs = std::max(maxabs, fabsf(w[i]));
        w_scale = maxabs > 0.f ? maxabs / 127.f : 1.f;
        const float inv = 1.f / w_scale;

        const int K = kernel_w, IC = in_channels, OC = num_output;
        w_q.assign((size_t)K * K * IC * OC, 0);
        for (int oc = 0; oc < OC; oc++)
            for (int ic = 0; ic < IC; ic++)
                for (int y = 0; y < K; y++)
                    for (int x = 0; x < K; x++)
                    {
                        float v = w[((oc * IC + ic) * K + y) * K + x];
                        w_q[((size_t)(y * K + x) * IC + ic) * OC + oc] = quant(v, inv);
                    }
    }

    // Quantize, run the int8 convolution on the NPU (Npu) or on the CPU (CpuInt8), dequantize.
    int forward_int8(const ncnn::Mat& bottom, ncnn::Mat& top, const ncnn::Option& opt, ConvStat& st) const
    {
        if (bottom.dims != 3 || bottom.elempack != 1 || bottom.elemsize != 4) return -1;
        const int W = bottom.w, H = bottom.h, IC = bottom.c, OC = num_output;
        const int OW = st.out_w, OH = st.out_h, K = kernel_w;

        uint64_t c0 = read_cycles();

        // input: fp32 NCHW -> int8 NHWC, per-tensor scale from this input's |max|
        float maxabs = 0.f;
        for (int q = 0; q < IC; q++)
        {
            const float* p = bottom.channel(q);
            for (int i = 0; i < W * H; i++) maxabs = std::max(maxabs, fabsf(p[i]));
        }
        const float in_scale = maxabs > 0.f ? maxabs / 127.f : 1.f;
        const float in_inv = 1.f / in_scale;
        std::vector<int8_t> in_q((size_t)H * W * IC);
        for (int q = 0; q < IC; q++)
        {
            const float* p = bottom.channel(q);
            for (int i = 0; i < W * H; i++) in_q[(size_t)i * IC + q] = quant(p[i], in_inv);
        }

        // bias in accumulator units, output scale from calibration
        const float acc_scale = in_scale * w_scale;
        std::vector<int32_t> bias_q(OC, 0);
        if (bias_term)
        {
            const float* b = bias_data;
            for (int oc = 0; oc < OC; oc++) bias_q[oc] = (int32_t)lrintf(b[oc] / acc_scale);
        }
        const float out_max = config().calib.at(name);
        const float out_scale = out_max > 0.f ? out_max / 127.f : 1e-6f;
        std::vector<int8_t> out_q((size_t)OH * OW * OC);

        vnpu_profile_t prof;
        memset(&prof, 0, sizeof(prof));
        std::vector<int32_t> acc(OC); // CPU kernel scratch, allocated outside the timed region

        uint64_t c1 = read_cycles();

        if (use_npu())
            vnpu_conv_int8(H, W, IC, OC, OH, OW, stride_w, pad_left, K,
                           in_q.data(), w_q.data(), bias_q.data(), out_q.data(),
                           activation_type == 1, acc_scale / out_scale, &prof);
        else
            cpu_conv_int8_kernel(H, W, IC, OC, OH, OW, stride_w, pad_left, K,
                                 in_q.data(), w_q.data(), bias_q.data(), out_q.data(),
                                 activation_type == 1, acc_scale / out_scale, acc.data());

        uint64_t c2 = read_cycles();
        if (!use_npu()) st.cpu_cycles = c2 - c1;

        // output: int8 NHWC -> fp32 NCHW
        top.create(OW, OH, OC, 4u, opt.blob_allocator);
        if (top.empty()) return -100;
        for (int oc = 0; oc < OC; oc++)
        {
            float* p = top.channel(oc);
            for (int i = 0; i < OW * OH; i++) p[i] = out_q[(size_t)i * OC + oc] * out_scale;
        }

        uint64_t c3 = read_cycles();
        st.pre_cycles = c1 - c0;
        st.call_cycles = prof.call_cycles;
        st.npu_cycles = prof.span_cycles;
        st.post_cycles = c3 - c2;
        st.npu_events = prof.total_events;
        st.npu_stale_events = prof.stale_events;
        st.profile_overflow = prof.overflow != 0;
        st.npu_counter_busy = prof.counter_busy;
        for (int t = 0; t < 3; t++) st.npu_counter_unit[t] = prof.counter_unit_busy[t];
        st.npu_tlb_miss_cycles = prof.tlb_miss_cycles;
        return 0;
    }

    void record_calib(const ncnn::Mat& top) const
    {
        if (top.elemsize / top.elempack != 4) return; // fp32 only
        float m = 0.f;
        const int n = top.w * top.h * top.d * top.elempack;
        for (int q = 0; q < top.c; q++)
        {
            const float* p = top.channel(q);
            for (int i = 0; i < n; i++) m = std::max(m, fabsf(p[i]));
        }
        float& slot = config().calib[name];
        slot = std::max(slot, m);
    }

    // ncnn Convolution parameters
    int num_output = 0, kernel_w = 0, kernel_h = 0, dilation_w = 1, dilation_h = 1;
    int stride_w = 1, stride_h = 1, pad_left = 0, pad_right = 0, pad_top = 0, pad_bottom = 0;
    float pad_value = 0.f;
    int bias_term = 0, weight_data_size = 0, int8_scale_term = 0, activation_type = 0, dynamic_weight = 0;
    int in_channels = 0;
    ncnn::Mat weight_data, bias_data;

    ncnn::Layer* cpu_conv = nullptr;
    std::string fallback;      // non-empty: NPU path not usable for this layer
    std::vector<int8_t> w_q;   // int8 weights (NPU and CpuInt8)
    float w_scale = 1.f;
};

ncnn::Layer* VelaNpuConvolution_layer_creator(void* /*userdata*/)
{
    return new VelaNpuConvolution;
}

} // namespace vela
