// NCNN convolution test executables for the AI acceleration system test procedure:
//   cpu_lenet.riscv  cpu_vgg16.riscv  cpu_resnet18.riscv   NPU backend disabled (Rocket core)
//   npu_lenet.riscv  npu_vgg16.riscv  npu_resnet18.riscv   NPU backend enabled  (velaNPU)
//   gemmini_lenet.riscv gemmini_vgg16.riscv gemmini_resnet18.riscv
//                                                          NPU backend on the upstream, unmodified
//                                                          Gemmini (FireSimNoCustomGemminiOnlyTest)
//   npu_lenet_fp.riscv  velaNPU fp16 (FpExeUnit, fp16 multiply / fp32 accumulate; support_fp bitstream)
//   cpu_lenet_fp.riscv  Rocket core fp32 (ncnn Convolution), the CPU baseline for npu_*_fp
//                       (-DTEST_CONV_FP=1; npu_*_fp also needs the HAL built with -DVNPU_FP=1)
// Built from this one file with -DTEST_MODEL_ID=0|1|2 -DTEST_BACKEND_NPU=0|1 (and
// -DTEST_NPU_STOCK_GEMMINI=1 + gemmini_stock_hal.c for the gemmini_* ones), so CPU, velaNPU and
// stock Gemmini use the same code, the same .param/.bin, the same int8 scheme and the same input.
//
// usage: <exe> [--seed N] [-r repeat] [-w warmup] [-d model_root] [--freq-hz F] [--npu-prof-mb N]
//              [--profiler hw|counter] [--fp32] [-s] [--metric conv|e2e|all]
//   --profiler (npu_* incl. npu_*_fp): where the NPU conv cycles come from
//     hw      (default) velaNPU hardware profiler, first event start -> last event end
//     counter Gemmini perf counters only, profiler off: the same method as gemmini_* on stock Gemmini
//             (like-for-like comparison, no profiler memory traffic)
//   gemmini_* always use the counters (stock Gemmini has no profiler).
//   -s / --summary: print only the key metrics (conv cycles/time/GOPS/utilization, freq, repeat,
//   inference time, STATUS) instead of the full per-layer report
//   --metric (-m): which metrics are reported
//     all  (default) conv-only metrics and the end-to-end (whole network) metrics
//     conv conv-only metrics (CONV_* / TOTAL.CONV_*), as before the E2E metrics were added
//     e2e  end-to-end metrics (E2E_*); in summary mode the conv-only lines are left out
//   defaults: --seed 1234, model_root /media/ncnn_models, freq 1 GHz = target clock of the FireSim SoC
//   (FireSimFVelaSoCConfig: one 1000 MHz clock domain, timebase 1 MHz; the U280 only emulates it at 60 MHz)
//
// CPU executables run the convolutions in int8 by default (Backend::CpuInt8): the NPU's int8 scheme
// (same quantization, int32 accumulate, bias, output scale, ReLU, clip) as scalar integer code on the
// Rocket core, so CPU and NPU do the same work and their outputs should be bit-identical
// (OUTPUT.CHECKSUM). --fp32 runs ncnn's own fp32 Convolution instead (the earlier behaviour).
//
// Convolution cycles per layer (median over `repeat` runs):
//   CPU int8: rdcycle before/after the int8 conv kernel (quantize/dequantize reported separately)
//   CPU fp32 (--fp32): rdcycle before/after ncnn's Convolution::forward
//   NPU: velaNPU hardware profiler, first event start -> last event end
//   NPU fp16: same profiler around the fp16 matmul; CPU im2col + fp16 convert (+ bias D) is
//        CPU_QUANT_CYCLE and ReLU + NHWC->NCHW is CPU_DEQUANT_CYCLE
//   --profiler counter (velaNPU, profiler off) and stock Gemmini (no profiler): Gemmini perf counters,
//        cycles with the load, execute or store controller busy; rdcycle around the call is in NPU_CALL_CYCLE
// Like-for-like NPU metrics, measured identically on velaNPU and stock Gemmini:
//   NPU_COUNTER_BUSY_CYCLE (Gemmini perf counters, gemmini_perf_counters.h) and NPU_CALL_CYCLE.
// OPS = 2 x MACs. TIME = cycles / CLOCK_FREQ_HZ. GOP/s = OPS / TIME.
// UTILIZATION = OPS / (cycles x PEAK_OPS_PER_CYCLE):
//   CPU int8 peak 1 OPS/cycle: one int8 MAC per 2 cycles. The Rocket integer multiplier is iterative
//            (mulUnroll 8, not pipelined); with early-out an 8-bit multiply takes at least 2 cycles
//            (8 cycles when the multiplier operand is negative)
//   CPU fp32 peak 2 OPS/cycle (one fp32 FMA per cycle on the pipelined scalar Rocket FPU)
//   NPU peak 2 x DIM x DIM OPS/cycle (int8 systolic array; the fp16 FpExeUnit has the same DIM x DIM lanes)
// The output is compared with the PyTorch reference for the same seed (ref_seed<seed>.bin):
// STATUS: PASS requires top-1 match and cos >= 0.98 (int8: NPU and CPU) / 0.999 (CPU fp32, NPU fp16).
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <algorithm>
#include <string>
#include <vector>

#include "net.h"
#include "vela_npu_convolution.h"

#ifndef TEST_MODEL_ID
#define TEST_MODEL_ID 0
#endif
#ifndef TEST_BACKEND_NPU
#define TEST_BACKEND_NPU 0
#endif
#ifndef TEST_NPU_STOCK_GEMMINI
#define TEST_NPU_STOCK_GEMMINI 0
#endif
// fp convolutions: NPU fp16 (FpExeUnit) / CPU fp32 by default (cpu_<model>_fp, npu_<model>_fp)
#ifndef TEST_CONV_FP
#define TEST_CONV_FP 0
#endif

#if TEST_CONV_FP
static const char* kNpuName = "velaNPU";
static const char* kNpuExec = "velaNPU fp16 (CPU im2col + tiled_matmul_auto_fp, WS, fp32 accumulate)";
static const char* kNpuProfilerSource = "NPU hardware profiler (matmul start -> completion)";
#elif TEST_NPU_STOCK_GEMMINI
static const char* kNpuName = "stock Gemmini";
static const char* kNpuExec = "stock Gemmini int8 (tiled_conv_auto, WS)";
static const char* kNpuProfilerSource = NULL; // no profiler: always the counters
#else
static const char* kNpuName = "velaNPU";
static const char* kNpuExec = "velaNPU int8 (tiled_conv_auto, WS)";
static const char* kNpuProfilerSource = "NPU hardware profiler (conv start -> completion)";
#endif
static const char* kNpuCounterSource = TEST_NPU_STOCK_GEMMINI
                                           ? "Gemmini perf counters (ld|ex|st controller busy cycles)"
                                           : "Gemmini perf counters (ld|ex|st controller busy cycles, profiler off)";

struct ModelSpec
{
    const char* id;      // directory / file prefix
    const char* display; // TEST_MODEL
    int c, h, w, num_out;
    int repeat;          // default number of measured runs
};

static const ModelSpec kModels[] = {
    {"lenet", "LeNet", 1, 32, 32, 10, 5},
    {"vgg16", "VGG16", 3, 224, 224, 1000, 3},
    {"resnet18", "ResNet18", 3, 224, 224, 1000, 3},
};

// Same generator as seeded_input() in models/export_models.py (bit-identical in float32).
static void seeded_input(float* x, size_t n, uint64_t seed)
{
    uint64_t state = seed;
    for (size_t i = 0; i < n; i++)
    {
        state += 0x9E3779B97F4A7C15ull;
        uint64_t z = state;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        x[i] = (float)(uint32_t)(z >> 40) * (1.0f / 16777216.0f) * 4.0f - 2.0f;
    }
}

// rdcycle on RISC-V (target cycles); on the host (sanity runs) nanoseconds instead.
static inline uint64_t cycles_now()
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

// MACs of the InnerProduct (FC) layers from the .param file: "2=" is the weight count = in x out.
static uint64_t fc_macs_from_param(const std::string& path, int* n_fc)
{
    *n_fc = 0;
    FILE* fp = fopen(path.c_str(), "r");
    if (!fp) return 0;
    uint64_t macs = 0;
    char line[1024];
    while (fgets(line, sizeof(line), fp))
    {
        if (strncmp(line, "InnerProduct", 12) != 0) continue;
        const char* w = strstr(line, " 2=");
        if (!w) continue;
        macs += strtoull(w + 3, NULL, 10);
        (*n_fc)++;
    }
    fclose(fp);
    return macs;
}

static double now_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static bool read_floats(const std::string& path, std::vector<float>& out, size_t count)
{
    FILE* fp = fopen(path.c_str(), "rb");
    if (!fp) return false;
    out.resize(count);
    size_t n = fread(out.data(), sizeof(float), count, fp);
    fclose(fp);
    return n == count;
}

static uint64_t median_u64(std::vector<uint64_t> v)
{
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return n % 2 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

static double median_d(std::vector<double> v)
{
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return n % 2 ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

static int argmax(const float* v, int n)
{
    return (int)(std::max_element(v, v + n) - v);
}

// -s / --summary: only the key metrics block at the end; DETAIL() prints the full report otherwise
static bool g_summary = false;
#define DETAIL(...)                              \
    do                                           \
    {                                            \
        if (!g_summary) printf(__VA_ARGS__);     \
    } while (0)

int main(int argc, char** argv)
{
    const ModelSpec& m = kModels[TEST_MODEL_ID];
    const bool npu = TEST_BACKEND_NPU != 0;

    uint64_t seed = 1234;
    int repeat = m.repeat, warmup = 1, prof_mb = 128;
    double freq_hz = 1e9;
    bool cpu_fp32 = TEST_CONV_FP != 0;
    bool use_counter = TEST_NPU_STOCK_GEMMINI != 0; // --profiler counter
    bool bad_arg = false;
    bool show_conv = true, show_e2e = true; // --metric
    std::string root = "/media/ncnn_models";
    for (int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        if (a == "--seed" && i + 1 < argc) seed = strtoull(argv[++i], NULL, 0);
        else if (a == "-r" && i + 1 < argc) repeat = std::max(1, atoi(argv[++i]));
        else if (a == "-w" && i + 1 < argc) warmup = std::max(0, atoi(argv[++i]));
        else if (a == "-d" && i + 1 < argc) root = argv[++i];
        else if (a == "--freq-hz" && i + 1 < argc) freq_hz = atof(argv[++i]);
        else if (a == "--npu-prof-mb" && i + 1 < argc) prof_mb = std::max(1, atoi(argv[++i]));
        else if (a == "--fp32" && !npu) cpu_fp32 = true;
        else if (a == "-s" || a == "--summary") g_summary = true;
        else if ((a == "--metric" || a == "-m") && i + 1 < argc)
        {
            const std::string v = argv[++i];
            if (v == "conv") show_conv = true, show_e2e = false;
            else if (v == "e2e") show_conv = false, show_e2e = true;
            else if (v == "all") show_conv = show_e2e = true;
            else bad_arg = true;
        }
        else if ((a == "--profiler" || a == "-p") && i + 1 < argc && npu)
        {
            const std::string v = argv[++i];
            if (v == "counter") use_counter = true;
            else if (v == "hw" && kNpuProfilerSource) use_counter = false;
            else bad_arg = true; // stock Gemmini has no hardware profiler
        }
        else bad_arg = true;
        if (bad_arg)
        {
            fprintf(stderr, "usage: %s [--seed N] [-r repeat] [-w warmup] [-d model_root] [--freq-hz F]%s%s"
                            " [-s|--summary] [-m|--metric conv|e2e|all]\n",
                    argv[0],
                    npu ? (kNpuProfilerSource ? " [--npu-prof-mb N] [--profiler hw|counter]" : " [--profiler counter]") : "",
                    npu ? "" : " [--fp32]");
            return 2;
        }
    }
    const char* npu_cycle_source = use_counter ? kNpuCounterSource : kNpuProfilerSource;
    const size_t npu_prof_mb = use_counter ? 0 : (size_t)prof_mb; // 0: HAL leaves the profiler off

    const char* exe = strrchr(argv[0], '/') ? strrchr(argv[0], '/') + 1 : argv[0];
    const std::string dir = root + "/" + m.id + "/";
    const std::string param_name = std::string(m.id) + ".ncnn.param";
    const std::string bin_name = std::string(m.id) + ".ncnn.bin";
    const bool npu_fp16 = npu && TEST_CONV_FP;
    const bool int8 = !TEST_CONV_FP && (npu || !cpu_fp32);
    // CPU int8: 1 MAC per 2 cycles (iterative integer multiplier); CPU fp32: 1 FMA per cycle
    const int kCpuInt8PeakOps = 1, kCpuFp32PeakOps = 2;
    const int cpu_peak_ops = cpu_fp32 ? kCpuFp32PeakOps : kCpuInt8PeakOps;
    const int peak_ops = npu ? 2 * vela::npu_peak_macs_per_cycle() : cpu_peak_ops;

    std::vector<std::string> errors;

    DETAIL("=== %s 예제 시작 ===\n", exe);
    DETAIL("TEST_MODEL: %s\n", m.display);
    DETAIL("BACKEND: %s\n", npu ? "NPU" : "CPU");
    if (npu) DETAIL("NPU_HW: %s\n", kNpuName);
    DETAIL("PARAM_FILE: %s\n", param_name.c_str());
    DETAIL("BIN_FILE: %s\n", bin_name.c_str());
    DETAIL("MODEL_DIR: %s\n", dir.c_str());
    DETAIL("INPUT_TYPE: FP32\n");
    DETAIL("INPUT_SHAPE: %dx%dx%d\n", m.c, m.h, m.w);
    DETAIL("INPUT_SEED: %llu\n", (unsigned long long)seed);
    DETAIL("CONV_DATA_TYPE: %s\n", int8 ? "INT8 (int32 accumulate)" : (npu_fp16 ? "FP16 (fp32 accumulate)" : "FP32"));
    DETAIL("CLOCK_FREQ_HZ: %.0f\n", freq_hz);
    DETAIL("CONV_EXEC: %s\n", npu ? kNpuExec
                                   : (int8 ? "Rocket core int8 (scalar integer MAC, same int8 quantization as the NPU)"
                                           : "Rocket core fp32 (ncnn Convolution)"));
    DETAIL("CYCLE_SOURCE: %s\n", npu ? npu_cycle_source
                                      : (int8 ? "rdcycle (before/after the int8 conv kernel)" : "rdcycle (before/after convolution)"));
    DETAIL("PEAK_OPS_PER_CYCLE: %d\n", peak_ops);
    DETAIL("REPEAT: %d\n", repeat);
    DETAIL("WARMUP: %d\n", warmup);
    fflush(stdout);

    // ---- set up the Net with the VelaNpuConvolution layer ------------------------------------
    vela::Config& vc = vela::config();
    vc.backend = npu_fp16 ? vela::Backend::NpuFp16
                          : (npu ? vela::Backend::Npu : (int8 ? vela::Backend::CpuInt8 : vela::Backend::Cpu));
    if (int8)
    {
        // int8 output scales: the same calibration table for the NPU and the CPU int8 path
        if (!vela::load_calib(dir + m.id + ".npu_calib"))
            errors.push_back("missing int8 calibration " + dir + m.id + ".npu_calib");
        else if (npu && vela::npu_init(npu_prof_mb) != 0)
            errors.push_back(std::string(kNpuName) + " init failed");
    }
    else if (npu_fp16 && vela::npu_init(npu_prof_mb) != 0) // fp16 needs no calibration table
        errors.push_back(std::string(kNpuName) + " init failed");

    ncnn::Net net;
    net.opt.num_threads = 1;
    net.opt.use_fp16_packed = false;
    net.opt.use_fp16_storage = false;
    net.opt.use_fp16_arithmetic = false;
    net.opt.use_bf16_storage = false;
    net.register_custom_layer("Convolution", vela::VelaNpuConvolution_layer_creator);

    std::vector<std::vector<vela::ConvStat> > runs;
    std::vector<double> wall_ms;
    std::vector<uint64_t> e2e_cyc; // rdcycle around ex.input + ex.extract (whole network)
    ncnn::Mat out;

    if (errors.empty() && (net.load_param((dir + param_name).c_str()) != 0 || net.load_model((dir + bin_name).c_str()) != 0))
        errors.push_back("cannot load " + dir + param_name + " / " + bin_name);

    if (errors.empty())
    {
        ncnn::Mat in(m.w, m.h, m.c);
        std::vector<float> x((size_t)m.c * m.h * m.w);
        seeded_input(x.data(), x.size(), seed);
        for (int q = 0; q < m.c; q++) memcpy(in.channel(q), x.data() + (size_t)q * m.h * m.w, sizeof(float) * m.h * m.w);

        for (int i = 0; i < warmup + repeat; i++)
        {
            vc.stats.clear();
            ncnn::Extractor ex = net.create_extractor();
            double t0 = now_ms();
            uint64_t k0 = cycles_now();
            ex.input("in0", in);
            if (ex.extract("out0", out) != 0)
            {
                errors.push_back("inference failed");
                break;
            }
            uint64_t k1 = cycles_now() - k0;
            double t1 = now_ms() - t0;
            DETAIL("RUN[%d]: %s %.3f ms\n", i, i < warmup ? "warmup" : "measure", t1);
            fflush(stdout);
            if (i >= warmup)
            {
                runs.push_back(vc.stats);
                wall_ms.push_back(t1);
                e2e_cyc.push_back(k1);
            }
        }
    }

    // ---- per-layer report --------------------------------------------------------------------
    uint64_t total_ops = 0, total_cycle = 0, total_pre = 0, total_post = 0, total_call = 0;
    uint64_t total_ctr = 0, total_unit[3] = {0, 0, 0};
    uint64_t total_alloc = 0, total_cvt = 0, total_im2col = 0;
    int n_layers = runs.empty() ? 0 : (int)runs[0].size();
    for (int li = 0; li < n_layers; li++)
    {
        const vela::ConvStat& s0 = runs[0][li];
        std::vector<uint64_t> cyc, pre, post, call, ctr, unit[3], tlb, alloc, cvt, im2col;
        uint64_t stale = 0;
        bool overflow = false;
        for (const auto& r : runs)
        {
            if ((int)r.size() != n_layers) continue;
            const vela::ConvStat& s = r[li];
            cyc.push_back(s.on_npu ? s.npu_cycles : s.cpu_cycles);
            pre.push_back(s.pre_cycles);
            post.push_back(s.post_cycles);
            call.push_back(s.call_cycles);
            ctr.push_back(s.npu_counter_busy);
            for (int t = 0; t < 3; t++) unit[t].push_back(s.npu_counter_unit[t]);
            tlb.push_back(s.npu_tlb_miss_cycles);
            alloc.push_back(s.alloc_cycles);
            cvt.push_back(s.cvt_cycles);
            im2col.push_back(s.im2col_cycles);
            stale += s.npu_stale_events;
            overflow |= s.profile_overflow;
        }
        const uint64_t ops = 2 * s0.macs;
        const uint64_t mc = median_u64(cyc);
        const double t_us = mc / freq_hz * 1e6;
        const int layer_peak = s0.on_npu ? peak_ops : (s0.int8 ? kCpuInt8PeakOps : kCpuFp32PeakOps);
        const double util = mc ? 100.0 * ops / ((double)mc * layer_peak) : 0;
        const double gops = mc ? ops / (mc / freq_hz) / 1e9 : 0;
        total_ops += ops;
        total_cycle += mc;

        DETAIL("LAYER[%d].NAME: %s\n", li, s0.name.c_str());
        DETAIL("LAYER[%d].SHAPE: %dx%dx%d -> %dx%dx%d k%d s%d p%d\n", li, s0.in_c, s0.in_h, s0.in_w, s0.out_c, s0.out_h,
               s0.out_w, s0.kernel, s0.stride, s0.pad);
        DETAIL("LAYER[%d].EXEC: %s\n", li,
               s0.on_npu ? (s0.fp16 ? "NPU fp16" : "NPU")
                         : (!s0.fallback.empty() ? ("CPU fp32 (fallback: " + s0.fallback + ")").c_str()
                                                 : (s0.int8 ? "CPU int8" : "CPU fp32")));
        DETAIL("LAYER[%d].OPS: %llu\n", li, (unsigned long long)ops);
        DETAIL("LAYER[%d].MEDIAN_CYCLE: %llu\n", li, (unsigned long long)mc);
        DETAIL("LAYER[%d].TIME_US: %.3f\n", li, t_us);
        DETAIL("LAYER[%d].UTILIZATION: %.2f%%\n", li, util);
        DETAIL("LAYER[%d].GOPS: %.4f\n", li, gops);
        if (s0.int8 && !s0.on_npu)
        {
            // CPU int8: same CPU-side conversion work as the NPU path, reported the same way
            const uint64_t mp = median_u64(pre), mq = median_u64(post);
            total_pre += mp;
            total_post += mq;
            DETAIL("LAYER[%d].CPU_QUANT_CYCLE: %llu\n", li, (unsigned long long)mp);
            DETAIL("LAYER[%d].CPU_DEQUANT_CYCLE: %llu\n", li, (unsigned long long)mq);
        }
        if (s0.on_npu)
        {
            const uint64_t mp = median_u64(pre), mq = median_u64(post), mcall = median_u64(call);
            total_pre += mp;
            total_post += mq;
            total_call += mcall;
            DETAIL("LAYER[%d].NPU_CALL_CYCLE: %llu\n", li, (unsigned long long)mcall);
            if (s0.fp16)
            {
                // fp16: no quantization; CPU work before / after the NPU matmul
                const uint64_t ma = median_u64(alloc), mv = median_u64(cvt), mi = median_u64(im2col);
                total_alloc += ma;
                total_cvt += mv;
                total_im2col += mi;
                DETAIL("LAYER[%d].CPU_PRE_CYCLE: %llu\n", li, (unsigned long long)mp);
                DETAIL("LAYER[%d].CPU_PRE.ALLOC_CYCLE: %llu\n", li, (unsigned long long)ma);
                DETAIL("LAYER[%d].CPU_PRE.FP16_CVT_CYCLE: %llu\n", li, (unsigned long long)mv);
                DETAIL("LAYER[%d].CPU_PRE.IM2COL_CYCLE: %llu\n", li, (unsigned long long)mi);
                DETAIL("LAYER[%d].CPU_POST_CYCLE: %llu\n", li, (unsigned long long)mq);
            }
            else
            {
                DETAIL("LAYER[%d].CPU_QUANT_CYCLE: %llu\n", li, (unsigned long long)mp);
                DETAIL("LAYER[%d].CPU_DEQUANT_CYCLE: %llu\n", li, (unsigned long long)mq);
            }
            // Gemmini perf counters: same method on velaNPU and stock Gemmini (like-for-like metric)
            const uint64_t mctr = median_u64(ctr);
            const uint64_t bl = median_u64(unit[0]), be = median_u64(unit[1]), bs = median_u64(unit[2]);
            total_ctr += mctr;
            total_unit[0] += bl;
            total_unit[1] += be;
            total_unit[2] += bs;
            DETAIL("LAYER[%d].NPU_COUNTER_BUSY_CYCLE: %llu\n", li, (unsigned long long)mctr);
            DETAIL("LAYER[%d].NPU_LD_BUSY_CYCLE: %llu\n", li, (unsigned long long)bl);
            DETAIL("LAYER[%d].NPU_EX_BUSY_CYCLE: %llu\n", li, (unsigned long long)be);
            DETAIL("LAYER[%d].NPU_ST_BUSY_CYCLE: %llu\n", li, (unsigned long long)bs);
            DETAIL("LAYER[%d].NPU_TLB_MISS_CYCLE: %llu\n", li, (unsigned long long)median_u64(tlb));
            if (stale) DETAIL("LAYER[%d].PROFILER_STALE_EVENTS: %llu\n", li, (unsigned long long)stale);
            if (overflow) errors.push_back("profile buffer overflow at " + s0.name + " (use --npu-prof-mb)");
            if (mc == 0 && !vela::npu_is_emulated())
                errors.push_back(std::string(use_counter ? "no Gemmini counter cycles for " : "no profiler events for ") + s0.name);
        }
    }

    const double total_us = total_cycle / freq_hz * 1e6;
    DETAIL("TOTAL.CONV_LAYERS: %d\n", n_layers);
    DETAIL("TOTAL.CONV_OPS: %llu\n", (unsigned long long)total_ops);
    DETAIL("TOTAL.CONV_MEDIAN_CYCLE: %llu\n", (unsigned long long)total_cycle);
    DETAIL("TOTAL.CONV_TIME_US: %.3f\n", total_us);
    DETAIL("TOTAL.CONV_UTILIZATION: %.2f%%\n", total_cycle ? 100.0 * total_ops / ((double)total_cycle * peak_ops) : 0);
    DETAIL("TOTAL.CONV_GOPS: %.4f\n", total_cycle ? total_ops / (total_us * 1e-6) / 1e9 : 0);
    if (npu)
    {
        DETAIL("TOTAL.NPU_CALL_CYCLE: %llu\n", (unsigned long long)total_call);
        if (npu_fp16)
        {
            DETAIL("TOTAL.CPU_PRE_CYCLE: %llu\n", (unsigned long long)total_pre);
            DETAIL("TOTAL.CPU_PRE.ALLOC_CYCLE: %llu\n", (unsigned long long)total_alloc);
            DETAIL("TOTAL.CPU_PRE.FP16_CVT_CYCLE: %llu\n", (unsigned long long)total_cvt);
            DETAIL("TOTAL.CPU_PRE.IM2COL_CYCLE: %llu\n", (unsigned long long)total_im2col);
            DETAIL("TOTAL.CPU_POST_CYCLE: %llu\n", (unsigned long long)total_post);
        }
        else
        {
            DETAIL("TOTAL.CPU_QUANT_CYCLE: %llu\n", (unsigned long long)total_pre);
            DETAIL("TOTAL.CPU_DEQUANT_CYCLE: %llu\n", (unsigned long long)total_post);
        }
        DETAIL("TOTAL.NPU_COUNTER_BUSY_CYCLE: %llu\n", (unsigned long long)total_ctr);
        DETAIL("TOTAL.NPU_LD_BUSY_CYCLE: %llu\n", (unsigned long long)total_unit[0]);
        DETAIL("TOTAL.NPU_EX_BUSY_CYCLE: %llu\n", (unsigned long long)total_unit[1]);
        DETAIL("TOTAL.NPU_ST_BUSY_CYCLE: %llu\n", (unsigned long long)total_unit[2]);
    }
    else if (int8)
    {
        DETAIL("TOTAL.CPU_QUANT_CYCLE: %llu\n", (unsigned long long)total_pre);
        DETAIL("TOTAL.CPU_DEQUANT_CYCLE: %llu\n", (unsigned long long)total_post);
    }
    DETAIL("INFERENCE.MEDIAN_TIME_MS: %.3f\n", median_d(wall_ms));

    // ---- end to end: whole network, everything between input and output -----------------------
    // OPS = 2 x (conv MACs + FC MACs); pooling / ReLU / residual add / flatten are not counted (no
    // MACs). Time split per run, then the median of each part:
    //   conv     : the convolution itself (NPU: rdcycle around the call + fence, incl. the driver's
    //              tiling; CPU: the conv kernel)
    //   cpu_prep : CPU work around the NPU / int8 kernel (quantize, fp16 convert, im2col, layout)
    //   measure  : NPU only, perf counter / profiler setup and readout (not part of the inference)
    //   other    : the rest (non-conv layers on the CPU, ncnn glue)
    int n_fc = 0;
    const uint64_t fc_macs = fc_macs_from_param(dir + param_name, &n_fc);
    const uint64_t e2e_ops = total_ops + 2 * fc_macs;
    std::vector<uint64_t> part_conv, part_prep, part_meas, part_other;
    for (size_t r = 0; r < runs.size() && r < e2e_cyc.size(); r++)
    {
        uint64_t conv = 0, prep = 0, meas = 0;
        for (const auto& s : runs[r])
        {
            if (s.on_npu && vela::npu_is_emulated())
                conv += s.wrap_cycles; // host: software model of the NPU, no call/measure split
            else if (s.on_npu)
            {
                conv += s.call_cycles;
                meas += s.wrap_cycles > s.call_cycles ? s.wrap_cycles - s.call_cycles : 0;
            }
            else
                conv += s.cpu_cycles;
            prep += s.pre_cycles + s.post_cycles;
        }
        const uint64_t used = conv + prep + meas;
        part_conv.push_back(conv);
        part_prep.push_back(prep);
        part_meas.push_back(meas);
        part_other.push_back(e2e_cyc[r] > used ? e2e_cyc[r] - used : 0);
    }
    const uint64_t e2e = median_u64(e2e_cyc);
    const uint64_t e2e_net = e2e - std::min(e2e, median_u64(part_meas)); // without measurement overhead
    const double e2e_us = e2e / freq_hz * 1e6, e2e_net_us = e2e_net / freq_hz * 1e6;
    if (show_e2e)
    {
        DETAIL("E2E.OPS: %llu\n", (unsigned long long)e2e_ops);
        DETAIL("E2E.CONV_OPS: %llu\n", (unsigned long long)total_ops);
        DETAIL("E2E.FC_OPS: %llu\n", (unsigned long long)(2 * fc_macs));
        DETAIL("E2E.FC_LAYERS: %d\n", n_fc);
        DETAIL("E2E.MEDIAN_CYCLE: %llu\n", (unsigned long long)e2e);
        DETAIL("E2E.TIME_US: %.3f\n", e2e_us);
        DETAIL("E2E.GOPS: %.4f\n", e2e ? e2e_ops / (e2e_us * 1e-6) / 1e9 : 0);
        DETAIL("E2E.CONV_CYCLE: %llu\n", (unsigned long long)median_u64(part_conv));
        DETAIL("E2E.CPU_PREP_CYCLE: %llu\n", (unsigned long long)median_u64(part_prep));
        if (npu)
        {
            DETAIL("E2E.MEASURE_CYCLE: %llu\n", (unsigned long long)median_u64(part_meas));
            DETAIL("E2E.MEDIAN_CYCLE_WO_MEASURE: %llu\n", (unsigned long long)e2e_net);
            DETAIL("E2E.GOPS_WO_MEASURE: %.4f\n", e2e_net ? e2e_ops / (e2e_net_us * 1e-6) / 1e9 : 0);
        }
        DETAIL("E2E.OTHER_CYCLE: %llu\n", (unsigned long long)median_u64(part_other));
    }

    // ---- output check vs PyTorch -------------------------------------------------------------
    const std::string ref_path = dir + "ref_seed" + std::to_string((unsigned long long)seed) + ".bin";
    std::vector<float> ref;
    if (errors.empty())
    {
        ncnn::Mat flat = out.reshape(out.w * out.h * out.c);
        const float* o = flat;
        const int n = flat.w;
        DETAIL("OUTPUT.TOP1: %d\n", argmax(o, n));
        // FNV-1a over the output floats: CPU int8 and NPU runs of the same model/seed must match
        uint32_t h = 2166136261u;
        const uint8_t* ob = (const uint8_t*)o;
        for (size_t i = 0; i < (size_t)n * sizeof(float); i++)
        {
            h ^= ob[i];
            h *= 16777619u;
        }
        DETAIL("OUTPUT.CHECKSUM: 0x%08x\n", h);
        if (n != m.num_out)
            errors.push_back("unexpected output size " + std::to_string(n));
        else if (!read_floats(ref_path, ref, n))
            errors.push_back("no reference for this seed: " + ref_path);
        else
        {
            double dot = 0, na = 0, nb = 0;
            for (int i = 0; i < n; i++)
            {
                dot += (double)o[i] * ref[i];
                na += (double)o[i] * o[i];
                nb += (double)ref[i] * ref[i];
            }
            const double cos = dot / (sqrt(na) * sqrt(nb) + 1e-12);
            const double cos_min = int8 ? 0.98 : 0.999;
            const int rtop = argmax(ref.data(), n);
            DETAIL("REFERENCE.TOP1: %d\n", rtop);
            DETAIL("OUTPUT.COS_SIM: %.6f (min %.3f)\n", cos, cos_min);
            if (argmax(o, n) != rtop) errors.push_back("top-1 differs from PyTorch reference");
            if (!(cos >= cos_min)) errors.push_back("cosine similarity below threshold");
        }
    }

    for (size_t i = 0; i < errors.size(); i++) DETAIL("ERROR_MSG[%zu]: %s\n", i, errors[i].c_str());
    DETAIL("STATUS: %s\n", errors.empty() ? "PASS" : "FAIL");
    DETAIL("ERROR: %zu\n", errors.size());
    DETAIL("=== %s 예제 종료 ===\n", exe);

    if (g_summary)
    {
        const char* hw = npu ? (npu_fp16 ? "velaNPU fp16 (fp32 acc)"
                                         : (TEST_NPU_STOCK_GEMMINI ? "stock Gemmini int8" : "velaNPU int8"))
                             : (int8 ? "Rocket CPU int8" : "Rocket CPU fp32");
        printf("=== %s summary ===\n", exe);
        printf("MODEL       : %s\n", m.display);
        printf("HW          : %s\n", hw);
        if (npu)
            printf("CYCLE_SRC   : %s\n", !use_counter          ? "velaNPU hardware profiler"
                                           : TEST_NPU_STOCK_GEMMINI ? "Gemmini perf counters"
                                                                    : "Gemmini perf counters (--profiler counter)");
        printf("FREQ        : %.0f MHz (FireSim target clock; U280 emulates it at ~60 MHz)\n", freq_hz / 1e6);
        printf("REPEAT      : %d (+%d warmup), median\n", repeat, warmup);
        if (show_conv)
        {
            printf("CONV_CYCLES : %llu\n", (unsigned long long)total_cycle);
            printf("CONV_TIME   : %.3f us\n", total_us);
            printf("CONV_GOPS   : %.4f\n", total_cycle ? total_ops / (total_us * 1e-6) / 1e9 : 0);
            printf("UTILIZATION : %.2f%% (peak %d OPS/cycle)\n",
                   total_cycle ? 100.0 * total_ops / ((double)total_cycle * peak_ops) : 0, peak_ops);
            if (npu || int8)
                printf("CPU_PRE+POST: %llu cycles (%s, not in CONV_CYCLES)\n", (unsigned long long)(total_pre + total_post),
                       npu_fp16 ? "fp16 convert + im2col + ReLU/layout on the CPU" : "int8 quantize + dequantize on the CPU");
        }
        printf("INFER_TIME  : %.3f ms (wall clock, whole network)\n", median_d(wall_ms));
        if (show_e2e)
        {
            printf("E2E_OPS     : %llu (conv %llu + FC %llu; pool/ReLU/add not counted)\n", (unsigned long long)e2e_ops,
                   (unsigned long long)total_ops, (unsigned long long)(2 * fc_macs));
            printf("E2E_CYCLES  : %llu (rdcycle around the whole forward)\n", (unsigned long long)e2e);
            printf("E2E_TIME    : %.3f us\n", e2e_us);
            printf("E2E_GOPS    : %.4f\n", e2e ? e2e_ops / (e2e_us * 1e-6) / 1e9 : 0);
            const double pc = e2e ? 100.0 / e2e : 0;
            if (npu)
            {
                printf("E2E_SPLIT   : NPU conv %.1f%% | CPU prep %.1f%% | measure %.1f%% | other layers %.1f%%\n",
                       median_u64(part_conv) * pc, median_u64(part_prep) * pc, median_u64(part_meas) * pc,
                       median_u64(part_other) * pc);
                printf("E2E_GOPS_NET: %.4f (without the measure part: %llu cycles)\n",
                       e2e_net ? e2e_ops / (e2e_net_us * 1e-6) / 1e9 : 0, (unsigned long long)e2e_net);
            }
            else if (int8)
                printf("E2E_SPLIT   : conv %.1f%% | CPU quant %.1f%% | other layers %.1f%%\n", median_u64(part_conv) * pc,
                       median_u64(part_prep) * pc, median_u64(part_other) * pc);
            else
                printf("E2E_SPLIT   : conv %.1f%% | other layers %.1f%%\n", median_u64(part_conv) * pc,
                       median_u64(part_other) * pc);
        }
        for (size_t i = 0; i < errors.size(); i++) printf("ERROR_MSG[%zu]: %s\n", i, errors[i].c_str());
        printf("STATUS      : %s\n", errors.empty() ? "PASS" : "FAIL");
    }
    return errors.empty() ? 0 : 1;
}
