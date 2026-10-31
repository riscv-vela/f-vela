// NCNN convolution test executables for the AI acceleration system test procedure:
//   cpu_lenet.riscv  cpu_vgg16.riscv  cpu_resnet18.riscv   NPU backend disabled (Rocket core)
//   npu_lenet.riscv  npu_vgg16.riscv  npu_resnet18.riscv   NPU backend enabled  (velaNPU)
//   npu_lenet2.riscv npu_vgg16_2.riscv npu_resnet18_2.riscv
//                                                          velaNPU, cycles from the Gemmini perf
//                                                          counters like gemmini_* (-DTEST_NPU_CYCLE_COUNTER=1)
//   gemmini_lenet.riscv gemmini_vgg16.riscv gemmini_resnet18.riscv
//                                                          NPU backend on the upstream, unmodified
//                                                          Gemmini (FireSimNoCustomGemminiOnlyTest)
// Built from this one file with -DTEST_MODEL_ID=0|1|2 -DTEST_BACKEND_NPU=0|1 (and
// -DTEST_NPU_STOCK_GEMMINI=1 + gemmini_stock_hal.c for the gemmini_* ones), so CPU, velaNPU and
// stock Gemmini use the same code, the same .param/.bin, the same int8 scheme and the same input.
//
// usage: <exe> [--seed N] [-r repeat] [-w warmup] [-d model_root] [--freq-hz F] [--npu-prof-mb N] [--fp32]
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
//   npu_<model>2 (velaNPU, profiler off) and stock Gemmini (no profiler): Gemmini perf counters, cycles with the load, execute or store
//        controller busy (sum of MAIN_*_CYCLES); rdcycle around the call is in NPU_CALL_CYCLE
// Like-for-like NPU metrics, measured identically on velaNPU and stock Gemmini:
//   NPU_COUNTER_BUSY_CYCLE (Gemmini perf counters, gemmini_perf_counters.h) and NPU_CALL_CYCLE.
// OPS = 2 x MACs. TIME = cycles / CLOCK_FREQ_HZ. GOP/s = OPS / TIME.
// UTILIZATION = OPS / (cycles x PEAK_OPS_PER_CYCLE):
//   CPU int8 peak 1 OPS/cycle: one int8 MAC per 2 cycles. The Rocket integer multiplier is iterative
//            (mulUnroll 8, not pipelined); with early-out an 8-bit multiply takes at least 2 cycles
//            (8 cycles when the multiplier operand is negative)
//   CPU fp32 peak 2 OPS/cycle (one fp32 FMA per cycle on the pipelined scalar Rocket FPU)
//   NPU peak 2 x DIM x DIM OPS/cycle (int8 systolic array)
// The output is compared with the PyTorch reference for the same seed (ref_seed<seed>.bin):
// STATUS: PASS requires top-1 match and cos >= 0.98 (int8: NPU and CPU) / 0.999 (CPU --fp32).
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
// velaNPU measured like stock Gemmini (npu_<model>2.riscv, HAL built with -DVNPU_COUNTER_ONLY=1)
#ifndef TEST_NPU_CYCLE_COUNTER
#define TEST_NPU_CYCLE_COUNTER 0
#endif

#if TEST_NPU_STOCK_GEMMINI
static const char* kNpuName = "stock Gemmini";
static const char* kNpuExec = "stock Gemmini int8 (tiled_conv_auto, WS)";
static const char* kNpuCycleSource = "Gemmini perf counters (ld|ex|st controller busy cycles)";
#elif TEST_NPU_CYCLE_COUNTER
static const char* kNpuName = "velaNPU";
static const char* kNpuExec = "velaNPU int8 (tiled_conv_auto, WS)";
static const char* kNpuCycleSource = "Gemmini perf counters (ld|ex|st controller busy cycles, profiler off)";
#else
static const char* kNpuName = "velaNPU";
static const char* kNpuExec = "velaNPU int8 (tiled_conv_auto, WS)";
static const char* kNpuCycleSource = "NPU hardware profiler (conv start -> completion)";
#endif

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

int main(int argc, char** argv)
{
    const ModelSpec& m = kModels[TEST_MODEL_ID];
    const bool npu = TEST_BACKEND_NPU != 0;

    uint64_t seed = 1234;
    int repeat = m.repeat, warmup = 1, prof_mb = 128;
    double freq_hz = 1e9;
    bool cpu_fp32 = false;
    std::string root = "/media/ncnn_models";
    for (int i = 1; i < argc; i++)
    {
        std::string a = argv[i];
        if (a == "--seed" && i + 1 < argc) seed = strtoull(argv[++i], NULL, 0);
        else if (a == "-r" && i + 1 < argc) repeat = std::max(1, atoi(argv[++i]));
        else if (a == "-w" && i + 1 < argc) warmup = std::max(0, atoi(argv[++i]));
        else if (a == "-d" && i + 1 < argc) root = argv[++i];
        else if (a == "--freq-hz" && i + 1 < argc) freq_hz = atof(argv[++i]);
        else if (a == "--npu-prof-mb" && i + 1 < argc) prof_mb = atoi(argv[++i]);
        else if (a == "--fp32" && !npu) cpu_fp32 = true;
        else
        {
            fprintf(stderr, "usage: %s [--seed N] [-r repeat] [-w warmup] [-d model_root] [--freq-hz F] [--npu-prof-mb N]%s\n",
                    argv[0], npu ? "" : " [--fp32]");
            return 2;
        }
    }

    const char* exe = strrchr(argv[0], '/') ? strrchr(argv[0], '/') + 1 : argv[0];
    const std::string dir = root + "/" + m.id + "/";
    const std::string param_name = std::string(m.id) + ".ncnn.param";
    const std::string bin_name = std::string(m.id) + ".ncnn.bin";
    const bool int8 = npu || !cpu_fp32;
    // CPU int8: 1 MAC per 2 cycles (iterative integer multiplier); CPU fp32: 1 FMA per cycle
    const int kCpuInt8PeakOps = 1, kCpuFp32PeakOps = 2;
    const int cpu_peak_ops = cpu_fp32 ? kCpuFp32PeakOps : kCpuInt8PeakOps;
    const int peak_ops = npu ? 2 * vela::npu_peak_macs_per_cycle() : cpu_peak_ops;

    std::vector<std::string> errors;

    printf("=== %s 예제 시작 ===\n", exe);
    printf("TEST_MODEL: %s\n", m.display);
    printf("BACKEND: %s\n", npu ? "NPU" : "CPU");
    if (npu) printf("NPU_HW: %s\n", kNpuName);
    printf("PARAM_FILE: %s\n", param_name.c_str());
    printf("BIN_FILE: %s\n", bin_name.c_str());
    printf("MODEL_DIR: %s\n", dir.c_str());
    printf("INPUT_TYPE: FP32\n");
    printf("INPUT_SHAPE: %dx%dx%d\n", m.c, m.h, m.w);
    printf("INPUT_SEED: %llu\n", (unsigned long long)seed);
    printf("CONV_DATA_TYPE: %s\n", int8 ? "INT8 (int32 accumulate)" : "FP32");
    printf("CLOCK_FREQ_HZ: %.0f\n", freq_hz);
    printf("CONV_EXEC: %s\n", npu ? kNpuExec
                                   : (int8 ? "Rocket core int8 (scalar integer MAC, same int8 quantization as the NPU)"
                                           : "Rocket core fp32 (ncnn Convolution)"));
    printf("CYCLE_SOURCE: %s\n", npu ? kNpuCycleSource
                                      : (int8 ? "rdcycle (before/after the int8 conv kernel)" : "rdcycle (before/after convolution)"));
    printf("PEAK_OPS_PER_CYCLE: %d\n", peak_ops);
    printf("REPEAT: %d\n", repeat);
    printf("WARMUP: %d\n", warmup);
    fflush(stdout);

    // ---- set up the Net with the VelaNpuConvolution layer ------------------------------------
    vela::Config& vc = vela::config();
    vc.backend = npu ? vela::Backend::Npu : (int8 ? vela::Backend::CpuInt8 : vela::Backend::Cpu);
    if (int8)
    {
        // int8 output scales: the same calibration table for the NPU and the CPU int8 path
        if (!vela::load_calib(dir + m.id + ".npu_calib"))
            errors.push_back("missing int8 calibration " + dir + m.id + ".npu_calib");
        else if (npu && vela::npu_init(prof_mb) != 0)
            errors.push_back(std::string(kNpuName) + " init failed");
    }

    ncnn::Net net;
    net.opt.num_threads = 1;
    net.opt.use_fp16_packed = false;
    net.opt.use_fp16_storage = false;
    net.opt.use_fp16_arithmetic = false;
    net.opt.use_bf16_storage = false;
    net.register_custom_layer("Convolution", vela::VelaNpuConvolution_layer_creator);

    std::vector<std::vector<vela::ConvStat> > runs;
    std::vector<double> wall_ms;
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
            ex.input("in0", in);
            if (ex.extract("out0", out) != 0)
            {
                errors.push_back("inference failed");
                break;
            }
            double t1 = now_ms() - t0;
            printf("RUN[%d]: %s %.3f ms\n", i, i < warmup ? "warmup" : "measure", t1);
            fflush(stdout);
            if (i >= warmup)
            {
                runs.push_back(vc.stats);
                wall_ms.push_back(t1);
            }
        }
    }

    // ---- per-layer report --------------------------------------------------------------------
    uint64_t total_ops = 0, total_cycle = 0, total_pre = 0, total_post = 0, total_call = 0;
    uint64_t total_ctr = 0, total_unit[3] = {0, 0, 0};
    int n_layers = runs.empty() ? 0 : (int)runs[0].size();
    for (int li = 0; li < n_layers; li++)
    {
        const vela::ConvStat& s0 = runs[0][li];
        std::vector<uint64_t> cyc, pre, post, call, ctr, unit[3], tlb;
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

        printf("LAYER[%d].NAME: %s\n", li, s0.name.c_str());
        printf("LAYER[%d].SHAPE: %dx%dx%d -> %dx%dx%d k%d s%d p%d\n", li, s0.in_c, s0.in_h, s0.in_w, s0.out_c, s0.out_h,
               s0.out_w, s0.kernel, s0.stride, s0.pad);
        printf("LAYER[%d].EXEC: %s\n", li,
               s0.on_npu ? "NPU"
                         : (!s0.fallback.empty() ? ("CPU fp32 (fallback: " + s0.fallback + ")").c_str()
                                                 : (s0.int8 ? "CPU int8" : "CPU fp32")));
        printf("LAYER[%d].OPS: %llu\n", li, (unsigned long long)ops);
        printf("LAYER[%d].MEDIAN_CYCLE: %llu\n", li, (unsigned long long)mc);
        printf("LAYER[%d].TIME_US: %.3f\n", li, t_us);
        printf("LAYER[%d].UTILIZATION: %.2f%%\n", li, util);
        printf("LAYER[%d].GOPS: %.4f\n", li, gops);
        if (s0.int8 && !s0.on_npu)
        {
            // CPU int8: same CPU-side conversion work as the NPU path, reported the same way
            const uint64_t mp = median_u64(pre), mq = median_u64(post);
            total_pre += mp;
            total_post += mq;
            printf("LAYER[%d].CPU_QUANT_CYCLE: %llu\n", li, (unsigned long long)mp);
            printf("LAYER[%d].CPU_DEQUANT_CYCLE: %llu\n", li, (unsigned long long)mq);
        }
        if (s0.on_npu)
        {
            const uint64_t mp = median_u64(pre), mq = median_u64(post), mcall = median_u64(call);
            total_pre += mp;
            total_post += mq;
            total_call += mcall;
            printf("LAYER[%d].NPU_CALL_CYCLE: %llu\n", li, (unsigned long long)mcall);
            printf("LAYER[%d].CPU_QUANT_CYCLE: %llu\n", li, (unsigned long long)mp);
            printf("LAYER[%d].CPU_DEQUANT_CYCLE: %llu\n", li, (unsigned long long)mq);
            // Gemmini perf counters: same method on velaNPU and stock Gemmini (like-for-like metric)
            const uint64_t mctr = median_u64(ctr);
            const uint64_t bl = median_u64(unit[0]), be = median_u64(unit[1]), bs = median_u64(unit[2]);
            total_ctr += mctr;
            total_unit[0] += bl;
            total_unit[1] += be;
            total_unit[2] += bs;
            printf("LAYER[%d].NPU_COUNTER_BUSY_CYCLE: %llu\n", li, (unsigned long long)mctr);
            printf("LAYER[%d].NPU_LD_BUSY_CYCLE: %llu\n", li, (unsigned long long)bl);
            printf("LAYER[%d].NPU_EX_BUSY_CYCLE: %llu\n", li, (unsigned long long)be);
            printf("LAYER[%d].NPU_ST_BUSY_CYCLE: %llu\n", li, (unsigned long long)bs);
            printf("LAYER[%d].NPU_TLB_MISS_CYCLE: %llu\n", li, (unsigned long long)median_u64(tlb));
            if (stale) printf("LAYER[%d].PROFILER_STALE_EVENTS: %llu\n", li, (unsigned long long)stale);
            if (overflow) errors.push_back("profile buffer overflow at " + s0.name + " (use --npu-prof-mb)");
            if (mc == 0 && !vela::npu_is_emulated())
                errors.push_back(std::string(TEST_NPU_STOCK_GEMMINI || TEST_NPU_CYCLE_COUNTER ? "no Gemmini counter cycles for " : "no profiler events for ") + s0.name);
        }
    }

    const double total_us = total_cycle / freq_hz * 1e6;
    printf("TOTAL.CONV_LAYERS: %d\n", n_layers);
    printf("TOTAL.CONV_OPS: %llu\n", (unsigned long long)total_ops);
    printf("TOTAL.CONV_MEDIAN_CYCLE: %llu\n", (unsigned long long)total_cycle);
    printf("TOTAL.CONV_TIME_US: %.3f\n", total_us);
    printf("TOTAL.CONV_UTILIZATION: %.2f%%\n", total_cycle ? 100.0 * total_ops / ((double)total_cycle * peak_ops) : 0);
    printf("TOTAL.CONV_GOPS: %.4f\n", total_cycle ? total_ops / (total_us * 1e-6) / 1e9 : 0);
    if (npu)
    {
        printf("TOTAL.NPU_CALL_CYCLE: %llu\n", (unsigned long long)total_call);
        printf("TOTAL.CPU_QUANT_CYCLE: %llu\n", (unsigned long long)total_pre);
        printf("TOTAL.CPU_DEQUANT_CYCLE: %llu\n", (unsigned long long)total_post);
        printf("TOTAL.NPU_COUNTER_BUSY_CYCLE: %llu\n", (unsigned long long)total_ctr);
        printf("TOTAL.NPU_LD_BUSY_CYCLE: %llu\n", (unsigned long long)total_unit[0]);
        printf("TOTAL.NPU_EX_BUSY_CYCLE: %llu\n", (unsigned long long)total_unit[1]);
        printf("TOTAL.NPU_ST_BUSY_CYCLE: %llu\n", (unsigned long long)total_unit[2]);
    }
    else if (int8)
    {
        printf("TOTAL.CPU_QUANT_CYCLE: %llu\n", (unsigned long long)total_pre);
        printf("TOTAL.CPU_DEQUANT_CYCLE: %llu\n", (unsigned long long)total_post);
    }
    printf("INFERENCE.MEDIAN_TIME_MS: %.3f\n", median_d(wall_ms));

    // ---- output check vs PyTorch -------------------------------------------------------------
    const std::string ref_path = dir + "ref_seed" + std::to_string((unsigned long long)seed) + ".bin";
    std::vector<float> ref;
    if (errors.empty())
    {
        ncnn::Mat flat = out.reshape(out.w * out.h * out.c);
        const float* o = flat;
        const int n = flat.w;
        printf("OUTPUT.TOP1: %d\n", argmax(o, n));
        // FNV-1a over the output floats: CPU int8 and NPU runs of the same model/seed must match
        uint32_t h = 2166136261u;
        const uint8_t* ob = (const uint8_t*)o;
        for (size_t i = 0; i < (size_t)n * sizeof(float); i++)
        {
            h ^= ob[i];
            h *= 16777619u;
        }
        printf("OUTPUT.CHECKSUM: 0x%08x\n", h);
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
            printf("REFERENCE.TOP1: %d\n", rtop);
            printf("OUTPUT.COS_SIM: %.6f (min %.3f)\n", cos, cos_min);
            if (argmax(o, n) != rtop) errors.push_back("top-1 differs from PyTorch reference");
            if (!(cos >= cos_min)) errors.push_back("cosine similarity below threshold");
        }
    }

    for (size_t i = 0; i < errors.size(); i++) printf("ERROR_MSG[%zu]: %s\n", i, errors[i].c_str());
    printf("STATUS: %s\n", errors.empty() ? "PASS" : "FAIL");
    printf("ERROR: %zu\n", errors.size());
    printf("=== %s 예제 종료 ===\n", exe);
    return errors.empty() ? 0 : 1;
}
