// ggml-vela.c — runs BitNet's matmuls on the velaNPU (custom Gemmini), selected at run time.
//
// Environment (read once, at the first matmul):
//   VELA_MODE     preset, default cpu
//                   cpu       everything on the Rocket core (original BitNet code)
//                   mp        I2_S (ternary) matmuls on MpExeUnit (2-bit mpgemm, B = [K][N/4])
//                   mpt       I2_S matmuls on MpExeUnit with b_transpose (B^T = [N/4][K]); from the
//                             original model packing the repack is only a 4-byte interleave
//                   int8      I2_S matmuls on the int8 array + AdderTree (weights unpacked to int8)
//                   fp16      F16 weight matmuls (LM head) on FpExeUnit, ternary on the CPU
//                   npu       mp + fp16
//                   npu-mpt   mpt + fp16
//                   npu-int8  int8 + fp16
//   VELA_TERNARY  cpu | mp | mpt | int8      overrides the preset for I2_S matmuls
//   VELA_FP16     off | weights | all        overrides the preset for F16 matmuls
//                   weights: 2-D contiguous F16 weights only (token_embd as LM head)
//                   all:     also the attention KQ / KQV matmuls on the F16 KV cache
//   VELA_ROPE     off | on | force    RoPE (Q and K) on the velaVPU RoPE unit (vfrope.vx), any VELA_MODE.
//                 At start the unit is probed to find the base its ROM was built with (vela_rope.c);
//                 on: only when it equals the model's rope freq_base (BitNet 2B-4T: 500000), else CPU
//                 force: also with another base (wrong angles; to measure the unit, not for real output)
//   VELA_ROPE_BASE  an extra ROM base to look for (default candidates 10000, 500000)
//   VELA_CHECK    N: compare the first N NPU matmuls of each lane with a CPU reference
//                 (sampled columns; ternary must match exactly, fp16 within 1e-2), and the first N RoPE ops
//                 with a float reference (error within 2e-2 of each row's largest pair norm)
//   VELA_VERBOSE  1: one line per NPU matmul
//   VELA_STATS    0: no matmul summary at exit (default 1)
//
// Weight packing: each I2_S tensor is checked once (vnpu_detect_format). BitNet's packing runs as
// before; the original packing (model/i2s_original.gguf, MpPE codes along K) is read by every NPU
// lane, and in cpu mode by a scalar kernel here ("cpu-o"), since BitNet's kernel cannot read it.
//
// Numerics: the ternary lanes reproduce the CPU path bit-exactly. Activations are quantized
// per row with BitNet's own quantize_row_i8_s, the NPU returns the exact int32 sum of
// w * q (w in {-1, 0, 1}), and the result is acc / act_scale * weight_scale like the CPU kernel.
// The fp16 lane converts activations with ggml_fp32_to_fp16_row (as the CPU path does); the
// FpExeUnit rounds products to fp16 and accumulates in fp32, so results differ in the last bits.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "ggml-vela.h"
#include "vela_npu.h"
#include "vela_rope.h"

#include "ggml.h"
#include "ggml-quants.h"   // quantize_row_i8_s

#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

// LANE_SW: ternary on the Rocket core by this file (original packing in cpu mode)
enum { LANE_CPU = 0, LANE_MP, LANE_MPT, LANE_I8, LANE_F16, LANE_SW, LANE_N };
static const char *const lane_name[LANE_N] = {"cpu", "mp", "mpt", "int8", "fp16", "cpu-o"};
static const char *const lane_unit[LANE_N] = {"Rocket core", "MpExeUnit (mpgemm)", "MpExeUnit (mpgemm, b_transpose)",
                                              "int8 array + AdderTree", "FpExeUnit", "Rocket core (vela scalar)"};
enum { FP16_OFF = 0, FP16_WEIGHTS, FP16_ALL };
static const char *const fp16_name[] = {"off", "weights", "all"};

static struct {
    const char *mode;
    int ternary;   // LANE_CPU / LANE_MP / LANE_MPT / LANE_I8
    int fp16;      // FP16_*
    int check, verbose, stats;
    int npu;       // an NPU lane is enabled (NPU initialised)
    int rope;      // ROPE_*
    double rope_base;   // base of the RoPE unit's ROM (0: not found / not probed)
} cfg;
enum { ROPE_OFF = 0, ROPE_ON, ROPE_FORCE };
static const char *const rope_mode_name[] = {"off", "on", "force"};

static pthread_once_t init_once = PTHREAD_ONCE_INIT;

// ---- statistics (thread 0 only) -------------------------------------------------------------------
enum { WT_I2S = 0, WT_F16, WT_F32, WT_OTHER, WT_N };
static const char *const wt_name[WT_N] = {"I2_S", "F16", "F32", "other"};
static struct { uint64_t calls, time, macs; } st_op[WT_N][LANE_N];
static struct { uint64_t calls, prep, npu, post, pack, check, check_fail; size_t pack_bytes; } st_lane[LANE_N];
static struct { uint64_t calls, time, rows; } st_rope[2];   // [0] CPU, [1] RoPE unit
static uint64_t st_rope_check_fail;

// ---- environment ------------------------------------------------------------------------------------
static int env_pick(const char *var, const char *const *names, int n, int def) {
    const char *v = getenv(var);
    if (!v || !*v) return def;
    for (int i = 0; i < n; i++)
        if (!strcasecmp(v, names[i])) return i;
    fprintf(stderr, "[vela] %s=%s is not valid, using %s\n", var, v, names[def]);
    return def;
}
static int env_int(const char *var, int def) {
    const char *v = getenv(var);
    return (v && *v) ? atoi(v) : def;
}

static void print_stats(void);

static void vela_init(void) {
    static const struct { const char *name; int ternary, fp16; } presets[] = {
        {"cpu", LANE_CPU, FP16_OFF},      {"mp", LANE_MP, FP16_OFF},          {"mpt", LANE_MPT, FP16_OFF},
        {"int8", LANE_I8, FP16_OFF},      {"fp16", LANE_CPU, FP16_WEIGHTS},   {"npu", LANE_MP, FP16_WEIGHTS},
        {"npu-mpt", LANE_MPT, FP16_WEIGHTS}, {"npu-int8", LANE_I8, FP16_WEIGHTS},
    };
    const int npresets = (int)(sizeof(presets) / sizeof(presets[0]));
    const char *mode = getenv("VELA_MODE");
    int p = 0;
    if (mode && *mode) {
        for (p = 0; p < npresets && strcasecmp(mode, presets[p].name); p++) {}
        if (p == npresets) {
            fprintf(stderr, "[vela] VELA_MODE=%s is not valid (cpu|mp|mpt|int8|fp16|npu|npu-mpt|npu-int8), using cpu\n", mode);
            p = 0;
        }
    }
    cfg.mode = presets[p].name;
    cfg.ternary = presets[p].ternary;
    cfg.fp16 = presets[p].fp16;

    static const char *const tern_names[] = {"cpu", "mp", "mpt", "int8"};   // = LANE_CPU..LANE_I8
    cfg.ternary = env_pick("VELA_TERNARY", tern_names, 4, cfg.ternary);
    cfg.fp16    = env_pick("VELA_FP16", fp16_name, 3, cfg.fp16);
    cfg.check   = env_int("VELA_CHECK", 0);
    cfg.verbose = env_int("VELA_VERBOSE", 0);
    cfg.stats   = env_int("VELA_STATS", 1);

    if (cfg.ternary != LANE_CPU || cfg.fp16 != FP16_OFF) {
        if (vnpu_init(1) != 0)
            fprintf(stderr, "[vela] WARNING: memory could not be locked; NPU DMA may fault\n");
        cfg.npu = 1;
    }
    fprintf(stderr, "[vela] VELA_MODE=%s  I2_S -> %s, F16 -> %s, backend=%s, check=%d\n",
            cfg.mode, lane_unit[cfg.ternary],
            cfg.fp16 == FP16_OFF ? "Rocket core" : (cfg.fp16 == FP16_WEIGHTS ? "FpExeUnit (weights)" : "FpExeUnit (all)"),
            cfg.npu ? vnpu_backend() : "none", cfg.check);

    cfg.rope = env_pick("VELA_ROPE", rope_mode_name, 3, ROPE_OFF);
    if (cfg.rope != ROPE_OFF) {
        double bases[3] = {10000.0, 500000.0, 0.0};
        int nb = 2;
        const char *b = getenv("VELA_ROPE_BASE");
        if (b && *b) bases[nb++] = atof(b);
        cfg.rope_base = vrope_detect(bases, nb);
        if (cfg.rope_base > 0.0)
            fprintf(stderr, "[vela] VELA_ROPE=%s  RoPE -> velaVPU RoPE unit (ROM base %g)\n", rope_mode_name[cfg.rope],
                    cfg.rope_base);
        else
            fprintf(stderr, "[vela] WARNING: VELA_ROPE=%s but the RoPE unit does not match the emulation for any ROM base "
                            "(10000, 500000%s): RoPE stays on the Rocket core\n", rope_mode_name[cfg.rope],
                    nb > 2 ? ", VELA_ROPE_BASE" : "");
    }
    if (cfg.stats) atexit(print_stats);
}

// ---- weight packing of each I2_S tensor (detected once, shared by all threads) --------------------
static pthread_mutex_t fmt_lock = PTHREAD_MUTEX_INITIALIZER;
static struct { const void *key; int fmt; } *fmt_tab;
static int fmt_count, fmt_cap;

static int tensor_format(const struct ggml_tensor *t) {
    pthread_mutex_lock(&fmt_lock);
    int fmt = VNPU_FMT_UNKNOWN, found = 0;
    for (int i = 0; i < fmt_count; i++)
        if (fmt_tab[i].key == t->data) { fmt = fmt_tab[i].fmt; found = 1; break; }
    if (!found) {
        fmt = vnpu_detect_format((const uint8_t *)t->data, (int)t->ne[1], (int)t->ne[0]);
        if (fmt == VNPU_FMT_UNKNOWN)
            fprintf(stderr, "[vela] WARNING: %s: weight packing not recognised, left to BitNet's kernel\n", t->name);
        else if (fmt_count == 0)
            fprintf(stderr, "[vela] ternary weight packing: %s (%s)\n", vnpu_format_name(fmt), t->name);
        if (fmt_count == fmt_cap) {
            fmt_cap = fmt_cap ? 2 * fmt_cap : 256;
            fmt_tab = realloc(fmt_tab, sizeof(*fmt_tab) * fmt_cap);
        }
        fmt_tab[fmt_count].key = t->data;
        fmt_tab[fmt_count].fmt = fmt;
        fmt_count++;
    }
    pthread_mutex_unlock(&fmt_lock);
    return fmt;
}

// ---- dispatch decision (same answer on every thread) -------------------------------------------------
static int decide(const struct ggml_tensor *src0, const struct ggml_tensor *src1, const struct ggml_tensor *dst) {
    pthread_once(&init_once, vela_init);
    if (src1->type != GGML_TYPE_F32 || src1->nb[0] != sizeof(float) || dst->type != GGML_TYPE_F32) return LANE_CPU;

    if (src0->type == GGML_TYPE_I2_S) {
        const int fmt = tensor_format(src0);
        if (fmt == VNPU_FMT_UNKNOWN) return LANE_CPU;
        const int shape_ok = src0->ne[2] == 1 && src0->ne[3] == 1 && src1->ne[2] == 1 && src1->ne[3] == 1 &&
                             src0->ne[0] % 128 == 0 && ggml_is_contiguous(src0);
        // BitNet's own kernel only reads its own packing; the original one goes to the vela scalar kernel
        const int cpu_lane = fmt == VNPU_FMT_BITNET ? LANE_CPU : (shape_ok ? LANE_SW : LANE_CPU);
        if (!cfg.npu || cfg.ternary == LANE_CPU || !shape_ok) return cpu_lane;
        // mpgemm needs N % 64 == 0 (partial-J bug, 2026-11-02)
        if (src0->ne[1] % 64) return cpu_lane;
        return cfg.ternary;
    }
    if (!cfg.npu) return LANE_CPU;
    if (src0->type == GGML_TYPE_F16 && cfg.fp16 != FP16_OFF) {
        if (src0->nb[0] != sizeof(ggml_fp16_t)) return LANE_CPU;
        if (cfg.fp16 == FP16_WEIGHTS && !(src0->ne[2] == 1 && src0->ne[3] == 1 && ggml_is_contiguous(src0)))
            return LANE_CPU;
        if (src1->ne[2] % src0->ne[2] || src1->ne[3] % src0->ne[3]) return LANE_CPU;
        return LANE_F16;
    }
    return LANE_CPU;
}

// ---- buffers ------------------------------------------------------------------------------------------
static void *xalloc(size_t bytes) {
    void *p = aligned_alloc(64, (bytes + 63) & ~(size_t)63);
    if (!p) {
        fprintf(stderr, "[vela] out of memory (%zu bytes)\n", bytes);
        abort();
    }
    return p;
}

enum { S_A, S_SCALE, S_C, S_N };
static void *scratch(int slot, size_t bytes) {
    static void *buf[S_N];
    static size_t cap[S_N];
    if (bytes > cap[slot]) {
        free(buf[slot]);
        cap[slot] = bytes + bytes / 4;
        buf[slot] = xalloc(cap[slot]);
    }
    return buf[slot];
}

// Repacked weights, built on first use and kept (keyed by the tensor's data pointer).
typedef struct { const void *key; void *buf; } wentry_t;
static wentry_t *wcache[LANE_N];
static int wcount[LANE_N], wcap[LANE_N];

static const void *packed_weights(const struct ggml_tensor *src0, int lane) {
    for (int i = 0; i < wcount[lane]; i++)
        if (wcache[lane][i].key == src0->data) return wcache[lane][i].buf;

    const int K = (int)src0->ne[0], N = (int)src0->ne[1];
    const int fmt = tensor_format(src0);
    const uint8_t *w = (const uint8_t *)src0->data;
    const size_t bytes = lane == LANE_MP ? vnpu_mp_bytes(N, K) : lane == LANE_MPT ? vnpu_mpt_bytes(N, K) : vnpu_i8_bytes(N, K);
    const uint64_t t0 = vnpu_now();
    void *buf = xalloc(bytes);
    if (lane == LANE_MP)       vnpu_to_mp(fmt, w, N, K, (uint8_t *)buf);
    else if (lane == LANE_MPT) vnpu_to_mpt(fmt, w, N, K, (uint8_t *)buf);
    else                       vnpu_to_i8(fmt, w, N, K, (int8_t *)buf);
    st_lane[lane].pack += vnpu_now() - t0;
    st_lane[lane].pack_bytes += bytes;

    if (wcount[lane] == wcap[lane]) {
        wcap[lane] = wcap[lane] ? 2 * wcap[lane] : 256;
        wcache[lane] = (wentry_t *)realloc(wcache[lane], sizeof(wentry_t) * wcap[lane]);
    }
    wcache[lane][wcount[lane]++] = (wentry_t){src0->data, buf};
    if (cfg.verbose)
        fprintf(stderr, "[vela] repack %-24s %5d x %5d %s -> %s (%zu KiB)\n", src0->name, N, K, vnpu_format_name(fmt),
                lane_name[lane], bytes >> 10);
    return buf;
}

// ---- checks against a CPU reference (sampled columns) ------------------------------------------------
#define CHECK_COLS 256
#define CHECK_ROWS 8

static void check_ternary(const struct ggml_tensor *src0, int lane, const int8_t *A, const int32_t *C, int M, int N, int K) {
    const uint8_t *w = (const uint8_t *)src0->data;
    const int fmt = tensor_format(src0);
    const int ncols = N < CHECK_COLS ? N : CHECK_COLS, nrows = M < CHECK_ROWS ? M : CHECK_ROWS;
    int bad = 0, total = 0;
    for (int ri = 0; ri < nrows; ri++) {
        const int m = (int)((int64_t)ri * M / nrows);
        for (int ci = 0; ci < ncols; ci++) {
            const int n = (int)((int64_t)ci * N / ncols) + (ci & 3);  // also hit every n % 4
            if (n >= N) continue;
            int32_t ref = 0;
            for (int k = 0; k < K; k++) ref += A[(size_t)m * K + k] * vnpu_weight(fmt, w, K, n, k);
            const int32_t got = C[(size_t)m * N + n];
            total++;
            if (got != ref && bad++ < 4)
                fprintf(stderr, "[vela]   %s [%d][%d] got %d ref %d\n", src0->name, m, n, got, ref);
        }
    }
    st_lane[lane].check_fail += bad != 0;
    fprintf(stderr, "[vela] check %-5s %-24s M=%d N=%d K=%d: %s (%d/%d mismatches)\n", lane_name[lane], src0->name,
            M, N, K, bad ? "FAIL" : "ok", bad, total);
}

static void check_f16(const char *name, const ggml_fp16_t *A, const ggml_fp16_t *B, size_t ldb, const float *C, size_t ldc,
                      int M, int N, int K) {
    const int ncols = N < CHECK_COLS ? N : CHECK_COLS, nrows = M < CHECK_ROWS ? M : CHECK_ROWS;
    double worst = 0.0;
    int wm = 0, wn = 0;
    for (int ri = 0; ri < nrows; ri++) {
        const int m = (int)((int64_t)ri * M / nrows);
        for (int ci = 0; ci < ncols; ci++) {
            const int n = (int)((int64_t)ci * N / ncols);
            double ref = 0.0, mag = 0.0;
            for (int k = 0; k < K; k++) {
                const double p = (double)ggml_fp16_to_fp32(A[(size_t)m * K + k]) * (double)ggml_fp16_to_fp32(B[(size_t)n * ldb + k]);
                ref += p;
                mag += fabs(p);
            }
            // error relative to sum |a*b| (the scale of the rounding error of a dot product)
            const double err = fabs((double)C[(size_t)m * ldc + n] - ref) / (mag + 1e-30);
            if (err > worst) { worst = err; wm = m; wn = n; }
        }
    }
    const int bad = !(worst <= 1e-2);
    st_lane[LANE_F16].check_fail += bad;
    fprintf(stderr, "[vela] check fp16  %-24s M=%d N=%d K=%d: %s (max rel err %.2e at [%d][%d])\n", name, M, N, K,
            bad ? "FAIL" : "ok", worst, wm, wn);
}

// ---- lanes ---------------------------------------------------------------------------------------------
static void mm_ternary(const struct ggml_tensor *src0, const struct ggml_tensor *src1, struct ggml_tensor *dst, int lane) {
    const int K = (int)src0->ne[0], N = (int)src0->ne[1], M = (int)src1->ne[1];
    const uint64_t pack0 = st_lane[lane].pack;
    const uint64_t t0 = vnpu_now();

    // activations: per-row int8, exactly as the CPU path (quantize_row_i8_s)
    int8_t *A = (int8_t *)scratch(S_A, (size_t)M * K);
    float *ascale = (float *)scratch(S_SCALE, sizeof(float) * M);
    for (int m = 0; m < M; m++) {
        int32_t sum;
        quantize_row_i8_s((const float *)((const char *)src1->data + (size_t)m * src1->nb[1]), A + (size_t)m * K, K,
                          &ascale[m], &sum);
    }
    const void *W = lane == LANE_SW ? NULL : packed_weights(src0, lane);
    int32_t *C = (int32_t *)scratch(S_C, sizeof(int32_t) * (size_t)M * N);

    const uint64_t t1 = vnpu_now();
    if (lane == LANE_MP)       vnpu_gemm_mp(M, N, K, A, K, (const uint8_t *)W, C, N);
    else if (lane == LANE_MPT) vnpu_gemm_mpt(M, N, K, A, K, (const uint8_t *)W, C, N);
    else if (lane == LANE_I8)  vnpu_gemm_i8(M, N, K, A, K, (const int8_t *)W, C, N);
    else vnpu_cpu_ternary(tensor_format(src0), (const uint8_t *)src0->data, M, N, K, A, K, C, N);
    const uint64_t t2 = vnpu_now();

    if (lane != LANE_SW && st_lane[lane].calls < (uint64_t)cfg.check) check_ternary(src0, lane, A, C, M, N, K);
    const uint64_t t3 = vnpu_now();

    const float wscale = vnpu_weight_scale((const uint8_t *)src0->data, N, K);
    for (int m = 0; m < M; m++) {
        float *d = (float *)((char *)dst->data + (size_t)m * dst->nb[1]);
        const int32_t *c = C + (size_t)m * N;
        const float as = ascale[m];
        for (int n = 0; n < N; n++) d[n] = (float)c[n] / as * wscale;
    }
    const uint64_t t4 = vnpu_now();

    const uint64_t pack = st_lane[lane].pack - pack0;
    st_lane[lane].calls++;
    st_lane[lane].prep += (t1 - t0) - pack;
    st_lane[lane].npu += t2 - t1;
    st_lane[lane].check += t3 - t2;
    st_lane[lane].post += t4 - t3;
    if (cfg.verbose)
        fprintf(stderr, "[vela] %-5s %-24s M=%d N=%d K=%d  prep %llu  npu %llu  post %llu %s\n", lane_name[lane], src0->name,
                M, N, K, (unsigned long long)((t1 - t0) - pack), (unsigned long long)(t2 - t1),
                (unsigned long long)(t4 - t3), vnpu_time_unit());
}

static void mm_f16(const struct ggml_tensor *src0, const struct ggml_tensor *src1, struct ggml_tensor *dst) {
    const int K = (int)src0->ne[0], N = (int)src0->ne[1], M = (int)src1->ne[1];
    const int64_t r2 = src1->ne[2] / src0->ne[2], r3 = src1->ne[3] / src0->ne[3];
    const size_t ldb = src0->nb[1] / sizeof(ggml_fp16_t), ldc = dst->nb[1] / sizeof(float);
    ggml_fp16_t *A = (ggml_fp16_t *)scratch(S_A, sizeof(ggml_fp16_t) * (size_t)M * K);

    for (int64_t i13 = 0; i13 < src1->ne[3]; i13++)
        for (int64_t i12 = 0; i12 < src1->ne[2]; i12++) {
            const uint64_t t0 = vnpu_now();
            for (int m = 0; m < M; m++)
                ggml_fp32_to_fp16_row((const float *)((const char *)src1->data + m * src1->nb[1] + i12 * src1->nb[2] + i13 * src1->nb[3]),
                                      A + (size_t)m * K, K);
            const ggml_fp16_t *B = (const ggml_fp16_t *)((const char *)src0->data + (i12 / r2) * src0->nb[2] + (i13 / r3) * src0->nb[3]);
            float *C = (float *)((char *)dst->data + i12 * dst->nb[2] + i13 * dst->nb[3]);

            const uint64_t t1 = vnpu_now();
            vnpu_gemm_f16(M, N, K, A, K, B, ldb, C, ldc);
            const uint64_t t2 = vnpu_now();

            if (st_lane[LANE_F16].calls < (uint64_t)cfg.check) check_f16(src0->name, A, B, ldb, C, ldc, M, N, K);
            const uint64_t t3 = vnpu_now();

            st_lane[LANE_F16].calls++;
            st_lane[LANE_F16].prep += t1 - t0;
            st_lane[LANE_F16].npu += t2 - t1;
            st_lane[LANE_F16].check += t3 - t2;
            if (cfg.verbose)
                fprintf(stderr, "[vela] fp16  %-24s M=%d N=%d K=%d  prep %llu  npu %llu %s\n", src0->name, M, N, K,
                        (unsigned long long)(t1 - t0), (unsigned long long)(t2 - t1), vnpu_time_unit());
        }
}

// ---- RoPE on the velaVPU RoPE unit ------------------------------------------------------------------------
typedef struct { int n_dims, mode; float base, freq_scale, ext_factor, attn_factor; } rope_params_t;

static rope_params_t rope_params(const struct ggml_tensor *t) {
    rope_params_t p;
    const int32_t *op = (const int32_t *)t->op_params;
    p.n_dims = op[1];
    p.mode = op[2];
    memcpy(&p.base, op + 5, sizeof(float));
    memcpy(&p.freq_scale, op + 6, sizeof(float));
    memcpy(&p.ext_factor, op + 7, sizeof(float));
    memcpy(&p.attn_factor, op + 8, sizeof(float));
    return p;
}

// same answer on every thread; warnings only from thread 0
static int rope_decide(const struct ggml_tensor *t, int ith) {
    pthread_once(&init_once, vela_init);
    if (cfg.rope == ROPE_OFF || cfg.rope_base <= 0.0) return 0;
    const struct ggml_tensor *src = t->src[0], *pos = t->src[1];
    const rope_params_t p = rope_params(t);
    // plain RoPE of 128-wide heads: no YaRN / scaling / freq factors, NORMAL or NEOX pairs
    if (src->type != GGML_TYPE_F32 || t->type != GGML_TYPE_F32 || pos->type != GGML_TYPE_I32 || t->src[2]) return 0;
    if (p.n_dims != VROPE_DIM || src->ne[0] != VROPE_DIM || src->ne[3] != 1) return 0;
    if (p.mode != 0 && p.mode != GGML_ROPE_TYPE_NEOX) return 0;
    if (p.freq_scale != 1.0f || p.ext_factor != 0.0f || p.attn_factor != 1.0f) return 0;
    if (src->nb[0] != sizeof(float) || t->nb[0] != sizeof(float)) return 0;
    if (fabs((double)p.base - cfg.rope_base) > 1e-6 * (double)p.base && cfg.rope != ROPE_FORCE) {
        static int warned;
        if (ith == 0 && !warned++)
            fprintf(stderr, "[vela] WARNING: model rope freq_base %g != RoPE unit ROM base %g: RoPE stays on the Rocket core "
                            "(VELA_ROPE=force runs it anyway, with wrong angles)\n", (double)p.base, cfg.rope_base);
        return 0;
    }
    const int32_t *m = (const int32_t *)pos->data;
    for (int64_t i = 0; i < src->ne[2]; i++)
        if (m[i] < 0 || m[i] >= VROPE_MAXPOS) return 0;   // m is 12 bits in the unit
    return 1;
}

// float RoPE (double precision) of one row, for the check
static void rope_ref_row(const float *x, float *y, int m, double base, int neox) {
    for (int j = 0; j < VROPE_PAIRS; j++) {
        const double th = m * pow(base, -2.0 * j / VROPE_DIM), c = cos(th), s = sin(th);
        const int i0 = neox ? j : 2 * j, i1 = neox ? j + VROPE_PAIRS : 2 * j + 1;
        y[i0] = (float)((double)x[i0] * c - (double)x[i1] * s);
        y[i1] = (float)((double)x[i0] * s + (double)x[i1] * c);
    }
}

bool ggml_vela_rope(struct ggml_tensor *dst, int ith, int nth) {
    (void)nth;
    if (!rope_decide(dst, ith)) return false;
    if (ith != 0) return true;   // thread 0 drives the unit; the node barrier waits for it

    const struct ggml_tensor *src = dst->src[0];
    const int32_t *pos = (const int32_t *)dst->src[1]->data;
    const rope_params_t p = rope_params(dst);
    const int neox = p.mode == GGML_ROPE_TYPE_NEOX;
    const int nrows = (int)src->ne[1], ntok = (int)src->ne[2];
    const int check = st_rope[1].calls < (uint64_t)cfg.check;

    // the check needs the input; dst may be a view of src (in-place RoPE)
    float *in = NULL;
    if (check) {
        in = (float *)xalloc(sizeof(float) * VROPE_DIM * (size_t)nrows * ntok);
        for (int t = 0; t < ntok; t++)
            for (int r = 0; r < nrows; r++)
                memcpy(in + ((size_t)t * nrows + r) * VROPE_DIM, (const char *)src->data + t * src->nb[2] + r * src->nb[1],
                       sizeof(float) * VROPE_DIM);
    }

    const uint64_t t0 = vnpu_now();
    for (int t = 0; t < ntok; t++)
        vrope_rows((const float *)((const char *)src->data + t * src->nb[2]), src->nb[1],
                   (float *)((char *)dst->data + t * dst->nb[2]), dst->nb[1], nrows, pos[t], neox);
    const uint64_t t1 = vnpu_now();

    if (check) {
        // error against float RoPE with the model's base, relative to the row's largest pair norm
        double worst = 0.0;
        int wt = 0, wr = 0;
        float ref[VROPE_DIM];
        for (int t = 0; t < ntok; t++)
            for (int r = 0; r < nrows; r++) {
                const float *x = in + ((size_t)t * nrows + r) * VROPE_DIM;
                const float *y = (const float *)((const char *)dst->data + t * dst->nb[2] + r * dst->nb[1]);
                rope_ref_row(x, ref, pos[t], (double)p.base, neox);
                double n2 = 0.0, e = 0.0;
                for (int j = 0; j < VROPE_PAIRS; j++) {
                    const int i0 = neox ? j : 2 * j, i1 = neox ? j + VROPE_PAIRS : 2 * j + 1;
                    const double a = (double)x[i0] * (double)x[i0] + (double)x[i1] * (double)x[i1];
                    if (a > n2) n2 = a;
                }
                for (int i = 0; i < VROPE_DIM; i++) {
                    const double d = fabs((double)y[i] - (double)ref[i]);
                    if (d > e) e = d;
                }
                e = n2 > 0.0 ? e / sqrt(n2) : e;
                if (e > worst) { worst = e; wt = t; wr = r; }
            }
        const int bad = !(worst <= 2e-2);
        st_rope_check_fail += bad;
        fprintf(stderr, "[vela] check rope  %-24s rows=%d tokens=%d: %s (max err %.2e of the pair norm, token %d pos %d row %d)\n",
                dst->name, nrows, ntok, bad ? "FAIL" : "ok", worst, wt, pos[wt], wr);
        free(in);
    }
    if (cfg.verbose)
        fprintf(stderr, "[vela] rope  %-24s rows=%d tokens=%d pos=%d..  vpu %llu %s\n", dst->name, nrows, ntok, pos[0],
                (unsigned long long)(t1 - t0), vnpu_time_unit());
    return true;
}

// ---- hooks -----------------------------------------------------------------------------------------------
bool ggml_vela_mul_mat(const struct ggml_tensor *src0, const struct ggml_tensor *src1, struct ggml_tensor *dst,
                       int ith, int nth) {
    (void)nth;
    const int lane = decide(src0, src1, dst);
    if (lane == LANE_CPU) return false;
    if (ith != 0) return true;   // thread 0 drives the NPU; the node barrier waits for it

    if (lane == LANE_F16) mm_f16(src0, src1, dst);
    else                  mm_ternary(src0, src1, dst, lane);
    return true;
}

uint64_t ggml_vela_now(void) { return vnpu_now(); }

void ggml_vela_account(const struct ggml_tensor *dst, int ith, uint64_t t0) {
    if (ith != 0) return;
    const uint64_t t = vnpu_now() - t0;
    const struct ggml_tensor *src0 = dst->src[0], *src1 = dst->src[1];
    if (dst->op == GGML_OP_ROPE) {
        const int unit = rope_decide(dst, ith);
        st_rope[unit].calls++;
        st_rope[unit].time += t;
        st_rope[unit].rows += (uint64_t)(src0->ne[1] * src0->ne[2] * src0->ne[3]);
        return;
    }
    const int lane = decide(src0, src1, dst);
    const int wt = src0->type == GGML_TYPE_I2_S ? WT_I2S
                 : src0->type == GGML_TYPE_F16  ? WT_F16
                 : src0->type == GGML_TYPE_F32  ? WT_F32 : WT_OTHER;
    st_op[wt][lane].calls++;
    st_op[wt][lane].time += t;
    st_op[wt][lane].macs += (uint64_t)src0->ne[0] * (uint64_t)ggml_nelements(dst);
}

static void print_stats(void) {
    const char *u = vnpu_time_unit();
    fprintf(stderr, "\n[vela] ===== matmul summary: VELA_MODE=%s (I2_S -> %s, F16 -> %s), time in %s =====\n",
            cfg.mode, lane_name[cfg.ternary], fp16_name[cfg.fp16], u);
    fprintf(stderr, "[vela] %-6s %-5s %10s %16s %12s %12s\n", "weight", "lane", "calls", "time", "MMAC", "MAC/unit");
    uint64_t total = 0;
    for (int wt = 0; wt < WT_N; wt++)
        for (int l = 0; l < LANE_N; l++) {
            if (!st_op[wt][l].calls) continue;
            total += st_op[wt][l].time;
            fprintf(stderr, "[vela] %-6s %-5s %10llu %16llu %12.1f %12.3f\n", wt_name[wt], lane_name[l],
                    (unsigned long long)st_op[wt][l].calls, (unsigned long long)st_op[wt][l].time,
                    st_op[wt][l].macs / 1e6, st_op[wt][l].time ? (double)st_op[wt][l].macs / st_op[wt][l].time : 0.0);
        }
    fprintf(stderr, "[vela] all matmuls: %llu %s\n", (unsigned long long)total, u);
    for (int l = LANE_MP; l < LANE_N; l++) {   // NPU lanes and the vela scalar lane
        if (!st_lane[l].calls) continue;
        fprintf(stderr, "[vela] %-5s NPU calls %llu: prep %llu  npu %llu  post %llu  check %llu  | repack (once) %llu, %zu MiB%s\n",
                lane_name[l], (unsigned long long)st_lane[l].calls, (unsigned long long)st_lane[l].prep,
                (unsigned long long)st_lane[l].npu, (unsigned long long)st_lane[l].post, (unsigned long long)st_lane[l].check,
                (unsigned long long)st_lane[l].pack, st_lane[l].pack_bytes >> 20,
                st_lane[l].check_fail ? "  ** CHECK FAILED **" : "");
    }
    for (int k = 0; k < 2; k++) {
        if (!st_rope[k].calls) continue;
        fprintf(stderr, "[vela] rope  on %-11s calls %llu, rows %llu: %llu %s%s\n", k ? "RoPE unit" : "Rocket core",
                (unsigned long long)st_rope[k].calls, (unsigned long long)st_rope[k].rows, (unsigned long long)st_rope[k].time, u,
                k && st_rope_check_fail ? "  ** CHECK FAILED **" : "");
    }
}
