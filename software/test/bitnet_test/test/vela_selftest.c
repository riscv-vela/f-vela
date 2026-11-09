// vela_selftest.c — checks the three velaNPU lanes used by the BitNet port, at BitNet's shapes,
// before running llama-cli. Same kernels as llama (vela/vela_npu.c), CPU reference.
//
//   mp   : I2_S weights -> mp packing -> ternary_gemm_auto, int32 out (full_C)   exact compare
//   mpt  : I2_S weights -> B^T packing -> tiled_matmul(transpose_B, is_mpgemm)  exact compare
//   int8 : I2_S weights -> int8 [K][N] -> tiled_matmul_auto, int32 out           exact compare
// Ternary lanes run from both weight packings (BitNet I2_S and the original model packing).
//   fp16 : fp16 A x B^T (B = [N][K]) -> tiled_matmul_auto_fp, fp32 out          rel. error <= 1e-2
//   rope : velaVPU RoPE unit (vfrope.vx, vela/vela_rope.c): finds the ROM base, then random instructions
//          (16 rows each) bit-exact against the emulation, and float rows against float RoPE (<= 2e-2)
//
// usage: vela_selftest [-q] [-l] [mp] [mpt] [int8] [fp16] [rope]
//   -q  quick: small shapes only (seconds, also in simulation)
//   -l  also the full LM head (N = 128256, K = 2560) on fp16 (slow CPU reference: sampled)
//   no lane names = all lanes
//
// Activations are int8 in [-127, 127], like BitNet's quantize_row_i8_s (scale 127 / max|x|, never -128).
// One extra case per mp lane uses the full int8 range: on the 2026-11 hardware the MpMularray columns
// (16..63 of every 64-column block) compute (-128) x (-1) as -128 instead of +128 (8-bit negate), so each such
// product is off by -256. That case is reported as [KNOWN] when the mismatches are exactly this pattern.
#include "vela_npu.h"
#include "vela_rope.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint32_t rng = 0xB17E7000u;
static inline uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng; }

static void *xalloc(size_t bytes) {
    void *p = aligned_alloc(64, (bytes + 63) & ~(size_t)63);
    if (!p) { fprintf(stderr, "out of memory (%zu bytes)\n", bytes); exit(2); }
    return p;
}

// random ternary tensor in packing fmt: every 2-bit field is a valid code, plus a scale
static uint8_t *make_i2s(int fmt, int N, int K) {
    static const uint8_t orig_codes[3] = {0x0, 0x1, 0x3};
    const size_t nb = (size_t)N * K / 4;
    uint8_t *w = (uint8_t *)xalloc(nb + 32);
    for (size_t i = 0; i < nb; i++) {
        uint8_t b = 0;
        for (int f = 0; f < 4; f++) {
            const unsigned c = (rnd() >> 12) % 3;
            b |= (uint8_t)((fmt == VNPU_FMT_ORIG ? orig_codes[c] : c) << (2 * f));
        }
        w[i] = b;
    }
    const float s = 1.5f;
    memcpy(w + nb, &s, sizeof s);
    return w;
}

static int fails = 0, runs = 0;

static void report(const char *lane, const char *fmt, int M, int N, int K, int bad, int total, uint64_t pack, uint64_t t, double extra) {
    runs++;
    fails += bad != 0;
    const double macs = (double)M * N * K;
    printf("[%s] %-4s %-10s M=%-3d N=%-6d K=%-5d  %d/%d mismatches", bad ? "FAIL" : "PASS", lane, fmt, M, N, K, bad, total);
    if (extra >= 0) printf(" (max rel err %.2e)", extra);
    printf("  gemm %llu %s (%.2f MAC/%s)", (unsigned long long)t, vnpu_time_unit(), t ? macs / t : 0.0,
           vnpu_time_unit()[0] == 'c' ? "cycle" : "ns");
    if (pack) printf("  repack %llu", (unsigned long long)pack);
    printf("\n");
    fflush(stdout);
}

enum { T_MP, T_MPT, T_I8 };
static const char *const tname[] = {"mp", "mpt", "int8"};

static void test_ternary(int lane, int fmt, int M, int N, int K, int full_range) {
    uint8_t *w = make_i2s(fmt, N, K);
    if (vnpu_detect_format(w, N, K) != fmt) { printf("[FAIL] format detection\n"); fails++; runs++; }
    int8_t *A = (int8_t *)xalloc((size_t)M * K);
    for (size_t i = 0; i < (size_t)M * K; i++) {
        A[i] = (int8_t)(rnd() >> 24);
        if (A[i] == -128 && !full_range) A[i] = -127;   // BitNet activations: [-127, 127]
    }
    int32_t *C = (int32_t *)xalloc(sizeof(int32_t) * (size_t)M * N);
    memset(C, 0x5a, sizeof(int32_t) * (size_t)M * N);                              // catch unwritten outputs

    uint64_t t0 = vnpu_now(), pack, t;
    void *packed;
    if (lane == T_MP) {
        packed = xalloc(vnpu_mp_bytes(N, K));
        vnpu_to_mp(fmt, w, N, K, (uint8_t *)packed);
        pack = vnpu_now() - t0;
        t0 = vnpu_now();
        vnpu_gemm_mp(M, N, K, A, K, (const uint8_t *)packed, C, N);
    } else if (lane == T_MPT) {
        packed = xalloc(vnpu_mpt_bytes(N, K));
        vnpu_to_mpt(fmt, w, N, K, (uint8_t *)packed);
        pack = vnpu_now() - t0;
        t0 = vnpu_now();
        vnpu_gemm_mpt(M, N, K, A, K, (const uint8_t *)packed, C, N);
    } else {
        packed = xalloc(vnpu_i8_bytes(N, K));
        vnpu_to_i8(fmt, w, N, K, (int8_t *)packed);
        pack = vnpu_now() - t0;
        t0 = vnpu_now();
        vnpu_gemm_i8(M, N, K, A, K, (const int8_t *)packed, C, N);
    }
    t = vnpu_now() - t0;

    // reference on every output for small N, else whole 64-column groups with gaps
    const int step = (size_t)N * K > (1u << 22) ? 7 : 1;
    int bad = 0, total = 0, known = 0;
    int8_t *wcol = (int8_t *)xalloc(K);
    for (int n = 0; n < N; n += (n % 64 == 63) ? step : 1) {
        for (int k = 0; k < K; k++) wcol[k] = (int8_t)vnpu_weight(fmt, w, K, n, k);
        for (int m = 0; m < M; m++) {
            int32_t ref = 0;
            for (int k = 0; k < K; k++) ref += A[(size_t)m * K + k] * wcol[k];
            const int32_t got = C[(size_t)m * N + n];
            total++;
            if (got == ref) continue;
            if (full_range && lane != T_I8 && n % 64 >= 16) {   // the known -128 x -1 pattern?
                int c = 0;
                for (int k = 0; k < K; k++) c += A[(size_t)m * K + k] == -128 && wcol[k] == -1;
                if (got == ref - 256 * c) { known++; continue; }
            }
            if (bad++ < 4) printf("    [%d][%d] got %d ref %d\n", m, n, got, ref);
        }
    }
    if (full_range) {
        if (bad == 0 && known > 0)
            printf("[KNOWN] %-4s full int8 range: %d/%d outputs off by -256 per (-128 x -1) product (MpMularray "
                   "8-bit negate; BitNet activations never reach -128)\n", tname[lane], known, total);
        else
            report(tname[lane], "int8 -128", M, N, K, bad + known, total, pack, t, -1);
    } else {
        report(tname[lane], vnpu_format_name(fmt), M, N, K, bad, total, pack, t, -1);
    }
    free(wcol); free(packed); free(C); free(A); free(w);
}

static uint16_t f2h(float f) {  // round to nearest even, no denormals (inputs here are normal)
    uint32_t x;
    memcpy(&x, &f, 4);
    const uint32_t sign = (x >> 16) & 0x8000u;
    const int32_t exp = (int32_t)((x >> 23) & 0xff) - 127 + 15;
    uint32_t mant = x & 0x7fffffu;
    if (exp <= 0) return (uint16_t)sign;
    if (exp >= 0x1f) return (uint16_t)(sign | 0x7c00u);
    uint32_t h = sign | ((uint32_t)exp << 10) | (mant >> 13);
    const uint32_t rem = mant & 0x1fff;
    if (rem > 0x1000 || (rem == 0x1000 && (h & 1))) h++;
    return (uint16_t)h;
}
static float h2f(uint16_t h) {
    const uint32_t sign = (uint32_t)(h & 0x8000u) << 16, exp = (h >> 10) & 0x1f, mant = h & 0x3ff;
    uint32_t u = exp == 0 ? sign : sign | ((exp - 15 + 127) << 23) | (mant << 13);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static void test_f16(int M, int N, int K, int sampled) {
    uint16_t *A = (uint16_t *)xalloc(2 * (size_t)M * K), *B = (uint16_t *)xalloc(2 * (size_t)N * K);
    // LLM-like magnitudes: activations ~1, embedding weights ~0.02
    for (size_t i = 0; i < (size_t)M * K; i++) A[i] = f2h(((int)(rnd() >> 20) - 2048) / 1024.0f);
    for (size_t i = 0; i < (size_t)N * K; i++) B[i] = f2h(((int)(rnd() >> 20) - 2048) / 65536.0f);
    float *C = (float *)xalloc(sizeof(float) * (size_t)M * N);
    for (size_t i = 0; i < (size_t)M * N; i++) C[i] = NAN;

    const uint64_t t0 = vnpu_now();
    vnpu_gemm_f16(M, N, K, A, K, B, K, C, N);
    const uint64_t t = vnpu_now() - t0;

    int bad = 0, total = 0;
    double worst = 0;
    const int step = sampled ? 97 : 1;
    for (int n = 0; n < N; n += step)
        for (int m = 0; m < M; m++) {
            double ref = 0, mag = 0;
            for (int k = 0; k < K; k++) {
                const double p = (double)h2f(A[(size_t)m * K + k]) * h2f(B[(size_t)n * K + k]);
                ref += p;
                mag += fabs(p);
            }
            const double err = fabs(C[(size_t)m * N + n] - ref) / (mag + 1e-30);
            total++;
            if (!(err <= 1e-2)) {   // also catches NaN (output not written)
                if (bad++ < 4) printf("    [%d][%d] got %g ref %g\n", m, n, C[(size_t)m * N + n], ref);
            } else if (err > worst) worst = err;
        }
    report("fp16", "", M, N, K, bad, total, 0, t, worst);
    free(C); free(B); free(A);
}

// ---- RoPE unit ------------------------------------------------------------------------------------------------
static void test_rope(int quick) {
    static const double bases[] = {10000.0, 500000.0};
    const uint64_t t0 = vnpu_now();
    const double base = vrope_detect(bases, 2);
    runs++;
    if (base <= 0.0) {
        fails++;
        printf("[FAIL] rope ROM: the unit matches no emulated ROM (base 10000 / 500000)\n");
        return;
    }
    vrope_lut_t *lut = (vrope_lut_t *)xalloc(sizeof(*lut));
    vrope_lut_build(lut, base);
    printf("[PASS] rope ROM base %g  (7 probes x 16 rows bit-exact, %llu %s)\n", base,
           (unsigned long long)(vnpu_now() - t0), vnpu_time_unit());
    if (base != 500000.0)
        printf("[INFO] rope: BitNet b1.58 2B-4T uses rope base 500000; with this ROM llama keeps RoPE on the CPU\n");

    // random instructions, bit-exact
    static int16_t in[VROPE_VL] __attribute__((aligned(64))), out[VROPE_VL] __attribute__((aligned(64)));
    int16_t ref[VROPE_VL];
    const int ninsn = quick ? 64 : 1024;
    int bad = 0, total = 0;
    const uint64_t t1 = vnpu_now();
    for (int n = 0; n < ninsn; n++) {
        for (int e = 0; e < VROPE_VL; e++) in[e] = (int16_t)(rnd() >> 16);
        const int m = (int)(rnd() % VROPE_MAXPOS), i0 = (int)(rnd() % 61);
        vrope_exec(in, out, m, i0);
        vrope_emulate(lut, in, ref, m, i0);
        for (int r = 0; r < VROPE_ROWS; r++)
            for (int l = 0; l < 8; l++) {
                total++;
                if (out[r * VROPE_EG + l] != ref[r * VROPE_EG + l] && bad++ < 4)
                    printf("  m=%d i0=%d row %d lane %d: unit %d emulation %d\n", m, i0, r, l, out[r * VROPE_EG + l],
                           ref[r * VROPE_EG + l]);
            }
    }
    runs++;
    fails += bad != 0;
    printf("[%s] rope %d instructions x 16 rows: %d/%d lanes differ from the emulation (%llu %s)\n", bad ? "FAIL" : "PASS",
           ninsn, bad, total, (unsigned long long)(vnpu_now() - t1), vnpu_time_unit());
    free(lut);

    // float rows (llama's path): 20 heads x 128, NEOX pairs, against float RoPE with the ROM's base
    enum { R = 20 };
    float *x = (float *)xalloc(sizeof(float) * R * VROPE_DIM), *y = (float *)xalloc(sizeof(float) * R * VROPE_DIM);
    double worst = 0.0;
    const int npos = quick ? 4 : 32;
    for (int pi = 0; pi < npos; pi++) {
        const int m = pi == 0 ? 0 : (int)(rnd() % VROPE_MAXPOS);
        for (int i = 0; i < R * VROPE_DIM; i++) x[i] = ((float)(rnd() >> 8) / 16777216.0f - 0.5f) * 20.0f;
        vrope_rows(x, sizeof(float) * VROPE_DIM, y, sizeof(float) * VROPE_DIM, R, m, 1);
        for (int r = 0; r < R; r++) {
            const float *xr = x + r * VROPE_DIM, *yr = y + r * VROPE_DIM;
            double n2 = 0.0, e = 0.0;
            for (int j = 0; j < VROPE_PAIRS; j++) {
                const double a = xr[j], b = xr[j + VROPE_PAIRS];
                const double th = m * pow(base, -2.0 * j / VROPE_DIM);
                const double ya = a * cos(th) - b * sin(th), yb = a * sin(th) + b * cos(th);
                n2 = fmax(n2, a * a + b * b);
                e = fmax(e, fmax(fabs(ya - yr[j]), fabs(yb - yr[j + VROPE_PAIRS])));
            }
            worst = fmax(worst, e / sqrt(n2));
        }
    }
    runs++;
    fails += !(worst <= 2e-2);
    printf("[%s] rope float rows (%d heads x %d positions, NEOX): max err %.2e of the pair norm\n",
           worst <= 2e-2 ? "PASS" : "FAIL", R, npos, worst);
    free(x);
    free(y);
}

int main(int argc, char **argv) {
    int quick = 0, lmhead = 0, want[3] = {0, 0, 0}, want_f16 = 0, want_rope = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-q")) quick = 1;
        else if (!strcmp(argv[i], "-l")) lmhead = 1;
        else if (!strcmp(argv[i], "mp")) want[T_MP] = 1;
        else if (!strcmp(argv[i], "mpt")) want[T_MPT] = 1;
        else if (!strcmp(argv[i], "int8")) want[T_I8] = 1;
        else if (!strcmp(argv[i], "fp16")) want_f16 = 1;
        else if (!strcmp(argv[i], "rope")) want_rope = 1;
        else { fprintf(stderr, "usage: %s [-q] [-l] [mp] [mpt] [int8] [fp16] [rope]\n", argv[0]); return 2; }
    }
    if (!want[0] && !want[1] && !want[2] && !want_f16 && !want_rope) want[0] = want[1] = want[2] = want_f16 = want_rope = 1;

    if (vnpu_init(1) != 0) fprintf(stderr, "warning: memory not locked\n");
    printf("=== velaNPU self-test for BitNet (backend %s, %s) ===\n", vnpu_backend(), quick ? "quick" : "BitNet shapes");

    // BitNet b1.58 2B: attn_k/v 640x2560, attn_q/o 2560x2560, ffn_gate/up 6912x2560, ffn_down 2560x6912
    static const int quick_nk[][2] = {{64, 128}, {128, 256}, {640, 256}};
    static const int full_nk[][2] = {{640, 2560}, {2560, 2560}, {6912, 2560}, {2560, 6912}};
    const int (*nk)[2] = quick ? quick_nk : full_nk;
    const int nshapes = quick ? 3 : 4;
    static const int Ms[] = {1, 5};   // generation (1 token) and a short prompt

    for (int lane = T_MP; lane <= T_I8; lane++) {
        if (!want[lane]) continue;
        for (int fmt = VNPU_FMT_BITNET; fmt <= VNPU_FMT_ORIG; fmt++)
            for (int s = 0; s < nshapes; s++)
                for (int mi = 0; mi < 2; mi++) test_ternary(lane, fmt, Ms[mi], nk[s][0], nk[s][1], 0);
        if (lane != T_I8) test_ternary(lane, VNPU_FMT_BITNET, 5, 128, 256, 1);   // -128 edge case
    }
    if (want_f16) {
        if (quick) { test_f16(1, 64, 128, 0); test_f16(3, 200, 136, 0); }
        else { test_f16(1, 2048, 2560, 0); test_f16(5, 1000, 2560, 0); }
        if (lmhead) test_f16(1, 128256, 2560, 1);
    }
    if (want_rope) test_rope(quick);

    printf("=== vela self-test: %d/%d PASS ===\n", runs - fails, runs);
    return fails ? 1 : 0;
}
