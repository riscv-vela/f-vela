// vela_npu.c — velaNPU kernels for the BitNet port (see vela_npu.h).
//
// RISC-V build: Gemmini via software/include/gemmini.h (+ gemmini_fp16.h).
// Host build (-DVELA_EMULATE): the same operations in plain C.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "vela_npu.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- weight formats / repack -------------------------------------------------------------------

int vnpu_detect_format(const uint8_t *w, int N, int K) {
    // code 10 only exists in the BitNet packing (+1), code 11 only in the original one (-1)
    const size_t nb = (size_t)N * K / 4;
    uint64_t has2 = 0, has3 = 0;
    size_t i = 0;
    for (; i + 8 <= nb; i += 8) {
        uint64_t x;
        memcpy(&x, w + i, 8);
        const uint64_t hi = (x >> 1) & 0x5555555555555555ull, lo = x & 0x5555555555555555ull;
        has3 |= hi & lo;
        has2 |= hi & ~lo;
        if (has2 && has3) return VNPU_FMT_UNKNOWN;
    }
    for (; i < nb; i++) {
        const unsigned hi = (w[i] >> 1) & 0x55u, lo = w[i] & 0x55u;
        has3 |= hi & lo;
        has2 |= hi & ~lo;
    }
    if (has2 && !has3) return VNPU_FMT_BITNET;
    if (has3 && !has2) return VNPU_FMT_ORIG;
    return VNPU_FMT_UNKNOWN;   // both (corrupt) or neither (all-zero / single-sign tensor)
}

const char *vnpu_format_name(int fmt) {
    return fmt == VNPU_FMT_BITNET ? "bitnet-i2s" : fmt == VNPU_FMT_ORIG ? "original" : "unknown";
}

// Codes (0/1/2 = -1/0/+1) of weight row n, from either packing. K must be a multiple of 128.
static void row_codes(int fmt, const uint8_t *w, int K, int n, uint8_t *out) {
    const uint8_t *row = w + (size_t)n * K / 4;
    if (fmt == VNPU_FMT_ORIG) {
        static const uint8_t dec[4] = {1, 2, 1, 0};   // 00 -> 0, 01 -> +1, 11 -> -1 (10 unused)
        for (int b = 0; b < K / 4; b++) {
            const uint8_t q = row[b];
            out[4 * b]     = dec[q & 3];
            out[4 * b + 1] = dec[(q >> 2) & 3];
            out[4 * b + 2] = dec[(q >> 4) & 3];
            out[4 * b + 3] = dec[(q >> 6) & 3];
        }
        return;
    }
    for (int b = 0; b < K / 128; b++) {
        const uint8_t *xb = row + b * 32;
        uint8_t *o = out + b * 128;
        for (int p = 0; p < 32; p++) {
            const uint8_t q = xb[p];
            o[p]      = (q >> 6) & 3;
            o[32 + p] = (q >> 4) & 3;
            o[64 + p] = (q >> 2) & 3;
            o[96 + p] = q & 3;
        }
    }
}

// code (0/1/2) -> MpPE code: -1 -> 0b11, 0 -> 0b00, +1 -> 0b01
static const uint8_t mp_enc[4] = {0x3, 0x0, 0x1, 0x0};

size_t vnpu_mp_bytes(int N, int K) { return (size_t)K * (size_t)(N / 4); }
size_t vnpu_mpt_bytes(int N, int K) { return (size_t)K * (size_t)(N / 4); }
size_t vnpu_i8_bytes(int N, int K) { return (size_t)K * (size_t)N; }

void vnpu_to_mp(int fmt, const uint8_t *w, int N, int K, uint8_t *bp) {
    assert(K % 128 == 0 && N % 4 == 0);
    // bp[k][jp] = MpPE codes of rows 4jp..4jp+3 at k. Built JB packed columns at a time so the writes are
    // JB consecutive bytes (one byte per row k with stride N/4 was ~18 cycles/weight on Rocket: cache misses).
    // lut[f][q] = MpPE code of 2-bit field f (bits 2f+1:2f) of source byte q
    static uint8_t lut[4][256];
    static int lut_fmt = -1;
    if (lut_fmt != fmt) {
        for (int f = 0; f < 4; f++)
            for (int q = 0; q < 256; q++) {
                const int c = (q >> (2 * f)) & 3;
                lut[f][q] = fmt == VNPU_FMT_ORIG ? (uint8_t)c : mp_enc[c];   // original packing = MpPE codes
            }
        lut_fmt = fmt;
    }
    enum { JB = 32 };
    const int N4 = N / 4;
    const size_t rb = (size_t)K / 4;   // bytes per source row
    for (int jp0 = 0; jp0 < N4; jp0 += JB) {
        const int jn = N4 - jp0 < JB ? N4 - jp0 : JB;
        for (size_t x = 0; x < rb; x++) {          // source byte x of each row: 4 weights
            // k of field f: BitNet I2_S: 128-element blocks, byte p holds p, 32+p, 64+p, 96+p in bits 7:6 .. 1:0
            //               original:   byte x holds k = 4x + f in bits 2f+1:2f
            size_t kf[4];
            for (int f = 0; f < 4; f++)
                kf[f] = fmt == VNPU_FMT_ORIG ? 4 * x + f : (x / 32) * 128 + (3 - f) * 32 + x % 32;
            for (int j = 0; j < jn; j++) {
                const uint8_t *src = w + (size_t)4 * (jp0 + j) * rb + x;
                const uint8_t q0 = src[0], q1 = src[rb], q2 = src[2 * rb], q3 = src[3 * rb];
                for (int f = 0; f < 4; f++)
                    bp[kf[f] * N4 + jp0 + j] =
                        (uint8_t)(lut[f][q0] | (lut[f][q1] << 2) | (lut[f][q2] << 4) | (lut[f][q3] << 6));
            }
        }
    }
}

void vnpu_to_mpt(int fmt, const uint8_t *w, int N, int K, uint8_t *bt) {
    assert(K % 128 == 0 && N % 4 == 0);
    const int N4 = N / 4, KB = K / 16;
    if (fmt == VNPU_FMT_ORIG) {
        // the original rows already hold the MpPE codes along K: only interleave 4 rows in 4-byte words
        for (int n4 = 0; n4 < N4; n4++) {
            uint32_t *dst = (uint32_t *)(bt + (size_t)n4 * K);
            const uint32_t *r[4];
            for (int c = 0; c < 4; c++) r[c] = (const uint32_t *)(w + (size_t)(4 * n4 + c) * K / 4);
            for (int kb = 0; kb < KB; kb++)
                for (int c = 0; c < 4; c++) dst[4 * kb + c] = r[c][kb];
        }
        return;
    }
    uint8_t *codes = (uint8_t *)malloc((size_t)4 * K);
    for (int n4 = 0; n4 < N4; n4++) {
        for (int c = 0; c < 4; c++) row_codes(fmt, w, K, 4 * n4 + c, codes + (size_t)c * K);
        uint8_t *dst = bt + (size_t)n4 * K;
        for (int kb = 0; kb < KB; kb++)
            for (int c = 0; c < 4; c++)
                for (int q = 0; q < 4; q++) {
                    const uint8_t *cc = codes + (size_t)c * K + 16 * kb + 4 * q;
                    dst[16 * kb + 4 * c + q] = (uint8_t)(mp_enc[cc[0]] | (mp_enc[cc[1]] << 2) | (mp_enc[cc[2]] << 4) | (mp_enc[cc[3]] << 6));
                }
    }
    free(codes);
}

void vnpu_to_i8(int fmt, const uint8_t *w, int N, int K, int8_t *b8) {
    assert(K % 128 == 0);
    enum { R = 16 };  // rows per pass: 16 consecutive bytes per B8 row write
    uint8_t *codes = (uint8_t *)malloc((size_t)R * K);
    for (int n0 = 0; n0 < N; n0 += R) {
        const int r = N - n0 < R ? N - n0 : R;
        for (int i = 0; i < r; i++) row_codes(fmt, w, K, n0 + i, codes + (size_t)i * K);
        for (int k = 0; k < K; k++) {
            int8_t *dst = b8 + (size_t)k * N + n0;
            for (int i = 0; i < r; i++) dst[i] = (int8_t)codes[(size_t)i * K + k] - 1;
        }
    }
    free(codes);
}

// Scalar CPU matmul for the original packing (BitNet's own kernel only reads its packing):
// C[M][N] = A[M][K] . W^T, exact int32
void vnpu_cpu_ternary(int fmt, const uint8_t *w, int M, int N, int K, const int8_t *A, size_t lda, int32_t *C, size_t ldc) {
    uint8_t *codes = (uint8_t *)malloc((size_t)K);
    for (int n = 0; n < N; n++) {
        row_codes(fmt, w, K, n, codes);
        for (int m = 0; m < M; m++) {
            const int8_t *a = A + (size_t)m * lda;
            int32_t s = 0;
            for (int k = 0; k < K; k++) s += ((int)codes[k] - 1) * a[k];
            C[(size_t)m * ldc + n] = s;
        }
    }
    free(codes);
}

#if defined(VELA_EMULATE)
// =================================================================================================
// Host emulation
// =================================================================================================
#include <time.h>

int vnpu_init(int lock_memory) { (void)lock_memory; return 0; }
const char *vnpu_backend(void) { return "emulated"; }
const char *vnpu_time_unit(void) { return "ns"; }
uint64_t vnpu_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

// -DVELA_EMULATE_NEG128: also emulate the 2026-11 hardware's MpMularray limit: columns 16..63 of every 64-column
// block compute (-128) x (-1) as -128 (8-bit negate). BitNet activations are in [-127, 127], so it never shows there.
#if defined(VELA_EMULATE_NEG128)
static inline int mp_neg(int a, int n) { return (a == -128 && n % 64 >= 16) ? -128 : -a; }
#else
static inline int mp_neg(int a, int n) { (void)n; return -a; }
#endif

void vnpu_gemm_mp(int M, int N, int K, const int8_t *A, size_t lda, const uint8_t *bp,
                  int32_t *C, size_t ldc) {
    const int N4 = N / 4;
    for (int m = 0; m < M; m++) {
        int32_t *c = C + (size_t)m * ldc;
        memset(c, 0, sizeof(int32_t) * N);
        for (int k = 0; k < K; k++) {
            const int a = A[(size_t)m * lda + k];
            if (!a) continue;
            const uint8_t *row = bp + (size_t)k * N4;
            for (int jp = 0; jp < N4; jp++) {
                const uint8_t q = row[jp];
                for (int i = 0; i < 4; i++) {
                    const int code = (q >> (2 * i)) & 3;           // bit0 = enable, bit1 = sign
                    if (code & 1) c[4 * jp + i] += (code & 2) ? mp_neg(a, 4 * jp + i) : a;
                }
            }
        }
    }
}

void vnpu_gemm_mpt(int M, int N, int K, const int8_t *A, size_t lda, const uint8_t *bt,
                   int32_t *C, size_t ldc) {
    // bt[N/4][K]: column n = 4*n4 + c, k = 16*kb + 4*q + i at byte n4*K + 16*kb + 4*c + q, bits 2i
    for (int m = 0; m < M; m++) {
        const int8_t *a = A + (size_t)m * lda;
        for (int n = 0; n < N; n++) {
            const uint8_t *row = bt + (size_t)(n / 4) * K + 4 * (n % 4);
            int32_t s = 0;
            for (int k = 0; k < K; k++) {
                const int code = (row[16 * (k / 16) + (k % 16) / 4] >> (2 * (k % 4))) & 3;
                if (code & 1) s += (code & 2) ? mp_neg(a[k], n) : a[k];
            }
            C[(size_t)m * ldc + n] = s;
        }
    }
}

void vnpu_gemm_i8(int M, int N, int K, const int8_t *A, size_t lda, const int8_t *b8,
                  int32_t *C, size_t ldc) {
    for (int m = 0; m < M; m++) {
        int32_t *c = C + (size_t)m * ldc;
        memset(c, 0, sizeof(int32_t) * N);
        for (int k = 0; k < K; k++) {
            const int a = A[(size_t)m * lda + k];
            const int8_t *b = b8 + (size_t)k * N;
            for (int n = 0; n < N; n++) c[n] += a * b[n];
        }
    }
}

static float h2f(uint16_t h) {
    const uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
    uint32_t exp = (h >> 10) & 0x1f, mant = h & 0x3ff, u;
    if (exp == 0) {
        if (mant == 0) u = sign;
        else {  // denormal
            exp = 127 - 15 + 1;
            while (!(mant & 0x400)) { mant <<= 1; exp--; }
            u = sign | (exp << 23) | ((mant & 0x3ff) << 13);
        }
    } else if (exp == 0x1f) u = sign | 0x7f800000u | (mant << 13);
    else u = sign | ((exp - 15 + 127) << 23) | (mant << 13);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

// fp32 products, fp32 accumulation (the FpExeUnit rounds each product to fp16 first; the
// emulation only checks the data layout, not that rounding).
void vnpu_gemm_f16(int M, int N, int K, const uint16_t *A, size_t lda, const uint16_t *B, size_t ldb,
                   float *C, size_t ldc) {
    float *a = (float *)malloc(sizeof(float) * K);
    for (int m = 0; m < M; m++) {
        for (int k = 0; k < K; k++) a[k] = h2f(A[(size_t)m * lda + k]);
        for (int n = 0; n < N; n++) {
            const uint16_t *b = B + (size_t)n * ldb;
            float s = 0.f;
            for (int k = 0; k < K; k++) s += a[k] * h2f(b[k]);
            C[(size_t)m * ldc + n] = s;
        }
    }
    free(a);
}

#else
// =================================================================================================
// Gemmini (velaNPU)
// =================================================================================================
#include <sys/mman.h>
// gemmini.h is a header-only driver written without -Wpedantic/-Wshadow in mind
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#include "gemmini.h"
#include "gemmini_fp16.h"
#pragma GCC diagnostic pop

int vnpu_init(int lock_memory) {
    int rc = 0;
    // The NPU DMA translates through its own TLB/PTW and cannot take page faults:
    // every buffer it touches must be resident (same as the vela_npu_test Linux tests).
    if (lock_memory && mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("[vela] mlockall");
        rc = -1;
    }
    gemmini_flush(0);
    return rc;
}
const char *vnpu_backend(void) { return "gemmini"; }
const char *vnpu_time_unit(void) { return "cycles"; }
uint64_t vnpu_now(void) {
    uint64_t c;
    __asm__ volatile("rdcycle %0" : "=r"(c));
    return c;
}

void vnpu_gemm_mp(int M, int N, int K, const int8_t *A, size_t lda, const uint8_t *bp,
                  int32_t *C, size_t ldc) {
    // stride_B is the logical (unpacked) column count; ternary_gemm_auto divides it by 4.
    ternary_gemm_auto((size_t)M, (size_t)N, (size_t)K, (const elem_t *)A, (const elem_t *)bp, NULL, C,
                      lda, (size_t)N, 0, ldc,
                      MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
                      NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, false,
                      true /*full_C: int32 out*/, false /*low_D*/);
    gemmini_fence();
}

// mpgemm with b_transpose (LoopMatmul mpgemm_transpose). Tiling as in ternary_gemm_auto: the packed
// B tile costs the same spad rows as without transpose; dim_J is in packed (N/4) columns.
void vnpu_gemm_mpt(int M, int N, int K, const int8_t *A, size_t lda, const uint8_t *bt,
                   int32_t *C, size_t ldc) {
    const size_t dim_I = (size_t)M, dim_J = (size_t)N / 4, dim_K = (size_t)K;
    const size_t I_blk = (dim_I + DIM - 1) / DIM, J_blk = (dim_J + DIM - 1) / DIM, K_blk = (dim_K + DIM - 1) / DIM;
    const size_t max_spad_rows = BANK_NUM * BANK_ROWS / 2, max_acc_rows = ACC_ROWS / 2;
    const size_t max_tk = (BANK_NUM * BANK_ROWS / 4 / DIM) / (size_t)sqrt(ACC_ROWS / 2 / DIM);
    size_t ti = I_blk < 2 ? I_blk : 1, tj = J_blk < 4 ? J_blk : 2, tk = K_blk < max_tk ? K_blk : max_tk;
    for (;;) {
        bool inc = false;
        if ((size_t)tiled_mpgemm_total_spad_rows(ti, tj + 1, tk) <= max_spad_rows &&
            (size_t)tiled_matmul_total_acc_rows(ti, (tj + 1) * 4) <= max_acc_rows && tj + 1 <= J_blk) { tj++; inc = true; }
        if ((size_t)tiled_mpgemm_total_spad_rows(ti + 1, tj, tk) <= max_spad_rows &&
            (size_t)tiled_matmul_total_acc_rows(ti + 1, tj * 4) <= max_acc_rows && ti + 1 <= I_blk) { ti++; inc = true; }
        if ((size_t)tiled_mpgemm_total_spad_rows(ti, tj, tk + 1) <= max_spad_rows && tk + 1 <= K_blk) { tk++; inc = true; }
        if (!inc) break;
    }
    tiled_matmul(dim_I, dim_J, dim_K, (const elem_t *)A, (const elem_t *)bt, NULL, C,
                 lda, (size_t)K /*B^T row stride in bytes*/, 0, ldc,
                 MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
                 NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, false,
                 ti, tj, tk,
                 false, true,                         // transpose_A, transpose_B
                 true /*full_C*/, false, 0,
                 WS, true /*is_mpgemm*/, false);
    gemmini_fence();
}

void vnpu_gemm_i8(int M, int N, int K, const int8_t *A, size_t lda, const int8_t *b8,
                  int32_t *C, size_t ldc) {
    tiled_matmul_auto((size_t)M, (size_t)N, (size_t)K, (const elem_t *)A, (const elem_t *)b8, NULL, C,
                      lda, (size_t)N, 0, ldc,
                      MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
                      NO_ACTIVATION, ACC_SCALE_IDENTITY, 0, false,
                      false, false,                       // transpose_A, transpose_B
                      true /*full_C: int32 out*/, false /*low_D*/,
                      0, WS);
    gemmini_fence();
}

void vnpu_gemm_f16(int M, int N, int K, const uint16_t *A, size_t lda, const uint16_t *B, size_t ldb,
                   float *C, size_t ldc) {
    tiled_matmul_auto_fp((size_t)M, (size_t)N, (size_t)K, (const fp16_t *)A, (const fp16_t *)B, NULL, C,
                         lda, ldb, 0, ldc,
                         1 /*transpose_B: B is [N][K]*/, 1 /*output_as_float*/, ACC_SCALE_IDENTITY);
    gemmini_fence();
}
#endif
