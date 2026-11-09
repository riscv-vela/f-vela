// vela_npu.h — velaNPU (custom Gemmini) kernels used by the BitNet port, without ggml.
//
// Three compute lanes of the NPU (velaNPU/MpWontolic.scala):
//   mp   : MpExeUnit / MpArray     2-bit ternary weights, int8 activations (is_mpgemm = 1)
//   int8 : Mularray + AdderTree    plain int8 x int8 (is_mpgemm = 0, is_fp = 0)
//   fp16 : FpExeUnit               fp16 x fp16, fp32 accumulate (is_fp = 1, needs support_fp)
// All GEMMs wait for completion (fence) before they return.
//
// Built with -DVELA_EMULATE (host), every call is computed in C with the same data
// formats, so the packing and the ggml glue can be checked on x86 against the real model.
#ifndef VELA_NPU_H
#define VELA_NPU_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- ternary weight formats (I2_S tensors) -----------------------------------------------------
// W[N][K]; both packings are N*K/4 bytes followed by one fp32 scale.
//   VNPU_FMT_BITNET  BitNet I2_S (QK_I2_S = 128, non-ARM): element e = n*K + k lies in byte
//                    (e/128)*32 + (e%128)%32 at bit shift 6 - 2*((e%128)/32); codes 0/1/2 = -1/0/+1
//   VNPU_FMT_ORIG    model/i2s_original.gguf: row n = K/4 bytes, weight k at bits [2(k%4)+1:2(k%4)]
//                    of byte k/4; codes 00 = 0, 01 = +1, 11 = -1 (the MpPE codes)
enum { VNPU_FMT_UNKNOWN = -1, VNPU_FMT_BITNET = 0, VNPU_FMT_ORIG = 1 };
int         vnpu_detect_format(const uint8_t *w, int N, int K);   // scans the codes
const char *vnpu_format_name(int fmt);

static inline int vnpu_weight(int fmt, const uint8_t *w, int K, int n, int k) {
    if (fmt == VNPU_FMT_ORIG) {
        const int c = (w[((size_t)n * K + k) >> 2] >> (2 * (k & 3))) & 3;
        return c == 1 ? 1 : (c == 3 ? -1 : 0);
    }
    const size_t e = (size_t)n * K + k;
    const int j = (int)(e & 127);
    return ((w[(e >> 7) * 32 + (j & 31)] >> (6 - 2 * (j >> 5))) & 3) - 1;
}
static inline float vnpu_weight_scale(const uint8_t *w, int N, int K) {
    return *(const float *)(w + (size_t)N * K / 4);
}

// ---- repacking (one time per weight tensor) ---------------------------------------------------
// mp  : Bp[K][N/4], weight (k, n = 4*jp + i) in bits [2i+1:2i] of Bp[k][jp]; 01 = +1, 11 = -1, 00 = 0
//       (MpPE TernaryMulUnit: bit0 = enable, bit1 = sign)
// mpt : B^T for mpgemm with b_transpose: BT[N/4][K] bytes, bytes [16*kb + 4*c, +4) of row n4 hold
//       k = 16*kb .. 16*kb+15 of column 4*n4 + c (4 per byte, LSB first, MpPE codes). From the
//       original packing this is only a 4-byte-word interleave of 4 rows.
// int8: B8[K][N], values -1/0/+1
size_t vnpu_mp_bytes(int N, int K);
size_t vnpu_mpt_bytes(int N, int K);
size_t vnpu_i8_bytes(int N, int K);
void   vnpu_to_mp (int fmt, const uint8_t *w, int N, int K, uint8_t *bp);
void   vnpu_to_mpt(int fmt, const uint8_t *w, int N, int K, uint8_t *bt);
void   vnpu_to_i8 (int fmt, const uint8_t *w, int N, int K, int8_t *b8);
// CPU reference / fallback: C[M][N] int32 = A[M][K] int8 . W^T (scalar, any format)
void   vnpu_cpu_ternary(int fmt, const uint8_t *w, int M, int N, int K, const int8_t *A, size_t lda,
                        int32_t *C, size_t ldc);

// ---- NPU ---------------------------------------------------------------------------------------
// Lock memory (the NPU DMA needs resident pages) and flush the NPU TLB. Returns 0 on success.
int         vnpu_init(int lock_memory);
const char *vnpu_backend(void);   // "gemmini" or "emulated"
uint64_t    vnpu_now(void);       // rdcycle (RISC-V), nanoseconds (host)
const char *vnpu_time_unit(void); // "cycles" or "ns"

// C[M][N] int32 (row stride ldc) = A[M][K] int8 (row stride lda) x W, W in mp format (N % 64 == 0)
void vnpu_gemm_mp(int M, int N, int K, const int8_t *A, size_t lda, const uint8_t *bp,
                  int32_t *C, size_t ldc);
// same, W in mpt format (mpgemm + b_transpose)
void vnpu_gemm_mpt(int M, int N, int K, const int8_t *A, size_t lda, const uint8_t *bt,
                   int32_t *C, size_t ldc);
// same, W as int8 B8[K][N] on the int8 array (AdderTree lane)
void vnpu_gemm_i8(int M, int N, int K, const int8_t *A, size_t lda, const int8_t *b8,
                  int32_t *C, size_t ldc);
// C[M][N] fp32 (ldc) = A[M][K] fp16 (lda) x B^T, B[N][K] fp16 (row stride ldb)  (FpExeUnit)
void vnpu_gemm_f16(int M, int N, int K, const uint16_t *A, size_t lda, const uint16_t *B, size_t ldb,
                   float *C, size_t ldc);

#ifdef __cplusplus
}
#endif

#endif // VELA_NPU_H
