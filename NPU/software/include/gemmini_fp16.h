// =============================================================================
// gemmini_fp16.h — fp16 matmul helpers (runtime is_fp mode).
//
// fp16 runs on the SAME Gemmini as int8/int2 (DIM = 16, WS dataflow). It is
// selected at runtime by the `is_fp` bits the gemmini.h macros emit while the
// global `gemmini_is_fp` flag is set — not by a separate opcode or params header.
// Include AFTER include/gemmini.h. Requires a Gemmini built with support_fp
// (e.g. Fp16MatmulGemminiConfig).
//
// HW contract:
//   - is_fp (execute) : CONFIG_EX rs1 bit 10. 1 => fp16 multiply / fp32 accumulate.
//   - is_fp (load)    : CONFIG_LD rs1 bit 5. 1 => fp16 mvin: each logical row
//                       (32 byte = 16 fp16) is split into 2 spad rows (lo = cols
//                       0..7, hi = cols 8..15). mvin scale MUST be identity.
//   - is_fp (store)   : CONFIG_ST rs1 bit 13. acc holds raw fp32 (not int32).
//   - output_as_float : CONFIG_ST rs1 bit 12. 1 => write the scaled acc as fp32.
//   - is_fp (loop)    : LOOP_WS rs1 bit 21. A/B use 2*DIM spad rows per DIM-block.
//
// Data layout:
//   - fp16 = IEEE half, 2 bytes, little-endian. A spad row (128 bit) = 8 fp16.
//   - bias D and the accumulator are IEEE single (fp32, 4 bytes).
//   - C is fp32 (output_as_float) or int8 = sat8(rne(acc * acc_scale)).
// =============================================================================
#ifndef GEMMINI_FP16_H
#define GEMMINI_FP16_H

#include <stdint.h>
#include <stddef.h>

typedef uint16_t fp16_t;   // IEEE half bit pattern
typedef float    fp32_t;   // IEEE single (accumulator / output)

// Portable IEEE single <-> half bit conversions (truncating; no denormals/NaN
// payloads). Host-side helpers to build fp16 inputs and golden references.
static inline fp16_t fp16_from_float(float f) {
  union { float f; uint32_t u; } v; v.f = f;
  uint32_t x = v.u;
  uint32_t sign = (x >> 16) & 0x8000u;
  int32_t  exp  = (int32_t)((x >> 23) & 0xff) - 127 + 15;
  uint32_t mant = x & 0x7fffffu;
  if (((x >> 23) & 0xff) == 0) return (fp16_t)sign;     // zero/denormal -> signed zero
  if (exp <= 0)    return (fp16_t)sign;                  // underflow -> signed zero
  if (exp >= 0x1f) return (fp16_t)(sign | 0x7c00u);      // overflow -> inf
  return (fp16_t)(sign | ((uint32_t)exp << 10) | (mant >> 13));
}

static inline float fp16_to_float(fp16_t h) {
  uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
  uint32_t exp  = (h >> 10) & 0x1fu;
  uint32_t mant = h & 0x3ffu;
  union { uint32_t u; float f; } v;
  if (exp == 0)    { v.u = sign; return v.f; }
  if (exp == 0x1f) { v.u = sign | 0x7f800000u | (mant << 13); return v.f; }
  v.u = sign | ((exp - 15 + 127) << 23) | (mant << 13);
  return v.f;
}

// fp tile cost: 2*DIM spad rows per DIM-block (the fp16 load splits each row lo/hi);
// the acc tile is fp32-native (DIM rows / block, not doubled).
static inline size_t tiled_matmul_fp_spad_rows(size_t I, size_t J, size_t K) {
  return (I * K + K * J) * (size_t)(2 * DIM);
}
static inline size_t tiled_matmul_fp_acc_rows(size_t I, size_t J) {
  return (I * J) * (size_t)DIM;
}

// fp16 matmul on the hardware WS loop:
//
//   C[I][J] = ((D[I][J] or 0) + A[I][K] @ B) * acc_scale
//
//   A, B            : fp16, dims/strides in fp16 elements
//   D               : fp32 bias (NULL = none; stride_D == 0 broadcasts one row)
//   transpose_B     : B is [J][K] and C += A @ B^T
//   output_as_float : 1 => C is fp32 (4 B/elem); 0 => C is int8 (1 B/elem, pass an
//                     elem_t buffer cast to fp32_t*), stride_C in output elements
//   acc_scale       : fp32 multiply applied in both output modes (ACC_SCALE_IDENTITY = none)
static void tiled_matmul_auto_fp(
    size_t dim_I, size_t dim_J, size_t dim_K,
    const fp16_t *A, const fp16_t *B, const fp32_t *D, fp32_t *C,
    size_t stride_A, size_t stride_B, size_t stride_D, size_t stride_C,
    int transpose_B, int output_as_float, acc_scale_t acc_scale) {

  const size_t dim_I_padded = ((dim_I + DIM - 1) / DIM) * DIM;
  const size_t dim_J_padded = ((dim_J + DIM - 1) / DIM) * DIM;
  const size_t dim_K_padded = ((dim_K + DIM - 1) / DIM) * DIM;

  // WS double-buffers spad/acc (budget halves); an fp block costs 2x spad rows.
  const size_t max_spad_rows = (size_t)(BANK_NUM * BANK_ROWS) / 2;
  const size_t max_acc_rows  = (size_t)ACC_ROWS / 2;

  size_t tile_I = 1, tile_J = 1, tile_K = 1;
  while (1) {
    int inc = 0;
    if (tiled_matmul_fp_spad_rows(tile_I, tile_J + 1, tile_K) <= max_spad_rows &&
        tiled_matmul_fp_acc_rows(tile_I, tile_J + 1) <= max_acc_rows &&
        (tile_J + 1) * DIM <= dim_J_padded) { tile_J++; inc = 1; }
    if (tiled_matmul_fp_spad_rows(tile_I + 1, tile_J, tile_K) <= max_spad_rows &&
        tiled_matmul_fp_acc_rows(tile_I + 1, tile_J) <= max_acc_rows &&
        (tile_I + 1) * DIM <= dim_I_padded) { tile_I++; inc = 1; }
    if (tiled_matmul_fp_spad_rows(tile_I, tile_J, tile_K + 1) <= max_spad_rows &&
        (tile_K + 1) * DIM <= dim_K_padded) { tile_K++; inc = 1; }
    if (!inc) break;
  }

  // Stock WS tiled matmul in fp mode: the global flag makes every config_ex/st/ld and
  // gemmini_loop_ws carry is_fp. mvin scales = identity (fp16 byte passthrough).
  const bool o2f = output_as_float ? true : false;
  gemmini_is_fp = true;
  tiled_matmul(dim_I, dim_J, dim_K,
      (const elem_t *)A, (const elem_t *)B, (const void *)D, (void *)C,
      stride_A, stride_B, stride_D, stride_C,
      MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY, MVIN_SCALE_IDENTITY,
      NO_ACTIVATION, acc_scale, 0 /*bert_scale*/,
      (stride_D == 0) ? true : false /*repeating_bias*/,
      tile_I, tile_J, tile_K,
      false /*transpose_A*/, transpose_B ? true : false,
      o2f /*full_C: fp32 out = full acc row, int8 out = narrow*/, false /*low_D*/, 0 /*weightA*/,
      WS, false /*is_mpgemm*/, o2f);
  gemmini_is_fp = false;
}

#endif // GEMMINI_FP16_H
