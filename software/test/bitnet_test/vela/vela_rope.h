// vela_rope.h — RoPE on the velaVPU RoPE unit (custom instruction vfrope.vx, CustomRoPEUnit.scala).
//
// The unit (one instruction, SEW=16):
//   vd = vfrope(vs2, rs1),  rs1 = m << 16 | idx      m = position (0..4095), idx = 2 * i0
//   In each element group (dLen = 256 bits = 16 x e16 on the F-Vela SoC) lanes 0..7 hold 4 pairs
//   (x0,x1),(x2,x3),... in Q1.15; pair k is rotated by m * theta(i0 + k), theta(i) = base^(-2i/128).
//   Lanes 8..15 are not written. cos/sin come from two ROMs built from vela_vpu_rope_test/make_lut.py:
//     A[q][i] = fp16 {sin, cos}(64 q * theta(i)),  B[r][i] = Q1.7 {sin, cos}(r * theta(i)),  m = 64 q + r
//   The base is fixed in the bitstream (make_lut.py -> RoPELutData.scala: 500000 since 2026-11-05, 10000 before);
//   the head size is fixed to 128. vrope_detect finds out which one the running bitstream has.
//
// vrope_exec runs one instruction over 16 rows (LMUL 8, vl 256: every element group gets the same m, i0).
// With VELA_EMULATE (host) the instruction is emulated bit-exactly; the emulated ROM base is
// VELA_EMU_ROPE_BASE (default 500000, like make_lut.py).
#ifndef VELA_ROPE_H
#define VELA_ROPE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VROPE_DIM     128    // head size (n_dims) the ROM is built for
#define VROPE_PAIRS   64
#define VROPE_MAXPOS  4096   // m is 12 bits
#define VROPE_EG      16     // e16 elements per element group (dLen 256)
#define VROPE_ROWS    16     // element groups per instruction (LMUL 8, VLEN 512)
#define VROPE_VL      (VROPE_ROWS * VROPE_EG)

// ROM contents for a given base (exactly what make_lut.py writes into lutA.hex / lutB.hex)
typedef struct {
    double   base;
    uint32_t a[VROPE_MAXPOS];   // [q*64 + i] = sin_fp16 << 16 | cos_fp16
    uint16_t b[VROPE_MAXPOS];   // [r*64 + i] = sin_q1p7 << 8  | cos_q1p7
} vrope_lut_t;

void vrope_lut_build(vrope_lut_t *lut, double base);

// One instruction, emulated: in/out = VROPE_VL int16 (row r = lanes 16r..16r+7), pairs i0..i0+3, position m.
// Lanes 8..15 of each group are left as they are in out.
void vrope_emulate(const vrope_lut_t *lut, const int16_t *in, int16_t *out, int m, int i0);

// One instruction on the RoPE unit (RISC-V) or emulated (VELA_EMULATE). Returns vl (must be VROPE_VL).
int vrope_exec(const int16_t *in, int16_t *out, int m, int i0);

// Finds what the unit's ROM holds: runs probes on the unit (all 16 rows, several m / i0, incl. ROM entries
// of -1.0 of both bases) and compares them bit-exactly with the emulation of each candidate base.
// Returns the matching base, or 0 if none matches (or the vector length is not VROPE_VL).
double vrope_detect(const double *bases, int nbases);

// Rotates nrows rows of VROPE_DIM floats, all at position m (0 <= m < VROPE_MAXPOS), src -> dst
// (may alias); strides in bytes. neox: pairs (x[j], x[j+64]) (GGML_ROPE_TYPE_NEOX), else (x[2j], x[2j+1]).
// Each row is scaled into Q1.15 by its largest pair norm, so any magnitude works.
void vrope_rows(const float *src, size_t src_stride, float *dst, size_t dst_stride, int nrows, int m, int neox);

#ifdef __cplusplus
}
#endif

#endif // VELA_ROPE_H
