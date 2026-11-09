// vela_rope.c — velaVPU RoPE unit (vfrope.vx): bit-exact emulation, the instruction, and float rows.
// See vela_rope.h for the unit's behaviour.
#include "vela_rope.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ---- ROM contents (rv_rope_test/make_lut.py) ---------------------------------------------------------
// np.float16(x): round to nearest even, from the double
static uint16_t f16_bits(double x) {
    const uint16_t sign = signbit(x) ? 0x8000 : 0;
    const double a = fabs(x);
    if (a == 0.0) return sign;
    int e;
    frexp(a, &e);
    int E = e - 1;                                   // a = 1.f * 2^E
    if (E < -14) return sign | (uint16_t)rint(ldexp(a, 24));        // subnormal (rounds up into 0x0400 if needed)
    double mant = rint(ldexp(a, 10 - E));           // 1024..2048
    if (mant == 2048.0) { mant = 1024.0; E++; }
    if (E > 15) return sign | 0x7c00;
    return sign | (uint16_t)((E + 15) << 10) | (uint16_t)(mant - 1024.0);
}

// q1p7(x): clip to [-1, 127/128], Python round (half to even), as uint8
static uint8_t q1p7_bits(double x) {
    if (x < -1.0) x = -1.0;
    if (x > 0.9921875) x = 0.9921875;
    return (uint8_t)((int)rint(x * 128.0) & 0xff);
}

void vrope_lut_build(vrope_lut_t *lut, double base) {
    double theta[VROPE_PAIRS];
    for (int i = 0; i < VROPE_PAIRS; i++) theta[i] = pow(base, -2.0 * i / VROPE_DIM);
    for (int q = 0; q < 64; q++)
        for (int i = 0; i < 64; i++) {
            const double ph = (q * 64.0) * theta[i];
            lut->a[q * 64 + i] = (uint32_t)f16_bits(sin(ph)) << 16 | f16_bits(cos(ph));
        }
    for (int r = 0; r < 64; r++)
        for (int i = 0; i < 64; i++) {
            const double ph = (double)r * theta[i];
            lut->b[r * 64 + i] = (uint16_t)(q1p7_bits(sin(ph)) << 8 | q1p7_bits(cos(ph)));
        }
    lut->base = base;
}

// ---- datapath (CustomRoPEUnit.scala, S3/S4) ------------------------------------------------------------
// Checked against the generated Verilog of the 2026-11-02 bitstream (FireSim-generated.sv, module RoPEUnit).
// fp16_to_q15: |h| * 2^15 rounded half up, clamped to 32767 (+) / 32768 (-); -1.0 -> -32768 (16-bit negate).
static int32_t fp16_to_q15(uint16_t h) {
    const unsigned sign = h >> 15, exp = (h >> 10) & 31, frac = h & 1023;
    uint32_t mag;
    if (exp == 0 && frac == 0) mag = 0;
    else if (exp == 0)         mag = (frac + 256) >> 9;
    else if (exp == 31)        mag = 32768;
    else {
        const uint32_t mant = 1024 + frac;
        if (exp >= 10) mag = mant << (exp - 10);
        else {
            const unsigned sh = 10 - exp;
            mag = (mant + (1u << (sh - 1))) >> sh;
        }
    }
    if (!sign) return mag >= 32767 ? 32767 : (int32_t)mag;
    if (mag >= 32768) return -32768;
    return -(int32_t)mag;
}

static int32_t b8_to_q15(uint8_t x) { return (int32_t)(int8_t)x * 256; }

// mulQ15: (a*b +- 2^14) >> 15, arithmetic shift (negative results round down after -2^14)
static int64_t mul_q15(int64_t a, int64_t b) {
    const int64_t p = a * b;
    return (p >= 0 ? p + 16384 : p - 16384) >> 15;
}

static int32_t sat16(int64_t x) { return x > 32767 ? 32767 : x < -32768 ? -32768 : (int32_t)x; }

void vrope_emulate(const vrope_lut_t *lut, const int16_t *in, int16_t *out, int m, int i0) {
    const unsigned q = ((unsigned)m >> 6) & 63, r = (unsigned)m & 63;
    int32_t c[4], s[4];
    for (int k = 0; k < 4; k++) {
        const unsigned i = (unsigned)(i0 + k) & 63;
        const uint32_t A = lut->a[q * 64 + i];
        const uint16_t B = lut->b[r * 64 + i];
        const int32_t cosA = fp16_to_q15(A & 0xffff), sinA = fp16_to_q15(A >> 16);
        const int32_t cosB = b8_to_q15(B & 0xff), sinB = b8_to_q15(B >> 8);
        c[k] = sat16(mul_q15(cosA, cosB) - mul_q15(sinA, sinB));
        s[k] = sat16(mul_q15(sinA, cosB) + mul_q15(cosA, sinB));
    }
    for (int row = 0; row < VROPE_ROWS; row++) {
        const int16_t *x = in + row * VROPE_EG;
        int16_t *y = out + row * VROPE_EG;
        for (int k = 0; k < 4; k++) {
            const int32_t x0 = x[2 * k], x1 = x[2 * k + 1];
            const int16_t y0 = (int16_t)sat16(mul_q15(x0, c[k]) - mul_q15(x1, s[k]));
            const int16_t y1 = (int16_t)sat16(mul_q15(x1, c[k]) + mul_q15(x0, s[k]));
            y[2 * k] = y0;
            y[2 * k + 1] = y1;
        }
    }
}

// ---- the instruction ------------------------------------------------------------------------------------
#if defined(VELA_EMULATE)
static vrope_lut_t emu_lut;
static int emu_ready;

int vrope_exec(const int16_t *in, int16_t *out, int m, int i0) {
    if (!emu_ready) {
        const char *b = getenv("VELA_EMU_ROPE_BASE");
        vrope_lut_build(&emu_lut, (b && *b) ? atof(b) : 500000.0);
        emu_ready = 1;
    }
    vrope_emulate(&emu_lut, in, out, m, i0);
    return VROPE_VL;
}
#else
// vsetvli e16 m8 (vl = 256 with VLEN 512), vle16 v8, vfrope.vx v16, v8, rs1, vse16 v16.
// vfrope.vx = OP-V (0x57), funct3 OPIVX (4), funct6 0x12 + vm=1 -> funct7 0x25 (rv_rope_test: .word 0x4A21C1D7
// = vfrope v3, v2, gp). The file is built for rv64gc, so V is enabled for this block only.
int vrope_exec(const int16_t *in, int16_t *out, int m, int i0) {
    const unsigned long rs1 = (unsigned long)(m & 0xffff) << 16 | (unsigned long)(2 * i0);
    unsigned long vl;
    __asm__ volatile(".option push\n"
                     ".option arch, +v\n"
                     "vsetvli %0, %3, e16, m8, ta, ma\n"
                     "vle16.v v8, (%1)\n"
                     ".insn r 0x57, 4, 0x25, x16, %4, x8\n"   // vfrope.vx v16, v8, rs1
                     "vse16.v v16, (%2)\n"
                     ".option pop\n"
                     : "=&r"(vl)
                     : "r"(in), "r"(out), "r"((unsigned long)VROPE_VL), "r"(rs1)
                     : "memory"
#if defined(__riscv_vector)
                       , "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15",
                       "v16", "v17", "v18", "v19", "v20", "v21", "v22", "v23"
#endif
    );
    return (int)vl;
}
#endif

// ---- probe: which ROM does the unit hold? ----------------------------------------------------------------
double vrope_detect(const double *bases, int nbases) {
    // (m, i0): i0 > 0 tells the bases apart; (1088, 32) and (640, 8) read ROM A entries of -1.0
    // (base 10000: q=17 i=33, base 500000: q=10 i=8)
    static const int probe[][2] = {{0, 0}, {65, 0}, {1234, 4}, {511, 16}, {1088, 32}, {640, 8}, {4095, 60}};
    enum { NP = sizeof(probe) / sizeof(probe[0]) };
    static int16_t in[VROPE_VL] __attribute__((aligned(64)));
    static int16_t hw[NP][VROPE_VL] __attribute__((aligned(64)));
    int16_t emu[VROPE_VL];
    uint32_t s = 12345;
    for (int n = 0; n < VROPE_VL; n++) {
        s = s * 1664525u + 1013904223u;
        in[n] = (int16_t)(s >> 16);
    }
    for (int p = 0; p < NP; p++)
        if (vrope_exec(in, hw[p], probe[p][0], probe[p][1]) != VROPE_VL) return 0.0;   // other VLEN

    vrope_lut_t *lut = (vrope_lut_t *)malloc(sizeof(*lut));
    double found = 0.0;
    for (int b = 0; b < nbases && found == 0.0; b++) {
        vrope_lut_build(lut, bases[b]);
        int same = 1;
        for (int p = 0; p < NP && same; p++) {
            vrope_emulate(lut, in, emu, probe[p][0], probe[p][1]);
            for (int row = 0; row < VROPE_ROWS && same; row++)
                same = !memcmp(&emu[row * VROPE_EG], &hw[p][row * VROPE_EG], 8 * sizeof(int16_t));
        }
        if (same) found = bases[b];
    }
    free(lut);
    return found;
}

// ---- float rows -------------------------------------------------------------------------------------------
// Q1.15 rows are laid out per pair chunk c (pairs 4c..4c+3 = one instruction): buf[c][row*16 + 0..7].
static int16_t qin[VROPE_DIM / 8][VROPE_VL] __attribute__((aligned(64)));
static int16_t qout[VROPE_DIM / 8][VROPE_VL] __attribute__((aligned(64)));

static inline int16_t to_q15(float v) {
    const long q = lrintf(v);
    return (int16_t)(q > 32767 ? 32767 : q < -32768 ? -32768 : q);
}

void vrope_rows(const float *src, size_t src_stride, float *dst, size_t dst_stride, int nrows, int m, int neox) {
    float inv[VROPE_ROWS];
    for (int r0 = 0; r0 < nrows; r0 += VROPE_ROWS) {
        const int nb = nrows - r0 < VROPE_ROWS ? nrows - r0 : VROPE_ROWS;
        for (int b = 0; b < nb; b++) {
            const float *x = (const float *)((const char *)src + (size_t)(r0 + b) * src_stride);
            // scale: largest pair norm -> 0.97 (the rotation keeps pair norms; leaves room for the
            // unit's cos/sin, whose norm can exceed 1 by the Q1.7 rounding of ROM B)
            float n2 = 0.0f;
            for (int j = 0; j < VROPE_PAIRS; j++) {
                const float a = neox ? x[j] : x[2 * j], c = neox ? x[j + VROPE_PAIRS] : x[2 * j + 1];
                const float t = a * a + c * c;
                if (t > n2) n2 = t;
            }
            const float scale = n2 > 0.0f ? 0.97f * 32768.0f / sqrtf(n2) : 0.0f;
            inv[b] = n2 > 0.0f ? 1.0f / scale : 0.0f;
            for (int j = 0; j < VROPE_PAIRS; j++) {
                const float a = neox ? x[j] : x[2 * j], c = neox ? x[j + VROPE_PAIRS] : x[2 * j + 1];
                int16_t *l = &qin[j / 4][b * VROPE_EG + 2 * (j % 4)];
                l[0] = to_q15(a * scale);
                l[1] = to_q15(c * scale);
            }
        }
        for (int c = 0; c < VROPE_DIM / 8; c++) vrope_exec(qin[c], qout[c], m, 4 * c);
        for (int b = 0; b < nb; b++) {
            float *y = (float *)((char *)dst + (size_t)(r0 + b) * dst_stride);
            for (int j = 0; j < VROPE_PAIRS; j++) {
                const int16_t *l = &qout[j / 4][b * VROPE_EG + 2 * (j % 4)];
                const float a = l[0] * inv[b], c = l[1] * inv[b];
                if (neox) { y[j] = a; y[j + VROPE_PAIRS] = c; }
                else      { y[2 * j] = a; y[2 * j + 1] = c; }
            }
        }
    }
}
