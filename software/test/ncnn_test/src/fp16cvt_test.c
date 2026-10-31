// Hardware check for fp16 -> fp32 conversion (ncnn loads fp16 weights with this, see
// software/ncnn/src/layer/riscv/cast_riscv_zfh.cpp: vle16 e16m4 -> vfwcvt.f.f.v -> vse32 e32m8).
//
// Converts every fp16 bit pattern (65536) with
//   - a software reference (integer bit manipulation, no FP instructions)
//   - scalar fcvt.s.h (Zfh)
//   - vfwcvt.f.f.v at LMUL mf2->m1, m1->m2, m2->m4, m4->m8 (Zvfh)
// and compares the fp32 bit patterns (NaNs: any NaN accepted).
//
// usage: fp16cvt_test
#include <riscv_vector.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define N 65536

static uint16_t src[N] __attribute__((aligned(64)));
static uint32_t ref[N] __attribute__((aligned(64)));
static uint32_t out[N] __attribute__((aligned(64)));

static uint32_t f16_to_f32_bits(uint16_t h)
{
    uint32_t sign = (uint32_t)(h & 0x8000) << 16;
    uint32_t exp = (h >> 10) & 0x1f;
    uint32_t man = h & 0x3ff;
    if (exp == 0)
    {
        if (man == 0) return sign;
        // subnormal: normalize
        int e = -1;
        do { man <<= 1; e++; } while (!(man & 0x400));
        man &= 0x3ff;
        return sign | ((uint32_t)(127 - 15 - e) << 23) | (man << 13);
    }
    if (exp == 31) return sign | 0x7f800000 | (man << 13) | (man ? 0x00400000 : 0);
    return sign | ((exp + 127 - 15) << 23) | (man << 13);
}

static int is_nan32(uint32_t b) { return (b & 0x7f800000) == 0x7f800000 && (b & 0x7fffff); }

static int check(const char* name)
{
    int bad = 0;
    for (int i = 0; i < N; i++)
    {
        if (out[i] == ref[i] || (is_nan32(out[i]) && is_nan32(ref[i]))) continue;
        if (bad < 8) printf("  %s: fp16 0x%04x -> 0x%08x, expected 0x%08x\n", name, src[i], out[i], ref[i]);
        bad++;
    }
    printf("%-22s mismatches %d / %d\n", name, bad, N);
    return bad;
}

#define VEC_TEST(NAME, SETVL, LD, CVT, ST)                     \
    do {                                                       \
        memset(out, 0xa5, sizeof(out));                        \
        const _Float16* p = (const _Float16*)src;              \
        float* o = (float*)out;                                \
        size_t n = N;                                          \
        while (n > 0)                                          \
        {                                                      \
            size_t vl = SETVL(n);                              \
            ST(o, CVT(LD(p, vl), vl), vl);                     \
            p += vl; o += vl; n -= vl;                         \
        }                                                      \
        fails += check(NAME) != 0;                             \
    } while (0)

int main(void)
{
    for (int i = 0; i < N; i++) { src[i] = (uint16_t)i; ref[i] = f16_to_f32_bits((uint16_t)i); }
    int fails = 0;

    memset(out, 0xa5, sizeof(out));
    for (int i = 0; i < N; i++)
    {
        float f;
        uint32_t hb = src[i];
        asm volatile("fmv.h.x ft0, %1\n fcvt.s.h %0, ft0" : "=f"(f) : "r"(hb) : "ft0");
        memcpy(&out[i], &f, 4);
    }
    fails += check("scalar fcvt.s.h") != 0;

    VEC_TEST("vfwcvt e16mf2->e32m1", __riscv_vsetvl_e16mf2, __riscv_vle16_v_f16mf2, __riscv_vfwcvt_f_f_v_f32m1, __riscv_vse32_v_f32m1);
    VEC_TEST("vfwcvt e16m1->e32m2", __riscv_vsetvl_e16m1, __riscv_vle16_v_f16m1, __riscv_vfwcvt_f_f_v_f32m2, __riscv_vse32_v_f32m2);
    VEC_TEST("vfwcvt e16m2->e32m4", __riscv_vsetvl_e16m2, __riscv_vle16_v_f16m2, __riscv_vfwcvt_f_f_v_f32m4, __riscv_vse32_v_f32m4);
    VEC_TEST("vfwcvt e16m4->e32m8", __riscv_vsetvl_e16m4, __riscv_vle16_v_f16m4, __riscv_vfwcvt_f_f_v_f32m8, __riscv_vse32_v_f32m8);

    printf("result: %s (%d of 5 conversions wrong)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
