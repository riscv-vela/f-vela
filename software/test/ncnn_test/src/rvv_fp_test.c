// Hardware check for RVV floating-point ops that take a scalar FP operand (.vf, from an f register)
// versus vector-vector (.vv) ops. On the FPGA, ncnn conv layers gave only relu(bias): the
// weight*input part of vfmacc.vf was lost.
//
// Each test runs one instruction on 64 fp32 elements (LMUL=1..8) and compares with scalar C.
// usage: rvv_fp_test
#include <riscv_vector.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define N 64

static float a[N], b[N], c[N], out[N], exp_[N];
static int fails = 0;

static void report(const char* name)
{
    int bad = 0;
    for (int i = 0; i < N; i++)
    {
        uint32_t x, y;
        memcpy(&x, &out[i], 4);
        memcpy(&y, &exp_[i], 4);
        if (x == y) continue;
        if (bad < 4) printf("  %s[%d] = %.6g (0x%08x), expected %.6g (0x%08x)\n", name, i, out[i], x, exp_[i], y);
        bad++;
    }
    printf("%-28s %s (%d/%d wrong)\n", name, bad ? "FAIL" : "ok", bad, N);
    fails += bad != 0;
}

// volatile scalar so the compiler keeps it in an f register (not folded into a vector splat)
static volatile float vs = 1.5f;

int main(void)
{
    for (int i = 0; i < N; i++) { a[i] = 0.25f * (i + 1); b[i] = 2.0f - 0.125f * i; c[i] = 0.5f * i - 3.0f; }
    float s = vs;
    size_t vl;

    // ---- vector-vector ----
    vl = __riscv_vsetvl_e32m4(N);
    __riscv_vse32_v_f32m4(out, __riscv_vfmacc_vv_f32m4(__riscv_vle32_v_f32m4(c, vl), __riscv_vle32_v_f32m4(a, vl), __riscv_vle32_v_f32m4(b, vl), vl), vl);
    for (int i = 0; i < N; i++) exp_[i] = c[i] + a[i] * b[i];
    report("vfmacc.vv m4");

    __riscv_vse32_v_f32m4(out, __riscv_vfadd_vv_f32m4(__riscv_vle32_v_f32m4(a, vl), __riscv_vle32_v_f32m4(b, vl), vl), vl);
    for (int i = 0; i < N; i++) exp_[i] = a[i] + b[i];
    report("vfadd.vv m4");

    // ---- scalar FP operand (.vf) ----
#define VF_TESTS(LM, SET, LD, ST, MACC, MUL, ADD, MAX, MV)                                          \
    vl = SET(N);                                                                                    \
    for (size_t o = 0; o < N; o += vl) {                                                            \
        vl = SET(N - o);                                                                            \
        ST(out + o, MACC(LD(c + o, vl), s, LD(a + o, vl), vl), vl); }                               \
    for (int i = 0; i < N; i++) exp_[i] = c[i] + s * a[i];                                          \
    report("vfmacc.vf " LM);                                                                        \
    for (size_t o = 0; o < N; o += vl) { vl = SET(N - o); ST(out + o, MUL(LD(a + o, vl), s, vl), vl); } \
    for (int i = 0; i < N; i++) exp_[i] = a[i] * s;                                                 \
    report("vfmul.vf " LM);                                                                         \
    for (size_t o = 0; o < N; o += vl) { vl = SET(N - o); ST(out + o, ADD(LD(a + o, vl), s, vl), vl); } \
    for (int i = 0; i < N; i++) exp_[i] = a[i] + s;                                                 \
    report("vfadd.vf " LM);                                                                         \
    for (size_t o = 0; o < N; o += vl) { vl = SET(N - o); ST(out + o, MAX(LD(c + o, vl), 0.f, vl), vl); } \
    for (int i = 0; i < N; i++) exp_[i] = c[i] > 0.f ? c[i] : 0.f;                                  \
    report("vfmax.vf 0 (relu) " LM);                                                                \
    for (size_t o = 0; o < N; o += vl) { vl = SET(N - o); ST(out + o, MV(s, vl), vl); }             \
    for (int i = 0; i < N; i++) exp_[i] = s;                                                        \
    report("vfmv.v.f " LM);

    VF_TESTS("m1", __riscv_vsetvl_e32m1, __riscv_vle32_v_f32m1, __riscv_vse32_v_f32m1, __riscv_vfmacc_vf_f32m1,
             __riscv_vfmul_vf_f32m1, __riscv_vfadd_vf_f32m1, __riscv_vfmax_vf_f32m1, __riscv_vfmv_v_f_f32m1)
    VF_TESTS("m4", __riscv_vsetvl_e32m4, __riscv_vle32_v_f32m4, __riscv_vse32_v_f32m4, __riscv_vfmacc_vf_f32m4,
             __riscv_vfmul_vf_f32m4, __riscv_vfadd_vf_f32m4, __riscv_vfmax_vf_f32m4, __riscv_vfmv_v_f_f32m4)

    // ---- scalar FP operand loaded right before use (the ncnn conv pattern: flw then vfmacc.vf) ----
    {
        vl = __riscv_vsetvl_e32m1(16);
        vfloat32m1_t acc = __riscv_vfmv_v_f_f32m1(0.f, vl);
        for (int k = 0; k < 16; k++)
            acc = __riscv_vfmacc_vf_f32m1(acc, ((volatile float*)a)[k], __riscv_vle32_v_f32m1(b, vl), vl);
        __riscv_vse32_v_f32m1(out, acc, vl);
        float sa = 0;
        for (int k = 0; k < 16; k++) sa += a[k];
        for (int i = 0; i < N; i++) exp_[i] = 0;
        for (int i = 0; i < 16; i++) exp_[i] = 0;
        int bad = 0;
        for (int i = 0; i < 16; i++)
        {
            float e = 0;
            for (int k = 0; k < 16; k++) e += a[k] * b[i];
            float d = out[i] - e;
            if (d > 1e-4f || d < -1e-4f) { if (bad < 4) printf("  flw+vfmacc.vf[%d] = %g, expected %g\n", i, out[i], e); bad++; }
        }
        printf("%-28s %s (%d/16 wrong)\n", "flw+vfmacc.vf loop m1", bad ? "FAIL" : "ok", bad);
        fails += bad != 0;
        (void)sa;
    }

    printf("result: %s (%d tests failed)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
