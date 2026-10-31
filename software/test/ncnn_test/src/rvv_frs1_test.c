// Hardware check: scalar FP operand (.vf) passed from the Rocket FPU to the vector unit.
// The FPU only reads the f register for a vector instruction when its copy of the vector decoder
// says read_frs1. In FVelaSoC that decoder gets vsew = 0 (vconfig is DontCare in FPU.scala) and the
// velaVPU decode table restricts FP ops to SEW >= 16, so the FPU does not read the register and the
// vector unit gets a stale value (whatever the FPU's FP->int path last produced).
//
// Each case: put a value in fa5, then vfmv.v.f v8, fa5 (e32, vl 16) and check every element.
// Linux and bare-metal (-DBAREMETAL, htif printf).
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static float out[16];
static volatile float mem_val = 2.5f;
static int fails = 0;

static void check(const char* name, float expect)
{
    int bad = 0;
    for (int i = 0; i < 16; i++)
        if (out[i] != expect) bad++;
    uint32_t got;
    memcpy(&got, &out[0], 4);
    printf("%-40s %s (expected %d.%03d, elem0 bits 0x%08x, %d/16 wrong)\n", name, bad ? "FAIL" : "ok",
           (int)expect, (int)((expect - (int)expect) * 1000), (unsigned)got, bad);
    fails += bad != 0;
}

int main(void)
{
    // 1. fa5 from fmv.w.x (1000.0f), right before the vector instruction
    asm volatile(
        "li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n"
        "li t1, 0x447a0000\n fmv.w.x fa5, t1\n"
        "vfmv.v.f v8, fa5\n vse32.v v8, (%0)\n" ::"r"(out) : "t0", "t1", "fa5", "v8", "memory");
    check("fmv.w.x 1000.0 -> vfmv.v.f", 1000.0f);

    // 2. same, but 20 independent instructions between fmv.w.x and vfmv.v.f (no hazard)
    asm volatile(
        "li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n"
        "li t1, 0x40400000\n fmv.w.x fa5, t1\n"
        ".rept 20\n addi t2, t2, 1\n .endr\n"
        "vfmv.v.f v8, fa5\n vse32.v v8, (%0)\n" ::"r"(out) : "t0", "t1", "t2", "fa5", "v8", "memory");
    check("fmv.w.x 3.0, 20 insns later", 3.0f);

    // 3. fa5 from flw
    asm volatile(
        "li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n"
        "flw fa5, 0(%1)\n"
        "vfmv.v.f v8, fa5\n vse32.v v8, (%0)\n" ::"r"(out), "r"(&mem_val) : "t0", "fa5", "v8", "memory");
    check("flw 2.5 -> vfmv.v.f", 2.5f);

    // 4. fa5 from fadd.s
    asm volatile(
        "li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n"
        "li t1, 0x3f800000\n fmv.w.x fa4, t1\n fadd.s fa5, fa4, fa4\n"
        "vfmv.v.f v8, fa5\n vse32.v v8, (%0)\n" ::"r"(out) : "t0", "t1", "fa4", "fa5", "v8", "memory");
    check("fadd.s 1+1 -> vfmv.v.f", 2.0f);

    // 5. stale value check: fsw of 7.0 (FP->int path) just before, then fmv.w.x 5.0 -> vfmv.v.f
    {
        static volatile float sink;
        asm volatile(
            "li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n"
            "li t1, 0x40e00000\n fmv.w.x fa4, t1\n fsw fa4, 0(%1)\n"
            "li t1, 0x40a00000\n fmv.w.x fa5, t1\n"
            "vfmv.v.f v8, fa5\n vse32.v v8, (%0)\n" ::"r"(out), "r"(&sink) : "t0", "t1", "fa4", "fa5", "v8", "memory");
        check("fsw 7.0, then fmv.w.x 5.0 -> vfmv.v.f", 5.0f);
    }

    // 6. vfadd.vf with e64 (vsew 3): fmv.d.x 1.0
    {
        static double outd[8];
        asm volatile(
            "li t0, 8\n vsetvli zero, t0, e64, m1, ta, ma\n"
            "li t1, 0x3ff0000000000000\n fmv.d.x fa5, t1\n"
            "vfmv.v.f v8, fa5\n vse64.v v8, (%0)\n" ::"r"(outd) : "t0", "t1", "fa5", "v8", "memory");
        int bad = 0;
        for (int i = 0; i < 8; i++) bad += outd[i] != 1.0;
        printf("%-40s %s (%d/8 wrong)\n", "e64 fmv.d.x 1.0 -> vfmv.v.f", bad ? "FAIL" : "ok", bad);
        fails += bad != 0;
    }

    // 7. exact sequence that failed on the FPGA (rvv_pf_test.linux init loop, clang -O2):
    //    vid / vfcvt.f.x.v / vfadd.vf (fa5 = 1000.0 from fmv.w.x) / vs2r.v, e32 m2 vlmax, 4 iterations
    {
        static float buf[256] __attribute__((aligned(64)));
        memset(buf, 0, sizeof(buf));
        asm volatile(
            "vsetvli a3, zero, e32, m2, ta, ma\n"
            "vid.v v8\n"
            "lui a4, 0x447a0\n fmv.w.x fa5, a4\n"
            "mv a5, %0\n li a6, 4\n"
            "1: vfcvt.f.x.v v10, v8\n vfadd.vf v10, v10, fa5\n vs2r.v v10, (a5)\n"
            "vadd.vx v8, v8, a3\n slli t0, a3, 2\n add a5, a5, t0\n addi a6, a6, -1\n bnez a6, 1b\n"
            ::"r"(buf) : "a3", "a4", "a5", "a6", "t0", "fa5", "v8", "v9", "v10", "v11", "memory");
        unsigned long vlmax;
        asm volatile("vsetvli %0, zero, e32, m2, ta, ma" : "=r"(vlmax));
        int bad = 0, n = (int)(4 * vlmax);
        for (int i = 0; i < n && i < 256; i++)
            if (buf[i] != 1000.0f + i) { if (bad < 4) printf("  buf[%d] = %d.%03d\n", i, (int)buf[i], (int)((buf[i] - (int)buf[i]) * 1000)); bad++; }
        printf("%-40s %s (%d/%d wrong)\n", "vid/vfcvt/vfadd.vf/vs2r loop (pf_test)", bad ? "FAIL" : "ok", bad, n);
        fails += bad != 0;
    }

    // 8. vfadd.vf / vfmacc.vf / vfmul.vf, fa5 from fmv.w.x, e32 m1 and m4
    {
        static float src[64], dst[64];
        for (int i = 0; i < 64; i++) src[i] = (float)i;
        const char* names[3] = {"vfadd.vf m1 (fmv.w.x 1000)", "vfmul.vf m4 (fmv.w.x 2)", "vfmacc.vf m4 (fmv.w.x 2)"};
        for (int k = 0; k < 3; k++)
        {
            memset(dst, 0, sizeof(dst));
            if (k == 0)
                asm volatile("li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n li t1, 0x447a0000\n fmv.w.x fa5, t1\n"
                             "vle32.v v8, (%1)\n vfadd.vf v8, v8, fa5\n vse32.v v8, (%0)\n" ::"r"(dst), "r"(src)
                             : "t0", "t1", "fa5", "v8", "memory");
            else if (k == 1)
                asm volatile("li t0, 64\n vsetvli zero, t0, e32, m4, ta, ma\n li t1, 0x40000000\n fmv.w.x fa5, t1\n"
                             "vle32.v v8, (%1)\n vfmul.vf v8, v8, fa5\n vse32.v v8, (%0)\n" ::"r"(dst), "r"(src)
                             : "t0", "t1", "fa5", "v8", "v9", "v10", "v11", "memory");
            else
                asm volatile("li t0, 64\n vsetvli zero, t0, e32, m4, ta, ma\n li t1, 0x40000000\n fmv.w.x fa5, t1\n"
                             "vle32.v v8, (%1)\n vmv.v.i v12, 0\n vfmacc.vf v12, fa5, v8\n vse32.v v12, (%0)\n" ::"r"(dst), "r"(src)
                             : "t0", "t1", "fa5", "v8", "v9", "v10", "v11", "v12", "v13", "v14", "v15", "memory");
            int n = k == 0 ? 16 : 64, bad = 0;
            for (int i = 0; i < n; i++)
            {
                float e = k == 0 ? 1000.0f + i : 2.0f * i;
                if (dst[i] != e) bad++;
            }
            printf("%-40s %s (%d/%d wrong)\n", names[k], bad ? "FAIL" : "ok", bad, n);
            fails += bad != 0;
        }
    }

    printf("result: %s (%d cases failed)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
