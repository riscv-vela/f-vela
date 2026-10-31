// Hardware check: vector segment / strided accesses that cross a 4 KiB page boundary.
// ncnn's RVV winograd kernels (conv3x3s1_winograd_transpose_pack_B_tile_rvv) use vsseg8e32,
// vssseg8e32 (stride 64 B), vsseg4e32, vsseg2e32 and vlse32/vsse32 with e32, m1, vl = 16.
// In velaVPU, strided accesses and segment accesses that span two pages go through the iterative
// fault check (one element / segment half at a time), which plain unit-stride tests do not reach.
//
// Each case: base address = page boundary - d, for every d (step 4 B) so that the accesses land
// on both sides of the boundary; store v8..v15 (field f, element e = f*1000 + e) or load them
// back, and compare with the expected memory image. Guard words around the data are checked too.
// Linux and bare-metal (-DBAREMETAL, htif printf).
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define PAGE 4096
#define VL 16
#ifndef DSTEP
#define DSTEP 4 // step of the base offset d (a larger step gives a quicker sampled sweep)
#endif
static uint32_t buf[4 * PAGE / 4] __attribute__((aligned(PAGE)));
static uint32_t ld[8][VL];
static const uint32_t GUARD = 0xdeadbeefu;
static int fails = 0;

// only the words around the accessed range are written and checked (keeps the simulation short)
static long win_lo, win_hi; // word indices
static void set_window(long off, long span)
{
    win_lo = (off - 64) / 4;
    win_hi = (off + span + 64) / 4;
}
static void fill_guard(void)
{
    for (long i = win_lo; i < win_hi; i++) buf[i] = GUARD;
}

// v8..v15 = f*1000 + e (vl 16, e32, m1)
#define SET_V8_15                                                                                    \
    "li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n vid.v v8\n"                                      \
    "li t1, 1000\n vadd.vx v9, v8, t1\n li t1, 2000\n vadd.vx v10, v8, t1\n"      \
    "li t1, 3000\n vadd.vx v11, v8, t1\n li t1, 4000\n vadd.vx v12, v8, t1\n li t1, 5000\n"           \
    "vadd.vx v13, v8, t1\n li t1, 6000\n vadd.vx v14, v8, t1\n li t1, 7000\n vadd.vx v15, v8, t1\n"

enum { SEG8, SSEG8, SEG4, SEG2, SSE, LSEG8, LSSEG8, LSE, NKIND };
static const char* kname[NKIND] = {
    "vsseg8e32  (unit, 32 B seg)", "vssseg8e32 (stride 64 B)", "vsseg4e32  (unit, 16 B seg)",
    "vsseg2e32  (unit, 8 B seg)",  "vsse32     (stride 260 B)", "vlseg8e32  (unit, 32 B seg)",
    "vlsseg8e32 (stride 64 B)",    "vlse32     (stride 260 B)",
};
static const int knf[NKIND] = {8, 8, 4, 2, 1, 8, 8, 1};
static const long kstride[NKIND] = {32, 64, 16, 8, 260, 32, 64, 260};

// vstart is written right before the memory instruction (any other vector instruction resets it),
// as when the kernel resumes a vector access after a page fault in the middle of it
static long g_vs = 0;

#define ST_ASM(insn) asm volatile(SET_V8_15 "csrw vstart, %[vs]\n" insn "\n" ::[p] "r"(p), [s] "r"(s), [vs] "r"(g_vs) : "t0", "t1", "memory")
#define ZERO_V8_15 "li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n vmv.v.i v8, 0\n vmv.v.i v9, 0\n vmv.v.i v10, 0\n vmv.v.i v11, 0\n" \
                   "vmv.v.i v12, 0\n vmv.v.i v13, 0\n vmv.v.i v14, 0\n vmv.v.i v15, 0\n"
#define OUT_V8_15 "vse32.v v8, (%[ld])\n addi t1, %[ld], 64\n vse32.v v9, (t1)\n addi t1, t1, 64\n vse32.v v10, (t1)\n"   \
                  "addi t1, t1, 64\n vse32.v v11, (t1)\n addi t1, t1, 64\n vse32.v v12, (t1)\n addi t1, t1, 64\n"        \
                  "vse32.v v13, (t1)\n addi t1, t1, 64\n vse32.v v14, (t1)\n addi t1, t1, 64\n vse32.v v15, (t1)\n"
#define LD_ASM(insn) asm volatile(ZERO_V8_15 "csrw vstart, %[vs]\n" insn "\n" OUT_V8_15 ::[p] "r"(p), [s] "r"(s), [ld] "r"(ld), [vs] "r"(g_vs) : "t0", "t1", "memory")

static void run(int k, uint32_t* p)
{
    long s = kstride[k];
    switch (k)
    {
    case SEG8: ST_ASM("vsseg8e32.v v8, (%[p])"); break;
    case SSEG8: ST_ASM("vssseg8e32.v v8, (%[p]), %[s]"); break;
    case SEG4: ST_ASM("vsseg4e32.v v8, (%[p])"); break;
    case SEG2: ST_ASM("vsseg2e32.v v8, (%[p])"); break;
    case SSE: ST_ASM("vsse32.v v8, (%[p]), %[s]"); break;
    case LSEG8: LD_ASM("vlseg8e32.v v8, (%[p])"); break;
    case LSSEG8: LD_ASM("vlsseg8e32.v v8, (%[p]), %[s]"); break;
    case LSE: LD_ASM("vlse32.v v8, (%[p]), %[s]"); break;
    }
}

// returns number of wrong words
static int check(int k, long off)
{
    const int nf = knf[k];
    const long s = kstride[k];
    const int is_load = k >= LSEG8;
    int bad = 0;
    if (is_load)
    {
        for (int f = 0; f < nf; f++)
            for (int e = 0; e < VL; e++)
                if (ld[f][e] != (e < g_vs ? 0u : (uint32_t)(f * 1000 + e + 1))) bad++;
        return bad;
    }
    // expected image of the whole buffer
    for (long w = win_lo; w < win_hi; w++)
    {
        uint32_t want = GUARD;
        long rel = w * 4 - off; // byte offset from the base
        if (rel >= 0 && rel / s < VL && rel / s >= g_vs && (rel % s) / 4 < nf && (rel % s) % 4 == 0)
            want = (uint32_t)(((rel % s) / 4) * 1000 + rel / s);
        if (buf[w] != want) bad++;
    }
    return bad;
}

static void prepare_load(int k, long off)
{
    const int nf = knf[k];
    const long s = kstride[k];
    fill_guard();
    for (int e = 0; e < VL; e++)
        for (int f = 0; f < nf; f++) buf[(off + e * s) / 4 + f] = (uint32_t)(f * 1000 + e + 1);
    memset(ld, 0, sizeof(ld));
}

int main(void)
{
    // the page boundary at byte 2*PAGE of buf; base = boundary - d, d = 4 .. span (step 4)
    static const long vstarts[] = {0, 3, 15};
#ifdef VERBOSE
    setvbuf(stdout, NULL, _IONBF, 0); // a file log is fully buffered; keep the last case visible on a hang
#endif
    for (int vi = 0; vi < 3; vi++)
    for (int k = 0; k < NKIND; k++)
    {
#ifdef KIND
        if (k != KIND) continue; // -DKIND=k runs one access kind only (parallel simulations)
#endif
        g_vs = vstarts[vi];
        const long span = (VL - 1) * kstride[k] + knf[k] * 4;
        int nbad_cases = 0, ncases = 0;
        printf("vstart %2ld  %-30s ", g_vs, kname[k]);
        for (long d = 4; d <= span; d += DSTEP)
        {
            long off = 2 * PAGE - d;
            set_window(off, span);
            if (k >= LSEG8) prepare_load(k, off);
            else fill_guard();
#ifdef VERBOSE
            printf("\nrun vstart=%ld k=%d d=%ld base=%p ...", g_vs, k, d, (void*)((char*)buf + off));
#endif
            run(k, (uint32_t*)((char*)buf + off));
#ifdef VERBOSE
            printf(" done");
#endif
            int bad = check(k, off);
            fill_guard(); // leave the buffer clean for the next case
            ncases++;
            if (bad)
            {
                if (nbad_cases < 3) printf("\n   d=%ld (base = boundary-%ld): %d words wrong", d, d, bad);
                nbad_cases++;
            }
        }
        printf("%s%s (%d/%d offsets wrong)\n", nbad_cases ? "\n   -> " : "", nbad_cases ? "FAIL" : "ok", nbad_cases, ncases);
        fails += nbad_cases != 0;
    }
    printf("result: %s (%d kinds failed)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
