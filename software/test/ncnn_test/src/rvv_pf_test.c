// Hardware check: RVV loads/stores that take a page fault (first touch of fresh mmap'd memory).
// Linux maps a page on the first access, so a vector store to a new page traps (store page fault),
// the kernel maps the page and returns to the same instruction with vstart = faulting element.
// ncnn writes its transformed weights into fresh buffers with vector stores; on the FPGA those
// weights read back as all zero.
//
// Each case writes a known pattern with one vector instruction into untouched pages, then checks
// with scalar loads.
// usage: rvv_pf_test
#define _GNU_SOURCE
#include <riscv_vector.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#define PG 4096
static int fails = 0;

static float* fresh(size_t pages)
{
    void* p = mmap(0, pages * PG, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) { perror("mmap"); return 0; }
    return (float*)p;
}

static void check(const char* name, const float* p, size_t n, size_t stride_elems, float base)
{
    int bad = 0;
    for (size_t i = 0; i < n; i++)
    {
        float e = base + (float)i;
        float v = p[i * stride_elems];
        if (v != e) { if (bad < 4) printf("  %s[%zu] = %g, expected %g\n", name, i, v, e); bad++; }
    }
    printf("%-44s %s (%d/%zu wrong)\n", name, bad ? "FAIL" : "ok", bad, n);
    fails += bad != 0;
}

int main(void)
{
    float src[512];
    for (int i = 0; i < 512; i++) src[i] = 1000.f + i;

    // 1. unit-stride store, m8 (vl 128 = 512 B), at the start of an untouched page: faults at element 0
    {
        float* d = fresh(2);
        size_t vl = __riscv_vsetvl_e32m8(128);
        __riscv_vse32_v_f32m8(d, __riscv_vle32_v_f32m8(src, vl), vl);
        check("vse32 m8, fresh page (fault at elem 0)", d, vl, 1, 1000.f);
    }
    // 2. unit-stride store crossing from a touched page into an untouched one: faults mid-instruction
    {
        float* d = fresh(2);
        d[0] = 1.f; // touch page 0 only
        float* p = d + PG / 4 - 64; // 64 elements in page 0, 64 in page 1
        size_t vl = __riscv_vsetvl_e32m8(128);
        __riscv_vse32_v_f32m8(p, __riscv_vle32_v_f32m8(src, vl), vl);
        check("vse32 m8, crosses into fresh page (elem 64)", p, vl, 1, 1000.f);
    }
    // 3. same with m1 (vl 16), crossing at element 8
    {
        float* d = fresh(2);
        d[0] = 1.f;
        float* p = d + PG / 4 - 8;
        size_t vl = __riscv_vsetvl_e32m1(16);
        __riscv_vse32_v_f32m1(p, __riscv_vle32_v_f32m1(src, vl), vl);
        check("vse32 m1, crosses into fresh page (elem 8)", p, vl, 1, 1000.f);
    }
    // 4. strided store, one element per page, 16 untouched pages
    {
        float* d = fresh(16);
        size_t vl = __riscv_vsetvl_e32m1(16);
        __riscv_vsse32_v_f32m1(d, PG, __riscv_vle32_v_f32m1(src, vl), vl);
        check("vsse32 m1, 16 fresh pages", d, vl, PG / 4, 1000.f);
    }
    // 5. many stores into a large fresh buffer (like ncnn weight transform), m4
    {
        size_t n = 64 * PG / 4; // 64 pages
        float* d = fresh(64);
        for (size_t o = 0; o < n;)
        {
            size_t vl = __riscv_vsetvl_e32m4(n - o);
            vfloat32m4_t v = __riscv_vfadd_vf_f32m4(__riscv_vfcvt_f_xu_v_f32m4(__riscv_vid_v_u32m4(vl), vl), 1000.f + (float)o, vl);
            __riscv_vse32_v_f32m4(d + o, v, vl);
            o += vl;
        }
        check("vse32 m4 loop, 64 fresh pages", d, n, 1, 1000.f);
    }
    // 6. load from a fresh page (maps the zero page), then store into it
    {
        float* d = fresh(1);
        size_t vl = __riscv_vsetvl_e32m8(128);
        vfloat32m8_t z = __riscv_vle32_v_f32m8(d, vl);
        __riscv_vse32_v_f32m8(d, __riscv_vfadd_vv_f32m8(z, __riscv_vle32_v_f32m8(src, vl), vl), vl);
        check("vle32 then vse32 m8, fresh page", d, vl, 1, 1000.f);
    }

    printf("result: %s (%d cases failed)\n", fails ? "FAIL" : "PASS", fails);
    return fails ? 1 : 0;
}
