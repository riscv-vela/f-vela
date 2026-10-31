// Hardware check for the ncnn crash in conv3x3s1_winograd_pack_A_tile_rvv.
//
// The crashing loop (clang -O2) was:
//   loop: vsetvli zero, a6, e32, m1, ta, ma
//         vlse32.v v8, (s1), t3        # strided load into v8
//         vse32.v  v8, (s0)            # store to s0  <-- faulted, s0 had its upper 32 bits cleared
//         addiw a5, a5, -1 ; add s1, s1, s8 ; add s0, s0, ra ; bnez a5, loop
//
// This test runs the same instruction sequence with the same registers on large mmap'd
// buffers (addresses above 4 GiB, like ncnn's) and checks after every call that the scalar
// pointer registers advanced exactly as expected and the copied data is right.
//
// usage: rvv_ptr_test [iterations] [trap] [contig]
//   default 2000 iterations; "trap" adds rdtime after each vector store;
//   "contig" makes every vector load read one aligned 64 B block (no page crossing)
#define _GNU_SOURCE
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif

#define PACKN_MAX 64

// Same register assignment as the crashing code: s0=dst, s1=src, t3=stride, s8=src step,
// ra=dst step, a5=count, a6=vl.  Returns final s0 / s1 through out[].
static void copy_loop(float* dst, const float* src, long stride_bytes, long src_step_bytes,
                      long dst_step_bytes, long count, long vl, uintptr_t out[2])
{
    register uintptr_t r_s0 asm("s0") = (uintptr_t)dst;
    register uintptr_t r_s1 asm("s1") = (uintptr_t)src;
    register long r_t3 asm("t3") = stride_bytes;
    register long r_s8 asm("s8") = src_step_bytes;
    register long r_t4 asm("t4") = dst_step_bytes;
    register long r_a5 asm("a5") = count;
    register long r_a6 asm("a6") = vl;
    // ra is the return address, so it is saved in t0 around the loop.
    asm volatile(
        "mv t0, ra\n"
        "mv ra, t4\n"
        "1:\n"
        "vsetvli zero, a6, e32, m1, ta, ma\n"
        "vlse32.v v8, (s1), t3\n"
        "vse32.v v8, (s0)\n"
        "addiw a5, a5, -1\n"
        "add s1, s1, s8\n"
        "add s0, s0, ra\n"
        "bnez a5, 1b\n"
        "mv ra, t0\n"
        : "+r"(r_s0), "+r"(r_s1), "+r"(r_a5)
        : "r"(r_t3), "r"(r_s8), "r"(r_a6), "r"(r_t4)
        : "t0", "v8", "memory");
    out[0] = r_s0;
    out[1] = r_s1;
}

// Same loop, plus `rdtime` right after the scalar adds. Rocket has no time CSR, so on Linux
// each rdtime traps to M-mode (OpenSBI emulates it): this forces a trap shortly after every
// vector store. If traps taken while a vector op retires record a wrong epc, the adds get
// re-executed and the pointer errors jump.
// noinline + a global label so the bare-metal trap handler knows the exact rdtime PC.
__attribute__((noinline)) static void copy_loop_trap(float* dst, const float* src, long stride_bytes, long src_step_bytes,
                           long dst_step_bytes, long count, long vl, uintptr_t out[2])
{
    register uintptr_t r_s0 asm("s0") = (uintptr_t)dst;
    register uintptr_t r_s1 asm("s1") = (uintptr_t)src;
    register long r_t3 asm("t3") = stride_bytes;
    register long r_s8 asm("s8") = src_step_bytes;
    register long r_t4 asm("t4") = dst_step_bytes;
    register long r_a5 asm("a5") = count;
    register long r_a6 asm("a6") = vl;
    asm volatile(
        "mv t0, ra\n"
        "mv ra, t4\n"
        "1:\n"
        "vsetvli zero, a6, e32, m1, ta, ma\n"
        "vlse32.v v8, (s1), t3\n"
        "vse32.v v8, (s0)\n"
        "addiw a5, a5, -1\n"
        "add s1, s1, s8\n"
        "add s0, s0, ra\n"
        ".globl rvv_rdtime_pc\n"
        "rvv_rdtime_pc:\n"
        "rdtime t5\n"
        "bnez a5, 1b\n"
        "mv ra, t0\n"
        : "+r"(r_s0), "+r"(r_s1), "+r"(r_a5)
        : "r"(r_t3), "r"(r_s8), "r"(r_a6), "r"(r_t4)
        : "t0", "t5", "v8", "memory");
    out[0] = r_s0;
    out[1] = r_s1;
}

#ifdef BM_CHECK
// With -DBM_CHECK: copy_loop plus an in-loop consistency check, to catch the first iteration where
// the pointer registers diverge (a re-executed `add`). Every iteration s0 advances by dst_step and
// s1 by src_step, so (s0 - dst) == (s1 - src) << shift with shift = log2(dst_step / src_step).
// On the first mismatch it records rdcycle and the registers in g_check and leaves the loop.
// g_check: [0]=cycle [1]=s0 [2]=s1 [3]=a5 (remaining count) [4]=flag
volatile uint64_t g_check[5] __attribute__((aligned(8)));
__attribute__((noinline)) static void copy_loop_check(float* dst, const float* src, long stride_bytes, long src_step_bytes,
                                                      long dst_step_bytes, long count, long vl, long shift, uintptr_t out[2])
{
    // no function calls after the register variables below are set (a call would clobber them)
    register uintptr_t r_s0 asm("s0") = (uintptr_t)dst;
    register uintptr_t r_s1 asm("s1") = (uintptr_t)src;
    register long r_t3 asm("t3") = stride_bytes;
    register long r_s8 asm("s8") = src_step_bytes;
    register long r_t4 asm("t4") = dst_step_bytes;
    register long r_a5 asm("a5") = count;
    register long r_a6 asm("a6") = vl;
    register uintptr_t r_a2 asm("a2") = (uintptr_t)dst;
    register uintptr_t r_a3 asm("a3") = (uintptr_t)src;
    register volatile uint64_t* r_a4 asm("a4") = g_check;
    register long r_a1 asm("a1") = shift;
    asm volatile(
        "mv t0, ra\n"
        "mv ra, t4\n"
        "1:\n"
        "vsetvli zero, a6, e32, m1, ta, ma\n"
        "vlse32.v v8, (s1), t3\n"
        "vse32.v v8, (s0)\n"
        "addiw a5, a5, -1\n"
        "add s1, s1, s8\n"
        "add s0, s0, ra\n"
        "sub t5, s0, a2\n"
        "sub t6, s1, a3\n"
        "sll t6, t6, a1\n"
        "bne t5, t6, 9f\n"
        "bnez a5, 1b\n"
        "j 8f\n"
        "9:\n"
        "rdcycle t5\n"
        "ld t6, 32(a4)\n"
        "bnez t6, 8f\n"
        "sd t5, 0(a4)\n"
        "sd s0, 8(a4)\n"
        "sd s1, 16(a4)\n"
        "sd a5, 24(a4)\n"
        "li t6, 1\n"
        "sd t6, 32(a4)\n"
        "8:\n"
        "mv ra, t0\n"
        : "+r"(r_s0), "+r"(r_s1), "+r"(r_a5)
        : "r"(r_t3), "r"(r_s8), "r"(r_a6), "r"(r_t4), "r"(r_a2), "r"(r_a3), "r"(r_a4), "r"(r_a1)
        : "t0", "t5", "t6", "v8", "memory");
    out[0] = r_s0;
    out[1] = r_s1;
}
#endif

static long read_vlenb(void)
{
    long v;
    asm volatile("csrr %0, vlenb" : "=r"(v));
    return v;
}

#ifdef BAREMETAL
// Bare-metal (htif) build for Verilator/FireSim without Linux: no virtual memory, no interrupts.
// Smaller so it finishes quickly in RTL simulation.
#ifndef BM_ROWS
#define BM_ROWS 64
#endif
#define BM_KK 256
#ifndef BM_ITERS
#define BM_ITERS 4
#endif
static float g_src[BM_ROWS * BM_KK] __attribute__((aligned(64)));
static float g_dst[BM_ROWS * BM_KK] __attribute__((aligned(64)));

// With -DBM_TRAP: M-mode trap handler for the rdtime in copy_loop_trap (Rocket has no time CSR,
// so it raises illegal-instruction). The handler checks that mepc is exactly the rdtime PC,
// logs it otherwise, then skips the instruction (mepc += 4) and returns.
// g_trap[0]=traps, [1]=mepc mismatches, [2]=first bad mepc, [3]=its mcause, [5..6]=t1/t2 save,
// [7]=timer interrupts.
//
// With -DBM_TIMER: machine timer interrupts every BM_TIMER_PERIOD mtime ticks (CLINT, ~1000 core
// cycles per tick here), asynchronous to the program like Linux's timer tick. The handler only
// re-arms mtimecmp and returns to the interrupted instruction.
#define CLINT_MTIMECMP0 0x2004000UL
#define CLINT_MTIME 0x200bff8UL
#ifndef BM_TIMER_PERIOD
#define BM_TIMER_PERIOD 1
#endif
#define BM_STR2(x) #x
#define BM_STR(x) BM_STR2(x)
#if defined(BM_TRAP) || defined(BM_TIMER)
#define BM_HANDLER 1
volatile uint64_t g_trap[8] __attribute__((aligned(8)));
void bm_trap_entry(void);
#ifdef BM_TRAP
extern char rvv_rdtime_pc[];
#define BM_EXPECTED_PC "  la t2, rvv_rdtime_pc\n"
#else
#define BM_EXPECTED_PC "  li t2, 0\n" // no synchronous trap expected at all
#endif
asm(".section .text\n"
    ".align 4\n"
    ".globl bm_trap_entry\n"
    "bm_trap_entry:\n"
    "  csrw mscratch, t0\n"
    "  la t0, g_trap\n"
    "  sd t1, 40(t0)\n"
    "  sd t2, 48(t0)\n"
    "  csrr t1, mcause\n"
    "  bltz t1, 3f\n"       // interrupt
    "  ld t1, 0(t0)\n"      // traps++
    "  addi t1, t1, 1\n"
    "  sd t1, 0(t0)\n"
    "  csrr t1, mepc\n"
    BM_EXPECTED_PC
    "  beq t1, t2, 2f\n"
    "  ld t2, 8(t0)\n"      // mismatches++
    "  bnez t2, 1f\n"
    "  sd t1, 16(t0)\n"     // remember the first bad mepc and mcause
    "  csrr t1, mcause\n"
    "  sd t1, 24(t0)\n"
    "  csrr t1, mepc\n"
    "1:\n"
    "  addi t2, t2, 1\n"
    "  sd t2, 8(t0)\n"
    "2:\n"
    "  addi t1, t1, 4\n"    // skip the trapping instruction (all 4-byte here)
    "  csrw mepc, t1\n"
    "4:\n"
    "  ld t1, 40(t0)\n"
    "  ld t2, 48(t0)\n"
    "  csrr t0, mscratch\n"
    "  mret\n"
    "3:\n"                  // machine timer interrupt: count, re-arm, return to mepc unchanged
    "  ld t1, 56(t0)\n"
    "  addi t1, t1, 1\n"
    "  sd t1, 56(t0)\n"
    "  ld t1, 32(t0)\n"     // g_trap[4] != 0: stop request -> mtimecmp = max
    "  li t2, -1\n"
    "  bnez t1, 5f\n"
    "  li t1, " BM_STR(CLINT_MTIME) "\n"
    "  ld t1, 0(t1)\n"
    "  addi t2, t1, " BM_STR(BM_TIMER_PERIOD) "\n"
    "5:\n"
    "  li t1, " BM_STR(CLINT_MTIMECMP0) "\n"
    "  sd t2, 0(t1)\n"
    "  j 4b\n");
#endif // BM_TRAP || BM_TIMER

#ifdef BM_SV39
// With -DBM_SV39: run the test in S-mode under Sv39 with 4 KiB pages, like Linux user code.
// Identity map: the first 8 MiB of DRAM (code, data, 1 MiB+ of buffers, heap, stack) with
// 4 KiB pages, the rest of 0x80000000..0xbfffffff with 2 MiB pages, 0..1 GiB (MMIO) as a 1 GiB page.
// The DTLB has 32 entries and no L2 TLB, so the 256 buffer pages keep missing.
//
// -DBM_UMODE: run in U-mode instead (all leaf PTEs get U), like a Linux user program.
// -DBM_ALIAS: also map the first 8 MiB of DRAM a second time at VA = PA + 0x8000_0000
//   (0x1_0000_0000..) and run the vector loop through that alias, so VA != PA as under Linux.
//   With an identity map, a vector access that is translated with the wrong privilege (e.g. as
//   M-mode = no translation) would still hit the right memory and go unnoticed.
#define PTE_V 0x01
#ifdef BM_UMODE
#define PTE_RWXAD (0x02 | 0x04 | 0x08 | 0x10 | 0x40 | 0x80)
#else
#define PTE_RWXAD (0x02 | 0x04 | 0x08 | 0x40 | 0x80)
#endif
#define PTE_PPN(pa) ((((uint64_t)(pa)) >> 12) << 10)
#define BM_ALIAS_OFFSET 0x80000000UL
static uint64_t pt_root[512] __attribute__((aligned(4096)));
static uint64_t pt_l1[512] __attribute__((aligned(4096)));
static uint64_t pt_l1_alias[512] __attribute__((aligned(4096)));
static uint64_t pt_l0[4][512] __attribute__((aligned(4096)));

static void sv39_enter_smode(void)
{
    const uint64_t dram = 0x80000000UL;
    pt_root[0] = PTE_PPN(0) | PTE_V | PTE_RWXAD;
    pt_root[2] = PTE_PPN(pt_l1) | PTE_V;
#ifdef BM_ALIAS
    // VA 0x1_0000_0000 + off -> PA 0x8000_0000 + off (VPN2 = 4), sharing the 4 KiB tables
    pt_root[(dram + BM_ALIAS_OFFSET) >> 30] = PTE_PPN(pt_l1_alias) | PTE_V;
    for (int j = 0; j < 4; j++) pt_l1_alias[j] = PTE_PPN(pt_l0[j]) | PTE_V;
#endif
    for (int j = 0; j < 512; j++)
    {
        uint64_t pa = dram + (uint64_t)j * (2UL << 20);
        pt_l1[j] = j < 4 ? (PTE_PPN(pt_l0[j]) | PTE_V) : (PTE_PPN(pa) | PTE_V | PTE_RWXAD);
    }
    for (int j = 0; j < 4; j++)
        for (int k = 0; k < 512; k++)
            pt_l0[j][k] = PTE_PPN(dram + (uint64_t)j * (2UL << 20) + (uint64_t)k * 4096) | PTE_V | PTE_RWXAD;

    uint64_t satp = (8UL << 60) | ((uint64_t)pt_root >> 12);
    asm volatile(
        // PMP entry 0: whole address space RWX (NAPOT), otherwise S-mode cannot access memory
        "li t0, -1\n csrw pmpaddr0, t0\n"
        "li t0, 0x1f\n csrw pmpcfg0, t0\n"
        // S/U-mode may read cycle/instret; time stays unavailable so rdtime still traps to M-mode
        "li t0, 0x5\n csrw mcounteren, t0\n csrw scounteren, t0\n"
        "csrw satp, %0\n"
        "sfence.vma\n"
        // mret into S-mode (MPP = 01) or U-mode (MPP = 00) at label 1
        "li t0, 0x1800\n csrc mstatus, t0\n"
#ifndef BM_UMODE
        "li t0, 0x0800\n csrs mstatus, t0\n"
#endif
        "la t0, 1f\n csrw mepc, t0\n"
        "mret\n"
        "1:\n" ::"r"(satp)
        : "t0", "memory");
}
#endif // BM_SV39
#endif

int main(int argc, char** argv)
{
#ifdef BAREMETAL
    // Enable FP and vector state (mstatus.FS/VS = dirty); Linux does this for user programs.
    asm volatile("li t0, 0x6600\n csrs mstatus, t0" ::: "t0");
#ifdef BM_NAME
    printf("rvv_ptr_test: binary %s\n", BM_NAME);
#endif
    long iters = BM_ITERS;
    (void)argc;
    (void)argv;
#ifdef BM_HANDLER
    asm volatile("csrw mtvec, %0" ::"r"(bm_trap_entry));
#endif
#ifdef BM_TRAP
    int trap_mode = 1;
#else
    int trap_mode = 0;
#endif
#ifdef BM_TIMER
    // arm the machine timer; it fires in S/U-mode regardless of mstatus.MIE, in M-mode only with MIE
    *(volatile uint64_t*)CLINT_MTIMECMP0 = *(volatile uint64_t*)CLINT_MTIME + BM_TIMER_PERIOD;
    asm volatile("li t0, 0x80\n csrs mie, t0" ::: "t0"); // MTIE
#ifndef BM_SV39
    asm volatile("csrsi mstatus, 0x8"); // MIE: the test itself runs in M-mode
#endif
#endif
    int contig = 0;
#ifdef BM_SV39
    sv39_enter_smode();
#ifdef BM_UMODE
    printf("rvv_ptr_test: running in U-mode, Sv39, 4 KiB pages%s\n",
#else
    printf("rvv_ptr_test: running in S-mode, Sv39, 4 KiB pages%s\n",
#endif
#ifdef BM_ALIAS
           ", buffers via alias VA = PA + 0x80000000");
#else
           ", identity map");
#endif
#endif
#else
    long iters = argc > 1 ? atol(argv[1]) : 2000;
    int trap_mode = 0, contig = 0;
    for (int i = 2; i < argc; i++)
    {
        if (strcmp(argv[i], "trap") == 0) trap_mode = 1;
        else if (strcmp(argv[i], "contig") == 0) contig = 1;
        else
        {
            fprintf(stderr, "usage: %s [iterations] [trap] [contig]\n", argv[0]);
            return 2;
        }
    }
#endif
    long vlenb = read_vlenb();
    long packn = vlenb / 4; // fp32 lanes per register, 16 for VLEN=512
    if (packn > PACKN_MAX) packn = PACKN_MAX;

    // Shape similar to the ResNet18 layer that crashed: N = 256 rows apart, 256 kk steps.
#ifdef BAREMETAL
    const long batch = 1, max_kk = BM_KK, N = max_kk * batch, rows = BM_ROWS;
#else
    const long batch = 1, max_kk = 256, N = max_kk * batch, rows = 512;
#endif
    size_t src_bytes = (size_t)rows * N * sizeof(float);
    size_t dst_bytes = (size_t)rows * max_kk * sizeof(float);

#ifdef BAREMETAL
#ifdef BM_ALIAS
    float* src = (float*)((uintptr_t)g_src + BM_ALIAS_OFFSET);
    float* dst = (float*)((uintptr_t)g_dst + BM_ALIAS_OFFSET);
#else
    float* src = g_src;
    float* dst = g_dst;
#endif
#else
    float* src = mmap(NULL, src_bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    float* dst = mmap(NULL, dst_bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (src == MAP_FAILED || dst == MAP_FAILED)
    {
        perror("mmap");
        return 2;
    }
#endif
    (void)src_bytes;
    for (long i = 0; i < rows * N; i++) src[i] = (float)i * 0.5f - 7.0f;

    // Load pattern (in floats):
    //   default: lanes N apart (1 KiB), next kk +1  -> each vlse32 spans 16 KiB = crosses 4 pages
    //   contig : lanes 1 apart (4 B),   next kk +packn -> each vlse32 reads one aligned 64 B block,
    //            never crossing a page (same instructions, only the stride registers change)
    const long lane_stride = contig ? 1 : N;
    const long kk_step = contig ? packn : batch;

    printf("rvv_ptr_test: vlenb=%ld packn=%ld src=%p dst=%p iters=%ld mode=%s%s\n", vlenb, packn, (void*)src,
           (void*)dst, iters, trap_mode ? "trap (rdtime after each vector store)" : "plain",
           contig ? " + contig (no page crossing)" : " + strided (each load crosses 4 pages)");
    fflush(stdout);

    long bad_ptr = 0, bad_data = 0;
    for (long it = 0; it < iters; it++)
    {
        memset(dst, 0, dst_bytes);
        for (long ii = 0; ii + packn - 1 < rows; ii += packn)
        {
            const float* p0 = src + ii * N;
            float* pp = dst + ii * max_kk;
            uintptr_t out[2];
            // max read index = (rows-packn)*N + max_kk*packn - 1 < rows*N since max_kk == N
            if (trap_mode)
                copy_loop_trap(pp, p0, lane_stride * sizeof(float), kk_step * sizeof(float), packn * sizeof(float), max_kk, packn, out);
            else
#ifdef BM_CHECK
            {
                long shift = 0;
                while ((kk_step << shift) < packn) shift++; // log2(dst_step / src_step), both powers of 2
                copy_loop_check(pp, p0, lane_stride * sizeof(float), kk_step * sizeof(float), packn * sizeof(float), max_kk, packn,
                                shift, out);
            }
            if (g_check[4] == 1)
            {
                // first divergence: report once, with how far each pointer is from the iteration count
                long done = max_kk - (long)g_check[3];
                printf("  FIRST DIVERGENCE at cycle %lu (iter %ld ii %ld): after %ld of %ld steps, s0 is %+ld steps, s1 is %+ld steps\n",
                       (unsigned long)g_check[0], it, ii, done, max_kk,
                       (long)(((long)g_check[1] - (long)(uintptr_t)pp) / (long)(packn * sizeof(float))) - done,
                       (long)(((long)g_check[2] - (long)(uintptr_t)p0) / (long)(kk_step * sizeof(float))) - done);
                g_check[4] = 2;
            }
#else
                copy_loop(pp, p0, lane_stride * sizeof(float), kk_step * sizeof(float), packn * sizeof(float), max_kk, packn, out);
#endif

            uintptr_t want_s0 = (uintptr_t)(pp + max_kk * packn);
            uintptr_t want_s1 = (uintptr_t)(p0 + max_kk * kk_step);
            if (out[0] != want_s0 || out[1] != want_s1)
            {
#ifdef BAREMETAL
                // cycle stamp to pick a waveform window (the error happened shortly before)
                unsigned long cyc;
                asm volatile("rdcycle %0" : "=r"(cyc));
                if (bad_ptr < 10) printf("  [cycle %lu]", cyc);
#endif
                if (bad_ptr < 10)
                    printf("  iter %ld ii %ld: POINTER MISMATCH s0=%#lx (want %#lx) s1=%#lx (want %#lx)\n",
                           it, ii, (unsigned long)out[0], (unsigned long)want_s0, (unsigned long)out[1], (unsigned long)want_s1);
                bad_ptr++;
            }
        }
        // dst[ii*max_kk + kk*packn + l] must equal src[ii*N + kk*kk_step + l*lane_stride]
        for (long ii = 0; ii + packn - 1 < rows; ii += packn)
            for (long kk = 0; kk < max_kk; kk++)
                for (long l = 0; l < packn; l++)
                {
                    float got = dst[ii * max_kk + kk * packn + l];
                    float want = src[ii * N + kk * kk_step + l * lane_stride];
                    if (got != want)
                    {
                        if (bad_data < 10)
                        {
                            // raw bits: the bare-metal printf (newlib nano) has no %g
                            uint32_t gb, wb;
                            memcpy(&gb, &got, 4);
                            memcpy(&wb, &want, 4);
                            printf("  iter %ld ii %ld kk %ld lane %ld: DATA MISMATCH got 0x%08x want 0x%08x\n", it, ii, kk, l,
                                   (unsigned)gb, (unsigned)wb);
                        }
                        bad_data++;
                    }
                }
        if ((it + 1) % 500 == 0)
        {
            printf("  %ld/%ld iterations, pointer errors %ld, data errors %ld\n", it + 1, iters, bad_ptr, bad_data);
            fflush(stdout);
        }
    }

#if defined(BAREMETAL) && defined(BM_HANDLER)
#ifdef BM_TIMER
    // stop the timer before reporting (we may be in U-mode: ask the handler to disarm it)
    g_trap[4] = 1;
    for (uint64_t n = g_trap[7]; g_trap[7] == n;) {}
    printf("timer interrupts %lu (period %d mtime ticks)\n", (unsigned long)g_trap[7], BM_TIMER_PERIOD);
#endif
#ifdef BM_TRAP
    printf("traps %lu (expected %ld), mepc mismatches %lu", (unsigned long)g_trap[0],
           iters * (rows / packn) * max_kk, (unsigned long)g_trap[1]);
    if (g_trap[1])
        printf(", first bad mepc %#lx (rdtime at %p) mcause %lu", (unsigned long)g_trap[2], (void*)rvv_rdtime_pc,
               (unsigned long)g_trap[3]);
#else
    printf("unexpected synchronous traps %lu", (unsigned long)g_trap[1]);
    if (g_trap[1]) printf(", first at mepc %#lx mcause %lu", (unsigned long)g_trap[2], (unsigned long)g_trap[3]);
#endif
    printf("\n");
    if (g_trap[1]) bad_ptr++;
#endif
    printf("result: %s (pointer errors %ld, data errors %ld)\n", (bad_ptr || bad_data) ? "FAIL" : "PASS", bad_ptr, bad_data);
    return (bad_ptr || bad_data) ? 1 : 0;
}
