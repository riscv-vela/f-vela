// velaNPU (custom Gemmini) wrapper: int8 conv via tiled_conv_auto + hardware profiler readout.
// Linux userspace: the NPU uses virtual addresses through its own TLB, so all buffers must stay
// resident -> mlockall() in vnpu_init().
//
// vnpu_init(0) (ncnn_conv_test --profiler counter): the hardware profiler is turned off (it then
// records nothing) and the conv cycles come from the Gemmini perf counters only, measured exactly
// as in gemmini_stock_hal.c (fence, reset, call, fence, snapshot), so velaNPU and stock Gemmini are
// compared with the same method and without profiler memory traffic.
//
// -DVNPU_FP=1 (npu_<model>_fp.riscv): built on include/gemmini.h, which is pf_gemmini.h plus the
// runtime is_fp mode, and adds vnpu_matmul_fp16 (gemmini_fp16.h). Needs a support_fp bitstream.
#include "vela_npu_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#if VNPU_FP
#include "gemmini.h"
#include "gemmini_fp16.h"
#else
#include "pf_gemmini.h"
#endif

#include "gemmini_perf_counters.h" // same counter readout as gemmini_stock_hal.c

// Profiler entry, packed by hardware as Cat(type[2], start[31], end[31]), written sequentially
// (8 bytes each) from the registered address on every profiled mvin/compute/mvout.
#define PROF_TYPE(e) ((int)(((e) >> 62) & 0x3ULL))
#define PROF_START(e) ((uint32_t)(((e) >> 31) & 0x7FFFFFFFULL))
#define PROF_END(e) ((uint32_t)((e)&0x7FFFFFFFULL))

static volatile uint64_t* g_prof = NULL; // profile buffer
static size_t g_prof_entries = 0;        // capacity
static size_t g_prof_dirty = 0;          // entries [0, g_prof_dirty) may be non-zero
static int g_counter_only = 0;           // vnpu_init(0): no profiler, cycles from the perf counters

// a - b on the 31-bit wrapping counter, as a signed value
static int64_t diff31(uint32_t a, uint32_t b)
{
    int64_t d = (int64_t)((a - b) & 0x7FFFFFFFu);
    return d >= (1 << 30) ? d - (1LL << 31) : d;
}

static inline uint64_t rdcycle(void)
{
    uint64_t c;
    asm volatile("rdcycle %0" : "=r"(c));
    return c;
}

int vnpu_peak_macs_per_cycle(void)
{
    return DIM * DIM; // systolic array: DIM x DIM int8 MACs per cycle
}

// The profiler keeps its registered address after the process that set it exits: a later process
// that never registers its own buffer would have every NPU command's event written to that stale
// virtual address, which is unmapped in the new address space -> page fault on the NPU's side
// (Linux does not handle those) -> NPU stuck -> the core hangs in the next fence. So the profiler
// is turned off (address 0 = disabled in Profiler.scala) at init in counter-only mode and at exit.
static void prof_disable(void)
{
    gemmini_fence();       // the profiler only takes a new address while idle
    gemmini_profiler(0);   // profiler_vaddr_valid := false
    gemmini_fence();
}

static void prof_disable_at_exit(void)
{
    if (g_prof) prof_disable();
}

int vnpu_init(size_t buffer_mb)
{
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    {
        perror("vnpu_init: mlockall");
        return -1;
    }
    g_counter_only = buffer_mb == 0;
    if (g_counter_only)
    {
        prof_disable(); // may still point into a previous process's buffer
        gemmini_flush(0);
        gpc_init();
        return 0;
    }
    size_t bytes = buffer_mb << 20;
    void* p = mmap(NULL, bytes, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE, -1, 0);
    if (p == MAP_FAILED)
    {
        perror("vnpu_init: mmap profile buffer");
        return -1;
    }
    g_prof = (volatile uint64_t*)p; // zero-filled by the kernel
    g_prof_entries = bytes / sizeof(uint64_t);
    g_prof_dirty = 0;
    atexit(prof_disable_at_exit); // do not leave the profiler pointing at this buffer

    gemmini_flush(0);
    gpc_init();
    return 0;
}

// Before every NPU call: clear what the last call wrote (plus a margin for late writes) and
// re-register the buffer, so this call's events start at entry 0 and nothing stale follows them.
// The profiler only accepts a new address while idle; we are after the previous fence here.
static void prof_prepare(void)
{
    size_t n = g_prof_dirty + 4096;
    if (n > g_prof_entries) n = g_prof_entries;
    memset((void*)g_prof, 0, n * sizeof(uint64_t));
    g_prof_dirty = 0;
    gemmini_profiler(g_prof);
}

// After the fence: the profiler's DMA writes can land slightly later, so keep polling until the
// count has not grown for a while. Events outside this call's time window are dropped as stale.
static void prof_collect(vnpu_profile_t* prof, uint64_t window_cycles)
{
    size_t end = 0;
    uint64_t last_growth = rdcycle();
    while (rdcycle() - last_growth < 20000)
    {
        while (end < g_prof_entries && g_prof[end] != 0)
        {
            end++;
            last_growth = rdcycle();
        }
    }
    g_prof_dirty = end;
    prof->total_events = end;
    prof->overflow = end >= g_prof_entries;
    if (end == 0) return;

    // reference = the latest end time; this call's events must lie within window_cycles before it
    uint32_t ref = PROF_END(g_prof[0]);
    for (size_t i = 1; i < end; i++)
        if (diff31(PROF_END(g_prof[i]), ref) > 0) ref = PROF_END(g_prof[i]);
    const int64_t window = (int64_t)window_cycles + 1000;

    int64_t min_s = 0, max_e = 0;
    int first = 1;
    for (size_t i = 0; i < end; i++)
    {
        uint64_t e = g_prof[i];
        int64_t s = diff31(PROF_START(e), ref); // <= 0
        int64_t f = diff31(PROF_END(e), ref);   // <= 0
        if (s < -window || f < -window || f < s)
        {
            prof->stale_events++;
            continue;
        }
        if (first || s < min_s) min_s = s;
        if (first || f > max_e) max_e = f;
        first = 0;
        int t = PROF_TYPE(e);
        if (t >= 0 && t <= 2)
        {
            prof->busy_cycles[t] += (uint64_t)(f - s);
            prof->events[t]++;
        }
    }
    prof->span_cycles = first ? 0 : (uint64_t)(max_e - min_s);
}

// After gpc_stop: perf counters into *prof, then the conv cycles from the profiler, or in counter-only
// mode from the counters (ld|ex|st controller busy, as for stock Gemmini).
static void prof_finish(vnpu_profile_t* prof, const gpc_result_t* r)
{
    prof->counter_busy = r->busy;
    for (int t = 0; t < 3; t++) prof->counter_unit_busy[t] = r->unit[t];
    prof->tlb_miss_cycles = r->tlb_miss;
    if (g_counter_only)
    {
        for (int t = 0; t < 3; t++) prof->busy_cycles[t] = r->unit[t];
        prof->span_cycles = r->busy;
    }
    else
        prof_collect(prof, prof->call_cycles);
}

void vnpu_conv_int8(int in_h, int in_w, int in_c, int out_c, int out_h, int out_w,
                    int stride, int padding, int kernel,
                    const int8_t* input, const int8_t* weights, const int32_t* bias, int8_t* output,
                    int relu, float scale, vnpu_profile_t* prof)
{
    memset(prof, 0, sizeof(*prof));
    if (g_counter_only)
        gemmini_fence(); // nothing of a previous call still counting
    else
        prof_prepare();
    gpc_start();

    uint64_t c0 = rdcycle();
    tiled_conv_auto(
        1, in_h, in_w, in_c,
        out_c, out_h, out_w,
        stride, 1, 1, padding, kernel,
        false, false, false, false, false,
        (const elem_t*)input, (const elem_t*)weights, (const acc_t*)bias, (elem_t*)output,
        relu ? RELU : NO_ACTIVATION, (acc_scale_t)scale, 0, 0, 0,
        WS);
    gemmini_fence();
    uint64_t c1 = rdcycle();

    prof->call_cycles = c1 - c0;

    gpc_result_t r;
    gpc_stop(&r);
    prof_finish(prof, &r);
}

#if VNPU_FP
void vnpu_matmul_fp16(int M, int N, int K,
                      const uint16_t* A, int lda, const uint16_t* B, int ldb,
                      const float* D, int ldd, float* C, int ldc, vnpu_profile_t* prof)
{
    memset(prof, 0, sizeof(*prof));
    if (g_counter_only)
        gemmini_fence(); // nothing of a previous call still counting
    else
        prof_prepare();
    gpc_start();

    uint64_t c0 = rdcycle();
    tiled_matmul_auto_fp(M, N, K, (const fp16_t*)A, (const fp16_t*)B, (const fp32_t*)D, (fp32_t*)C,
                         lda, ldb, ldd /*0 = repeating bias*/, ldc, 0 /*transpose_B*/, 1 /*output_as_float*/, ACC_SCALE_IDENTITY);
    gemmini_fence();
    uint64_t c1 = rdcycle();

    prof->call_cycles = c1 - c0;

    gpc_result_t r;
    gpc_stop(&r);
    prof_finish(prof, &r);
}
#endif
