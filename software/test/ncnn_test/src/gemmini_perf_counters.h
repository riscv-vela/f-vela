// Gemmini performance-counter readout, shared by vela_npu_hal.c (velaNPU) and gemmini_stock_hal.c
// (upstream Gemmini) so both report the same metric measured the same way.
// Include after gemmini.h / pf_gemmini.h (needs ROCC_INSTRUCTION, k_COUNTER, counter_* and the
// event codes of gemmini_counter.h, identical in both).
//
// counters 0..6 = MAIN_{LD,ST,EX,LD_ST,LD_EX,ST_EX,LD_ST_EX}_CYCLES: one counter per combination of
// busy load/store/execute controllers (Controller.scala), so in any cycle at most one of them counts.
//   sum            = cycles in which at least one of ld/ex/st was busy
//   ld | ex | st   = sum of the combinations that contain that unit
// counter 7 = DMA_TLB_MISS_CYCLE.
#ifndef GEMMINI_PERF_COUNTERS_H
#define GEMMINI_PERF_COUNTERS_H

#include <stdint.h>

typedef struct
{
    uint64_t busy;     // ld, ex or st busy
    uint64_t unit[3];  // 0 = ld, 1 = ex, 2 = st busy
    uint64_t tlb_miss; // DMA TLB miss cycles
} gpc_result_t;

static const struct
{
    int event;
    int ld, ex, st;
} gpc_main[7] = {
    {MAIN_LD_CYCLES, 1, 0, 0},    {MAIN_ST_CYCLES, 0, 0, 1},    {MAIN_EX_CYCLES, 0, 1, 0},
    {MAIN_LD_ST_CYCLES, 1, 0, 1}, {MAIN_LD_EX_CYCLES, 1, 1, 0}, {MAIN_ST_EX_CYCLES, 0, 1, 1},
    {MAIN_LD_ST_EX_CYCLES, 1, 1, 1},
};
#define GPC_TLB_MISS_COUNTER 7

// counter_read() in gemmini.h truncates to 32 bits; the counters are xLen (64) bits wide.
static inline uint64_t gpc_read(uint64_t index)
{
    uint64_t cfg = (index & 0x7) << 4;
    uint64_t zero = 0;
    uint64_t res;
    ROCC_INSTRUCTION(XCUSTOM_ACC, res, cfg, zero, k_COUNTER);
    return res;
}

// Once: the configuration survives counter_reset(); configuring also zeroes a counter.
static inline void gpc_init(void)
{
    for (int i = 0; i < 7; i++) counter_configure(i, gpc_main[i].event);
    counter_configure(GPC_TLB_MISS_COUNTER, DMA_TLB_MISS_CYCLE);
    counter_reset();
}

// Before the measured work (Gemmini must be idle: call after a fence).
static inline void gpc_start(void)
{
    counter_reset();
}

// After the measured work + fence: freeze all counters at the same instant and read them.
static inline void gpc_stop(gpc_result_t* r)
{
    r->busy = r->unit[0] = r->unit[1] = r->unit[2] = 0;
    counter_snapshot_take();
    for (int i = 0; i < 7; i++)
    {
        uint64_t v = gpc_read(i);
        r->busy += v;
        if (gpc_main[i].ld) r->unit[0] += v;
        if (gpc_main[i].ex) r->unit[1] += v;
        if (gpc_main[i].st) r->unit[2] += v;
    }
    r->tlb_miss = gpc_read(GPC_TLB_MISS_COUNTER);
    counter_snapshot_reset();
}

#endif // GEMMINI_PERF_COUNTERS_H
