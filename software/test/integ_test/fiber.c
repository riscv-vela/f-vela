// See LICENSE for license details.
//
// Scheduler bookkeeping around fiber_switch() (fiber_asm.S). See fiber.h for why
// this exists. There is a single, fixed scheduler "instance" (module-level
// globals) because integ_multi.c only ever runs one round of fibers at a
// time -- no need for a re-entrant/allocatable scheduler object here.

#include "fiber.h"

static fiber_t **g_fibers;
static int g_nfibers;
static int g_cur;          // index of the fiber currently running, -1 in the boot context
static fiber_regs_t g_main; // boot ("caller of fiber_run_all") context
static fiber_t *g_starting; // fiber about to run for the very first time

// Landing pad for a fiber's very first resume: fiber_switch() "returns" here
// because fiber_init() points regs.ra at it. Reads which fiber is starting
// from g_starting (set by the scheduler immediately before every switch;
// harmless to re-set on a plain resume, since this trampoline never runs
// twice for the same fiber).
static void fiber_trampoline(void) {
    fiber_t *f = g_starting;
    f->entry();
    f->done = 1;
    fiber_yield();
    for (;;) { /* unreachable: fiber_yield() never resumes a finished fiber */ }
}

void fiber_init(fiber_t *f, void *stack_top, fiber_entry_t entry) {
    f->entry = entry;
    f->done = 0;
    f->regs.ra = (uint64_t)fiber_trampoline;
    f->regs.sp = (uint64_t)stack_top;
    for (int i = 0; i < 12; i++) {
        f->regs.s[i] = 0;
    }
}

// Next not-done fiber after index `from`, round-robin; -1 if none.
static int next_runnable(int from) {
    for (int i = 1; i <= g_nfibers; i++) {
        int cand = (from + i) % g_nfibers;
        if (!g_fibers[cand]->done) {
            return cand;
        }
    }
    return -1;
}

void fiber_run_all(fiber_t *fibers[], int n) {
    g_fibers = fibers;
    g_nfibers = n;
    g_cur = -1;

    int first = -1;
    for (int i = 0; i < n; i++) {
        if (!fibers[i]->done) {
            first = i;
            break;
        }
    }
    if (first < 0) {
        return; // nothing to run
    }

    g_cur = first;
    g_starting = fibers[first];
    fiber_switch(&g_main, &fibers[first]->regs);
    // Execution resumes here once every fiber has finished (fiber_yield()
    // switches back to g_main when next_runnable() has nothing left).
}

void fiber_yield(void) {
    int prev = g_cur;
    int next = next_runnable(prev);

    if (next < 0) {
        g_cur = -1;
        fiber_switch(&g_fibers[prev]->regs, &g_main);
        return;
    }

    if (next == prev) {
        return; // only this fiber is still runnable; nothing to switch to
    }

    g_cur = next;
    g_starting = g_fibers[next];
    fiber_switch(&g_fibers[prev]->regs, &g_fibers[next]->regs);
}
