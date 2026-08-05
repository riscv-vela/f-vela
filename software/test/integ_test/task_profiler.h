// See LICENSE for license details.
#ifndef INTEG_TEST_TASK_PROFILER_H
#define INTEG_TEST_TASK_PROFILER_H

// Runs the Gemmini profiler workload (ported from profiler_test/gemm_profiler_test.c):
// a tiled matmul instrumented with gemmini_profiler(), dumping "type, start, end"
// events consumable by profiler_test/plot/profile.py. Safe to call repeatedly.
void profiler_task_run(void);

#endif
