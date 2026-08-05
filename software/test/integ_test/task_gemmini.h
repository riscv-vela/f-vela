// See LICENSE for license details.
#ifndef INTEG_TEST_TASK_GEMMINI_H
#define INTEG_TEST_TASK_GEMMINI_H

// Runs one Gemmini tiled-matmul pass (ported from rv_gemmini_test/gemm.c) and
// prints cycle count / utilization / operand dumps. Safe to call repeatedly.
void gemmini_task_run(void);

#endif
