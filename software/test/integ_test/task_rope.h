// See LICENSE for license details.
#ifndef INTEG_TEST_TASK_ROPE_H
#define INTEG_TEST_TASK_ROPE_H

// Runs the vector RoPE custom-instruction test (ported from
// rv_rope_test/vfrope_test2.c / vfrope_test2_linux.c, the HW-validated
// variant per rv_rope_test/README.md). Safe to call repeatedly.
void rope_task_run(void);

#endif
