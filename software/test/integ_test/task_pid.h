// See LICENSE for license details.
#ifndef INTEG_TEST_TASK_PID_H
#define INTEG_TEST_TASK_PID_H

// Runs the vector PID custom-instruction test (ported from
// rv_pid_test/vfpid_4f.c / vfpid_4f_linux.c). Safe to call repeatedly.
void pid_task_run(void);

#endif
