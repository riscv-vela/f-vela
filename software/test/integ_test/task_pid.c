// See LICENSE for license details.
//
// Vector PID custom-instruction workload, ported from rv_pid_test/vfpid_4f.c
// (bare-metal) and rv_pid_test/vfpid_4f_linux.c (Linux) into a single
// callable task (`pid_task_run`), merged with #ifdef BAREMETAL instead of
// being two separate files, mirroring how rv_gemmini_test/gemm.c already
// handles bare-metal vs. Linux with one dual-mode source.
//
// The only functional difference between the two modes is which CSR times
// the vector op: `mcycle` (M-mode only -- traps under Linux/U-mode) for
// bare-metal, vs. the unprivileged `cycle` shadow CSR under Linux.

#include <stdio.h>
#include <stdlib.h>
#include "task_pid.h"

typedef struct {
    float setpoint;
    float currpoint;
    float Kp, Ki, Kd;
    float I_prev;
    float error_prev;
    float output;
} Actuator;

void pid_task_run(void) {
    const int N = 4;
    Actuator actuator[4] = {
        {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f},
        {11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f},
        {21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f, 28.0f},
        {31.0f, 32.0f, 33.0f, 34.0f, 35.0f, 36.0f, 37.0f, 38.0f}
    };
    float dt = 0.1f;
    int start, end;
#ifdef BAREMETAL
    __asm__ volatile ("csrr %0, mcycle" : "=r"(start) :: "memory");
#else
    __asm__ volatile ("csrr %0, cycle" : "=r"(start) :: "memory");
#endif
    __asm__ volatile (
        "vsetivli t0, 8, e32, m1, ta, ma\n\t"
        "flw fa0, (%1)\n\t"
        "mv t0, %0\n\t"
        "vle32.v v1, (%0)\n\t"
        "addi t0, t0, 32\n\t"
        "vle32.v v2, (t0)\n\t"
        "addi t0, t0, 32\n\t"
        "vle32.v v3, (t0)\n\t"
        "addi t0, t0, 32\n\t"
        "vle32.v v4, (t0)\n\t"

        // funct6 + vm(1) + vs2(1) + rs1(10) + funct3(101) + vd(xxxxx) + op(1010111)
        // vd = v4 (bits[11:7] = 00100)
        ".word 0b00000110000101010101001001010111 \n\t"

        "vse32.v v1, (%0)\n\t"
        "mv t1, %0\n\t"
        "addi t1, t1, 32\n\t"
        "vse32.v v2, (t1)\n\t"
        "addi t1, t1, 32\n\t"
        "vse32.v v3, (t1)\n\t"
        "addi t1, t1, 32\n\t"
        "vse32.v v4, (t1)\n\t"
        :
        : "r"(actuator), "r"(&dt)
        : "v1", "v2", "v3", "v4", "fa0", "t0", "t1", "memory"
    );
#ifdef BAREMETAL
    __asm__ volatile ("csrr %0, mcycle" : "=r"(end) :: "memory");
#else
    __asm__ volatile ("csrr %0, cycle" : "=r"(end) :: "memory");
#endif
    int a = end - start;

    printf("PID Computation Results:\n");
    for (int i = 0; i < N; i++) {
        printf("Actuator[%d]:\n", i);
        printf("  Setpoint   : %.2f\n", actuator[i].setpoint);
        printf("  Currpoint  : %.2f\n", actuator[i].currpoint);
        printf("  Output     : %.2f\n", actuator[i].output);
        printf("  Error Prev : %.2f\n", actuator[i].error_prev);
        printf("  I Prev     : %.2f\n", actuator[i].I_prev);
        printf("---------------------------\n");
    }
    printf("Execution time: %d cycles\n", a);
}
