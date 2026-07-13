#include <stdio.h>
// #include "xprintf.h"
// #include "uart.h"
#include <stdlib.h>

typedef struct {
    double setpoint;
    double currpoint;
    double Kp, Ki, Kd;
    double I_prev;
    double error_prev;
    double output;
} Actuator;

int main() {
    printf("\n--- Simulation Started! ---\n");
    fflush(stdout);
    // 1. mstatus 레지스터의 FS(FPU, 13~14번 비트)와 VS(Vector, 9~10번 비트)를 모두 켬
    // unsigned long mstatus_val;
    // __asm__ volatile (
    //     "csrr %0, mstatus\n\t"               // 현재 mstatus 값을 읽어옴
    //     "li t0, (3 << 13) | (3 << 9)\n\t"    // FS=3(Dirty), VS=3(Dirty) 마스크 생성
    //     "or %0, %0, t0\n\t"                  // 기존 값에 OR 연산으로 비트 켬
    //     "csrw mstatus, %0\n\t"               // 다시 mstatus에 덮어씀
    //     : "+r" (mstatus_val)
    //     : 
    //     : "t0"
    // );

    Actuator actuators[4] = {
        {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0},
        {11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0},
        {21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0},
        {31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0}
    };

    printf("\n---Custom PID Instruction Start ---\n");
    double dt = 0.1;
    int start, end;
    __asm__ volatile ("csrr %0, mcycle" : "=r"(start) :: "memory");
    __asm__ volatile (
        // sew=64bit, vl=8, vlen=512, lmul=1
        // 벡터 길이 설정 (2 actuator, 8개 필드)
        "vsetivli t0, 8, e64, m1, ta, ma\n\t"
        // dt를 fa0에 로드
        "fld fa0, (%1)\n\t"
        // actuator_array를 v1, v2에 로드
        "vle64.v v1, (%0)\n\t"
        "addi t1, %0, 64\n\t"
        "vle64.v v2, (t1)\n\t"
        "addi t2, t1, 64\n\t"
        "vle64.v v3, (t2)\n\t"
        "addi t3, t2, 64\n\t"
        "vle64.v v4, (t3)\n\t"
        :
        : "r"(actuators), "r"(&dt)
        : "t0", "t1", "t2", "t3",
        "fa0",
        "v1", "v2", "v3", "v4",
        "memory"
    );
    // printf("Custom PID Instruction Executed\n");
    __asm__ volatile (
        // vfpid64b.vf v2, v1, fa0\n\t
        // funct6 + vm(1) + vs2(1) + rs1(10) + funct3(101) + vd(xxxxx) + op(1010111)
        // vd = v4 (bits[11:7] = 00100)
        ".word 0b00001110000101010101001001010111\n\t"
        // ".word 0b00000110000101010101001001010111 \n\t"
    );
    // printf("Custom PID Instruction Completed\n");
    // printf("Storing results back to memory...\n");
    __asm__ volatile (
        // 결과를 메모리로 저장
        "vse64.v v1, (%0)\n\t"
        "vse64.v v2, (t1)\n\t"
        "vse64.v v3, (t2)\n\t"
        "vse64.v v4, (t3)\n\t"
        :
        : "r"(actuators), "r"(&dt)
        : "t0", "t1", "t2", "t3", "fa0", "v1", "v2", "v3", "v4", "memory"
    );
    __asm__ volatile ("csrr %0, mcycle" : "=r"(end) :: "memory");
    int a = end - start;

    printf("PID Computation Results:\n");

    for (int i = 0; i < 4; i++) {
        printf("Actuator[%d]:\n", i);
        printf("  Setpoint   : %.2f\n", actuators[i].setpoint);
        printf("  Currpoint  : %.2f\n", actuators[i].currpoint);
        printf("  Output     : %.2f\n", actuators[i].output);
        printf("  Error Prev : %.2f\n", actuators[i].error_prev);
        printf("  I Prev     : %.2f\n", actuators[i].I_prev);
        printf("---------------------------\n");
    }
    printf("Execution time: %d cycles\n", a);

    printf("--- Simulation Finished! ---\n");
    fflush(stdout);
    return 0;

    exit(0);
}
