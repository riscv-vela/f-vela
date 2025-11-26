#include <stdio.h>
typedef struct {
    float setpoint;
    float currpoint;
    float Kp, Ki, Kd;
    float I_prev;
    float error_prev;
    float output;
} Actuator;
int start, end;
int main() {
    volatile Actuator actuator4[8] = {
        {1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f},
        {11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f},
        {21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f, 28.0f},
        {31.0f, 32.0f, 33.0f, 34.0f, 35.0f, 36.0f, 37.0f, 38.0f},
        {41.0f, 42.0f, 43.0f, 44.0f, 45.0f, 46.0f, 47.0f, 48.0f},
        {51.0f, 52.0f, 53.0f, 54.0f, 55.0f, 56.0f, 57.0f, 58.0f},
        {61.0f, 62.0f, 63.0f, 64.0f, 65.0f, 66.0f, 67.0f, 68.0f},
        {71.0f, 72.0f, 73.0f, 74.0f, 75.0f, 76.0f, 77.0f, 78.0f}
    };


    float dt = 0.1f;
    __asm__ volatile ("csrr %0, cycle\n\t" : "=r"(start));

    // unoptimized version
    for (int i = 0; i < 8; i++){
        float error = actuator4[i].setpoint - actuator4[i].currpoint;
        float I_curr = actuator4[i].I_prev + error * dt;
        float D_curr = (error - actuator4[i].error_prev) / dt;
        float output = actuator4[i].Kp * error + actuator4[i].Ki * I_curr + actuator4[i].Kd * D_curr;
        actuator4[i].I_prev = I_curr;
        actuator4[i].error_prev = error;
        actuator4[i].output = output;
    }



    // @@@@@@@@@@@@@@@@@@@ actuator1 @@@@@@@@@@@@@@@@@@
    // __asm__ volatile (
    //     // Load dt into f8
    //     "flw    f8, 0(%1)\n"

    //     // --- Actuator 1 (at act_ptr) ---
    //     "flw    f0,  0(%0)\n"  // setpoint
    //     "flw    f1,  4(%0)\n"  // currpoint
    //     "flw    f2, 8(%0)\n"  // Kp
    //     "flw    f3, 12(%0)\n"  // Ki
    //     "flw    f4, 16(%0)\n"  // Kd
    //     "flw    f5, 20(%0)\n"  // I_prev
    //     "flw    f6, 24(%0)\n"  // error_prev

    //     "fsub.s f7, f0, f1\n"      // error_curr
    //     "fmul.s f9, f7, f8\n"      // error_curr * dt
    //     "fadd.s f10, f5, f9\n"     // I_curr
    //     "fsub.s f11, f7, f6\n"     // deriv_num
    //     "fdiv.s f11, f11, f8\n"    // derivative

    //     "fmul.s f12, f2, f7\n"     // Kp*error
    //     "fmul.s f13, f3, f10\n"    // Ki*I_curr
    //     "fmul.s f14, f4, f11\n"    // Kd*derivative
    //     "fadd.s f15, f12, f13\n"
    //     "fadd.s f15, f15, f14\n"   // output

    //     "fsw    f10, 20(%0)\n" // store I_curr
    //     "fsw    f7,  24(%0)\n" // store error_curr
    //     "fsw    f15, 28(%0)\n" // store output
    //     :
    //     :"r" (actuator1), "r"(&dt)
    //     :
    //     // clobber all the FPU regs we used and memory
    //     "f0","f1","f2","f3","f4","f5","f6","f7",
    //     "f8","f9","f10","f11","f12","f13","f14","f15",
    //     "memory"
    // );
    __asm__ volatile ("csrr %0, cycle\n\t" : "=r"(end));
    printf("Cycles: %d\n", end - start);


    return 0;
}