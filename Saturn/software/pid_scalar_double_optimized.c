#include <stdio.h>
typedef struct {
    double setpoint;
    double currpoint;
    double Kp, Ki, Kd;
    double I_prev;
    double error_prev;
    double output;
} Actuator;
int start, end;
int main() {
    volatile Actuator actuator4[8] = {
        {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0},
        {11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0},
        {21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0},
        {31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0},
        {41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0},
        {51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0},
        {61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0},
        {71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0}
    };


    double dt = 0.1;
    __asm__ volatile ("csrr %0, cycle\n\t" : "=r"(start));

    // unoptimized version
    for (int i = 0; i < 8; i++){
        double error = actuator4[i].setpoint - actuator4[i].currpoint;
        double I_curr = actuator4[i].I_prev + error * dt;
        double D_curr = (error - actuator4[i].error_prev) / dt;
        double output = actuator4[i].Kp * error + actuator4[i].Ki * I_curr + actuator4[i].Kd * D_curr;
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