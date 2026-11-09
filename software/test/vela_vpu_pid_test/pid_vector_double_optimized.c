#include <stdio.h>
#include <riscv_vector.h>

typedef struct {
    double setpoint;
    double currpoint;
    double Kp, Ki, Kd;
    double I_prev;
    double error_prev;
    double output;
} Actuator;

int main(void) {
    Actuator actuators[8] = {
        {1.0,  2.0,  3.0,  4.0,  5.0,  6.0,  7.0,  8.0},
        {11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0},
        {21.0, 22.0, 23.0, 24.0, 25.0, 26.0, 27.0, 28.0},
        {31.0, 32.0, 33.0, 34.0, 35.0, 36.0, 37.0, 38.0},
        {41.0, 42.0, 43.0, 44.0, 45.0, 46.0, 47.0, 48.0},
        {51.0, 52.0, 53.0, 54.0, 55.0, 56.0, 57.0, 58.0},
        {61.0, 62.0, 63.0, 64.0, 65.0, 66.0, 67.0, 68.0},
        {71.0, 72.0, 73.0, 74.0, 75.0, 76.0, 77.0, 78.0}
    };

    const int N = 8;
    double dt = 0.1;
    double inv_dt = 1.0 / dt;

    // SoA 배열 준비 (측정 바깥에서 해도 됨)
    double setpoint[8], currpoint[8];
    double kp[8], ki[8], kd[8];
    double I_prev[8], err_prev[8], output[8];

    for (int i = 0; i < N; ++i) {
        setpoint[i]  = actuators[i].setpoint;
        currpoint[i] = actuators[i].currpoint;
        kp[i]        = actuators[i].Kp;
        ki[i]        = actuators[i].Ki;
        kd[i]        = actuators[i].Kd;
        I_prev[i]    = actuators[i].I_prev;
        err_prev[i]  = actuators[i].error_prev;
        output[i]    = actuators[i].output;
    }

    unsigned long start, end;

    __asm__ volatile ("csrr %0, cycle" : "=r"(start) :: "memory");

    // ===== RVV PID 연산 (8개 액추에이터 한 번에) =====
    size_t vl = __riscv_vsetvl_e64m1(N);   // 여기서는 vl == 8

    vfloat64m1_t v_set     = __riscv_vle64_v_f64m1(setpoint,  vl);
    vfloat64m1_t v_curr    = __riscv_vle64_v_f64m1(currpoint, vl);
    vfloat64m1_t v_kp      = __riscv_vle64_v_f64m1(kp,        vl);
    vfloat64m1_t v_ki      = __riscv_vle64_v_f64m1(ki,        vl);
    vfloat64m1_t v_kd      = __riscv_vle64_v_f64m1(kd,        vl);
    vfloat64m1_t v_Iprev   = __riscv_vle64_v_f64m1(I_prev,    vl);
    vfloat64m1_t v_errprev = __riscv_vle64_v_f64m1(err_prev,  vl);

    // error = setpoint - currpoint
    vfloat64m1_t v_err =
        __riscv_vfsub_vv_f64m1(v_set, v_curr, vl);

    // I_curr = I_prev + error * dt  (vfmacc_vf: acc + scalar * vec)
    vfloat64m1_t v_Icurr =
        __riscv_vfmacc_vf_f64m1(v_Iprev, dt, v_err, vl);
    // D_curr = (error - error_prev) / dt
    vfloat64m1_t v_err_diff =
        __riscv_vfsub_vv_f64m1(v_err, v_errprev, vl);
    vfloat64m1_t v_Dcurr =
        __riscv_vfmul_vf_f64m1(v_err_diff, inv_dt, vl);

    // output = Kp*error + Ki*I_curr + Kd*D_curr
    vfloat64m1_t v_out =
        __riscv_vfmul_vv_f64m1(v_kp, v_err, vl);                 // Kp * error
    v_out =
        __riscv_vfmacc_vv_f64m1(v_out, v_ki, v_Icurr, vl);       // + Ki * I_curr
    v_out =
        __riscv_vfmacc_vv_f64m1(v_out, v_kd, v_Dcurr, vl);       // + Kd * D_curr
    // 상태 업데이트: I_prev, error_prev, output
    __riscv_vse64_v_f64m1(I_prev,    v_Icurr,   vl);
    __riscv_vse64_v_f64m1(err_prev,  v_err,     vl);
    __riscv_vse64_v_f64m1(output,    v_out,     vl);
    // dependency 있는 스칼라 명령어 추가
    // 결과 struct에 반영 (측정 바깥)
    for (int i = 0; i < N; ++i) {
        actuators[i].I_prev     = I_prev[i];
        actuators[i].error_prev = err_prev[i];
        actuators[i].output     = output[i];
    }

    __asm__ volatile ("csrr %0, cycle" : "=r"(end) :: "memory");

    printf("Cycles: %lu\n", end - start);


    // 디버그 출력
    for (int i = 0; i < N; ++i) {
        printf("act[%d].output = %f\n", i, actuators[i].output);
    }

    return 0;
}
