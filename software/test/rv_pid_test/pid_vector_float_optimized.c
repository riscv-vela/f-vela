#include <stdio.h>
#include <riscv_vector.h>

typedef struct {
    float setpoint;
    float currpoint;
    float Kp, Ki, Kd;
    float I_prev;
    float error_prev;
    float output;
} Actuator;

int main(void) {
    Actuator actuators[4] = {
        {1.0f,  2.0f,  3.0f,  4.0f,  5.0f,  6.0f,  7.0f,  8.0f},
        {11.0f, 12.0f, 13.0f, 14.0f, 15.0f, 16.0f, 17.0f, 18.0f},
        {21.0f, 22.0f, 23.0f, 24.0f, 25.0f, 26.0f, 27.0f, 28.0f},
        {31.0f, 32.0f, 33.0f, 34.0f, 35.0f, 36.0f, 37.0f, 38.0f},
        // {41.0f, 42.0f, 43.0f, 44.0f, 45.0f, 46.0f, 47.0f, 48.0f},
        // {51.0f, 52.0f, 53.0f, 54.0f, 55.0f, 56.0f, 57.0f, 58.0f},
        // {61.0f, 62.0f, 63.0f, 64.0f, 65.0f, 66.0f, 67.0f, 68.0f},
        // {71.0f, 72.0f, 73.0f, 74.0f, 75.0f, 76.0f, 77.0f, 78.0f}
    };

    const int N = 4;
    float dt = 0.1f;
    float inv_dt = 1.0f / dt;

    // SoA 배열 준비 (측정 바깥에서 해도 됨)
    float setpoint[4], currpoint[4];
    float kp[4], ki[4], kd[4];
    float I_prev[4], err_prev[4], output[4];

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
    size_t vl = __riscv_vsetvl_e32m1(N);   // 여기서는 vl == 8

    vfloat32m1_t v_set     = __riscv_vle32_v_f32m1(setpoint,  vl);
    vfloat32m1_t v_curr    = __riscv_vle32_v_f32m1(currpoint, vl);
    vfloat32m1_t v_kp      = __riscv_vle32_v_f32m1(kp,        vl);
    vfloat32m1_t v_ki      = __riscv_vle32_v_f32m1(ki,        vl);
    vfloat32m1_t v_kd      = __riscv_vle32_v_f32m1(kd,        vl);
    vfloat32m1_t v_Iprev   = __riscv_vle32_v_f32m1(I_prev,    vl);
    vfloat32m1_t v_errprev = __riscv_vle32_v_f32m1(err_prev,  vl);

    // error = setpoint - currpoint
    vfloat32m1_t v_err =
        __riscv_vfsub_vv_f32m1(v_set, v_curr, vl);

    // I_curr = I_prev + error * dt  (vfmacc_vf: acc + scalar * vec)
    vfloat32m1_t v_Icurr =
        __riscv_vfmacc_vf_f32m1(v_Iprev, dt, v_err, vl);

    // D_curr = (error - error_prev) / dt
    vfloat32m1_t v_err_diff =
        __riscv_vfsub_vv_f32m1(v_err, v_errprev, vl);
    vfloat32m1_t v_Dcurr =
        __riscv_vfmul_vf_f32m1(v_err_diff, inv_dt, vl);

    // output = Kp*error + Ki*I_curr + Kd*D_curr
    vfloat32m1_t v_out =
        __riscv_vfmul_vv_f32m1(v_kp, v_err, vl);                 // Kp * error
    v_out =
        __riscv_vfmacc_vv_f32m1(v_out, v_ki, v_Icurr, vl);       // + Ki * I_curr
    v_out =
        __riscv_vfmacc_vv_f32m1(v_out, v_kd, v_Dcurr, vl);       // + Kd * D_curr

    // 상태 업데이트: I_prev, error_prev, output
    __riscv_vse32_v_f32m1(I_prev,    v_Icurr,   vl);
    __riscv_vse32_v_f32m1(err_prev,  v_err,     vl);
    __riscv_vse32_v_f32m1(output,    v_out,     vl);

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
