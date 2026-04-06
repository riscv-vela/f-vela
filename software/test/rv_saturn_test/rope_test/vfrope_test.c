#include <stdint.h>
#include <stdio.h>

// // 1. 시뮬레이터 종료 및 출력을 위한 필수 변수 (지워지지 않게 섹션 강제 지정)
// volatile uint64_t tohost __attribute__((section(".tohost"))) = 0;
// volatile uint64_t fromhost __attribute__((section(".tohost"))) = 0;

// // 2. 테스트 데이터 (역시 .data 섹션에 명시)
// int16_t x_data[8] __attribute__((section(".data"))) = {0x2000, 0x4000, 0x6000, 0x7FFF, 0xE000, 0xC000, 0xA000, 0x8000};
// int16_t y_data[8] __attribute__((section(".data"))) = {0};

// // 프로그램 종료를 위해 강제로 값을 쓰는 함수 (printf가 안될 때 대비)
// void terminate_simulation() {
//     tohost = 1;
//     while(1);
// }

// void _start() {
//     printf("Starting RoPE Test...\n");
//     // m=65, idx=0 설정
//     uint32_t rs1_val = (65 << 16) | (0 & 0xFFFF);

//     // 인라인 어셈블리로 하드웨어 제어
//     asm volatile (
//         "vsetvli zero, %0, e16, m1, ta, ma \n\t"
//         "vle16.v v2, (%1)                  \n\t"
//         : : "r"(8), "r"(x_data) : "v2"
//     );

//     // 커스텀 명령어 실행 (rs1으로 x3 대신 a0 사용)
//     asm volatile (
//         "mv a0, %0 \n\t"
//         ".word 0x4A21C1D7"
//         : : "r"(rs1_val) : "a0"
//     );

//     // 결과 저장
//     asm volatile (
//         "vse16.v v3, (%0)"
//         : : "r"(y_data) : "memory"
//     );
//     for(int i = 0; i < 8; i++) {
//         printf("y_data[%d] = 0x%04x\n", i, (uint16_t)y_data[i]);
//     }

//     // 시뮬레이션 종료 신호 (tohost = 1)
//     terminate_simulation();
// }

// 커스텀 명령어 호출을 위한 매크로 (0x4A21C1D7)
// vfrope.fvx v3, v2, x3 (x3는 m과 idx가 합쳐진 값)
#define ROPE_INST(vd, vs2, rs1) \
  asm volatile (".word 0x4A21C1D7" \
                : : "f"(vd), "f"(vs2), "r"(rs1))

//시뮬레이터 종료 및 출력을 위한 필수 변수 (지워지지 않게 섹션 강제 지정) --> 컴파일 옵션에 -specs=htif_nano.specs 사용으로 인해 .tohost 섹션이 자동으로 생성되고, tohost와 fromhost 변수가 정의되어 시뮬레이터와의 통신이 가능해짐. 따라서 코드 내에서 명시적으로 선언할 필요 없음.
// volatile uint64_t tohost __attribute__((section(".tohost"))) = 0;
// volatile uint64_t fromhost __attribute__((section(".tohost"))) = 0;

// 1. 입력 데이터 준비 (Half-precision 대신 int16_t 사용)
int16_t x_data[8] = {0x2000, 0x4000, 0x6000, 0x7FFF, 0xE000, 0xC000, 0xA000, 0x8000};
int16_t y_data[8] = {0};

int main() {
    
    // m과 idx 설정 (t2=65, t3=0 이었던 로직)
    uint32_t m = 65;
    uint32_t idx = 0;
    uint32_t rs1_val = (m << 16) | (idx & 0xFFFF);

    printf("Starting RoPE Test...\n");

    // 벡터 유닛 및 데이터 로드/연산 (인라인 어셈블리 활용)
    // 실제 환경에선 vsetvli와 vle16.v 등이 필요하므로 __asm__으로.
    asm volatile (
        "vsetvli zero, %1, e16, m1, ta, ma \n\t"
        "vle16.v v2, (%0)                  \n\t"
        : : "r"(x_data), "r"(8) : "v2"
    );

    // 커스텀 명령어 실행 (x3 대신 일반적인 C 변수 전달)
    asm volatile (
        "mv a0, %0 \n\t"        // rs1_val을 a0 레지스터로 강제 이동
        ".word 0x4A21C1D7"      // 커스텀 명령어 실행 (rs1이 a0임을 하드웨어와 약속했을 경우)
        : : "r"(rs1_val) : "a0" // rs1_val을 입력으로 사용하고, a0를 사용함을 명시
    );

    // 결과값 v3를 y_data로 복사
    asm volatile (
        "vse16.v v3, (%0) \n\t"
        : : "r"(y_data) : "memory"
    );

    // 결과 출력
    for(int i = 0; i < 8; i++) {
        printf("y_data[%d] = 0x%04x\n", i, (uint16_t)y_data[i]);
    }

    return 0;
}