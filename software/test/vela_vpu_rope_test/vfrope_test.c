#include <stdint.h>
#include <stdio.h>

// 커스텀 명령어 호출을 위한 매크로 (0x4A21C1D7)
// vfrope.fvx v3, v2, x3 (x3는 m과 idx가 합쳐진 값)
#define ROPE_INST(vd, vs2, rs1) \
  asm volatile (".word 0x4A21C1D7" \
                : : "f"(vd), "f"(vs2), "r"(rs1))

int16_t x_data[8] = {0x2000, 0x4000, 0x6000, 0x7FFF, 0xE000, 0xC000, 0xA000, 0x8000};
int16_t y_data[8] = {0};

int main() {
    
    // m과 idx 설정 (t2=65, t3=0 이었던 로직)
    uint32_t m = 65;
    uint32_t idx = 0;
    uint32_t rs1_val = (m << 16) | (idx & 0xFFFF);

    printf("Starting RoPE Test...\n");

    asm volatile (
        "vsetvli zero, %1, e16, m1, ta, ma \n\t"
        "vle16.v v2, (%0)                  \n\t"
        : : "r"(x_data), "r"(8) : "v2"
    );

    printf("Executing custom RoPE instruction with m=%d, idx=%d\n", m, idx);

    asm volatile(
        "mv   t4, gp\n"
        "mv   gp, %0\n"    
        ".word 0x4A21C1D7\n"
        "mv   gp, t4\n"
        : : "r"(rs1_val) : "t4", "memory"
    );

    asm volatile (
        "vse16.v v3, (%0) \n\t"
        : : "r"(y_data) : "memory"
    );

    for(int i = 0; i < 8; i++) {
        printf("y_data[%d] = 0x%04x\n", i, (uint16_t)y_data[i]);
    }

    return 0;
}