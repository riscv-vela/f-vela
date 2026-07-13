#include <stdint.h>
#include <stdio.h>

static volatile uint16_t x_data[8]
    __attribute__((aligned(16))) = {
        0x2000, 0x4000, 0x6000, 0x7FFF,
        0xE000, 0xC000, 0xA000, 0x8000
};

static volatile uint16_t y_data[8]
    __attribute__((aligned(16))) = {0};

int main(void)
{
    printf("Starting RoPE Test...\n");

    /*
     * 주소는 x3(gp)를 변경하기 전에 일반 레지스터에 배치된다.
     *
     * t4 : 기존 gp 보관
     * t5 : m
     * t6 : idx 및 config 생성
     */
    printf("[1] Starting RoPE Test\n");

    asm volatile(
        "li t0, 0x1E00\n"
        "csrs mstatus, t0\n"
        :
        :
        : "t0", "memory"
    );

    printf("[2] mstatus configured\n");

    asm volatile(
        "li t0, 8\n"
        "vsetvli zero, t0, e16, m1, ta, ma\n"
        :
        :
        : "t0", "memory"
    );

    printf("[3] vsetvli passed\n");

    asm volatile(
        "vle16.v v2, (%0)\n"
        :
        : "r"(x_data)
        : "memory"
    );

    printf("[4] vector load passed\n");

    asm volatile(
        "mv      t4, gp\n"
        "li      t5, 65\n"
        "slli    t5, t5, 16\n"
        "li      t6, 0\n"
        "or      gp, t5, t6\n"
        ".word   0x4A21C1D7\n"
        "mv      gp, t4\n"
        :
        :
        : "t4", "t5", "t6", "memory"
    );

    printf("[5] custom instruction passed\n");

    asm volatile(
        "vse16.v v3, (%0)\n"
        :
        : "r"(y_data)
        : "memory"
    );

printf("[6] vector store passed\n");

    printf("Input Data:\n");

    for (int i = 0; i < 8; i++) {
        printf(
            "x_data[%d] = 0x%04x\n",
            i,
            (unsigned int)x_data[i]
        );
    }

    printf("RoPE Result:\n");

    for (int i = 0; i < 8; i++) {
        printf(
            "y_data[%d] = 0x%04x\n",
            i,
            (unsigned int)y_data[i]
        );
    }

    printf("RoPE Test Finished.\n");

    return 0;
}