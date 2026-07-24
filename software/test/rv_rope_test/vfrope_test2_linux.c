// Linux userspace variant of vfrope_test2.c (custom vector RoPE instruction).
//
// Only difference from the bare-metal version: the explicit
//   csrs mstatus, t0   (sets FS/VS to "Dirty" so the vector unit is enabled)
// is removed. mstatus is an M-mode-only CSR, so writing it from a Linux user
// process (U-mode) raises an illegal-instruction trap. Under Linux the kernel
// already owns mstatus and enables vector state (sstatus.VS) lazily the first
// time the process executes a vector instruction, so no explicit enable is needed.
//
// Build with the riscv64-unknown-linux-gnu toolchain (see Makefile `linux` target).

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
    printf("Starting RoPE Test (linux)...\n");

    printf("[1] Starting vector sequence\n");

    asm volatile(
        "li      t0, 8\n"
        "vsetvli zero, t0, e16, m1, ta, ma\n"

        "vle16.v v2, (%0)\n"

        "mv      t4, gp\n"
        "li      t5, 65\n"
        "slli    t5, t5, 16\n"
        "li      t6, 0\n"
        "or      gp, t5, t6\n"

        ".word   0x4A21C1D7\n"

        "mv      gp, t4\n"
        "vse16.v v3, (%1)\n"
        :
        : "r"(x_data), "r"(y_data)
        : "t0", "t4", "t5", "t6", "memory"
    );

    printf("[2] vector sequence passed\n");

    // hw debugging
    // printf("[1] Starting RoPE Test\n");

    // asm volatile(
    //     "li t0, 8\n"
    //     "vsetvli zero, t0, e16, m1, ta, ma\n"
    //     :
    //     :
    //     : "t0", "memory"
    // );

    // printf("[2] vsetvli passed\n");

    // asm volatile(
    //     "vle16.v v2, (%0)\n"
    //     :
    //     : "r"(x_data)
    //     : "memory"
    // );

    // printf("[3] vector load passed\n");

    // asm volatile(
    //     "mv      t4, gp\n"
    //     "li      t5, 65\n"
    //     "slli    t5, t5, 16\n"
    //     "li      t6, 0\n"
    //     "or      gp, t5, t6\n"
    //     ".word   0x4A21C1D7\n"
    //     "mv      gp, t4\n"
    //     :
    //     :
    //     : "t4", "t5", "t6", "memory"
    // );

    // printf("[4] custom instruction passed\n");

    // asm volatile(
    //     "vse16.v v3, (%0)\n"
    //     :
    //     : "r"(y_data)
    //     : "memory"
    // );

    // printf("[5] vector store passed\n");

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
