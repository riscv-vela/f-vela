// See LICENSE for license details.
//
// Vector RoPE custom-instruction workload, ported from
// rv_rope_test/vfrope_test2.c (bare-metal) and
// rv_rope_test/vfrope_test2_linux.c (Linux) into a single callable task
// (`rope_task_run`). Unlike rv_pid_test's pair, these two sources are
// otherwise identical (vfrope_test2.c doesn't touch the M-mode-only
// `mstatus` CSR either -- only the older vfrope_test.c, not test2, does
// that) -- see rv_rope_test/README.md for why *_test2 is the HW-validated
// variant reused here.

#include <stdint.h>
#include <stdio.h>
#include "task_rope.h"

static volatile uint16_t x_data[8]
    __attribute__((aligned(16))) = {
        0x2000, 0x4000, 0x6000, 0x7FFF,
        0xE000, 0xC000, 0xA000, 0x8000
};

static volatile uint16_t y_data[8]
    __attribute__((aligned(16))) = {0};

void rope_task_run(void) {
#ifdef BAREMETAL
    printf("Starting RoPE Test...\n");
#else
    printf("Starting RoPE Test (linux)...\n");
#endif

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
}
