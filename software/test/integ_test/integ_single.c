// See LICENSE for license details.
//
// integ_test single-thread test selector.
//
// Prints a menu and waits for a single keypress on the console (HTIF
// console under bare metal / Verilator+FireSim sim; the busybox tty over
// UART on the FireSim/Alveo Linux target) to pick which accelerator test to
// run: 1) gemmini, 2) pid, 3) rope, 4) profiler. Loops back to the menu
// after each test; 'q' exits.
//
// FALLBACK WHEN NO INTERACTIVE CONSOLE IS ATTACHED: under a batch Verilator
// run with nothing hooked up to stdin, the HTIF console's read() doesn't
// block for input the way a real terminal's would -- getchar() returns EOF
// immediately instead. Looping on the menu in that case just spins forever
// printing "unrecognized key", flooding the sim log until it's killed. So
// the first EOF from wait_for_key() is treated as "no interactive console
// available", and this falls back to running every test once, in a fixed
// order (rope, pid, gemmini, profiler), then exits -- still exercises all
// four workloads without needing a human (or a piped input file) at the
// other end of the console.
//
// Dual-mode (like rv_gemmini_test/gemm.c and profiler_test/gemm_profiler_test.c):
// the exact same source builds for bare metal (-DBAREMETAL, htif specs) and
// for the Linux/busybox userspace build (see Makefile).

#include <stdio.h>
#include <stdlib.h>
#include "task_gemmini.h"
#include "task_profiler.h"
#include "task_pid.h"
#include "task_rope.h"

static int wait_for_key(void) {
    int c;
    do {
        c = getchar();
    } while (c == '\n' || c == '\r');
    return c;
}

static void run_all_tests_in_order(void) {
    printf("\n[integ_single] no interactive console input detected -- running every\n");
    printf("[integ_single] test once, in order: rope, pid, gemmini, profiler.\n");
    rope_task_run();
    pid_task_run();
    gemmini_task_run();
    profiler_task_run();
    printf("\n[integ_single] all tests complete.\n");
}

static void print_menu(void) {
    printf("\n==================================================\n");
    printf(" F-Vela integ_test -- single-thread test selector\n");
#ifdef BAREMETAL
    printf(" (bare-metal / HTIF console)\n");
#else
    printf(" (Linux / busybox console)\n");
#endif
    printf("--------------------------------------------------\n");
    printf("  1) Gemmini tiled matmul test\n");
    printf("  2) PID vector custom-instruction test\n");
    printf("  3) RoPE vector custom-instruction test\n");
    printf("  4) Gemmini profiler test\n");
    printf("  q) Quit\n");
    printf("==================================================\n");
    printf("Select a test [1-4, q]: ");
    fflush(stdout);
}

int main(void) {
    for (;;) {
        print_menu();
        int key = wait_for_key();

        if (key == EOF) {
            run_all_tests_in_order();
            exit(0);
        }

        printf("%c\n", (char)key);

        switch (key) {
            case '1':
                gemmini_task_run();
                break;
            case '2':
                pid_task_run();
                break;
            case '3':
                rope_task_run();
                break;
            case '4':
                profiler_task_run();
                break;
            case 'q':
            case 'Q':
                printf("Bye.\n");
                exit(0);
            default:
                printf("Unrecognized key '%c' (0x%02x); choose 1-4 or q.\n", (char)key, (unsigned)key);
                break;
        }
    }
    return 0;
}
