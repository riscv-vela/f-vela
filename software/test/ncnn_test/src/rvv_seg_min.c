// Minimal repro: one vsseg8e32 (e32, m1, vl 16) whose first segment straddles a 4 KiB page
// boundary (field 0 on the first page, fields 1..7 on the next). Hangs on FVelaSoC (2026-10-29).
// Bare-metal (htif printf). -DOFF=<bytes before the boundary> (default 4).
#include <stdint.h>
#include <stdio.h>
#ifndef OFF
#define OFF 4
#endif
static uint32_t buf[2 * 4096 / 4] __attribute__((aligned(4096)));
int main(void)
{
    uint32_t* p = (uint32_t*)((char*)buf + 4096 - OFF);
    printf("before vsseg8e32 base=%p\n", (void*)p);
    asm volatile("li t0, 16\n vsetvli zero, t0, e32, m1, ta, ma\n vid.v v8\n vid.v v9\n vid.v v10\n vid.v v11\n"
                 "vid.v v12\n vid.v v13\n vid.v v14\n vid.v v15\n"
                 "vsseg8e32.v v8, (%0)\n fence\n" ::"r"(p) : "t0", "memory");
    printf("after vsseg8e32: buf[1023]=%u buf[1024]=%u\n", (unsigned)buf[1023], (unsigned)buf[1024]);
    return 0;
}
