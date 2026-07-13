    .section .text._start
    .globl _start

_start:
    li      t0, 0x1E00
    csrs    mstatus, t0

    la      t1, _after_mret
    csrw    mepc, t1
    mret

_after_mret:
    la      sp, _stack_top

    li      t2, 65
    slli    t2, t2, 16
    li      t3, 0
    or      x3, t2, t3

    li      t0, 8
    vsetvli zero, t0, e16, m1, ta, ma

    la      t1, x_data
    vle16.v v2, (t1)

    fence   iorw, iorw

    # custom instruction : vfrope.fvx
    .word   0x4A21C1D7

    la      t3, y_data
    vse16.v v3, (t3)

    # Finish simulation (tohost = 1)
    li      t0, 1
    la      t1, tohost
    sd      t0, 0(t1)

1:  j       1b

    .section .data
    .align 2

x_data:
    .half   0x2000, 0x4000, 0x6000, 0x7FFF
    .half   0xE000, 0xC000, 0xA000, 0x8000

y_data:
    .space  16

    .section .bss
    .align 4
_stack:
    .space 4096
_stack_top:

    .section .tohost
    .align 6
    .globl tohost
tohost:  .dword 0
    .globl fromhost
fromhost:.dword 0