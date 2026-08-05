# See LICENSE for license details.
#
# void fiber_switch(fiber_regs_t *from, fiber_regs_t *to)
#
# Saves {ra, sp, s0-s11} into *from (a0), then restores the same set from
# *to (a1) and returns -- into whatever `ra` *to had, i.e. either back into
# a previous fiber_switch() call site (resuming a fiber) or into
# fiber_trampoline (starting one for the first time; see fiber.c).
#
# fiber_regs_t layout (fiber.h), all uint64_t, naturally aligned / no padding:
#   offset  0: ra
#   offset  8: sp
#   offset 16..112: s0..s11

    .text
    .align 2
    .globl fiber_switch
    .type fiber_switch, @function
fiber_switch:
    sd ra,    0(a0)
    sd sp,    8(a0)
    sd s0,   16(a0)
    sd s1,   24(a0)
    sd s2,   32(a0)
    sd s3,   40(a0)
    sd s4,   48(a0)
    sd s5,   56(a0)
    sd s6,   64(a0)
    sd s7,   72(a0)
    sd s8,   80(a0)
    sd s9,   88(a0)
    sd s10,  96(a0)
    sd s11, 104(a0)

    ld ra,    0(a1)
    ld sp,    8(a1)
    ld s0,   16(a1)
    ld s1,   24(a1)
    ld s2,   32(a1)
    ld s3,   40(a1)
    ld s4,   48(a1)
    ld s5,   56(a1)
    ld s6,   64(a1)
    ld s7,   72(a1)
    ld s8,   80(a1)
    ld s9,   88(a1)
    ld s10,  96(a1)
    ld s11, 104(a1)
    ret
    .size fiber_switch, . - fiber_switch
