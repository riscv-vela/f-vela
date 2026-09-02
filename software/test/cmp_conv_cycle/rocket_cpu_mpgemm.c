// Rocket Core (RISC-V scalar, no accelerator) reference mpgemm (mixed-precision GEMM:
// int8 activations x 2-bit-packed ternary weights), timed with rdcycle.
//
// Mirrors rv_gemmini_test/mpgemm.c / profiler_test/ternary_profiler_test.c's workload
// (C = A @ unpack(B) + D, A in {0,1}, B packed 4 ternary {-1,0,1} values per byte) but on
// pure Rocket -- no RoCC/Gemmini instruction at all, so this exact binary runs on any SoC
// config. It's the CPU-only baseline for custom_gemmini_mpgemm_profiler.c (there is no
// vanilla-Gemmini counterpart: ternary_gemm_auto is a custom/F-Vela-only feature, not present
// in upstream gemmini.h).
//
// Same ld/ex/st split as rocket_cpu_conv.c, matched to what the ternary PE array has to do:
//   ld: unpack the 2-bit-packed ternary B matrix into a plain {-1,0,1} matrix (the software
//       equivalent of whatever unpacking the custom systolic array does while feeding weights
//       in)
//   ex: the A x B_unpacked MAC loop (systolic-execute equivalent)
//   st: clip + copy the result into the output matrix (mvout equivalent)

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef BAREMETAL
#include <sys/mman.h>
#endif

#include "mpgemm_dims.h"

typedef int8_t elem_t;
typedef int32_t acc_t;
#define ELEM_T_MIN INT8_MIN
#define ELEM_T_MAX INT8_MAX

#define NO_BIAS 1

// Same FNV-1a checksum custom_gemmini_mpgemm_profiler.c prints. NO_BIAS=1 makes the bias term a
// no-op and neither program calls srand(), so both compute A x unpack(B) on bit-identical
// inputs -- a checksum match here is what verifies the two implementations agree on the actual
// mpgemm result, not just that they ran in a comparable number of cycles.
static uint32_t output_checksum(const void * buf, size_t len) {
    const uint8_t * p = (const uint8_t *)buf;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < len; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static uint64_t read_cycles(void) {
    uint64_t cycles;
    asm volatile ("rdcycle %0" : "=r" (cycles));
    return cycles;
}

// Emit the ld/ex/st stage timestamps as a "type, start, end" PROFILE DUMP block, same format
// the HW profilers (custom_gemmini_mpgemm_profiler.c, ternary_profiler_test.c) use -- so
// run_cmp.sh / profiler_test/plot/profile.py can extract and plot this CPU run the same way
// (0=ld, 1=ex, 2=st). On CPU the three stages run back-to-back, so this is just t0..t3.
static void dump_profile(uint64_t t0, uint64_t t1, uint64_t t2, uint64_t t3) {
    printf("=== PROFILE DUMP BEGIN ===\n");
    printf("0, %lu, %lu\n", t0, t1);
    printf("1, %lu, %lu\n", t1, t2);
    printf("2, %lu, %lu\n", t2, t3);
    printf("=== PROFILE DUMP END ===\n");
    printf("Captured 3 profile events (0=ld,1=ex,2=st).\n");
}

// A[I][K] in {0,1}; B_packed[K][J/4], 4 ternary {-1,0,1} values packed 2 bits each per byte.
static void init_mats_packed(elem_t A[MAT_DIM_I][MAT_DIM_K], elem_t B_packed[MAT_DIM_K][MAT_DIM_J/4]) {
    for (int i = 0; i < MAT_DIM_I; i++)
        for (int k = 0; k < MAT_DIM_K; k++)
            A[i][k] = rand() % 2;

    for (int k = 0; k < MAT_DIM_K; k++) {
        for (int jp = 0; jp < MAT_DIM_J / 4; jp++) {
            uint8_t packed_val = 0;
            for (int i = 0; i < 4; i++) {
                int ternary_val = rand() % 3 - 1; // -1, 0, or 1
                uint8_t two_bit_val = 0;
                if (ternary_val == 1) two_bit_val = 1;        // 0b01
                else if (ternary_val == -1) two_bit_val = 3;  // 0b11
                packed_val |= (two_bit_val & 0x03) << (i * 2);
            }
            B_packed[k][jp] = (elem_t)packed_val;
        }
    }
}

static void init_random_acc(acc_t * buf, int len) {
    for (int i = 0; i < len; i++)
        buf[i] = (rand() % 5) - 2;
}

static int8_t decode_ternary_2b(uint8_t two_bit_val) {
    if (two_bit_val == 1) return 1;
    if (two_bit_val == 3) return -1;
    return 0;
}

// ld: B_packed[K][J/4] (2-bit-packed ternary) -> B_unpacked[K][J] ({-1,0,1} as elem_t)
static void unpack_ternary(
        elem_t B_packed[MAT_DIM_K][MAT_DIM_J/4],
        elem_t B_unpacked[MAT_DIM_K][MAT_DIM_J]) {

    for (int k = 0; k < MAT_DIM_K; k++) {
        for (int jp = 0; jp < MAT_DIM_J / 4; jp++) {
            uint8_t packed_val = (uint8_t)B_packed[k][jp];
            for (int i = 0; i < 4; i++) {
                uint8_t two_bit_val = (packed_val >> (i * 2)) & 0x03;
                B_unpacked[k][jp * 4 + i] = decode_ternary_2b(two_bit_val);
            }
        }
    }
}

// ex: C_acc[I][J] = bias[j] + sum_k A[i][k] * B_unpacked[k][j]
static void matmul_ex(
        elem_t A[MAT_DIM_I][MAT_DIM_K],
        elem_t B_unpacked[MAT_DIM_K][MAT_DIM_J],
        acc_t bias[MAT_DIM_J],
        acc_t C_acc[MAT_DIM_I][MAT_DIM_J]) {

    for (int i = 0; i < MAT_DIM_I; i++) {
        for (int j = 0; j < MAT_DIM_J; j++) {
            acc_t result = NO_BIAS ? 0 : bias[j];
            for (int k = 0; k < MAT_DIM_K; k++)
                result += (acc_t)A[i][k] * (acc_t)B_unpacked[k][j];
            C_acc[i][j] = result;
        }
    }
}

// st: clip C_acc (32-bit accumulator) into the elem_t output matrix.
static void writeback_st(
        acc_t C_acc[MAT_DIM_I][MAT_DIM_J],
        elem_t C[MAT_DIM_I][MAT_DIM_J]) {

    for (int i = 0; i < MAT_DIM_I; i++) {
        for (int j = 0; j < MAT_DIM_J; j++) {
            acc_t v = C_acc[i][j];
            C[i][j] = v > ELEM_T_MAX ? ELEM_T_MAX : (v < ELEM_T_MIN ? ELEM_T_MIN : v);
        }
    }
}

int main() {
#ifndef BAREMETAL
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        perror("mlockall failed");
        exit(1);
    }
#endif

    printf("==============================================\n");
    printf("  Rocket CPU mpgemm (no accelerator, int8 x ternary)\n");
    printf("  I=%d J=%d K=%d\n", MAT_DIM_I, MAT_DIM_J, MAT_DIM_K);
    printf("==============================================\n");

    static elem_t A[MAT_DIM_I][MAT_DIM_K];
    static elem_t B_packed[MAT_DIM_K][MAT_DIM_J/4];
    static elem_t B_unpacked[MAT_DIM_K][MAT_DIM_J];
    static acc_t bias[MAT_DIM_J];
    static acc_t C_acc[MAT_DIM_I][MAT_DIM_J];
    static elem_t C[MAT_DIM_I][MAT_DIM_J];

    init_mats_packed(A, B_packed);
    init_random_acc(bias, MAT_DIM_J);

    printf("Rocket CPU mpgemm...\n");
    uint64_t t0 = read_cycles();
    unpack_ternary(B_packed, B_unpacked);
    uint64_t t1 = read_cycles();
    matmul_ex(A, B_unpacked, bias, C_acc);
    uint64_t t2 = read_cycles();
    writeback_st(C_acc, C);
    uint64_t t3 = read_cycles();

    uint64_t ld_cycles = t1 - t0;
    uint64_t ex_cycles = t2 - t1;
    uint64_t st_cycles = t3 - t2;

    printf("RESULT rocket_cpu_mpgemm cycles: %lu\n", t3 - t0);
    printf("  LD_CYCLES (ternary unpack): %lu\n", ld_cycles);
    printf("  EX_CYCLES (MAC/matmul):     %lu\n", ex_cycles);
    printf("  ST_CYCLES (writeback):      %lu\n", st_cycles);
    printf("RESULT output_checksum: 0x%08x\n", output_checksum(&C[0][0], sizeof(C)));

    dump_profile(t0, t1, t2, t3);

    printf("DONE\n");
    return 0;
}
