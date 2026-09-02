// Shared Conv2D problem size for the cmp_conv_cycle comparison. All three programs
// (rocket_cpu_conv.c, vanilla_gemmini_conv.c, custom_gemmini_conv_profiler.c) use these
// exact dimensions so their reported cycle counts describe the same workload.
#ifndef CMP_CONV_DIMS_H
#define CMP_CONV_DIMS_H

#define BATCH_SIZE   2
#define IN_ROW_DIM   17
#define IN_COL_DIM   17
#define IN_CHANNELS  18
#define OUT_CHANNELS 19
#define KERNEL_DIM   3
#define PADDING      1
#define STRIDE       2

#define OUT_ROW_DIM ((IN_ROW_DIM + 2*PADDING - KERNEL_DIM) / STRIDE + 1)
#define OUT_COL_DIM ((IN_COL_DIM + 2*PADDING - KERNEL_DIM) / STRIDE + 1)

#endif // CMP_CONV_DIMS_H
