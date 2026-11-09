// ggml-vela.h — velaNPU offload hooks called from ggml.c (GGML_USE_VELA).
#ifndef GGML_VELA_H
#define GGML_VELA_H

#include <stdbool.h>
#include <stdint.h>

struct ggml_tensor;

#ifdef __cplusplus
extern "C" {
#endif

// Called at the top of ggml_compute_forward_mul_mat by every thread. Returns true when the
// matmul is done on the NPU (thread 0 runs it; the others return at once), false -> CPU path.
bool ggml_vela_mul_mat(const struct ggml_tensor *src0, const struct ggml_tensor *src1,
                       struct ggml_tensor *dst, int ith, int nth);

// Called for GGML_OP_ROPE by every thread. Returns true when the RoPE is done on the velaVPU RoPE unit
// (VELA_ROPE=on, thread 0 runs it; the others return at once), false -> CPU path.
bool ggml_vela_rope(struct ggml_tensor *dst, int ith, int nth);

// Timing of every GGML_OP_MUL_MAT and GGML_OP_ROPE (CPU or NPU/VPU), for the summary printed at exit.
uint64_t ggml_vela_now(void);
void     ggml_vela_account(const struct ggml_tensor *dst, int ith, uint64_t t0);

#ifdef __cplusplus
}
#endif

#endif // GGML_VELA_H
