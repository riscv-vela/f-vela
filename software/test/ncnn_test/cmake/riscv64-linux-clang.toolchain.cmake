# Cross toolchain for RISC-V Linux userspace using clang from riscv-gnu-toolchain (LLVM build).
# clang >= 17 is needed for the RVV tuple intrinsics ncnn uses (vfloat32m1x2_t, __riscv_vcreate_*).
# Set RISCV_LLVM_ROOT (default: /home/esca/_f_vela/toolchains/riscv64-llvm).
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR riscv64)

if(DEFINED ENV{RISCV_LLVM_ROOT})
    file(TO_CMAKE_PATH $ENV{RISCV_LLVM_ROOT} RISCV_LLVM_ROOT)
else()
    set(RISCV_LLVM_ROOT /home/esca/_f_vela/toolchains/riscv64-llvm)
endif()
set(RISCV_LLVM_ROOT ${RISCV_LLVM_ROOT} CACHE PATH "riscv llvm toolchain root")

set(CMAKE_C_COMPILER "${RISCV_LLVM_ROOT}/bin/clang")
set(CMAKE_CXX_COMPILER "${RISCV_LLVM_ROOT}/bin/clang++")
set(CMAKE_SYSROOT "${RISCV_LLVM_ROOT}/sysroot")
set(CMAKE_C_COMPILER_TARGET riscv64-unknown-linux-gnu)
set(CMAKE_CXX_COMPILER_TARGET riscv64-unknown-linux-gnu)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Base ISA. ncnn compiles its RVV/zfh kernels separately and picks them at runtime (hwcap).
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=rv64gc -mabi=lp64d" CACHE STRING "c flags")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=rv64gc -mabi=lp64d" CACHE STRING "c++ flags")
