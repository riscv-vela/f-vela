#!/bin/bash
CHIPYARD_DIR="/home/woojin/_risc_vela/chipyard"
source $CHIPYARD_DIR/env.sh 

# export RISCV_ROOT_PATH=$RISCV #/home/woojin/_risc_vela/chipyard/.conda-env/riscv-tools
# export RISCV_ROOT_PATH_IME1=$RISCV_ROOT_PATH
# Chipyard의 env.sh Riscv 툴체인에서 RVV 만 버젼 문제로 인해, 빌드는 Local Riscv 툴체인으로 진행 (spacemit-toolchain-linux-glibc-x86_64-v1.1.2 사용)
# 
export RISCV_ROOT_PATH="/home/woojin/riscv/spacemit-toolchain-linux-glibc-x86_64-v1.1.2"
export RISCV_ROOT_PATH_IME1=$RISCV_ROOT_PATH

cd ../

cmake -B build \
    -DBUILD_SHARED_LIBS=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    -DLLAMA_OPENSSL=OFF \
    -DGGML_RVV=ON \
    -DGGML_RV_ZFH=ON \
    -DGGML_RV_ZICBOP=ON \
    -DGGML_RV_ZIHINTPAUSE=ON \
    -DCMAKE_TOOLCHAIN_FILE=${PWD}/cmake/riscv64-spacemit-linux-gnu-gcc.cmake \
    -DCMAKE_INSTALL_PREFIX=build/installed

# cmake -B build \
#     -DBUILD_SHARED_LIBS=OFF \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DGGML_CPU_RISCV64_SPACEMIT=OFF \
#     -DLLAMA_OPENSSL=OFF \
#     -DGGML_RVV=OFF \
#     -DGGML_RV_ZFH=ON \
#     -DGGML_RV_ZICBOP=ON \
#     -DGGML_RV_ZIHINTPAUSE=ON \
#     -DRISCV64_SPACEMIT_IME_SPEC=RISCV64_SPACEMIT_IME1 \
#     -DCMAKE_TOOLCHAIN_FILE=${PWD}/cmake/riscv64-spacemit-linux-gnu-gcc.cmake \
#     -DCMAKE_INSTALL_PREFIX=build/installed

cmake --build build --config Release -j$(nproc)

# cmake -B build \
#     -DBUILD_SHARED_LIBS=OFF \
#     -DCMAKE_BUILD_TYPE=Release \
#     -DGGML_CPU_RISCV64_SPACEMIT=ON \
#     -DLLAMA_OPENSSL=OFF \
#     -DCMAKE_C_FLAGS="-march=rv64gcv -mabi=lp64d" \   ? 
#     -DCMAKE_CXX_FLAGS="-march=rv64gcv -mabi=lp64d" \ ?
#     -DGGML_RVV=ON  # 벡터 연산(RVV)을 쓸 경우
#     -DGGML_RV_ZFH=ON \
#     -DGGML_RV_ZICBOP=ON \
#     -DGGML_RV_ZIHINTPAUSE=ON \
#     -DRISCV64_SPACEMIT_IME_SPEC=RISCV64_SPACEMIT_IME1 \
#     -DCMAKE_TOOLCHAIN_FILE=${PWD}/cmake/riscv64-spacemit-linux-gnu-gcc.cmake \
#     -DCMAKE_INSTALL_PREFIX=build/installed

# cmake --build build --parallel $(nproc) --config Release

# pushd build
# make install
# popd

# /home/woojin/qemu/jdsk-qemu/bin/qemu-riscv64 -L ${RISCV_ROOT_PATH_IME1}/sysroot -cpu max,vlen=256,elen=64,vext_spec=v1.0 ${PWD}/build/bin/llama-cli -m ${PWD}/models/Qwen2.5-0.5B-Instruct-Q4_0.gguf -t 1
