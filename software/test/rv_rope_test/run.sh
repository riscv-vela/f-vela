#!/bin/bash
# 경로 설정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../../../../" && pwd)"
VERILATOR_DIR="$CHIPYARD_DIR/sims/verilator"

echo "Current Script Dir: $SCRIPT_DIR"
echo "Chipyard Root: $CHIPYARD_DIR"

# 환경 변수 로드
if [ -f "$CHIPYARD_DIR/env.sh" ]; then
    source "$CHIPYARD_DIR/env.sh"
else
    echo "Error: env.sh not found in $CHIPYARD_DIR"
    exit 1
fi

# LUT 생성
if [ -f "make_lut.py" ]; then
    python3 make_lut.py
fi

# 하드웨어의 loadMemoryFromFileInline이 LUT 파일을 찾을 수 있도록 시뮬레이터 디렉토리(sims/verilator)로 복사
if [ -d "$VERILATOR_DIR" ]; then
    echo "Copying LUT hex files to $VERILATOR_DIR..."
    cp -f "$SCRIPT_DIR"/*.hex "$VERILATOR_DIR/"
else
    echo "Warning: Verilator directory not found at $VERILATOR_DIR"
fi

# Risc-V Compile Make 파일 실행
mkdir build
cd build
cmake ..
make -j$(nproc)

# Compile 
riscv64-unknown-elf-gcc -march=rv64gcv -mabi=lp64d -O2 \
    -static -mcmodel=medany \
    -fno-common -fno-builtin-printf \
    -specs=htif_nano.specs \
    vfrope_test.c -o vfrope_test.elf -lm

# 빌드 결과 확인
if [ $? -eq 0 ]; then
    echo "-----------------------------------------------"
    echo "Build Success: vfrope_test.elf"
    riscv64-unknown-elf-nm vfrope_test.elf | grep tohost
    echo "-----------------------------------------------"
else
    echo "Build Failed!"
    exit 1
fi

# 데이터 확인
riscv64-unknown-elf-objdump -s -j .data vfrope_test.elf