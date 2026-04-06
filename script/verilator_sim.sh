#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
VERILATOR_DIR="$CHIPYARD_DIR/sims/verilator"
SIMULATOR="simulator-chipyard.harness-FVelaSoCConfigTest"
# CONFIG_NAME="simulator-chipyard.harness-FVelaGemminiConfigTest"
# CONFIG_NAME="simulator-chipyard.harness-FVelaGemminiConfigTest-debug"

# 프로젝트 설정 
BUILD_DIR="$SCRIPT_DIR/build"

# 실행할 ELF 파일 (인자가 있으면 해당 파일, 없으면 기본값)
ELF_FILE="${1:-$BUILD_DIR/mpgemm.elf}"
ELF_NAME=$(basename "$ELF_FILE")

# 시뮬레이터 경로 확인
SIM_PATH="$VERILATOR_DIR/$SIMULATOR"

if [ ! -f "$SIM_PATH" ]; then
    echo "시뮬레이터를 찾을 수 없습니다: $SIM_PATH"
    echo "먼저 sims/verilator에서 'make'를 완료하세요."
    exit 1
fi

if [ ! -f "$ELF_FILE" ]; then
    echo "ELF 파일을 찾을 수 없습니다: $ELF_FILE"
    exit 1
fi

PK_PATH=$(which pk)
if [ -z "$PK_PATH" ]; then
    PK_PATH="/home/woojin/_risc_vela/chipyard/.conda-env/riscv-tools/riscv64-unknown-elf/bin/pk"
fi

echo "=============================================="
echo "  Simulation Starting..."
echo "  Simulator: $SIMULATOR"
echo "  Simulator Path: $SIM_PATH"
echo "  PK Path: $PK_PATH"
echo "  Binary: $ELF_NAME"
echo "  Log: $BUILD_DIR/${ELF_NAME}.log"
echo "=============================================="

# 환경 변수 로드
source "$CHIPYARD_DIR/env.sh"

# 스택 제한 해제
ulimit -s unlimited

#argument parsing
while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--sim)
            SIM_NAME="$2"
            SIMULATOR="simulator-chipyard.harness-$SIM_NAME"
            shift 2
            ;;
        -pk) 
            PK_PATH="$2"
            shift 2
            ;;
        
        --clean)
            DO_CLEAN=true
            shift
            ;;       

        -h|--help)
            usage
            echo "Example usage:"
            echo "  $0 -s MyCustomSim (default: $SIMULATOR)"
            echo "  $0 -pk /path/to/pk (default: $PK_PATH)"
            echo "  $0 --clean"
            exit 0
            ;;
            
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done


# 시뮬레이션 실행 (+verbose 로그는 build 폴더 내에 저장)
# $SIM_PATH +max-cycles=10000000 -- $PK_PATH $ELF_FILE 2>&1 | tee "$BUILD_DIR/${ELF_NAME}.log"

# BareMetal SIM
$SIM_PATH \
    +permissive \
    +max-cycles=100000000 \
    +permissive-off \
    $ELF_FILE \
    2>&1 | tee "$BUILD_DIR/${ELF_NAME}.log"

# Proxy Kernel SIM
# $SIM_PATH \
#     +permissive \
#     +max-cycles=100000000 \
#     +permissive-off \
#     $PK_PATH \
#     $ELF_FILE \
#     2>&1 | tee "$BUILD_DIR/${ELF_NAME}.log"

if [ $? -eq 0 ]; then
    echo "  Simulation Completed Successfully!"
    tail -n 20 "$BUILD_DIR/${ELF_NAME}.log" | grep -E "SUCCESS|FAIL|PASSED"
else
    echo "  Simulation Encountered an Error"
    exit 1
fi
