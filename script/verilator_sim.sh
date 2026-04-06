#!/usr/bin/env bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

SOFTWARE_DIR="$CHIPYARD_DIR/generators/_f_vela/software/"
VERILATOR_DIR="$CHIPYARD_DIR/sims/verilator"
SIMULATOR="simulator-chipyard.harness-FVelaSoCConfigTest"
# CONFIG_NAME="simulator-chipyard.harness-FVelaGemminiConfigTest"
# CONFIG_NAME="simulator-chipyard.harness-FVelaGemminiConfigTest-debug"


# 프로젝트 설정 
BUILD_DIR="$SOFTWARE_DIR/build"

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
# 1. usage 함수 정의 (에러 방지)
usage() {
    echo "Usage: $0 [ELF_FILE] [-s|--sim CONFIG_NAME] [-pk PK_PATH] [--clean]"
}

# 2. 파싱 전 기본값 설정
ELF_FILE=""
DO_CLEAN=false

# 3. Argument Parsing
while [[ $# -gt 0 ]]; do
    case $1 in
        -s|--sim)
            # simulator-chipyard.harness- 부분을 포함해서 입력하면 중복되므로 
            # CONFIG_NAME만 받거나, 전체 이름을 받도록 처리
            SIM_NAME="$2"
            if [[ $SIM_NAME == simulator-chipyard.harness-* ]]; then
                SIMULATOR="$SIM_NAME"
            else
                SIMULATOR="simulator-chipyard.harness-$SIM_NAME"
            fi
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
            exit 0
            ;;
        -*) # 알 수 없는 옵션 처리
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
        *) # 옵션이 아닌 것은 ELF_FILE로 간주
            ELF_FILE="$1"
            shift
            ;;
    esac
done

# 4. ELF_FILE이 지정되지 않았을 경우 기본값 적용
if [ -z "$ELF_FILE" ]; then
    ELF_FILE="$BUILD_DIR/mpgemm.elf"
fi

# 5. 최종 경로 및 로그 설정 (파싱 후에 반영)
ELF_NAME=$(basename "$ELF_FILE")
SIM_PATH="$VERILATOR_DIR/$SIMULATOR"


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
