#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHIPYARD_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BUILD_DIR="$SCRIPT_DIR/build"

# 실행할 ELF 파일
ELF_FILE="${1:-$BUILD_DIR/mpgemm.elf}"

# --extension=gemmini 옵션을 통해 Spike가 커스텀 RoCC 명령어를 처리하게 합니다.
spike --extension=gemmini $ELF_FILE --isa=rv64gcv --log-commits
