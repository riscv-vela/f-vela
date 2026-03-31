# 경로 설정
include_dir = ./include
lib_dir = ./libgemmini
test_dir = ./test/rv_gemm_simple_test
build_dir = ./build

# 툴체인 설정 (Chipyard env.sh 로드 상태 기준)
CC = riscv64-unknown-elf-gcc
OBJDUMP = riscv64-unknown-elf-objdump

# 컴파일 플래그
# -fno-common, -fno-builtin-printf: bare-metal에서 안전하게 실행하기 위함
# -march=rv64gc: 일반적인 Rocket 코어 사양 (64-bit, 기본 명령어 + MAFD 확장)
# -mabi=lp64d: 64-bit 데이터 모델, double-precision 지원
# -DBAREMETAL: 소스 코드에서 bare-metal 컴파일을 구분, #define BAREMETAL
CFLAGS = -I$(include_dir) -I$(lib_dir) -O2 -fno-common -fno-builtin-printf \
         -specs=htif_nano.specs -march=rv64gc -mabi=lp64d \
         -DBAREMETAL

# 테스트 파일 이름 (확장자 제외)
PROG = mpgemm
SRCS = $(test_dir)/$(PROG).c
TARGET = $(build_dir)/$(PROG).elf
DUMP = $(build_dir)/$(PROG).dump

.PHONY: all clean prepare

all: prepare $(TARGET) $(DUMP)

# build 디렉토리 생성
prepare:
	mkdir -p $(build_dir)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) $(SRCS) -o $@ -lm

$(DUMP): $(TARGET)
	$(OBJDUMP) -D $(TARGET) > $@

clean:
	rm -rf $(build_dir)
