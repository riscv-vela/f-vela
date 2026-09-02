## Conv / mpgemm Cycle 비교 (Rocket CPU vs 순정 Gemmini vs Custom Gemmini)

같은 Conv2D 문제(`conv_dims.h`: BATCH=2, IN=17x17x18, OUT_CH=19, K=3, PAD=1, STRIDE=2)를
세 가지 방식으로, 같은 mpgemm(int8 x ternary matmul, `mpgemm_dims.h`: I=1,K=64,J=64) 문제를
두 가지 방식으로 돌려서 cycle을 비교한다.

### 무엇이 "공정한" 비교인가

**공정한 비교 = `RESULT ... cycles:` 한 줄, `rdcycle` 로 감싼 end-to-end 총 cycle뿐이다.**
CPU는 함수 전체를, Gemmini는 명령 발행부터 `gemmini_fence()`(=완료 동기화)까지를 재므로,
셋 다 "같은 경계"(작업 시작 ~ 완전히 끝남)를 기준으로 잰 wall-clock이라 서로 비교해도 된다.
실제로 이게 세 구현이 "얼마나 빠른가"를 말해주는 유일한 숫자다.

**LD/EX/ST breakdown은 그 반대다 — 각 구현이 자기 시간을 어디에 썼는지 보는 진단용이지,
행끼리(CPU의 EX vs Gemmini의 EX처럼) 값을 직접 비교하라고 만든 게 아니다.** 세 가지가 서로
완전히 다른 종류의 것을 재고 있기 때문이다.

| | 뭘 재는가 |
|---|---|
| Rocket CPU LD/EX/ST | C 함수 하나의 wall-clock 시간 (load/shift/AND/분기/함수 호출/store/loop 제어까지 전부 포함) |
| Gemmini active-cycle 카운터 (RDMA/EXE/WDMA) | HW 유닛이 "activated" 상태였던 시간 — 내부 FSM 신호(`firing`, `matmul_in_progress` 등) 기준 |
| Gemmini HW 프로파일러 dump 합산 | RS(reservation station)에서 관측 가능한 개별 명령 단위 start/end 구간의 합 — 타일마다 겹쳐 실행되므로 합이 실제 경과시간보다 커질 수 있음 |

그래서 예를 들어 CPU의 `EX_CYCLES`(수백만 cycle, scalar MAC loop 전체)와 Gemmini의
`EXE_ACTIVE_CYCLE`(수천 cycle, systolic array가 활성 상태였던 시간)을 "같아야 하는데 왜 다르지?"
라고 보면 안 된다 — 애초에 같을 이유가 없는, 서로 다른 정의의 숫자다. 또한 Gemmini 쪽은
`LD+EX+ST` 합이 총 cycle(`RESULT`)과 다를 수 있는데(카운터는 DMA/연산이 겹쳐 돌아가서 총합보다
크게, 프로파일러는 잡히지 않는 idle/hazard/큐잉 구간 때문에 총합보다 작게 나올 수 있다), 이것도
버그가 아니라 위 정의 차이 때문이다. `run_cmp.sh`가 breakdown 표 바로 위에 이 경고를 그대로
출력한다.

**cycle이 다르다고 결과값(수학적으로 같은 연산인지)까지 검증된 건 아니다.** 그래서 다섯 프로그램
모두 출력 버퍼에 대해 동일한 FNV-1a `output_checksum` 을 찍는다. 어느 프로그램도 `srand()` 를
호출하지 않아 libc 기본 시드로 항상 같은 난수열을 쓰고, `rand()` 호출 순서/횟수도 동일하게
맞춰놨기 때문에(mpgemm의 bias 초기화처럼 실제 연산에 안 쓰이는 부분만 순서가 다름) 세 conv
구현/두 mpgemm 구현은 항상 같은 입력으로 계산한다. `run_cmp.sh` 가 이 체크섬들을 서로 비교해서
`MATCH`/`MISMATCH` 를 출력하므로, "같은 conv를 계산했는지"는 cycle 표가 아니라 이 줄로 확인한다.

### Conv (3-way)

##### rocket_cpu_conv.c
- 가속기 없이 순수 C로 conv 수행. Gemmini 헤더를 전혀 include하지 않으므로, 같은 바이너리를 두
  시뮬레이터(`NoCustomFvelaTest`, `FVelaSoCConfigTest`) 양쪽에서 그대로 실행해서 "가속기 없는 core
  기준선"을 각 비교 대상과 같은 SoC 환경에서 얻는다.
- **LD/EX/ST breakdown**: 다른 두 프로파일러(HW 카운터, HW 프로파일러)와 같은 기준으로 비교하려고,
  conv를 `tiled_conv_auto()` 가 내부적으로 하는 것과 똑같이 im2col + matmul 두 단계로 재구성했다.
  - LD = im2col gather (padding 처리하며 receptive field를 patch 행렬로 모으는 단계, Gemmini의 mvin에 대응)
  - EX = patch 행렬 x weight 행렬 MAC 루프 (systolic 연산에 대응)
  - ST = 결과를 최종 output 텐서 레이아웃으로 복사 (Gemmini의 mvout에 대응)

  CPU는 세 단계가 완전히 순차 실행되므로 `LD+EX+ST == 전체 cycle` 이 항상 성립한다. 반면 HW는
  타일 단위로 DMA와 연산이 겹쳐 돌아가므로(pipelining) `LD+EX+ST` 합이 전체 wall-clock cycle보다 커질
  수 있다 — 그 차이가 곧 HW가 얻은 겹침(overlap) 효과다. 즉 완전히 동일한 기준은 아니고, "각 스테이지가
  각각 얼마나 걸렸는지"를 비교하는 것이지 "스테이지 합이 곧 총 시간이다"는 CPU에서만 성립한다는 점은
  숫자를 볼 때 감안해야 한다.
- **CSV/그래프**: `rdcycle`로 잰 `t0..t3` 스테이지 타임스탬프를 `dump_profile()`이 두 HW 프로파일러
  (`custom_gemmini_conv_profiler.c`, `ternary_profiler_test.c`)와 똑같은 `PROFILE DUMP BEGIN/END` +
  `type, start, end` 포맷으로 찍는다. 그래서 `run_cmp.sh`가 이 CPU 실행도 동일한 추출/플롯 파이프라인
  (`profiler_test/plot/profile.py`)에 태워 `build/rocket_cpu_conv.<sim>.csv`/`.png`로 저장한다.

##### vanilla_gemmini_conv.c
- 순정(upstream) Gemmini (`gemmini/software/gemmini-rocc-tests/include/gemmini_testutils.h`)의
  `tiled_conv_auto()` 로 conv 수행.
- **HW 프로파일러 없이 cycle을 재는 방법**: `gemmini_fence()` 는 실제로는 RISC-V `fence` 명령 한 줄
  (`#define gemmini_fence() asm volatile("fence")`)이고, RoCC 인터페이스 규약상 `fence` 는 가속기의
  busy 신호가 내려갈 때까지, 즉 mvin/mvout/systolic 연산과 그에 딸린 DMA까지 전부 끝날 때까지 Rocket
  파이프라인을 정지시킨다. Gemmini 커맨드 큐도 core에 backpressure를 걸기 때문에, `tiled_conv_auto(...)
  ; gemmini_fence();` 를 `rdcycle` 로 감싸면 core가 실제로 관측하는 cycle 델타 = Conv 전체(DMA+연산)
  cycle이 정확히 나온다. `gemmini-rocc-tests/bareMetalC/conv.c`, `conv_perf.c` 가 이미 이 패턴을 쓰고
  있어서 그대로 재사용했다.
- LD/EX/ST는 `gemmini_counter.h` 의 내장 active-cycle 카운터로 뽑는다: LD=`RDMA_ACTIVE_CYCLE`,
  EX=`EXE_ACTIVE_CYCLE`, ST=`WDMA_ACTIVE_CYCLE` (덤으로 `LOOP_MATMUL_ACTIVE_CYCLES` 도 출력).
- `simulator-chipyard.harness-NoCustomFvelaTest` (Rocket + 업스트림 `DefaultGemminiConfig`) 에서 실행.

##### custom_gemmini_conv_profiler.c
- `profiler_test/gemm_profiler_test.c` 와 동일한 흐름으로, F-Vela custom Gemmini
  (`pf_gemmini.h`) 의 실제 HW 프로파일러를 붙여서 conv (`tiled_conv_auto()`) 를 수행.
  `gemmini_profiler(P)` 로 프로파일 버퍼를 등록하고, `PROFILE DUMP BEGIN/END` 사이에
  `type, start, end` 이벤트를 찍는다 (`run_profiler.sh` / `profiler/profile.py` 와 같은 포맷).
- 이벤트를 타입별(0=ld,1=ex,2=st)로 합산해서 `LD_CYCLES`/`EX_CYCLES`/`ST_CYCLES` 도 같이 출력 —
  다른 둘과 동일한 라벨.
- **동시에 `counter_configure()` 로 같은 active-cycle 카운터도 등록**해서, 프로파일러 없이 카운터만
  으로도 값이 맞는지 직접 검증한다 (아래 "Custom Gemmini도 카운터로 뽑을 수 있나?" 참고).
- `simulator-chipyard.harness-FVelaSoCConfigTest` (Rocket + F-Vela custom Gemmini) 에서 실행.

### mpgemm / int8 x ternary matmul (2-way, vanilla 없음)

`ternary_gemm_auto()` 는 F-Vela 전용 명령이라 순정 Gemmini(`gemmini.h`)에는 아예 없다. 그래서 이
비교는 CPU vs Custom Gemmini 둘 뿐이다.

##### rocket_cpu_mpgemm.c
- `profiler_test/ternary_profiler_test.c` 와 같은 워크로드(A: int8 0/1, B: 4개씩 2bit로 packing된
  ternary(-1/0/1))를 CPU로 재현. LD=ternary unpack, EX=MAC 루프, ST=결과 clip+writeback.
- rocket_cpu_conv.c와 마찬가지로 `dump_profile()`이 `t0..t3`를 HW 프로파일러와 동일한
  `PROFILE DUMP BEGIN/END` 포맷으로 찍어서 `run_cmp.sh`가 `build/rocket_cpu_mpgemm.csv`/`.png`를
  만들어준다.

##### custom_gemmini_mpgemm_profiler.c
- `profiler_test/ternary_profiler_test.c` 기반. HW 프로파일러 dump + `counter_configure()` 카운터를
  둘 다 붙여서 conv 쪽과 동일한 출력 포맷(`RESULT ... cycles:`, `LD/EX/ST_CYCLES`)으로 정리했다.
- `simulator-chipyard.harness-FVelaSoCConfigTest` 에서 실행.

### Custom Gemmini도 `gemmini_counter` 로 뽑을 수 있나? (프로파일러가 필수인가?)

아니다, **프로파일러 없이 카운터만으로도 된다.** Custom Gemmini는 ternary matmul을 위해 systolic
PE datapath를 확장했지만, 카운터 인프라 자체는 건드리지 않았다:

- `f_vela_gemmini/CounterFile.scala` 는 업스트림 `gemmini/CounterFile.scala` 와 패키지명만 다르고
  **완전히 동일**하다 (`diff` 결과 한 줄 차이).
- `ExecuteController.scala` 의 `EXE_ACTIVE_CYCLE` 카운터는
  `io.counter.connectEventSignal(CounterEvent.EXE_ACTIVE_CYCLE, firing || matmul_in_progress)` 로,
  int8 경로든 ternary 경로든 상관없이 컨트롤 FSM의 공통 신호(`firing`/`matmul_in_progress`)에 연결돼
  있다. RDMA/WDMA 쪽 active-cycle 카운터도 마찬가지로 DMA busy 신호 기반이라 데이터 폭/precision과
  무관하다.
- `custom_gemmini_conv_profiler.c`/`custom_gemmini_mpgemm_profiler.c` 에서 실제로 카운터와
  프로파일러를 동시에 등록해서 실행해보면, 카운터로 읽은 LD/EX/ST(`counter, ...`)와 프로파일러 이벤트를
  합산한 LD/EX/ST(`profiler, ...`)가 같은 자릿수로 나온다 (`run_cmp.sh` 의 LD/EX/ST breakdown 표 참고).

즉 **HW 프로파일러는 "필요조건"이 아니라 "더 좋은 도구"**다: 카운터는 전체 active-cycle 총합만 주는
반면, 프로파일러는 개별 명령 단위의 start/end 타임스탬프를 줘서 타일별 겹침(pipelining) 양상을
간트차트로 볼 수 있게 해준다 (`profiler/profile.py` 가 그리는 그래프). 총 cycle 수만 필요하면 굳이
프로파일러 없이도 vanilla와 동일한 방식(카운터, 또는 `rdcycle`+`gemmini_fence()`)으로 충분하다.

### 빌드 / 실행

```sh
make            # 다섯 개(conv 3개 + mpgemm 2개) 전부 bare-metal 빌드
make linux      # firemarshal/busybox용 Linux 바이너리 빌드
./run_cmp.sh    # conv 4번 + mpgemm 2번 실행 후 cycle 비교표 + LD/EX/ST breakdown 표 출력
```

`run_cmp.sh` 는 네 프로그램(`custom_gemmini_conv_profiler`, `custom_gemmini_mpgemm_profiler`,
`rocket_cpu_conv`, `rocket_cpu_mpgemm`)의 실행 로그에서 프로파일 덤프를 추출해 `build/*.csv` 로
저장하고, matplotlib이 있으면 그래프도 그린다 (`profiler_test/run_profiler.sh` 와 동일한 파이프라인).
CPU 쪽 둘은 HW 프로파일러가 없으니 `t0..t3` 스테이지 타임스탬프를 같은 포맷으로 대신 찍는다.
출력 표는 순서대로: (1) 공정 비교 대상인 총 cycle(`RESULT`), (2) 출력 checksum MATCH/MISMATCH
(결과값 자체가 같은지), (3) 참고용 LD/EX/ST breakdown(위 "무엇이 공정한 비교인가" 참고) 이다.
