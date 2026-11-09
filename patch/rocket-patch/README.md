# rocket-chip 패치

velaVPU(Saturn 기반 벡터 유닛)와 custom PID/RoPE 명령을 Rocket 코어에 붙여 쓰기 위해 수정한
rocket-chip 파일입니다. 이 repo(f-vela)에는 rocket-chip 코드가 없으므로, 이 폴더가 chipyard와 같은
경로 구조로 수정된 파일 전체를 갖고 있습니다.

## 적용 방법

**`script/setup.sh`가 자동으로 적용합니다** (chipyard 1.13.x에서만).

```bash
cd <chipyard>/generators/_f_vela
bash ./script/setup.sh
```

- `patch/rocket-patch/generators/...`의 모든 `.scala`를 chipyard의 같은 경로에 덮어씁니다.
- 덮어쓰기 전에 원본을 `<file>.bak`으로 한 번만 백업합니다 (이미 `.bak`이 있으면 원본 보존을 위해 그대로 둠).
- 파일 전체를 덮어쓰는 방식이라 1.13.x가 아니면 건너뜁니다 (버전은 `CHANGELOG.md`에서 읽음).
- `script/raw_config_build.sh`(stock config 빌드)는 빌드 동안 이 패치를 원본으로 되돌렸다가 끝나면 다시 적용합니다.

적용 대상:

```
generators/rocket-chip/src/main/scala/rocket/RocketCore.scala
generators/rocket-chip/src/main/scala/rocket/VectorUnit.scala
generators/rocket-chip/src/main/scala/rocket/Instructions.scala
generators/rocket-chip/src/main/scala/rocket/Configs.scala
generators/rocket-chip/src/main/scala/tile/FPU.scala
```

이 폴더의 파일을 고친 뒤에는 `setup.sh`를 다시 실행하면 반영됩니다.

되돌리기:
```bash
# 각 <file>.bak 을 <file> 위에 복원, 예:
cp generators/rocket-chip/src/main/scala/rocket/RocketCore.scala.bak \
   generators/rocket-chip/src/main/scala/rocket/RocketCore.scala
# 또는 rocket-chip이 submodule이므로:
git -C generators/rocket-chip checkout src/main/scala/
```

## 변경 요약 (1.13.0 원본 대비)

### `rocket/VectorUnit.scala`
- `RocketVectorDecoder` IO 번들에 `val vector = Output(Bool())` 추가
  ("벡터 명령인지"를 legal/`vconfig`와 무관하게 알려줌).

### `rocket/RocketCore.scala`
- **ID 단계**: `id_ctrl.vec`를 `io.vector`로 구동해서, `vsetvl` 직후 `vconfig`가 확정되기 전의
  벡터 명령도 벡터 명령으로 인식하고 `vconfig` hazard에서 stall 합니다
  (그렇지 않으면 `vsetvl` 다음 첫 벡터 load/연산이 illegal instruction trap).
- **EX 단계**: 명령을 현재 `vconfig` 기준으로 다시 디코드하여 `io.legal`이 참인
  (legal한) 벡터 명령만 벡터 유닛에 발행하도록 `ex_vec_valid` 검증 추가
  (`v.ex.valid := ... && ex_vec_valid`).
- **MEM 단계 (2026-10-28, 인터럽트 + 벡터 trap check 버그 수정)**: `vec_kill_all`에
  `mem_reg_xcpt_interrupt`를 포함
  (`val vec_kill_all = (mem_reg_valid || mem_reg_xcpt_interrupt) && ...block_all`).
  원본은 벡터 유닛이 `block_all` 중일 때 유효한 명령만 replay하고, 인터럽트 슬롯은
  `wb_reg_xcpt`에서 버리기만 해서 fetch가 예측 경로로 앞서 나갔습니다. 그 결과 인터럽트가
  실행되지 않은 뒷 명령의 PC를 epc로 잡아 명령이 건너뛰어졌습니다
  (페이지 경계 벡터 load/store 도중 타이머 인터럽트 → 데이터 손상, ncnn RVV 오류의 원인).
- 그 밖에:
  - ID: `vsetvl`류(`id_set_vconfig`)도 `vector_illegal` 검사 대상에 포함하고, `vill` 검사는 EX 재디코드로 옮김.
  - FPU ↔ 벡터 kill 전달: `io.fpu.killm`에 `vec_kill_mem`, `v.killm`에 `fpu_kill_mem` 추가.
  - WB: 벡터 retire/xcpt 처리를 유효한 WB 슬롯(`wb_reg_valid`)으로 제한.
  - ID stall 조건에 `id_rocc_busy`(RoCC 명령 처리 중) 추가.

### `rocket/Instructions.scala`
- custom PID 명령의 bit pattern 추가: `VFPID64B_VF`, `VFPID32B_VF` (OP-V opcode, funct3 = 101(OPFVF)).

### `rocket/Configs.scala`
- `WithRationalCDCs`가 `direction: RationalDirection = Flexible` 인자를 받도록 변경
  (`RationalCrossing(direction)`), 이를 위해 `RationalDirection`, `Flexible` import 추가.

### `tile/FPU.scala` (2026-10-29, `.vf` 명령의 scalar FP 피연산자 버그 수정)
- FPU 안의 벡터 디코더 사본에 현재 SEW를 연결:
  `v_decode.io.vconfig.vtype.vsew := io.v_sew` (원본은 vconfig 전체가 `DontCare` → RTL에서 vsew = 0).
- velaVPU 디코드 테이블은 FP 연산을 SEW e16/e32/e64에서만 매칭합니다(`restrictSEW(1,2,3)`).
  vsew가 0(e8)으로 고정되어 `vfadd.vf`, `vfmul.vf`, `vfmacc.vf` 같은 FP `.vf` 명령이 FPU 디코더에서
  매칭되지 않았고, FPU가 f 레지스터를 읽지 않아 벡터 유닛에 이전 `store_data` 값(대개 0)이 전달됐습니다.
  `vfmv.v.f`처럼 SEW 제한이 없는 명령은 정상이었습니다.
  (ncnn RVV LeNet 오답의 원인: Conv 출력이 ReLU(bias)만 남음. 재현: ncnn_test `rvv_frs1_test`)

<!-- docs/issue/rocekt_core/rv_intr_load_issue.md -->
