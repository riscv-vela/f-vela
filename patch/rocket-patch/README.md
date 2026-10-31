# rocket-chip 벡터 패치 (RocketCore.scala / VectorUnit.scala)

Saturn 벡터 유닛을 Rocket 코어에 붙여 쓰기 위해 수정한 rocket-chip 파일입니다.
이 repo(f-vela)에는 rocket 코드가 포함되어 있지 않으므로, 아래 파일을
**chipyard에 직접 복사**해서 사용하시면 됩니다.

## 적용 방법

이 폴더의 파일을 chipyard의 동일 경로에 그대로 덮어쓰면 됩니다.(백업으로 만들어 둔 뒤 호환 체크해보시기 바랍니다.)

```
generators/rocket-chip/src/main/scala/rocket/RocketCore.scala
generators/rocket-chip/src/main/scala/rocket/VectorUnit.scala
```

예:
```bash
cp generators/rocket-chip/src/main/scala/rocket/RocketCore.scala  <YOUR_CHIPYARD>/generators/rocket-chip/src/main/scala/rocket/
cp generators/rocket-chip/src/main/scala/rocket/VectorUnit.scala  <YOUR_CHIPYARD>/generators/rocket-chip/src/main/scala/rocket/
```


## 변경 요약 (1.13.0 원본 대비)

### `VectorUnit.scala`
- `RocketVectorDecoder` IO 번들에 `val vector = Output(Bool())` 추가.

### `RocketCore.scala`
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

### `tile/FPU.scala` (2026-10-29, `.vf` 명령의 scalar FP 피연산자 버그 수정)
- FPU 안의 벡터 디코더 사본에 현재 SEW를 연결:
  `v_decode.io.vconfig.vtype.vsew := io.v_sew` (원본은 vconfig 전체가 `DontCare` → RTL에서 vsew = 0).
- velaVPU 디코드 테이블은 FP 연산을 SEW e16/e32/e64에서만 매칭합니다(`restrictSEW(1,2,3)`).
  vsew가 0(e8)으로 고정되어 `vfadd.vf`, `vfmul.vf`, `vfmacc.vf` 같은 FP `.vf` 명령이 FPU 디코더에서
  매칭되지 않았고, FPU가 f 레지스터를 읽지 않아 벡터 유닛에 이전 `store_data` 값(대개 0)이 전달됐습니다.
  `vfmv.v.f`처럼 SEW 제한이 없는 명령은 정상이었습니다.
  (ncnn RVV LeNet 오답의 원인: Conv 출력이 ReLU(bias)만 남음. 재현: ncnn_test `rvv_frs1_test`)
- 이 파일을 적용하려면 위 목록에 `generators/rocket-chip/src/main/scala/tile/FPU.scala`도 복사합니다.

<!-- docs/issue/rocekt_core/rv_intr_load_issue.md -->
