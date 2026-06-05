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