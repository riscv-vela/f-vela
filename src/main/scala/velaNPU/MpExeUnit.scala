package velaNPU

import chisel3._
import chisel3.util._

import velaNPU.Util._

class MpExeUnit[T <: Data](
  inputType: T, weightType: T, outputType: T,
  ma_length: Int, ma_num: Int, max_simultaneous_matmuls: Int
)(implicit ev: Arithmetic[T]) extends Module {
  import ev._

  val io = IO(new Bundle {
    val in_a = Input(Vec(ma_length, inputType))
    val in_b = Input(Vec(ma_num, weightType))   // 그대로 유지 (int2 Vec)
    val in_d = Input(Vec(ma_num, inputType))

    val in_last = Input(Vec(ma_length, Bool()))
    val in_prop = Input(Vec(ma_length, Bool()))
    val in_valid = Input(Vec(ma_length, Bool()))
    val in_id = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))
    val in_fire_counter = Input(UInt(log2Up(ma_length).W))
    val in_b_transpose = Input(Bool())

    // 추가
    val in_is_mpgemm = Input(Bool())
    val out_is_mpgemm = Output(Bool())

    val out_c = Output(Vec(ma_num, outputType))
    val out_last = Output(Vec(ma_num, Bool()))
    val out_id = Output(Vec(ma_num, UInt(log2Up(max_simultaneous_matmuls).W)))
    val out_valid = Output(Vec(ma_num, Bool()))
  })

  val buffadderarray = Seq.fill(ma_num) {
    Module { new Buffadderlight(inputType, outputType, max_simultaneous_matmuls) }
  }
  val buffvectorarray = Seq.fill(ma_length) {
    Module { new Buffvector(inputType, max_simultaneous_matmuls) }
  }

  val mularraybundle = Seq.fill(ma_num - ma_num / 4) {
    Module(new MpMularray(inputType, weightType, outputType, ma_length, max_simultaneous_matmuls))
  }
  val mularraybundle_int8 = Seq.fill(ma_num / 4) {
    Module(new Mularray(inputType, outputType, ma_length, max_simultaneous_matmuls))
  }

  val BVECTYPE  = Vec(4, Vec(ma_length, weightType))
  val BVECTYPE2 = Vec(ma_length, weightType)

  val BVECTYPE_I8  = Vec(4, Vec(ma_length, inputType))
  val BVECTYPE2_I8 = Vec(ma_length, inputType)

  val mpCount = ma_num - ma_num / 4
  val i8Count = ma_num / 4

  // -----------------------------
  // 핵심: 전체 비트를 int8 Vec로 재해석 (is_mpgemm=false에서 사용)
  // ma_num*2 bits == (ma_num/4)*8 bits
  // -----------------------------
  val b_i8_all = io.in_b.asUInt.asTypeOf(Vec(ma_length, inputType))

  // sign-extend: int2 -> int8
  def sext2toInput(x: T): T = {
    val wIn = inputType.getWidth
    val wW  = weightType.getWidth
    val xu  = x.asUInt
    val sign = xu(wW-1)
    Cat(Fill(wIn - wW, sign), xu(wW-1, 0)).asTypeOf(inputType)
  }

  // buffvectorarray 입력
  for (i <- 0 until ma_length) {
    buffvectorarray(i).io.in_a := io.in_a(i)
    buffvectorarray(i).io.in_valid := io.in_valid(i)
    buffvectorarray(i).io.in_last := io.in_last(i)
    buffvectorarray(i).io.in_id := io.in_id(i)
    buffvectorarray(i).io.in_prop := io.in_prop(i)
  }

  // -----------------------------
  // 1) MpMularray (상위 3/4) : is_mpgemm=true일 때만 동작
  // -----------------------------
  for (i <- 0 until mpCount) {
    val laneIdx = i8Count + i
    mularraybundle(i).io.in_a := VecInit(buffvectorarray.map(_.io.out_a))
    val b_fire = RegNext(io.in_valid.head)

    mularraybundle(i).io.in_fire_counter := io.in_fire_counter
    mularraybundle(i).io.in_valid := Mux(io.in_is_mpgemm, VecInit(buffvectorarray.map(_.io.out_valid)), VecInit(Seq.fill(ma_length)(false.B)))
    mularraybundle(i).io.in_last  := Mux(io.in_is_mpgemm, VecInit(buffvectorarray.map(_.io.out_last )), VecInit(Seq.fill(ma_length)(false.B)))
    mularraybundle(i).io.in_id    := Mux(io.in_is_mpgemm, VecInit(buffvectorarray.map(_.io.out_id    )), 0.U.asTypeOf(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W))))
    mularraybundle(i).io.in_prop  := Mux(io.in_is_mpgemm, VecInit(buffvectorarray.map(_.io.out_prop )), VecInit(Seq.fill(ma_length)(false.B)))
    mularraybundle(i).io.in_b_transpose := io.in_b_transpose

    val in_b_sel = Wire(weightType)
    val in_b_fireSel = Wire(Bool())
    val in_b_vec = Wire(BVECTYPE2)

    when (!io.in_b_transpose) {
      in_b_sel := Mux(io.in_is_mpgemm, io.in_b(laneIdx), 0.U.asTypeOf(weightType))
      in_b_fireSel := b_fire && io.in_is_mpgemm
      in_b_vec := 0.U.asTypeOf(BVECTYPE2)
    } .otherwise {
      val sel = io.in_fire_counter === (laneIdx / 4).U
      val b_vec = io.in_b.asTypeOf(BVECTYPE)

      in_b_vec := Mux(sel && io.in_is_mpgemm, b_vec(laneIdx % 4), 0.U.asTypeOf(BVECTYPE2))
      in_b_sel := 0.U.asTypeOf(weightType)
      in_b_fireSel := sel && io.in_is_mpgemm
    }

    mularraybundle(i).io.in_b := in_b_sel
    mularraybundle(i).io.in_b_vec := in_b_vec
    mularraybundle(i).io.in_b_fire := in_b_fireSel
  }

  // -----------------------------
  // 2) int8 Mularray (하위 1/4) :
  //   - is_mpgemm=false: b_i8_all(j) (전체 비트 재해석 결과) 사용
  //   - is_mpgemm=true : io.in_b(laneIdx)의 int2를 sign-extend해서 사용
  // -----------------------------
  for (j <- 0 until i8Count) {
    mularraybundle_int8(j).io.in_a := VecInit(buffvectorarray.map(_.io.out_a))
    val b_fire = RegNext(io.in_valid.head)

    mularraybundle_int8(j).io.in_fire_counter := io.in_fire_counter
    mularraybundle_int8(j).io.in_valid := VecInit(buffvectorarray.map(_.io.out_valid))
    mularraybundle_int8(j).io.in_last  := VecInit(buffvectorarray.map(_.io.out_last))
    mularraybundle_int8(j).io.in_id    := VecInit(buffvectorarray.map(_.io.out_id))
    mularraybundle_int8(j).io.in_prop  := VecInit(buffvectorarray.map(_.io.out_prop))
    mularraybundle_int8(j).io.in_b_transpose := io.in_b_transpose

    val in_b_sel_i8 = Wire(inputType)
    val in_b_fireSel_i8 = Wire(Bool())
    val in_b_vec_i8 = Wire(BVECTYPE2_I8)

    when (!io.in_b_transpose) {
      val as_i8 = b_i8_all(j)
      val sext  = sext2toInput(io.in_b(j))
      in_b_sel_i8 := Mux(io.in_is_mpgemm, sext, as_i8)
      in_b_fireSel_i8 := b_fire
      in_b_vec_i8 := 0.U.asTypeOf(BVECTYPE2_I8)
    } .otherwise {
      // transpose일 때도 기존 sel 규약 유지 (laneIdx 기반)
      val sel = Mux(io.in_is_mpgemm, io.in_fire_counter === (j / 4).U, io.in_fire_counter === j.U)

      // non-mp: raw int8 view에서 laneIdx%4에 해당하는 벡터
      val b_vec_tw = io.in_b.asTypeOf(BVECTYPE)
      val tw_vec = b_vec_tw(j % 4)
      val i8raw = VecInit(tw_vec.map(sext2toInput)).asTypeOf(BVECTYPE2_I8)

      // mp: int2 벡터를 sign-extend해서 int8 벡터 생성
      // val b_vec_tw = io.in_b.asTypeOf(BVECTYPE)
      // val tw2 = b_vec_tw(laneIdx % 4)
      // val i8sext = VecInit(tw2.map(sext2toInput))

      // in_b_vec_i8 := Mux(sel, Mux(io.in_is_mpgemm, i8sext, i8raw), 0.U.asTypeOf(BVECTYPE2_I8))
  
      in_b_vec_i8 := Mux(sel, Mux(io.in_is_mpgemm, i8raw, b_i8_all), 0.U.asTypeOf(BVECTYPE2_I8))
      in_b_sel_i8 := 0.U.asTypeOf(inputType)
      in_b_fireSel_i8 := sel
    }

    mularraybundle_int8(j).io.in_b := in_b_sel_i8
    mularraybundle_int8(j).io.in_b_vec := in_b_vec_i8
    mularraybundle_int8(j).io.in_b_fire := in_b_fireSel_i8
  }

  // -----------------------------
  // 3) adder 연결:
  //   - is_mpgemm=false: 상위 3/4는 무조건 0/invalid, 하위 1/4만 int8 결과
  //   - is_mpgemm=true : 상위 3/4는 mp 결과, 하위 1/4는 int8(sign-extend) 결과
  // -----------------------------
  for (i <- 0 until ma_num) {

    val sumSel   = Wire(outputType)
    val validSel = Wire(Bool())
    val lastSel  = Wire(Bool())
    val idSel    = Wire(UInt(log2Up(max_simultaneous_matmuls).W))

    if (i < i8Count) {
        // 🔹 int8 lane (j는 항상 >= 0)
        sumSel   := mularraybundle_int8(i).io.out_sum
        validSel := mularraybundle_int8(i).io.out_valid
        lastSel  := mularraybundle_int8(i).io.out_last
        idSel    := mularraybundle_int8(i).io.out_id

    } else {
      val k = i - i8Count
        // 🔹 mp lane (j를 아예 계산하지 않음)
        when (io.in_is_mpgemm) {
            sumSel   := mularraybundle(k).io.out_sum
            validSel := mularraybundle(k).io.out_valid
            lastSel  := mularraybundle(k).io.out_last
            idSel    := mularraybundle(k).io.out_id
        } .otherwise {
            sumSel   := 0.U.asTypeOf(outputType)
            validSel := false.B
            lastSel  := false.B
            idSel    := 0.U
        }
    }

    buffadderarray(i).io.in_d := io.in_d(i)
    buffadderarray(i).io.in_result := sumSel
    buffadderarray(i).io.in_valid := validSel
    buffadderarray(i).io.in_last := lastSel
    buffadderarray(i).io.in_id := idSel

    io.out_c(i) := buffadderarray(i).io.out_c
    io.out_valid(i) := buffadderarray(i).io.out_valid
    io.out_last(i) := buffadderarray(i).io.out_last
    io.out_id(i) := buffadderarray(i).io.out_id
    io.out_is_mpgemm := ShiftRegister(io.in_is_mpgemm, 3)
  }
}