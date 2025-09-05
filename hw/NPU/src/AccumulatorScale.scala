
package gemmini

import chisel3._
import chisel3.util._
import Util._

class AccumulatorReadRespWithFullData[T <: Data: Arithmetic, U <: Data](fullDataType: Vec[Vec[T]], scale_t: U)
  extends Bundle {
  val resp = new AccumulatorReadResp(fullDataType, scale_t)
  val full_data = fullDataType.cloneType
}

class AccumulatorScaleResp[T <: Data: Arithmetic](fullDataType: Vec[Vec[T]], rDataType: Vec[Vec[T]]) extends Bundle {
  val full_data = fullDataType.cloneType
  val data = rDataType.cloneType
  val acc_bank_id = UInt(2.W)
  val fromDMA = Bool()
}

class AccumulatorScaleIO[T <: Data: Arithmetic, U <: Data](
  fullDataType: Vec[Vec[T]], scale_t: U,
  rDataType: Vec[Vec[T]]
) extends Bundle {
  val in = Flipped(Decoupled(new NormalizedOutput[T,U](fullDataType, scale_t)))
  val out = Decoupled(new AccumulatorScaleResp[T](fullDataType, rDataType))
}

class AccScaleDataWithIndex[T <: Data: Arithmetic, U <: Data](t: T, u: U) extends Bundle {
  val scale = u.cloneType
  val act = UInt(Activation.bitwidth.W)
  val igelu_qb = t.cloneType
  val igelu_qc = t.cloneType
  val iexp_qln2 = t.cloneType
  val iexp_qln2_inv = t.cloneType
  val mean = t.cloneType
  val max = t.cloneType
  val inv_stddev = u.cloneType
  val inv_sum_exp = u.cloneType
  val data = t.cloneType
  val full_data = t.cloneType
  val id = UInt(2.W) // TODO hardcoded
  val index = UInt()
}

class AccScalePipe[T <: Data, U <: Data](t: T, rDataType: Vec[Vec[T]], scale_func: (T, U) => T, scale_t: U,
                                         latency: Int, has_nonlinear_activations: Boolean, has_normalizations: Boolean)
                                        (implicit ev: Arithmetic[T]) extends Module {
  val u = scale_t
  val io = IO(new Bundle {
    val in = Input(Valid(new AccScaleDataWithIndex(t, u)(ev)))
    val out = Output(Valid(new AccScaleDataWithIndex(t, u)(ev)))
  })
  import ev._
  val out = WireInit(io.in)

  val e = io.in.bits.data

  val act = io.in.bits.act
  // make sure no normalizations gets passed in if no functional units present
  assert(has_normalizations.B || (!io.in.fire) ||
    (act =/= Activation.LAYERNORM && act =/= Activation.SOFTMAX && act =/= Activation.IGELU))

  val e_act = MuxCase(e, Seq(
    (has_nonlinear_activations.B && act === Activation.RELU) -> e.relu,
    (has_nonlinear_activations.B && has_normalizations.B && act === Activation.LAYERNORM) ->
      (e - io.in.bits.mean),
    (has_nonlinear_activations.B && has_normalizations.B && act === Activation.IGELU) ->
      AccumulatorScale.igelu(e, io.in.bits.igelu_qb, io.in.bits.igelu_qc),
    (has_nonlinear_activations.B && has_normalizations.B && act === Activation.SOFTMAX) ->
      AccumulatorScale.iexp(e - io.in.bits.max, io.in.bits.iexp_qln2, io.in.bits.iexp_qln2_inv, io.in.bits.igelu_qb, io.in.bits.igelu_qc),
  ))

  val e_scaled = scale_func(e_act, MuxCase(io.in.bits.scale, Seq(
    (has_nonlinear_activations.B && has_normalizations.B && act === Activation.LAYERNORM) ->
      io.in.bits.inv_stddev,
    (has_nonlinear_activations.B && has_normalizations.B && act === Activation.SOFTMAX) ->
      io.in.bits.inv_sum_exp.asTypeOf(scale_t)
  )).asTypeOf(scale_t))

  val e_clipped = e_scaled.clippedToWidthOf(rDataType.head.head)

  out.bits.data := e_clipped
  io.out := Pipe(out, latency)
}

// class AccScalePipe[T <: Data, U <: Data](
//   t: T,
//   rDataType: Vec[Vec[T]],
//   scale_func: (T, U) => T,
//   scale_t: U,
//   latency: Int,
//   has_nonlinear_activations: Boolean,
//   has_normalizations: Boolean
// )(implicit ev: Arithmetic[T]) extends Module {
//   import ev._
//   val io = IO(new Bundle {
//     val in  = Input(Valid(new AccScaleDataWithIndex(t, scale_t)(ev)))
//     val out = Output(Valid(new AccScaleDataWithIndex(t, scale_t)(ev)))
//   })

//   // S1: activation + scale 선택
//   val s1w = Wire(Valid(new AccScaleDataWithIndex(t, scale_t)(ev)))
//   s1w.valid := io.in.valid
//   s1w.bits  := io.in.bits

//   val act = io.in.bits.act
//   assert(has_normalizations.B || (!io.in.valid) ||
//     (act =/= Activation.LAYERNORM && act =/= Activation.SOFTMAX && act =/= Activation.IGELU))

//   val e0 = io.in.bits.data
//   val e_act = MuxCase(e0, Seq(
//     (has_nonlinear_activations.B && act === Activation.RELU)      -> e0.relu,
//     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.LAYERNORM) ->
//       (e0 - io.in.bits.mean),
//     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.IGELU) ->
//       AccumulatorScale.igelu(e0, io.in.bits.igelu_qb, io.in.bits.igelu_qc),
//     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.SOFTMAX) ->
//       AccumulatorScale.iexp(e0 - io.in.bits.max, io.in.bits.iexp_qln2, io.in.bits.iexp_qln2_inv, io.in.bits.igelu_qb, io.in.bits.igelu_qc)
//   ))

//   val scale_sel = MuxCase(io.in.bits.scale, Seq(
//     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.LAYERNORM) -> io.in.bits.inv_stddev,
//     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.SOFTMAX)   -> io.in.bits.inv_sum_exp.asTypeOf(scale_t)
//   )).asTypeOf(scale_t)

//   s1w.bits.data  := e_act
//   s1w.bits.scale := scale_sel
//   val s1 = Pipe(s1w, 1)  // 1단

//   // S2a: 곱의 피연산자 레지스터
//   val s2aw = Wire(Valid(new AccScaleDataWithIndex(t, scale_t)(ev)))
//   s2aw.valid := s1.valid
//   s2aw.bits  := s1.bits
//   val s2a = Pipe(s2aw, 1) // 1단

//   // S3: 스케일 곱/라운딩 결과를 바로 레지스터에
//   val e_scaled = scale_func(s2a.bits.data, s2a.bits.scale)
//   val s3w = Wire(Valid(new AccScaleDataWithIndex(t, scale_t)(ev)))
//   s3w.valid := s2a.valid
//   s3w.bits  := s2a.bits
//   s3w.bits.data := e_scaled
//   val s3 = Pipe(s3w, 1) // 1단

//   // S4: clip만 별도
//   val e_clipped = s3.bits.data.clippedToWidthOf(rDataType.head.head)
//   val s4w = Wire(Valid(new AccScaleDataWithIndex(t, scale_t)(ev)))
//   s4w.valid := s3.valid
//   s4w.bits  := s3.bits
//   s4w.bits.data := e_clipped
//   val s4 = Pipe(s4w, 1) // 1단

//   // 정렬용 추가 latency
//   io.out := Pipe(s4, latency)
// }



class AccumulatorScale[T <: Data, U <: Data](
  fullDataType: Vec[Vec[T]], rDataType: Vec[Vec[T]],
  scale_t: U,
  read_small_data: Boolean, read_full_data: Boolean,
  scale_func: (T, U) => T,
  num_scale_units: Int,
  latency: Int,
  has_nonlinear_activations: Boolean, has_normalizations: Boolean)(implicit ev: Arithmetic[T]) extends Module {

  import ev._

  val io = IO(new AccumulatorScaleIO[T,U](
    fullDataType, scale_t, rDataType
  )(ev))
  val t = io.in.bits.acc_read_resp.data(0)(0).cloneType
  val acc_read_data = io.in.bits.acc_read_resp.data
  val out = Wire(Decoupled(new AccumulatorScaleResp[T](
    fullDataType, rDataType)(ev)))

  if (num_scale_units == -1) {
    val data = io.in.bits.acc_read_resp.data
    val act = io.in.bits.acc_read_resp.act
    val igelu_qb = io.in.bits.acc_read_resp.igelu_qb
    val igelu_qc = io.in.bits.acc_read_resp.igelu_qc
    val iexp_qln2 = io.in.bits.acc_read_resp.iexp_qln2
    val iexp_qln2_inv = io.in.bits.acc_read_resp.iexp_qln2_inv
    val scale = io.in.bits.acc_read_resp.scale

    // val activated_data = VecInit(data.map(v => VecInit(v.map { e =>
    //   val e_act = MuxCase(e, Seq(
    //     (has_nonlinear_activations.B && act === Activation.RELU) -> e.relu,
    //     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.LAYERNORM) ->
    //       (e - io.in.bits.mean),
    //     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.IGELU) ->
    //       AccumulatorScale.igelu(e, igelu_qb, igelu_qc),
    //     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.SOFTMAX) ->
    //       AccumulatorScale.iexp(e - io.in.bits.max, iexp_qln2, iexp_qln2_inv, igelu_qb, igelu_qc),
    //   ))

    //   val e_scaled = scale_func(e_act, MuxCase(scale, Seq(
    //     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.LAYERNORM) ->
    //       io.in.bits.inv_stddev,
    //     (has_nonlinear_activations.B && has_normalizations.B && act === Activation.SOFTMAX) ->
    //       io.in.bits.inv_sum_exp.asTypeOf(scale_t)
    //   )).asTypeOf(scale_t))

    //   val e_clipped = e_scaled.clippedToWidthOf(rDataType.head.head)

    //   e_clipped
    // })))

    // val in = Wire(Decoupled(new AccumulatorReadRespWithFullData(fullDataType, scale_t)(ev)))
    // in.valid := io.in.valid
    // io.in.ready := in.ready
    // in.bits.resp := io.in.bits.acc_read_resp
    // in.bits.full_data := acc_read_data
    // in.bits.resp.data := activated_data

    // val pipe_out = Pipeline(in, latency)

    // out.valid := pipe_out.valid
    // pipe_out.ready := out.ready
    // out.bits.full_data := pipe_out.bits.full_data
    // out.bits.data      := pipe_out.bits.resp.data
    // out.bits.fromDMA   := pipe_out.bits.resp.fromDMA
    // out.bits.acc_bank_id := pipe_out.bits.resp.acc_bank_id
    
    // S0: 그대로
    val s0 = Pipeline(io.in, 1) // s0: Decoupled[NormalizedOutput]

    // S1/S1'에서 사용할 로컬 payload: resp + full_data + inv_* 를 함께 운반
    class ActStagePayload[T2 <: Data : Arithmetic, U2 <: Data](fullDataType2: Vec[Vec[T2]], scale_t2: U2) extends Bundle {
      val resp       = new AccumulatorReadResp(fullDataType2, scale_t2) // data/act/scale/igelu/iexp 등 포함
      val full_data  = fullDataType2.cloneType                           // 원본 full 데이터 유지
      val inv_stddev = scale_t2.cloneType
      val inv_sum_exp= scale_t2.cloneType
    }

    // ---------- S1: activation만 적용 ----------
    val s1 = Wire(Decoupled(new ActStagePayload(fullDataType, scale_t)(ev)))
    s1.valid := s0.valid
    s0.ready := s1.ready

    val s0_resp         = s0.bits.acc_read_resp
    val s0_data         = s0_resp.data
    val s0_act          = s0_resp.act
    val s0_igelu_qb     = s0_resp.igelu_qb
    val s0_igelu_qc     = s0_resp.igelu_qc
    val s0_iexp_qln2    = s0_resp.iexp_qln2
    val s0_iexp_qln2inv = s0_resp.iexp_qln2_inv
    val s0_mean         = s0.bits.mean
    val s0_max          = s0.bits.max

    val activated_data = VecInit(s0_data.map { row =>
      VecInit(row.map { e =>
        MuxCase(e, Seq(
          (has_nonlinear_activations.B && s0_act === Activation.RELU) ->
            e.relu,
          (has_nonlinear_activations.B && has_normalizations.B && s0_act === Activation.LAYERNORM) ->
            (e - s0_mean),
          (has_nonlinear_activations.B && has_normalizations.B && s0_act === Activation.IGELU) ->
            AccumulatorScale.igelu(e, s0_igelu_qb, s0_igelu_qc),
          (has_nonlinear_activations.B && has_normalizations.B && s0_act === Activation.SOFTMAX) ->
            AccumulatorScale.iexp(e - s0_max, s0_iexp_qln2, s0_iexp_qln2inv, s0_igelu_qb, s0_igelu_qc)
        ))
      })
    })

    s1.bits.resp       := s0_resp
    s1.bits.resp.data  := activated_data         // 활성화 결과로 교체
    s1.bits.full_data  := s0_resp.data           // 원본 full-data 유지
    s1.bits.inv_stddev := s0.bits.inv_stddev.asTypeOf(scale_t)
    s1.bits.inv_sum_exp:= s0.bits.inv_sum_exp.asTypeOf(scale_t)

    // ---------- S1' : activation→scale 사이 1단 더 ----------
    val s1p = Pipeline(s1, 1) // s1p: Decoupled[ActStagePayload]

    // ---------- S2: scale + clip ----------
    val in = Wire(Decoupled(new AccumulatorReadRespWithFullData(fullDataType, scale_t)(ev)))
    in.valid := s1p.valid
    s1p.ready := in.ready

    val s1p_resp      = s1p.bits.resp
    val s1p_act       = s1p_resp.act
    val s1p_scale     = s1p_resp.scale.asTypeOf(scale_t)
    val s1p_invStd    = s1p.bits.inv_stddev
    val s1p_invSumExp = s1p.bits.inv_sum_exp

    val scaledClipped = VecInit(s1p_resp.data.map { row =>
      VecInit(row.map { e_act =>
        val scale_sel = MuxCase(s1p_scale, Seq(
          (has_nonlinear_activations.B && has_normalizations.B && s1p_act === Activation.LAYERNORM) -> s1p_invStd,
          (has_nonlinear_activations.B && has_normalizations.B && s1p_act === Activation.SOFTMAX)  -> s1p_invSumExp
        )).asTypeOf(scale_t)
        val e_scaled  = scale_func(e_act, scale_sel)
        e_scaled.clippedToWidthOf(rDataType.head.head)
      })
    })

    in.bits.resp       := s1p_resp
    in.bits.resp.data  := scaledClipped
    in.bits.full_data  := s1p.bits.full_data     // 원본 full-width 데이터 그대로 유지

    // 기존 최종 파이프 유지
    val pipe_out = Pipeline(in, latency)

    out.valid := pipe_out.valid
    pipe_out.ready := out.ready
    out.bits.full_data   := pipe_out.bits.full_data
    out.bits.data        := pipe_out.bits.resp.data
    out.bits.fromDMA     := pipe_out.bits.resp.fromDMA
    out.bits.acc_bank_id := pipe_out.bits.resp.acc_bank_id

  } else {
    val width = acc_read_data.size * acc_read_data(0).size
    val nEntries = 3
    /*val regs = Reg(Vec(nEntries, Valid(new AccumulatorReadResp[T,U](
      fullDataType, scale_t)(ev))))*/
    val regs = Reg(Vec(nEntries, Valid(new NormalizedOutput[T,U](
      fullDataType, scale_t)(ev))))
    val out_regs = Reg(Vec(nEntries, new AccumulatorScaleResp[T](
      fullDataType, rDataType)(ev)))

    val fired_masks = Reg(Vec(nEntries, Vec(width, Bool())))
    val completed_masks = Reg(Vec(nEntries, Vec(width, Bool())))
    val head_oh = RegInit(1.U(nEntries.W))
    val tail_oh = RegInit(1.U(nEntries.W))
    out.valid := Mux1H(head_oh.asBools, (regs zip completed_masks).map({case (r, c) => r.valid && c.reduce(_&&_)}))
    out.bits  := Mux1H(head_oh.asBools, out_regs)
    when (out.fire) {
      for (i <- 0 until nEntries) {
        when (head_oh(i)) {
          regs(i).valid := false.B
        }
      }
      head_oh := (head_oh << 1).asUInt | head_oh(nEntries-1)
    }

    io.in.ready := !Mux1H(tail_oh.asBools, regs.map(_.valid)) || (tail_oh === head_oh && out.fire)
    when (io.in.fire) {
      for (i <- 0 until nEntries) {
        when (tail_oh(i)) {
          regs(i).valid := true.B
          regs(i).bits  := io.in.bits
          out_regs(i).fromDMA := io.in.bits.acc_read_resp.fromDMA
          out_regs(i).acc_bank_id := io.in.bits.acc_read_resp.acc_bank_id
          fired_masks(i).foreach(_ := false.B)
          completed_masks(i).foreach(_ := false.B)
        }
      }
      tail_oh := (tail_oh << 1).asUInt | tail_oh(nEntries-1)
    }

    val num_units_with_norm = 4 // TODO: move to configs

    val inputs_norm = Seq.fill(width*nEntries) { Wire(Decoupled(new AccScaleDataWithIndex(t, scale_t)(ev))) }
    val inputs_non_norm = Seq.fill(width*nEntries) { Wire(Decoupled(new AccScaleDataWithIndex(t, scale_t)(ev))) }

    val norm_mask = regs.map(r => r.valid && (
      (r.bits.acc_read_resp.act === Activation.SOFTMAX) ||
      (r.bits.acc_read_resp.act === Activation.LAYERNORM) ||
      (r.bits.acc_read_resp.act === Activation.IGELU)
    ))

    // input: norm_mask
    // output: {b2, b1, b0} <-> b_i = whether entry i should use functional units with norm (1 = should)
    val static_assignment_policy = Wire(Vec(1 << nEntries, UInt(nEntries.W)))
    println("static policy for " + num_units_with_norm + " norm units:")
    for (i <- 0 until (1 << nEntries)) {
      val binaryString = String.format("%" + nEntries + "s", i.toBinaryString)
        .replace(' ', '0').toCharArray.toList
      val num_norm : Int = binaryString.count(_ == '1')
      val ratio_of_norm_entries = num_norm.toFloat / nEntries.toFloat
      val ratio_of_norm_units = num_units_with_norm.toFloat / num_scale_units.toFloat
      if (ratio_of_norm_entries >= ratio_of_norm_units) {
        // use norm units for all norm entries
        static_assignment_policy(i.U) := i.U
        println("input pattern " + binaryString.mkString("") + ": " + binaryString.mkString(""))
      } else {
        def flip_n_zeros (s: List[Char], n: Int): List[Char] = {
          if (s.nonEmpty) {
            if ((s.head == '0') && (n > 0))
              '1' :: flip_n_zeros(s.tail, n - 1)
            else
              s.head :: flip_n_zeros(s.tail, n)
          } else {
            assert(n == 0, "cannot flip " + n + " zeros in an empty string")
            List.empty
          }
        }
        val flippedString = flip_n_zeros(
          binaryString, Math.round(ratio_of_norm_units * nEntries) - num_norm)
        val flipped = Integer.parseInt(flippedString.mkString(""), 2)
        static_assignment_policy(i.U) := flipped.U
        println("input pattern " + binaryString.mkString("") + ": " + flipped.toBinaryString)
      }
    }

    //    val inputs = Seq.fill(width*nEntries) { Wire(Decoupled(new AccScaleDataWithIndex(t, scale_t)(ev))) }
    val current_policy = Wire(UInt(nEntries.W))
    val norm_mask_int = Wire(UInt(nEntries.W))
    norm_mask_int := VecInit(norm_mask).asUInt
    dontTouch(norm_mask_int)
    current_policy := static_assignment_policy(norm_mask_int)


    for (i <- 0 until nEntries) {
      for (w <- 0 until width) {
        val input = inputs_norm(i*width+w)

        val acc_read_resp = regs(i).bits.acc_read_resp

        input.valid       := regs(i).valid && !fired_masks(i)(w) && /*norm_mask(i)*/ current_policy(i)
        input.bits.data   := acc_read_resp.data(w / acc_read_data(0).size)(w % acc_read_data(0).size)
        input.bits.full_data := acc_read_resp.data(w / acc_read_data(0).size)(w % acc_read_data(0).size)
        input.bits.scale  := acc_read_resp.scale
        input.bits.act    := acc_read_resp.act
        input.bits.igelu_qb := acc_read_resp.igelu_qb
        input.bits.igelu_qc := acc_read_resp.igelu_qc
        input.bits.iexp_qln2 := acc_read_resp.iexp_qln2
        input.bits.iexp_qln2_inv := acc_read_resp.iexp_qln2_inv
        input.bits.mean := regs(i).bits.mean
        input.bits.max := regs(i).bits.max
        input.bits.inv_stddev := regs(i).bits.inv_stddev
        input.bits.inv_sum_exp := regs(i).bits.inv_sum_exp
        input.bits.id := i.U
        input.bits.index := w.U
        when (input.fire) {
          fired_masks(i)(w) := true.B
        }
      }
    }

    for (i <- 0 until nEntries) {
      for (w <- 0 until width) {
        val input = inputs_non_norm(i*width+w)

        val acc_read_resp = regs(i).bits.acc_read_resp

        input.valid       := regs(i).valid && !fired_masks(i)(w) && (!current_policy(i))
        input.bits.data   := acc_read_resp.data(w / acc_read_data(0).size)(w % acc_read_data(0).size)
        input.bits.full_data := acc_read_resp.data(w / acc_read_data(0).size)(w % acc_read_data(0).size)
        input.bits.scale  := acc_read_resp.scale
        input.bits.act    := acc_read_resp.act
        input.bits.igelu_qb := DontCare
        input.bits.igelu_qc := DontCare
        input.bits.iexp_qln2 := DontCare
        input.bits.iexp_qln2_inv := DontCare
        input.bits.mean := DontCare
        input.bits.max := DontCare
        input.bits.inv_stddev := DontCare
        input.bits.inv_sum_exp := DontCare
        input.bits.id := i.U
        input.bits.index := w.U
        if (num_scale_units == num_units_with_norm) {
          input.ready := false.B
        }
        when (input.fire) {
          fired_masks(i)(w) := true.B
        }
      }
    }

    for (i <- 0 until num_scale_units) {
      val norm_supported = (i < num_units_with_norm) && has_normalizations

      val arbIn =
        if (norm_supported)
          // for norm units, prioritize norm operations
          inputs_norm.zipWithIndex.filter({ case (_, w) => w % num_units_with_norm == i }).map(_._1)
        else
          inputs_non_norm.zipWithIndex.filter({ case (_, w) => w % (num_scale_units - num_units_with_norm) == (i - num_units_with_norm) }).map(_._1)

      val arb = Module(new RRArbiter(new AccScaleDataWithIndex(t, scale_t)(ev), arbIn.length))
      arb.io.in <> arbIn
      arb.io.out.ready := true.B
      val arbOut = Reg(Valid(new AccScaleDataWithIndex(t, scale_t)(ev)))
      arbOut.valid := arb.io.out.valid
      arbOut.bits  := arb.io.out.bits
      when (reset.asBool) {
        arbOut.valid := false.B
      }
      val pipe = Module(new AccScalePipe(t, rDataType, scale_func, scale_t, latency,
            has_nonlinear_activations, norm_supported))

      pipe.io.in := arbOut
      val pipe_out = pipe.io.out

      for (j <- 0 until nEntries) {
        for (w <- 0 until width) {
          val id0 = w % acc_read_data(0).size
          val id1 = w / acc_read_data(0).size
            if ((j*width+w) % num_units_with_norm == i) {
              when (pipe_out.fire && pipe_out.bits.id === j.U && pipe_out.bits.index === w.U) {
                out_regs(j).data     (id1)(id0) := pipe_out.bits.data
                out_regs(j).full_data(id1)(id0) := pipe_out.bits.full_data
                completed_masks(j)(w) := true.B
              }
            }
            if (num_scale_units > num_units_with_norm) {
              if ((j*width+w) % (num_scale_units - num_units_with_norm) == (i - num_units_with_norm)) {
                val id0 = w % acc_read_data(0).size
                val id1 = w / acc_read_data(0).size
                when (pipe_out.fire && pipe_out.bits.id === j.U && pipe_out.bits.index === w.U) {
                  out_regs(j).data     (id1)(id0) := pipe_out.bits.data
                  out_regs(j).full_data(id1)(id0) := pipe_out.bits.full_data
                  completed_masks(j)(w) := true.B
                }
              }
            }
        }
      }
    }
    when (reset.asBool) {
      regs.foreach(_.valid := false.B)
    }
  }

  io.out <> out

  if (read_small_data)
    io.out.bits.data := out.bits.data
  else
    io.out.bits.data := DontCare

  if (read_full_data)
    io.out.bits.full_data := out.bits.full_data
  else
    io.out.bits.full_data := DontCare
}

object AccumulatorScale {
  def igelu[T <: Data](q: T, qb: T, qc: T)(implicit ev: Arithmetic[T]): T = {
    import ev._

    val zero = q.zero
    val one = q.identity
    def neg(x: T) = zero-x

    val q_sign = Mux(q.zero > q, neg(one), one)
    val q_abs = Mux(q.zero > q, neg(q), q)
    val q_clipped = Mux(q_abs > neg(qb), neg(qb), q_abs)
    val q_poly = qc.mac(q_clipped + qb, q_clipped + qb).withWidthOf(q)
    val q_erf = (q_sign * q_poly).withWidthOf(q)
    (q * (q_erf + qc)).withWidthOf(q)
  }

  def iexp[T <: Data](q: T, qln2: T, qln2_inv: T, qb: T, qc: T)(implicit ev: Arithmetic[T]): T = {
    import ev._

    val zero = q.zero
    def neg(x: T) = zero-x

    // qln2_inv needs scale to be 1 / (2 ** 16) / S
    // qln2_inv / S / (2 ** 16) = 1 / ln2
    // q * qln2_inv = x / S / ln2 * S * (2 ** 16) = x / ln2 * (2 ** 16)
    val neg_q_iexp = neg(q)
    val z_iexp = (neg_q_iexp * qln2_inv).asUInt.do_>>(16).asTypeOf(q) // q is non-positive
    val z_iexp_saturated = Wire(z_iexp.cloneType)
    z_iexp_saturated := Mux((5 until 16).map(z_iexp.asUInt(_)).reduce(_ | _), 32.S.asTypeOf(z_iexp), z_iexp)
    val qp_iexp = q.mac(z_iexp, qln2).withWidthOf(q)
    val q_poly_iexp = qc.mac(qp_iexp + qb, qp_iexp + qb).withWidthOf(q)
    // we dont want a rounding shift
    //  TODO: z overflow
    (q_poly_iexp.asUInt.do_>>(z_iexp_saturated.asUInt)).asTypeOf(q)
  }}

