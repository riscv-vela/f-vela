
package gemmini

import chisel3._
import chisel3.util._
import gemmini.AccumulatorScale.iexp
import hardfloat.{DivSqrtRecFN_small, INToRecFN, MulRecFN, consts, fNFromRecFN, recFNFromFN}

class NormalizedInput[T <: Data: Arithmetic, U <: Data](max_len: Int, num_stats: Int, fullDataType: Vec[Vec[T]],
                                                        scale_t: U) extends Bundle {
  val acc_read_resp = new AccumulatorReadResp[T,U](fullDataType, scale_t)
  val len = UInt(log2Up(max_len + 1).W)
  val stats_id = UInt(log2Up(num_stats).W)
  val cmd = NormCmd()
}

class NormalizedOutput[T <: Data: Arithmetic, U <: Data](fullDataType: Vec[Vec[T]], scale_t: U) extends Bundle {
  val acc_read_resp = new AccumulatorReadResp[T,U](fullDataType, scale_t)
  val mean = fullDataType.head.head.cloneType
  val max = fullDataType.head.head.cloneType
  val inv_stddev = scale_t.cloneType
  val inv_sum_exp = scale_t.cloneType
}

class IExpConst[T <: Data](acc_t: T) extends Bundle {
  val qb = acc_t.cloneType
  val qc = acc_t.cloneType
  val qln2 = acc_t.cloneType
  val qln2_inv = acc_t.cloneType
}

class AccumulationLanes[T <: Data](num_stats: Int, acc_t: T, n_lanes: Int, latency: Int)(implicit ev: Arithmetic[T])
  extends Module {
  import ev._

  class LaneOutput extends Bundle {
    val result = acc_t.cloneType
    val stats_id = UInt(log2Up(num_stats).W)
  }

  val io = IO(new Bundle {
    val ins = Flipped(Valid(new Bundle {
      val len = UInt(log2Up(n_lanes+1).W)
      val data = Vec(n_lanes, acc_t)
      val mean = acc_t.cloneType
      val max = acc_t.cloneType
      val iexp_const = new IExpConst(acc_t)
      val cmd = NormCmd()
      val stats_id = UInt(log2Up(num_stats).W)
    }))

    val out = Valid(new LaneOutput)

    val busy = Output(Bool())
  })

  // -------------------------
  // S0) 입력/상수 로컬 래치 (팬아웃 절감)
  // -------------------------
  val s0w = Wire(Valid(new Bundle {
    val len = io.ins.bits.len.cloneType
    val data = Vec(n_lanes, acc_t.cloneType)
    val mean = acc_t.cloneType
    val max  = acc_t.cloneType
    val cmd  = NormCmd()
    val stats_id = io.ins.bits.stats_id.cloneType
    val iexp_const = new IExpConst(acc_t)
  }))
  s0w.valid := io.ins.valid
  s0w.bits.len := io.ins.bits.len
  s0w.bits.data := io.ins.bits.data
  s0w.bits.mean := io.ins.bits.mean
  s0w.bits.max  := io.ins.bits.max
  s0w.bits.cmd  := io.ins.bits.cmd
  s0w.bits.stats_id := io.ins.bits.stats_id
  s0w.bits.iexp_const := io.ins.bits.iexp_const
  val s0 = Pipe(s0w, 1) // ★ 1단: 상수/메타 로컬 복제

  // -------------------------
  // S1) 요소별 변환 (iexp, (d-mean)^2, etc.)
  // -------------------------
  val s1w = Wire(Valid(new Bundle {
    val mapped = Vec(n_lanes, acc_t.cloneType)
    val stats_id = io.ins.bits.stats_id.cloneType
  }))
  s1w.valid := s0.valid
  s1w.bits.stats_id := s0.bits.stats_id
  s1w.bits.mapped := VecInit(s0.bits.data.zipWithIndex.map { case (d, i) =>
    val doUse = i.U < s0.bits.len
    val iexp_c = s0.bits.iexp_const
    val iexp_result = iexp(d - s0.bits.max, iexp_c.qln2, iexp_c.qln2_inv, iexp_c.qb, iexp_c.qc)
    val transformed = MuxCase(d, Seq(
      (s0.bits.cmd === NormCmd.VARIANCE || s0.bits.cmd === NormCmd.INV_STDDEV) -> ((d - s0.bits.mean) * (d - s0.bits.mean)).withWidthOf(acc_t),
      (s0.bits.cmd === NormCmd.SUM_EXP || s0.bits.cmd === NormCmd.INV_SUM_EXP) -> iexp_result.withWidthOf(acc_t)
    )).withWidthOf(acc_t)
    Mux(doUse, transformed, d.zero)
  })
  val s1 = Pipe(s1w, 1) // ★ 1단: 무거운 per-lane 연산 뒤를 절단

  // -------------------------
  // S2) 부분합(Adder tree) 1단계
  // -------------------------
  def groupReduce4(xs: Seq[T]): Seq[T] = xs.grouped(4).map(_.reduce(_ + _)).toSeq
  val lvl1 = groupReduce4(s1.bits.mapped)
  val s2w = Wire(Valid(new Bundle {
    val partial = Vec(lvl1.length, acc_t.cloneType)
    val stats_id = io.ins.bits.stats_id.cloneType
  }))
  s2w.valid := s1.valid
  s2w.bits.stats_id := s1.bits.stats_id
  s2w.bits.partial := VecInit(lvl1)
  val s2 = Pipe(s2w, 1) // ★ 1단: adder-tree 중간 절단

  // -------------------------
  // S3) 부분합(Adder tree) 2단계 + 최종
  //    (필요시 한 번 더 groupReduce4 + Pipe 추가하세요)
  // -------------------------
  val lvl2 = groupReduce4(s2.bits.partial)
  val finalSum =
    if (lvl2.length == 1) lvl2.head
    else lvl2.reduce(_ + _)

  // 최종 파이프 (외부 파라미터 latency에서 우리가 넣은 3단 만큼 보정)
  val extraStages = 3 // s0, s1, s2
  val pipe = Module(new Pipeline[LaneOutput](new LaneOutput, math.max(0, latency - extraStages))())

  pipe.io.in.valid := s2.valid
  pipe.io.in.bits.result := finalSum
  pipe.io.in.bits.stats_id := s2.bits.stats_id

  io.out.valid := pipe.io.out.valid
  io.out.bits  := pipe.io.out.bits
  pipe.io.out.ready := true.B

  io.busy := pipe.io.busy
}

class MaxLanes[T <: Data](num_stats: Int, acc_t: T, n_lanes: Int, latency: Int)(implicit ev: Arithmetic[T])
  extends Module {
  import ev._

  class LaneOutput extends Bundle {
    val result = acc_t.cloneType
    val stats_id = UInt(log2Up(num_stats).W)
  }

  val io = IO(new Bundle {
    val ins = Flipped(Valid(new Bundle {
      val len = UInt(log2Up(n_lanes + 1).W)
      val data = Vec(n_lanes, acc_t)
      val stats_id = UInt(log2Up(num_stats).W)
    }))

    val out = Valid(new LaneOutput)

    val busy = Output(Bool())
  })

  // S0) 입력 래치
  val s0w = Wire(Valid(new Bundle {
    val len = io.ins.bits.len.cloneType
    val data = Vec(n_lanes, acc_t.cloneType)
    val stats_id = io.ins.bits.stats_id.cloneType
  }))
  s0w.valid := io.ins.valid
  s0w.bits := io.ins.bits
  val s0 = Pipe(s0w, 1)

  // S1) 마스킹 적용
  val s1w = Wire(Valid(new Bundle {
    val masked = Vec(n_lanes, acc_t.cloneType)
    val stats_id = io.ins.bits.stats_id.cloneType
  }))
  s1w.valid := s0.valid
  s1w.bits.stats_id := s0.bits.stats_id
  s1w.bits.masked := VecInit(s0.bits.data.zipWithIndex.map { case (d, i) =>
    Mux(i.U < s0.bits.len, d.withWidthOf(acc_t), d.minimum)
  })
  val s1 = Pipe(s1w, 1)

  // S2) 부분-최대 (group-of-4) + 파이프
  def groupMax4(xs: Seq[T]): Seq[T] = xs.grouped(4).map(_.reduce((a,b) => Mux(a > b, a, b))).toSeq
  val lvl1 = groupMax4(s1.bits.masked)
  val s2w = Wire(Valid(new Bundle {
    val partial = Vec(lvl1.length, acc_t.cloneType)
    val stats_id = io.ins.bits.stats_id.cloneType
  }))
  s2w.valid := s1.valid
  s2w.bits.stats_id := s1.bits.stats_id
  s2w.bits.partial := VecInit(lvl1)
  val s2 = Pipe(s2w, 1)

  // S3) 최종 최대
  val lvl2 = groupMax4(s2.bits.partial)
  val finalMax = if (lvl2.length == 1) lvl2.head else lvl2.reduce((a,b) => Mux(a > b, a, b))

  val extraStages = 3
  val pipe = Module(new Pipeline[LaneOutput](new LaneOutput, math.max(0, latency - extraStages))())
  pipe.io.in.valid := s2.valid
  pipe.io.in.bits.result := finalMax
  pipe.io.in.bits.stats_id := s2.bits.stats_id

  io.out.valid := pipe.io.out.valid
  io.out.bits  := pipe.io.out.bits
  pipe.io.out.ready := true.B

  io.busy := pipe.io.busy
}


// class AccumulationLanes[T <: Data](num_stats: Int, acc_t: T, n_lanes: Int, latency: Int)(implicit ev: Arithmetic[T])
//   extends Module {
//   // Each lane computes a sum, or an error-squared sum

//   import ev._

//   class LaneOutput extends Bundle {
//     val result = acc_t.cloneType
//     val stats_id = UInt(log2Up(num_stats).W)
//   }

//   val io = IO(new Bundle {
//     val ins = Flipped(Valid(new Bundle {
//       val len = UInt(log2Up(n_lanes+1).W)
//       val data = Vec(n_lanes, acc_t)
//       val mean = acc_t.cloneType
//       val max = acc_t.cloneType
//       val iexp_const = new IExpConst(acc_t)
//       val cmd = NormCmd()
//       val stats_id = UInt(log2Up(num_stats).W)
//     }))

//     val out = Valid(new LaneOutput)

//     val busy = Output(Bool())
//   })

//   val cmd = io.ins.bits.cmd
//   val mean = io.ins.bits.mean
//   val iexp_c = io.ins.bits.iexp_const

//   val data = io.ins.bits.data.zipWithIndex.map { case (d, i) =>
//     val iexp_result = iexp(d - io.ins.bits.max, iexp_c.qln2, iexp_c.qln2_inv, iexp_c.qb, iexp_c.qc)
//     Mux(i.U < io.ins.bits.len,
//       MuxCase(d, Seq(
//         (cmd === NormCmd.VARIANCE || cmd === NormCmd.INV_STDDEV) -> (d-mean)*(d-mean),
//         (cmd === NormCmd.SUM_EXP || cmd === NormCmd.INV_SUM_EXP) ->
//           iexp_result //iexp(d - io.ins.bits.max, iexp_c.qln2, iexp_c.qln2_inv, iexp_c.qb, iexp_c.qc)
//       )).withWidthOf(acc_t),
//       d.zero)
//   }

//   val result = data.reduce(_ + _)

//   val pipe = Module(new Pipeline[LaneOutput](new LaneOutput, latency)())

//   pipe.io.in.valid := io.ins.valid
//   // io.ins.ready := pipe.io.in.ready
//   pipe.io.in.bits.result := result
//   pipe.io.in.bits.stats_id := io.ins.bits.stats_id

//   io.out.valid := pipe.io.out.valid
//   pipe.io.out.ready := true.B
//   // pipe.io.out.ready := io.out.ready
//   io.out.bits := pipe.io.out.bits

//   io.busy := pipe.io.busy
// }

// class MaxLanes[T <: Data](num_stats: Int, acc_t: T, n_lanes: Int, latency: Int)(implicit ev: Arithmetic[T])
//   extends Module {
//   // Each lane computes a sum, or an error-squared sum

//   import ev._

//   class LaneOutput extends Bundle {
//     val result = acc_t.cloneType
//     val stats_id = UInt(log2Up(num_stats).W)
//   }

//   val io = IO(new Bundle {
//     val ins = Flipped(Valid(new Bundle {
//       val len = UInt(log2Up(n_lanes + 1).W)
//       val data = Vec(n_lanes, acc_t)
//       val stats_id = UInt(log2Up(num_stats).W)
//     }))

//     val out = Valid(new LaneOutput)

//     val busy = Output(Bool())
//   })

//   val data = io.ins.bits.data.zipWithIndex.map { case (d, i) =>
//     Mux(i.U < io.ins.bits.len, d.withWidthOf(acc_t), d.minimum)
//   }

//   def treeMax(x: Seq[T]): T = {
//     if (x.length == 1) {
//       x.head
//     } else {
//       val a = treeMax(x.slice(0, x.length / 2)) // ayy slice
//       val b = treeMax(x.slice(x.length / 2, x.length))
//       Mux(a > b, a, b)
//     }
//   }

//   val result = treeMax(data)

//   val pipe = Module(new Pipeline[LaneOutput](new LaneOutput, latency)())

//   pipe.io.in.valid := io.ins.valid
//   // io.ins.ready := pipe.io.in.ready
//   pipe.io.in.bits.result := result
//   pipe.io.in.bits.stats_id := io.ins.bits.stats_id

//   io.out.valid := pipe.io.out.valid
//   pipe.io.out.ready := true.B
//   // pipe.io.out.ready := io.out.ready
//   io.out.bits := pipe.io.out.bits

//   io.busy := pipe.io.busy
// }


class IntSqrt(width: Int) extends Module {
  val N = (width + 1) >> 1

  val input = IO(Flipped(Decoupled(UInt(width.W))))
  val output = IO(Decoupled(UInt(N.W)))

  val x           = Reg(UInt(width.W))
  val a           = Reg(UInt(width.W))
  val t           = Wire(UInt(width.W))
  val q           = Reg(UInt(N.W))
  val sign        = Wire(UInt(1.W))
  val busy        = RegInit(false.B)
  val resultValid = RegInit(false.B)
  val counter     = Reg(UInt(log2Ceil(N).W))

  input.ready := ! busy
  output.valid := resultValid
  output.bits := DontCare

  t := Cat(a, x(width - 1, width - 2)) - Cat(q, 1.U(2.W))
  sign := t(width - 1)
  output.bits := q

  when(busy)  {
    when (!resultValid) {
      counter := counter - 1.U
      x := Cat(x(width - 3, 0), 0.U(2.W))
      a := Mux(sign.asBool, Cat(a(width - 3, 0), x(width - 1, width - 2)), t)
      q := Cat(q(N - 2, 0), ~sign)

      when(counter === 0.U) {
        resultValid := true.B
      }
    }

    when(output.ready && resultValid) {
      busy := false.B
      resultValid := false.B
    }
  }.otherwise {
    when(input.valid) {
      val inputBundle = input.deq()
      x := inputBundle
      a := 0.U
      q := 0.U
      busy := true.B
      counter := (N - 1).U
    }
  }
}

class MulPipe[T <: Data, U <: Data](scale_t: U)(implicit ev: Arithmetic[T])
  extends Module {

  val io = IO(new Bundle {
    val ins = Flipped(Decoupled(new Bundle {
      val x = scale_t.cloneType
      val y = scale_t.cloneType
    }))

    val out = Decoupled(scale_t.cloneType)
  })

  scale_t match {
    case Float(expWidth, sigWidth) =>
      val self_rec = recFNFromFN(expWidth, sigWidth, io.ins.bits.x.asUInt)
      val scale_rec = recFNFromFN(expWidth, sigWidth, io.ins.bits.y.asUInt)

      val mul = Module(new MulRecFN(expWidth, sigWidth))

      mul.io.roundingMode := consts.round_near_even
      mul.io.detectTininess := consts.tininess_afterRounding

      mul.io.a := self_rec
      mul.io.b := scale_rec

      val mul_result = fNFromRecFN(expWidth, sigWidth, mul.io.out).asTypeOf(scale_t)

      val pipe = Module(new Pipeline(scale_t.cloneType, 2)())

      pipe.io.in.valid := io.ins.valid
      pipe.io.in.bits := mul_result
      io.ins.ready := pipe.io.in.ready

//      pipe.io.out.ready := io.out.ready
//      io.out.bits := pipe.io.out.bits
//      io.out.valid := pipe.io.out.valid
      io.out <> pipe.io.out
  }
}

class Normalizer[T <: Data, U <: Data](max_len: Int, num_reduce_lanes: Int, num_stats: Int, latency: Int,
                                       fullDataType: Vec[Vec[T]], scale_t: U)
                                      (implicit ev: Arithmetic[T]) extends Module {
  import ev._
  val acc_t = fullDataType.head.head.cloneType
  val vec_size = fullDataType.flatten.size
  val n_lanes = if (num_reduce_lanes < 0) vec_size else num_reduce_lanes

  assert(isPow2(n_lanes))

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new NormalizedInput[T,U](max_len, num_stats, fullDataType, scale_t)))
    val out = Decoupled(new NormalizedOutput(fullDataType, scale_t))
  })

  object State extends ChiselEnum {
    // NOTE: We assume that "idle" and "output" are the first two states. We also assume that all the enums on the same
    //   line keep the order below
    val idle, output = Value
    val get_sum = Value
    val get_mean, waiting_for_mean = Value
    val get_variance, waiting_for_variance, get_stddev, waiting_for_stddev = Value
    val get_inv_stddev, waiting_for_inv_stddev = Value
    val get_scaled_inv_stddev, waiting_for_scaled_inv_stddev = Value
    val get_max = Value
    val get_inv_sum_exp, waiting_for_inv_sum_exp = Value
    val get_scaled_inv_sum_exp, waiting_for_scaled_inv_sum_exp = Value
  }
  import State._

  // Buffers for normalization stats
  class Stats extends Bundle {
    val req = new NormalizedInput[T,U](max_len, num_stats, fullDataType, scale_t)
    val state = State()

    // Running state
    val sum = acc_t.cloneType
    val count = UInt(16.W) // TODO magic number
    val running_max = acc_t.cloneType
    val max = acc_t.cloneType

    // Iterative state
    val mean = acc_t.cloneType
    val inv_stddev = acc_t.cloneType
    val inv_sum_exp = acc_t.cloneType

    val elems_left = req.len.cloneType

    def vec_grouped = VecInit(req.acc_read_resp.data.flatten.grouped(n_lanes).map(v => VecInit(v)).toSeq)
    def vec_groups_left = elems_left / n_lanes.U + (elems_left % n_lanes.U =/= 0.U)

    def cmd = req.cmd

    def waiting_for_lanes_to_drain =
      (cmd === NormCmd.MEAN && (state === get_sum || state === get_mean)) ||
        (cmd === NormCmd.INV_STDDEV && (state === get_sum || state === get_variance)) ||
        (cmd === NormCmd.MAX && (state === get_max)) ||
        (cmd === NormCmd.INV_SUM_EXP && (state === get_sum))
  }

  val stats = Reg(Vec(num_stats, new Stats))
  val done_with_functional_units = Wire(Vec(num_stats, Bool()))
  val next_states = Wire(Vec(num_stats, State()))

  (stats.map(_.state) zip next_states).foreach { case (s, ns) => s := ns }

  // IO
  val in_stats_id = io.in.bits.stats_id
  io.in.ready := (stats(in_stats_id).state === idle || done_with_functional_units(in_stats_id)) &&
    stats.map(!_.waiting_for_lanes_to_drain).reduce(_ && _)

  val out_stats_id = MuxCase((num_stats-1).U,
    stats.zipWithIndex.map { case (s,i) => (s.state === output) -> i.U }
  )

  io.out.valid := stats(out_stats_id).state === output
  io.out.bits.acc_read_resp := stats(out_stats_id).req.acc_read_resp

  io.out.bits.mean := stats(out_stats_id).mean
  io.out.bits.max := stats(out_stats_id).max
  io.out.bits.inv_stddev := stats(out_stats_id).inv_stddev.asTypeOf(scale_t)
  io.out.bits.inv_sum_exp := stats(out_stats_id).inv_sum_exp.asTypeOf(scale_t)
/////////////////////////////////////////////////////////////////////////

  // // 1) one-hot 선택 (콤비)
  // val out_sel_oh   = VecInit(stats.map(_.state === output))
  // val out_valid_pre = out_sel_oh.asUInt.orR
  // val out_id_pre    = OHToUInt(out_sel_oh)

  // val out_bits_pre = Wire(new NormalizedOutput(fullDataType, scale_t))
  // out_bits_pre.acc_read_resp := Mux1H(out_sel_oh, stats.map(_.req.acc_read_resp))
  // out_bits_pre.mean          := Mux1H(out_sel_oh, stats.map(_.mean))
  // out_bits_pre.max           := Mux1H(out_sel_oh, stats.map(_.max))
  // out_bits_pre.inv_stddev    := Mux1H(out_sel_oh, stats.map(_.inv_stddev)).asTypeOf(scale_t)
  // out_bits_pre.inv_sum_exp   := Mux1H(out_sel_oh, stats.map(_.inv_sum_exp)).asTypeOf(scale_t)

  // // 2) 데이터 + id를 한 번에 담는 Bundle
  // class OutPack extends Bundle {
  //   val bits = new NormalizedOutput(fullDataType, scale_t)
  //   val id   = UInt(log2Ceil(num_stats).W)
  // }

  // // 3) 1-딥 fall-through(=flow) + pipe 큐
  // val out_q = Module(new Queue(new OutPack, 1, flow = true, pipe = true))

  // // enq
  // out_q.io.enq.valid      := out_valid_pre
  // out_q.io.enq.bits.bits  := out_bits_pre
  // out_q.io.enq.bits.id    := out_id_pre

  // // deq → 외부 io.out
  // io.out.valid            := out_q.io.deq.valid
  // io.out.bits             := out_q.io.deq.bits.bits
  // out_q.io.deq.ready      := io.out.ready

  // // FSM 완료 판정용 id
  // val out_id_inflight     = out_q.io.deq.bits.id
  // // when (io.out.fire) { ... out_id_inflight 사용 ... }


//////////////////////////////////////////////////////////////////////////////

  // Lanes and functional units
  val lanes = Module(new AccumulationLanes(num_stats, acc_t, n_lanes, latency))
  val max_lanes = Module(new MaxLanes(num_stats, acc_t, n_lanes, latency)) // TODO: change latency?

  {
    // Lanes input
    val in_lanes_stats_id = MuxCase((num_stats-1).U,
      stats.zipWithIndex.map { case (s,i) => (s.state === get_sum) -> i.U }
    )

    val stat = stats(in_lanes_stats_id)

    val len = Mux(stat.elems_left % n_lanes.U === 0.U, n_lanes.U, stat.elems_left % n_lanes.U)

    lanes.io.ins.valid := stat.state === get_sum && stat.vec_groups_left > 0.U &&
      !max_lanes.io.busy // TODO We should be able to start the accumulation lanes if the max-lanes are busy with a different stat-id
    lanes.io.ins.bits.data := stat.vec_grouped(stat.vec_groups_left-1.U)
    lanes.io.ins.bits.mean := stat.mean
    lanes.io.ins.bits.max := stat.max

    val iexp_const = Wire(new IExpConst(acc_t))
    iexp_const.qln2 := io.in.bits.acc_read_resp.iexp_qln2.asTypeOf(iexp_const.qln2)
    iexp_const.qln2_inv := io.in.bits.acc_read_resp.iexp_qln2_inv.asTypeOf(iexp_const.qln2_inv)
    iexp_const.qb := io.in.bits.acc_read_resp.igelu_qb.asTypeOf(iexp_const.qb)
    iexp_const.qc := io.in.bits.acc_read_resp.igelu_qc.asTypeOf(iexp_const.qc)

    lanes.io.ins.bits.cmd := stat.cmd
    lanes.io.ins.bits.len := len
    lanes.io.ins.bits.stats_id := in_lanes_stats_id
    lanes.io.ins.bits.iexp_const := iexp_const

    when (lanes.io.ins.fire) {
      stat.elems_left := stat.elems_left - len
    }
  }

  {
    // Lanes output
    val out_lanes_stats_id = lanes.io.out.bits.stats_id

    val stat = stats(out_lanes_stats_id)

    when (lanes.io.out.fire) {
      stat.sum := stat.sum + lanes.io.out.bits.result
    }
  }

  {
    // Max lanes input
    val max_in_lanes_stats_id = MuxCase((num_stats-1).U,
      stats.zipWithIndex.map { case (s,i) => (s.state === get_max) -> i.U }
    )

    val stat = stats(max_in_lanes_stats_id)

    val len = Mux(stat.elems_left % n_lanes.U === 0.U, n_lanes.U, stat.elems_left % n_lanes.U)

    max_lanes.io.ins.valid := stat.state === get_max && stat.vec_groups_left > 0.U
    max_lanes.io.ins.bits.data := stat.vec_grouped(stat.vec_groups_left-1.U)
    max_lanes.io.ins.bits.len := len
    max_lanes.io.ins.bits.stats_id := max_in_lanes_stats_id

    when (max_lanes.io.ins.fire) {
      stat.elems_left := stat.elems_left - len
    }
  }

  {
    // Max lanes output
    val max_out_lanes_stats_id = max_lanes.io.out.bits.stats_id

    val stat = stats(max_out_lanes_stats_id)

    when (max_lanes.io.out.fire) {
      val new_max = Mux(max_lanes.io.out.bits.result > stat.running_max, max_lanes.io.out.bits.result, stat.running_max)
      stat.running_max := new_max
      stat.max := new_max
    }
  }

  val sum_to_divide_id = MuxCase((num_stats-1).U,
    stats.zipWithIndex.map { case (s,i) =>
      (s.state === get_mean || s.state === get_variance) -> i.U }
  )
  val sum_to_divide = stats(sum_to_divide_id).sum
  val (divider_in, divider_out) = sum_to_divide.divider(stats.head.count, 16).get

  {
    // Divider input
    val stat = stats(sum_to_divide_id)

    divider_in.valid := (stat.state === get_mean || stat.state === get_variance) && !lanes.io.busy
    divider_in.bits := stat.count
  }

  {
    // Divider output
    val waiting_for_divide_id = MuxCase((num_stats-1).U,
      stats.zipWithIndex.map { case (s,i) =>
        (s.state === waiting_for_mean || s.state === waiting_for_variance) -> i.U }
    )
    val stat = stats(waiting_for_divide_id)

    divider_out.ready := stat.state === waiting_for_mean || stat.state === waiting_for_variance

    when(stat.state === waiting_for_mean) {
      stat.mean := divider_out.bits
    }.elsewhen(stat.state === waiting_for_variance) {
      stat.inv_stddev := divider_out.bits
    }
  }

  val variance_to_sqrt_id = MuxCase((num_stats-1).U,
    stats.zipWithIndex.map { case (s,i) =>
      (s.state === get_stddev) -> i.U }
  )
  val variance_to_sqrt = stats(variance_to_sqrt_id).inv_stddev

  val sqrt_unit = Module(new IntSqrt(acc_t.getWidth))
  val sqrt_in = sqrt_unit.input
  val sqrt_out = sqrt_unit.output

//  val (sqrt_in, sqrt_out) = variance_to_sqrt.sqrt.get

  {
    // Sqrt input
    val stat = stats(variance_to_sqrt_id)

    sqrt_in.bits := variance_to_sqrt.asUInt
    sqrt_in.valid := stat.state === get_stddev
  }

  {
    // Sqrt output
    val waiting_for_sqrt_id = MuxCase((num_stats-1).U,
      stats.zipWithIndex.map { case (s,i) =>
        (s.state === waiting_for_stddev) -> i.U }
    )
    val stat = stats(waiting_for_sqrt_id)

    sqrt_out.ready := stat.state === waiting_for_stddev

    // TODO this fallback for stddev === 0 only works if acc_t is an SInt
    assert(acc_t.isInstanceOf[SInt])

    when (stat.state === waiting_for_stddev) {
      stat.inv_stddev := Mux(sqrt_out.bits.asUInt === acc_t.zero.asUInt,
        1.S(acc_t.getWidth.W).asTypeOf(acc_t),
        sqrt_out.bits.asTypeOf(acc_t)
      )
    }
  }

  val stddev_to_inv_id = MuxCase((num_stats-1).U,
    stats.zipWithIndex.map { case (s,i) =>
      (s.state === get_inv_stddev) -> i.U }
  )
  val stddev_to_inv = stats(stddev_to_inv_id).inv_stddev
  val (reciprocal_in, reciprocal_out) = stddev_to_inv.reciprocal(scale_t, 16).get

  {
    // Reciprocal input
    val stat = stats(stddev_to_inv_id)

    reciprocal_in.valid := stat.state === get_inv_stddev
    reciprocal_in.bits := DontCare
  }

  {
    // Reciprocal output
    val waiting_for_reciprocal_id = MuxCase((num_stats-1).U,
      stats.zipWithIndex.map { case (s,i) =>
        (s.state === waiting_for_inv_stddev) -> i.U }
    )
    val stat = stats(waiting_for_reciprocal_id)

    reciprocal_out.ready := stat.state === waiting_for_inv_stddev

    when (stat.state === waiting_for_inv_stddev) {
      stat.inv_stddev := reciprocal_out.bits.asTypeOf(stat.inv_stddev)
    }
  }

  val inv_stddev_to_scale_id = MuxCase((num_stats-1).U,
    stats.zipWithIndex.map { case (s,i) =>
      (s.state === get_scaled_inv_stddev) -> i.U }
  )

  val inv_stddev_scale_mul_pipe = Module(new MulPipe(scale_t))

  {
    // Scale input
    val stat = stats(inv_stddev_to_scale_id)

    val ins = inv_stddev_scale_mul_pipe.io.ins
    ins.bits.x := stats(inv_stddev_to_scale_id).inv_stddev.asTypeOf(scale_t)
    ins.bits.y := stats(inv_stddev_to_scale_id).req.acc_read_resp.scale
    ins.valid := stat.state === get_scaled_inv_stddev
  }

  {
    // Scale output
    val waiting_for_scale_id = MuxCase((num_stats-1).U,
      stats.zipWithIndex.map { case (s,i) =>
        (s.state === waiting_for_scaled_inv_stddev) -> i.U }
    )
    val stat = stats(waiting_for_scale_id)

    val out = inv_stddev_scale_mul_pipe.io.out
    out.ready := stat.state === waiting_for_scaled_inv_stddev

    when (stat.state === waiting_for_scaled_inv_stddev) {
      stat.inv_stddev := out.bits.asTypeOf(stat.inv_stddev)
    }
  }


  val sum_exp_to_inv_id = MuxCase((num_stats-1).U,
    stats.zipWithIndex.map { case (s,i) =>
      (s.state === get_inv_sum_exp) -> i.U }
  )
  val sum_exp_to_inv = stats(sum_exp_to_inv_id).sum
  val exp_divider_in = Wire(Decoupled(UInt(0.W)))
  val exp_divider_out = Wire(Decoupled(scale_t.cloneType))

  scale_t match {
    case Float(expWidth, sigWidth) =>

      exp_divider_in.bits := DontCare

      // We translate our integer to floating-point form so that we can use the hardfloat divider
      def in_to_float(x: SInt) = {
        val in_to_rec_fn = Module(new INToRecFN(intWidth = sum_exp_to_inv.getWidth, expWidth, sigWidth))
        in_to_rec_fn.io.signedIn := true.B
        in_to_rec_fn.io.in := x.asUInt
        in_to_rec_fn.io.roundingMode := consts.round_near_even // consts.round_near_maxMag
        in_to_rec_fn.io.detectTininess := consts.tininess_afterRounding

        in_to_rec_fn.io.out
      }

      val self_rec = in_to_float(sum_exp_to_inv.asUInt.asSInt)
      val one_rec = in_to_float(127.S) // softmax maximum is 127 for signed int8

      // Instantiate the hardloat divider
      val divider = Module(new DivSqrtRecFN_small(expWidth, sigWidth, 16))

      exp_divider_in.ready := divider.io.inReady
      divider.io.inValid := exp_divider_in.valid
      divider.io.sqrtOp := false.B
      divider.io.a := one_rec
      divider.io.b := self_rec
      divider.io.roundingMode := consts.round_near_even
      divider.io.detectTininess := consts.tininess_afterRounding

      exp_divider_out.valid := divider.io.outValid_div
      exp_divider_out.bits := fNFromRecFN(expWidth, sigWidth, divider.io.out).asTypeOf(scale_t)
  }


  {
    // Divider input
    val stat = stats(sum_exp_to_inv_id)

    exp_divider_in.valid := (stat.state === get_inv_sum_exp) && !lanes.io.busy
    exp_divider_in.bits := sum_exp_to_inv.asUInt
  }

  {
    // Divider output
    val waiting_for_divide_id = MuxCase((num_stats-1).U,
      stats.zipWithIndex.map { case (s,i) =>
        (s.state === waiting_for_inv_sum_exp) -> i.U }
    )
    val stat = stats(waiting_for_divide_id)

    exp_divider_out.ready := stat.state === waiting_for_inv_sum_exp

    when (stat.state === waiting_for_inv_sum_exp) {
      stat.inv_sum_exp := exp_divider_out.bits.asTypeOf(stat.inv_sum_exp)
    }
  }

  val inv_sum_exp_to_scale_id = MuxCase((num_stats-1).U,
    stats.zipWithIndex.map { case (s,i) =>
      (s.state === get_scaled_inv_sum_exp) -> i.U }
  )

  val inv_sum_exp_scale_mul_pipe = Module(new MulPipe(scale_t))

  {
    // Scale input
    val stat = stats(inv_sum_exp_to_scale_id)

    val ins = inv_sum_exp_scale_mul_pipe.io.ins
    ins.bits.x := stats(inv_sum_exp_to_scale_id).inv_sum_exp.asTypeOf(scale_t)
    ins.bits.y := stats(inv_sum_exp_to_scale_id).req.acc_read_resp.scale
    ins.valid := stat.state === get_scaled_inv_sum_exp
  }

  {
    // Scale output
    val waiting_for_scale_id = MuxCase((num_stats-1).U,
      stats.zipWithIndex.map { case (s,i) =>
        (s.state === waiting_for_scaled_inv_sum_exp) -> i.U }
    )
    val stat = stats(waiting_for_scale_id)

    val out = inv_sum_exp_scale_mul_pipe.io.out
    out.ready := stat.state === waiting_for_scaled_inv_sum_exp

    when (stat.state === waiting_for_scaled_inv_sum_exp) {
      stat.inv_sum_exp := out.bits.asTypeOf(stat.inv_sum_exp)
    }
  }

  // State transitions
  for (((stat, next_state), id) <- (stats zip next_states).zipWithIndex) {
    val state = stat.state
    val cmd = stat.cmd

    val done = done_with_functional_units(id)

    when (state === idle) {
      // We have a different "when" statement below to support the case where a new row is input into the normalizer
      next_state := idle
      done := DontCare
    }.elsewhen(state === output) {
      next_state := Mux(io.out.fire && out_stats_id === id.U, idle, state)
      done := io.out.fire && out_stats_id === id.U
      // next_state := Mux(io.out.fire && out_id_inflight === id.U, idle, state)
      // done       := io.out.fire && out_id_inflight === id.U
    }.elsewhen(state === get_max) {
      val is_last_lane_input = stat.vec_groups_left === 0.U ||
        (stat.vec_groups_left === 1.U &&
          max_lanes.io.ins.bits.stats_id === id.U &&
          max_lanes.io.ins.fire)

      next_state := Mux(
        is_last_lane_input,
        MuxCase(state, Seq(
          (cmd === NormCmd.MAX) -> idle,
          (cmd === NormCmd.SUM_EXP || cmd === NormCmd.INV_SUM_EXP) -> get_sum
        )),
        state
      )

      done := is_last_lane_input && cmd === NormCmd.MAX
    }.elsewhen(state === get_sum) {
      val is_last_lane_input = stat.vec_groups_left === 0.U ||
        (stat.vec_groups_left === 1.U &&
          lanes.io.ins.bits.stats_id === id.U &&
          lanes.io.ins.fire)

      next_state := Mux(
        is_last_lane_input,
        MuxCase(state, Seq(
          (cmd === NormCmd.SUM || cmd === NormCmd.VARIANCE || cmd === NormCmd.SUM_EXP) -> idle,
          (cmd === NormCmd.MEAN) -> get_mean,
          (cmd === NormCmd.INV_STDDEV) -> get_variance,
          (cmd === NormCmd.INV_SUM_EXP) -> get_inv_sum_exp,
        )),
        state
      )
//      next_state := Mux(cmd === NormCmd.SUM || cmd === NormCmd.VARIANCE,
//        Mux(is_last_lane_input, idle, state),
//        Mux(is_last_lane_input,
//          Mux(cmd === NormCmd.MEAN, get_mean, get_variance),
//          state)
//      )

      done := is_last_lane_input && cmd =/= NormCmd.MEAN && cmd =/= NormCmd.INV_STDDEV && cmd =/= NormCmd.INV_SUM_EXP
    }.elsewhen(state === get_mean || state === get_variance) {
      next_state := Mux(divider_in.fire && sum_to_divide_id === id.U, state.next, state)
      done := false.B
    }.elsewhen(state === waiting_for_mean) {
      next_state := Mux(divider_out.fire, idle, state)
      done := divider_out.fire
    }.elsewhen(state === waiting_for_variance) {
      next_state := Mux(divider_out.fire, get_stddev, state)
      done := false.B
    }.elsewhen(state === get_stddev) {
      next_state := Mux(sqrt_in.fire && variance_to_sqrt_id === id.U, state.next, state)
      done := false.B
    }.elsewhen(state === waiting_for_stddev) {
      next_state := Mux(sqrt_out.fire, state.next, state)
      done := false.B
    }.elsewhen(state === get_inv_stddev) {
      next_state := Mux(reciprocal_in.fire && stddev_to_inv_id === id.U, state.next, state)
      done := false.B
    }.elsewhen(state === waiting_for_inv_stddev) {
      next_state := Mux(reciprocal_out.fire, state.next, state)
      done := false.B
    }.elsewhen(state === get_scaled_inv_stddev) {
      next_state := Mux(inv_stddev_scale_mul_pipe.io.ins.fire && inv_stddev_to_scale_id === id.U, state.next, state)
      done := false.B
    }.elsewhen(state === waiting_for_scaled_inv_stddev) {
      next_state := Mux(inv_stddev_scale_mul_pipe.io.out.fire, idle, state)
      done := inv_stddev_scale_mul_pipe.io.out.fire
    }.elsewhen(state === get_inv_sum_exp) {
      next_state := Mux(exp_divider_in.fire && sum_exp_to_inv_id === id.U, state.next, state)
      done := false.B
    }.elsewhen(state === waiting_for_inv_sum_exp) {
      next_state := Mux(exp_divider_out.fire, state.next, state)
      done := false.B
    }.elsewhen(state === get_scaled_inv_sum_exp) {
      next_state := Mux(inv_sum_exp_scale_mul_pipe.io.ins.fire && inv_sum_exp_to_scale_id === id.U, state.next, state)
      done := false.B
    }.elsewhen(state === waiting_for_scaled_inv_sum_exp) {
      next_state := Mux(inv_sum_exp_scale_mul_pipe.io.out.fire, idle, state)
      done := inv_sum_exp_scale_mul_pipe.io.out.fire
    }.otherwise {
      assert(false.B, "invalid state in Normalizer")
      next_state := DontCare
      done := DontCare
    }

    when (io.in.fire && in_stats_id === id.U) {
      next_state := Mux(io.in.bits.cmd === NormCmd.RESET, output,
        Mux(io.in.bits.cmd === NormCmd.MAX, get_max, get_sum))
    }
  }

  // Update stats variables
  for (((stat, next_state), id) <- (stats zip next_states).zipWithIndex) {
    val state = stat.state

    val reset_running_state =
      state === output ||
        (state === get_mean && next_state =/= get_mean) ||
        (state === get_variance && next_state =/= get_variance)

    val is_input = io.in.fire && in_stats_id === id.U

    when (is_input) {
      stat.req := io.in.bits
      stat.count := stat.count + io.in.bits.len
      stat.elems_left := io.in.bits.len
    }

    when(reset_running_state) {
      stat.sum := acc_t.zero
      stat.count := Mux(is_input, io.in.bits.len, 0.U)
      stat.running_max := acc_t.minimum
    }

//    when (state =/= get_max && next_state === get_max) {
//      stat.running_max := acc_t.minimum
//      stat.max := acc_t.minimum
//    }
  }

  // Assertions
  assert(PopCount(stats.map(s => s.state === waiting_for_mean || s.state === waiting_for_variance)) <= 1.U, "we don't support pipelining the divider/sqrt-unit/inv-unit right now")
  assert(PopCount(stats.map(_.state === waiting_for_stddev)) <= 1.U, "we don't support pipelining the divider/sqrt-unit/inv-unit right now")
  assert(PopCount(stats.map(_.state === waiting_for_inv_stddev)) <= 1.U, "we don't support pipelining the divider/sqrt-unit/inv-unit right now")
//  assert(PopCount(stats.map(_.state === output)) <= 1.U, "multiple outputs at same time")
  assert(acc_t.getWidth == scale_t.getWidth, "we use the same variable to hold both the variance and the inv-stddev, so we need them to see the width")

  // Resets
  when (reset.asBool) {
    stats.foreach(_.state := idle)
    stats.foreach(_.sum := acc_t.zero)
    stats.foreach(_.max := acc_t.minimum)
    stats.foreach(_.running_max := acc_t.minimum)
    stats.foreach(_.count := 0.U)
    stats.foreach(_.inv_sum_exp := acc_t.zero)
  }
}

object Normalizer {
  def apply[T <: Data, U <: Data](is_passthru: Boolean, max_len: Int, num_reduce_lanes: Int, num_stats: Int,
                                  latency: Int, fullDataType: Vec[Vec[T]], scale_t: U)(implicit ev: Arithmetic[T]):
  (DecoupledIO[NormalizedInput[T,U]], DecoupledIO[NormalizedOutput[T,U]]) = {
    if (is_passthru) {
      passthru(max_len = max_len, num_stats = num_stats, fullDataType = fullDataType, scale_t = scale_t)
    } else {
      gen(max_len = max_len, num_reduce_lanes = num_reduce_lanes, num_stats = num_stats, latency = latency,
        fullDataType = fullDataType, scale_t = scale_t)
    }
  }

  def gen[T <: Data, U <: Data](max_len: Int, num_reduce_lanes: Int, num_stats: Int, latency: Int,
                                  fullDataType: Vec[Vec[T]], scale_t: U)(implicit ev: Arithmetic[T]): (DecoupledIO[NormalizedInput[T,U]], DecoupledIO[NormalizedOutput[T,U]]) = {
    val norm_unit_module = Module(new Normalizer(max_len, num_reduce_lanes, num_stats, latency, fullDataType, scale_t))
    (norm_unit_module.io.in, norm_unit_module.io.out)
  }

  def passthru[T <: Data, U <: Data](max_len: Int, num_stats: Int, fullDataType: Vec[Vec[T]], scale_t: U)
                                    (implicit ev: Arithmetic[T]): (DecoupledIO[NormalizedInput[T,U]], DecoupledIO[NormalizedOutput[T,U]]) = {

    val norm_unit_passthru_q = Module(new Queue(new NormalizedInput[T,U](max_len, num_stats, fullDataType, scale_t), 2))
    val norm_unit_passthru_out = Wire(Decoupled(new NormalizedOutput(fullDataType, scale_t)))

    norm_unit_passthru_out.valid := norm_unit_passthru_q.io.deq.valid
    norm_unit_passthru_out.bits.acc_read_resp := norm_unit_passthru_q.io.deq.bits.acc_read_resp
    norm_unit_passthru_out.bits.mean := DontCare
    norm_unit_passthru_out.bits.max := DontCare
    norm_unit_passthru_out.bits.inv_stddev := DontCare
    norm_unit_passthru_out.bits.inv_sum_exp := DontCare

    norm_unit_passthru_q.io.deq.ready := norm_unit_passthru_out.ready

    (norm_unit_passthru_q.io.enq, norm_unit_passthru_out)
  }
}
