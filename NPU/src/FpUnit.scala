package gemmini

import chisel3._
import chisel3.util._
import hardfloat._

// =============================================================================
// Floating-point datapath units for the mixed-precision Gemmini path.
// (See memory: gemmini-mixed-precision-design — build order step 1.)
//
//   Fp16MulUnit : fp16 x fp16 -> fp32 product, emitted in hardfloat RECODED
//                 form (recFN) so the adder tree can stay recoded.
//   FpAdderTree : reduces ma_length recoded fp32 products into one standard
//                 fp32 result using cheap AddRecFN nodes (NO FMA), staying in
//                 recoded form through the whole tree (a single decode at root).
//
// Formats (hardfloat Float(expW, sigW); standard "FN" bits = expW + sigW):
//   fp16 = Float(5,11) : standard 16b ; recoded recFN = expW+sigW+1 = 17b
//   fp32 = Float(8,24) : standard 32b ; recoded recFN = 33b
//
// Why recoded through the tree: hardfloat ops work on recFN. Recoding once at
// the multiplier output and decoding once at the tree root avoids a recode/
// decode pair at every node (the TODO noted in Arithmetic.scala).
// =============================================================================

object FpUnitConsts {
  val f16ExpW = 5;  val f16SigW = 11   // fp16, standard 16b, recFN 17b
  val f32ExpW = 8;  val f32SigW = 24   // fp32, standard 32b, recFN 33b
  def fnW(expW:  Int, sigW: Int): Int = expW + sigW       // standard FN width
  def recW(expW: Int, sigW: Int): Int = expW + sigW + 1   // recoded FN width
}

/** fp16 x fp16 multiply. Output is the product in recoded fp32 (recFN(8,24)).
  * The multiply itself is fp16-precision (cheap 11x11 mantissa); widening to
  * fp32 is a pure reformat (no extra rounding of the value) so the downstream
  * tree accumulates in fp32 — matching the stock Gemmini fp16 config behavior. */
class Fp16MulUnit extends Module {
  import FpUnitConsts._
  val io = IO(new Bundle {
    val in_a = Input(UInt(fnW(f16ExpW, f16SigW).W))    // fp16 standard bits (16b)
    val in_b = Input(UInt(fnW(f16ExpW, f16SigW).W))
    val out  = Output(UInt(recW(f32ExpW, f32SigW).W))  // recFN(8,24) (33b)
  })

  // Recode fp16 inputs: standard FN -> recFN(5,11)
  val a_rec = recFNFromFN(f16ExpW, f16SigW, io.in_a)
  val b_rec = recFNFromFN(f16ExpW, f16SigW, io.in_b)

  // fp16 x fp16 multiply (recFN(5,11) in/out)
  val mul = Module(new MulRecFN(f16ExpW, f16SigW))
  mul.io.a := a_rec
  mul.io.b := b_rec
  mul.io.roundingMode   := consts.round_near_even
  mul.io.detectTininess := consts.tininess_afterRounding

  // Widen recoded product fp16 -> fp32 (recFN(5,11) -> recFN(8,24))
  val widen = Module(new RecFNToRecFN(f16ExpW, f16SigW, f32ExpW, f32SigW))
  widen.io.in := mul.io.out
  widen.io.roundingMode   := consts.round_near_even
  widen.io.detectTininess := consts.tininess_afterRounding

  io.out := widen.io.out                                // recFN(8,24)
}

/** Binary adder tree over ma_length recoded fp32 products. Nodes are AddRecFN
  * (cheap: AddRawFN + RoundRawFNToRecFN, no multiplier). Values stay recoded
  * through the tree; a single fNFromRecFN at the root yields standard fp32. */
class FpAdderTree(ma_length: Int, max_simultaneous_matmuls: Int) extends Module {
  import FpUnitConsts._
  require(ma_length >= 1, "FpAdderTree needs at least one input")

  val io = IO(new Bundle {
    val in       = Input(Vec(ma_length, UInt(recW(f32ExpW, f32SigW).W)))  // recFN(8,24)
    val in_last  = Input(Vec(ma_length, Bool()))
    val in_valid = Input(Vec(ma_length, Bool()))
    val in_id    = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))

    val out       = Output(UInt(fnW(f32ExpW, f32SigW).W))  // standard fp32 (32b)
    val out_last  = Output(Bool())
    val out_valid = Output(Bool())
    val out_id    = Output(UInt(log2Up(max_simultaneous_matmuls).W))
  })

  // Pipeline the tree: one register per binary-tree level. Every element of a
  // level (including a carried-up odd element) is registered, so all paths have
  // the SAME depth and the reduction stays balanced. Added latency = number of
  // levels = log2Up(ma_length). The critical path becomes a single AddRecFN per
  // stage instead of log2Up(ma_length) AddRecFNs in series.
  // NOTE: FpExeUnit must delay in_d by this same treeLatency and bump
  // out_is_mpgemm accordingly (keep the output bundle uniform).
  val treeLatency = log2Up(ma_length)

  def treeAdd(n: Seq[UInt]): UInt = {
    if (n.length == 1) {
      n.head
    } else {
      val next = n.grouped(2).map {
        case Seq(a, b) =>
          val add = Module(new AddRecFN(f32ExpW, f32SigW))
          add.io.subOp          := false.B
          add.io.a              := a
          add.io.b              := b
          add.io.roundingMode   := consts.round_near_even
          add.io.detectTininess := consts.tininess_afterRounding
          add.io.out
        case Seq(a) => a
      }.toSeq
      treeAdd(next.map(RegNext(_)))   // register this level (1 pipeline stage)
    }
  }

  val sumRec = treeAdd(io.in.toSeq)                       // recFN(8,24)
  io.out := fNFromRecFN(f32ExpW, f32SigW, sumRec)         // standard fp32

  // delay control by treeLatency so valid/last/id stay aligned with the data
  io.out_valid := ShiftRegister(io.in_valid.reduce(_ || _), treeLatency)
  io.out_last  := ShiftRegister(io.in_last.reduce(_ || _), treeLatency)
  io.out_id    := ShiftRegister(io.in_id.head, treeLatency)
}

/** fp16 PE: double-buffered weight (mirrors PE.scala), fp16 multiply via
  * Fp16MulUnit. Output is the recoded fp32 product (recFN(8,24)) for FpAdderTree.
  * 1-cycle latency on the product (ShiftRegister), matching the int PE. */
class FpPE(max_simultaneous_matmuls: Int) extends Module {
  import FpUnitConsts._
  val io = IO(new Bundle {
    val in_a = Input(UInt(fnW(f16ExpW, f16SigW).W))         // fp16 (16b)
    val in_b = Input(UInt(fnW(f16ExpW, f16SigW).W))         // fp16 (16b)
    val out_result = Output(UInt(recW(f32ExpW, f32SigW).W)) // recFN(8,24)

    val in_last  = Input(Bool())
    val in_valid = Input(Bool())
    val in_id    = Input(UInt(log2Up(max_simultaneous_matmuls).W))
    val in_prop  = Input(Bool())
    val in_b_fire = Input(Bool())
    // 2-weight (lo/hi) support for the merged 128b fp 2-pass: a logical 16-deep
    // fp contraction is run as 2 passes of 8, so each PE holds TWO weights —
    // `lo` (k) used on pass 0, `hi` (k+DIM/2) on pass 1.
    val in_pass   = Input(Bool())   // multiply select: false=lo, true=hi
    val in_b_pass = Input(Bool())   // weight-load target: false=lo, true=hi

    val out_last  = Output(Bool())
    val out_valid = Output(Bool())
    val out_id    = Output(UInt(log2Up(max_simultaneous_matmuls).W))
  })

  val mul = Module(new Fp16MulUnit)
  mul.io.in_a := Mux(io.in_valid, io.in_a, 0.U)

  // Double-buffered weight (preload into one buffer while multiplying the other);
  // each buffer now holds TWO weights (lo/hi) for the 2-pass contraction.
  // num_passes=1 usage drives in_pass=in_b_pass=false -> only the `lo` regs are
  // ever read/written, i.e. identical to the original single-weight PE.
  val c1_lo = RegInit(0.U(fnW(f16ExpW, f16SigW).W))
  val c1_hi = RegInit(0.U(fnW(f16ExpW, f16SigW).W))
  val c2_lo = RegInit(0.U(fnW(f16ExpW, f16SigW).W))
  val c2_hi = RegInit(0.U(fnW(f16ExpW, f16SigW).W))

  mul.io.in_b := 0.U
  when (io.in_prop) {
    when (io.in_valid)  { mul.io.in_b := Mux(io.in_pass, c2_hi, c2_lo) }
    when (io.in_b_fire) { when (io.in_b_pass) { c1_hi := io.in_b } .otherwise { c1_lo := io.in_b } }
  } .otherwise {
    when (io.in_valid)  { mul.io.in_b := Mux(io.in_pass, c1_hi, c1_lo) }
    when (io.in_b_fire) { when (io.in_b_pass) { c2_hi := io.in_b } .otherwise { c2_lo := io.in_b } }
  }

  io.out_result := ShiftRegister(mul.io.out, 1)
  io.out_valid  := io.in_valid
  io.out_last   := io.in_last
  io.out_id     := io.in_id
}

/** fp16 dot-product lane: ma_length FpPEs + an FpAdderTree (mirrors Mularray).
  * in_a is broadcast activation; in_b is a scalar weight selected by
  * in_fire_counter, or in_b_vec when transposed. out_sum is standard fp32. */
class FpMularray(ma_length: Int, max_simultaneous_matmuls: Int) extends Module {
  import FpUnitConsts._
  val io = IO(new Bundle {
    val in_a     = Input(Vec(ma_length, UInt(fnW(f16ExpW, f16SigW).W)))
    val in_b     = Input(UInt(fnW(f16ExpW, f16SigW).W))
    val in_b_vec = Input(Vec(ma_length, UInt(fnW(f16ExpW, f16SigW).W)))

    val in_last  = Input(Vec(ma_length, Bool()))
    val in_valid = Input(Vec(ma_length, Bool()))
    val in_id    = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))
    val in_prop  = Input(Vec(ma_length, Bool()))
    val in_fire_counter = Input(UInt(log2Up(ma_length).W))
    val in_b_fire = Input(Bool())
    val in_b_transpose = Input(Bool())
    val in_pass   = Input(Bool())   // 2-pass weight select (false=lo, true=hi)
    val in_b_pass = Input(Bool())   // 2-pass weight-load target (false=lo, true=hi)

    val out_sum   = Output(UInt(fnW(f32ExpW, f32SigW).W))   // standard fp32
    val out_last  = Output(Bool())
    val out_valid = Output(Bool())
    val out_id    = Output(UInt(log2Up(max_simultaneous_matmuls).W))
  })

  val tree = Module(new FpAdderTree(ma_length, max_simultaneous_matmuls))
  val pes  = Seq.fill(ma_length)(Module(new FpPE(max_simultaneous_matmuls)))

  pes.zipWithIndex.foreach { case (pe, i) =>
    pe.io.in_a     := io.in_a(i)
    pe.io.in_valid := io.in_valid(i)
    pe.io.in_last  := io.in_last(i)
    pe.io.in_id    := io.in_id(i)
    pe.io.in_prop  := io.in_prop(i)
    pe.io.in_pass   := io.in_pass
    pe.io.in_b_pass := io.in_b_pass
  }

  for ((pe, idx) <- pes.zipWithIndex) {
    val sel           = io.in_fire_counter === idx.U
    val b_scalar      = Mux(sel, io.in_b, 0.U)
    val b_fire_scalar = Mux(sel, io.in_b_fire, false.B)
    val b_vector      = io.in_b_vec(idx)
    pe.io.in_b      := Mux(io.in_b_transpose, b_vector, b_scalar)
    pe.io.in_b_fire := Mux(io.in_b_transpose, io.in_b_fire, b_fire_scalar)
  }

  tree.io.in       := VecInit(pes.map(_.io.out_result))
  tree.io.in_valid := VecInit(pes.map(_.io.out_valid))
  tree.io.in_last  := VecInit(pes.map(_.io.out_last))
  tree.io.in_id    := VecInit(pes.map(_.io.out_id))

  io.out_sum   := tree.io.out
  io.out_valid := tree.io.out_valid
  io.out_last  := tree.io.out_last
  io.out_id    := tree.io.out_id
}
