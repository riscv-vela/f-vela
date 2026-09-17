package gemmini

import chisel3._
import chisel3.util._

import gemmini.Util._

// =============================================================================
// FpExeUnit — fp16-mul / fp32-accumulate execute unit for the DualGemmini fp
// side. IO is IDENTICAL to MpExeUnit so WontolicWithDelays can instantiate it
// as a drop-in (type-dispatch on Float). Internals mirror MpExeUnit's int8
// lane path but are fp-only: no int2/int8 tiers, no is_mpgemm packing.
//
//   activations -> Buffvector (double-buffer)  [reused as-is, dtype-agnostic]
//               -> FpMularray (FpPE array + FpAdderTree)   [FpUnit.scala]
//               -> Buffadderlight[Float]  (fp32 accumulate) [reused as-is]
//
// Generic over T so the constructor signature matches MpExeUnit; in practice
// T = Float (inputType/weightType = Float(5,11), outputType = Float(8,24)).
// The container <-> 16-bit conversions use .asUInt / .asTypeOf.
// =============================================================================
class FpExeUnit[T <: Data](
  inputType: T, weightType: T, outputType: T,
  ma_length: Int, ma_num: Int, max_simultaneous_matmuls: Int, num_passes: Int = 1
)(implicit ev: Arithmetic[T]) extends Module with HasExeUnitIO[T] {
  import ev._
  require(num_passes >= 1, "FpExeUnit num_passes must be >= 1")

  // Same shared ExeUnitIO as MpExeUnit (drop-in). in_is_mpgemm/out_is_mpgemm
  // are present for compatibility but unused on the fp path.
  val io = IO(new ExeUnitIO(inputType, weightType, outputType, ma_length, ma_num, max_simultaneous_matmuls))

  val W16 = 16  // fp16 container width
  // Extra latency added by the pipelined FpAdderTree (must match
  // FpAdderTree.treeLatency = log2Up(ma_length)). in_d is delayed by this so the
  // bias aligns with the (now later) dot-product result; out_is_mpgemm too.
  val treeLatency = log2Up(ma_length)

  val buffvectorarray = Seq.fill(ma_length) {
    Module(new Buffvector(inputType, max_simultaneous_matmuls))
  }
  val fpmularray = Seq.fill(ma_num) {
    Module(new FpMularray(ma_length, max_simultaneous_matmuls))
  }
  val buffadderarray = Seq.fill(ma_num) {
    Module(new FpBuffadderlight(inputType, outputType, max_simultaneous_matmuls, num_passes))
  }

  // ---- activation double-buffering (mirrors MpExeUnit) ----
  for (i <- 0 until ma_length) {
    buffvectorarray(i).io.in_a     := io.in_a(i)
    buffvectorarray(i).io.in_valid := io.in_valid(i)
    buffvectorarray(i).io.in_last  := io.in_last(i)
    buffvectorarray(i).io.in_id    := io.in_id(i)
    buffvectorarray(i).io.in_prop  := io.in_prop(i)
  }

  // buffered activation vector as raw 16-bit fp16 bits, broadcast to every lane
  val act_bits = VecInit(buffvectorarray.map(_.io.out_a.asUInt))

  // ---- per-column fp dot-product lanes ----
  if (num_passes <= 1) {
    // Original single-weight feed (Float-only config). in_fire_counter is the
    // normal per-PE select; no 2-pass.
    for (i <- 0 until ma_num) {
      val fpm = fpmularray(i)
      val b_fire = RegNext(io.in_valid.head)
      fpm.io.in_a            := act_bits
      fpm.io.in_fire_counter := io.in_fire_counter
      fpm.io.in_valid        := VecInit(buffvectorarray.map(_.io.out_valid))
      fpm.io.in_last         := VecInit(buffvectorarray.map(_.io.out_last))
      fpm.io.in_id           := VecInit(buffvectorarray.map(_.io.out_id))
      fpm.io.in_prop         := VecInit(buffvectorarray.map(_.io.out_prop))
      fpm.io.in_b_transpose  := io.in_b_transpose
      fpm.io.in_pass         := false.B
      fpm.io.in_b_pass       := false.B
      when (!io.in_b_transpose) {
        fpm.io.in_b      := io.in_b(i).asUInt
        fpm.io.in_b_fire := b_fire
        fpm.io.in_b_vec  := VecInit(Seq.fill(ma_length)(0.U(W16.W)))
      } .otherwise {
        val sel   = io.in_fire_counter === i.U
        val b_vec = io.in_b.asTypeOf(Vec(ma_length, weightType))
        fpm.io.in_b      := 0.U(W16.W)
        fpm.io.in_b_vec  := VecInit(b_vec.map(_.asUInt))
        fpm.io.in_b_fire := sel
      }
    }
  } else {
    // Merged 2-pass: decode PE/pass/lo-hi/col-half from the RAW fire counter.
    // Bit fields are compile-time from DIM (num_passes, ma_length=fp_dim, ma_num):
    //   non-transpose fire = [ lo/hi | PE-index | col-half ]
    //   transpose     fire = [ lo/hi | column-index ]
    //   compute multiply pass = the low (pass/col) field.
    val fp_dim   = ma_length                       // physical fp PEs per lane (= DIM/num_passes)
    val pBits    = log2Up(num_passes)              // pass / col-half field width
    val peBits   = log2Up(fp_dim)                  // PE-index field width
    val colBits  = log2Up(ma_num)                  // column-index field width (transpose)
    val fc       = io.in_fire_counter
    val nt_col   = fc(pBits - 1, 0)                // non-transpose col-half
    val nt_pe    = fc(pBits + peBits - 1, pBits)   // non-transpose PE index (k % fp_dim)
    val nt_lohi  = fc(pBits + peBits)              // weight lo/hi (k >= fp_dim)
    // transpose: B arrives along k with lo/hi CONSECUTIVE per column, so lo/hi is
    // the LOW field and column index is the UPPER field (opposite of non-transpose).
    val t_lohi   = fc(pBits - 1, 0).orR            // weight lo/hi (consecutive, low)
    val t_col    = fc(pBits + colBits - 1, pBits)  // column index (upper)
    val mul_pass = fc(pBits - 1, 0).orR           // compute: multiply weight half (pass != 0)

    for (i <- 0 until ma_num) {
      val fpm = fpmularray(i)
      val b_fire = RegNext(io.in_valid.head)
      fpm.io.in_a            := act_bits
      fpm.io.in_valid        := VecInit(buffvectorarray.map(_.io.out_valid))
      fpm.io.in_last         := VecInit(buffvectorarray.map(_.io.out_last))
      fpm.io.in_id           := VecInit(buffvectorarray.map(_.io.out_id))
      fpm.io.in_prop         := VecInit(buffvectorarray.map(_.io.out_prop))
      fpm.io.in_b_transpose  := io.in_b_transpose
      fpm.io.in_pass         := mul_pass

      when (!io.in_b_transpose) {
        // PE[nt_pe] of every column loads its weight (io.in_b is the 16-col row
        // pre-assembled upstream) into slot lo/hi=nt_lohi, on staging-complete.
        fpm.io.in_fire_counter := nt_pe
        fpm.io.in_b      := io.in_b(i).asUInt
        fpm.io.in_b_pass := nt_lohi
        fpm.io.in_b_fire := b_fire && (nt_col === (num_passes - 1).U)
        fpm.io.in_b_vec  := VecInit(Seq.fill(ma_length)(0.U(W16.W)))
      } .otherwise {
        // column t_col loads its full ma_length weight vector into slot lo/hi.
        val sel   = t_col === i.U
        val b_vec = io.in_b.asTypeOf(Vec(ma_length, weightType))
        fpm.io.in_fire_counter := 0.U
        fpm.io.in_b      := 0.U(W16.W)
        fpm.io.in_b_vec  := VecInit(b_vec.map(_.asUInt))
        fpm.io.in_b_pass := t_lohi
        fpm.io.in_b_fire := sel && b_fire
      }
    }
  }

  // ---- accumulate (mirrors MpExeUnit output section) ----
  for (i <- 0 until ma_num) {
    buffadderarray(i).io.in_d     := ShiftRegister(io.in_d(i), treeLatency)
    buffadderarray(i).io.in_result := fpmularray(i).io.out_sum.asTypeOf(outputType)
    buffadderarray(i).io.in_valid := fpmularray(i).io.out_valid
    buffadderarray(i).io.in_last  := fpmularray(i).io.out_last
    buffadderarray(i).io.in_id    := fpmularray(i).io.out_id

    io.out_c(i)     := buffadderarray(i).io.out_c
    io.out_valid(i) := buffadderarray(i).io.out_valid
    io.out_last(i)  := buffadderarray(i).io.out_last
    io.out_id(i)    := buffadderarray(i).io.out_id
  }

  io.out_is_mpgemm := ShiftRegister(io.in_is_mpgemm, 4 + treeLatency)
}
