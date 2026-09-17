package gemmini

import chisel3._
import chisel3.util._

// Shared IO for the spatial-array execute units (MpExeUnit, FpExeUnit) so that
// WontolicWithDelays can instantiate either one (type-dispatch on Float) and
// wire it uniformly via `exeunit.io`. This is a pure IO-type extraction — the
// field set is identical to MpExeUnit's original inline bundle (no behavior
// change to the int2/int8 datapath).
class ExeUnitIO[T <: Data](
  inputType: T, weightType: T, outputType: T,
  ma_length: Int, ma_num: Int, max_simultaneous_matmuls: Int
) extends Bundle {
  val in_a = Input(Vec(ma_length, inputType))
  val in_b = Input(Vec(ma_num, weightType))
  val in_d = Input(Vec(ma_num, inputType))

  val in_last = Input(Vec(ma_length, Bool()))
  val in_prop = Input(Vec(ma_length, Bool()))
  val in_valid = Input(Vec(ma_length, Bool()))
  val in_id = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))
  // wide enough to carry the RAW fire counter (up to 2*ma_num-1 for the merged
  // fp 2-pass). The fp control (pass / PE / lo-hi / col-half) is decoded from
  // this INSIDE FpExeUnit; MpExeUnit uses the low bits as before.
  val in_fire_counter = Input(UInt(log2Up(2*ma_num).W))
  val in_b_transpose = Input(Bool())

  val in_is_mpgemm = Input(Bool())
  val out_is_mpgemm = Output(Bool())

  val out_c = Output(Vec(ma_num, outputType))
  val out_last = Output(Vec(ma_num, Bool()))
  val out_id = Output(Vec(ma_num, UInt(log2Up(max_simultaneous_matmuls).W)))
  val out_valid = Output(Vec(ma_num, Bool()))
}

// Marker so a common reference type exists for the MpExeUnit/FpExeUnit choice.
trait HasExeUnitIO[T <: Data] { def io: ExeUnitIO[T] }
