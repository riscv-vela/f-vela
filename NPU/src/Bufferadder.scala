package gemmini

import chisel3._
import chisel3.util._
import hardfloat._
import GemminiISA._
import Util._

class Buffadderlight[T <: Data : Arithmetic](inputType: T, outputType: T, max_simultaneous_matmuls: Int) (implicit ev: Arithmetic[T])  extends Module {
    import ev._
    val io = IO(new Bundle {
        val in_d = Input(inputType)
        val in_result = Input(outputType)
        val out_c = Output(outputType)

        val in_last = Input(Bool())
        val in_valid = Input(Bool())
        val in_id = Input(UInt(log2Up(max_simultaneous_matmuls).W))

        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))
    })

    val in_d_ext = io.in_d.withWidthOf(outputType)
    val c1 = RegInit(0.U.asTypeOf(outputType))
    val c2 = RegInit(0.U.asTypeOf(outputType))

    //internal control signal for double buffering
    val in_d_ext_next = RegNext(in_d_ext, 0.U.asTypeOf(outputType))
    val in_valid_next = RegNext(io.in_valid)
    
    val toggle_reg = RegInit(false.B)
    val toggle = Wire(Bool())
    toggle := false.B

    when(in_valid_next) {
        toggle := ~toggle_reg
        toggle_reg := toggle
    }
    
    when(toggle) {
        c1 := in_d_ext_next
    }.otherwise {
        c2 := in_d_ext_next
    }

    val base_d   = Mux(toggle, c2, c1)    
    val out_c = base_d + io.in_result

    io.out_c := Mux(in_valid_next, out_c, 0.U.asTypeOf(outputType))

    io.out_valid := in_valid_next
    io.out_last := RegNext(io.in_last)
    io.out_id := RegNext(io.in_id)

}

// class Buffadder[T <: Data : Arithmetic](inputType: T, outputType: T, max_simultaneous_matmuls: Int) (implicit ev: Arithmetic[T])  extends Module {
//     import ev._
//     val io = IO(new Bundle {
//         val in_d = Input(inputType)
//         val in_result = Input(outputType)
//         val out_c = Output(outputType)

//         val in_last = Input(Bool())
//         val in_valid = Input(Bool())
//         val in_id = Input(UInt(log2Up(max_simultaneous_matmuls).W))
//         val in_acc = Input(Bool())
//         val in_preload = Input(Bool())

//         val out_last = Output(Bool())
//         val out_valid = Output(Bool())
//         val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))
//     })

//     val in_d_ext = io.in_d.withWidthOf(outputType)
//     val c1 = RegInit(0.U.asTypeOf(outputType))
//     val c2 = RegInit(0.U.asTypeOf(outputType))
//     val c3 = RegInit(0.U.asTypeOf(outputType))

//     //internal control signal for double buffering
//     val in_d_ext_next = RegNext(in_d_ext, 0.U.asTypeOf(outputType))
//     val in_valid_next = RegNext(io.in_valid)
    
//     val toggle_reg = RegInit(false.B)
//     val toggle = Wire(Bool())
//     toggle := false.B

//     when(in_valid_next) {
//         toggle := ~toggle_reg
//         toggle_reg := toggle
//     }
    
//     when(toggle) {
//         c1 := in_d_ext_next
//     }.otherwise {
//         c2 := in_d_ext_next
//     }


//     io.out_c := 0.U.asTypeOf(outputType)

//     // ────────── 누적 / 출력 로직 ──────────
//     val base_d   = Mux(toggle, c2, c1)      // 현재 싸이클에 더할 D
//     val c3_next = c3 + io.in_result


//     when(in_valid_next){
//         when(io.in_acc){
//             c3 := c3_next                       // 누적 단계
//         } .elsewhen(io.in_preload){
//             c3 := c3
//         }.otherwise {
//             io.out_c := c3_next + base_d        // 출력 단계
//             c3 := 0.U.asTypeOf(outputType)      // 다음 누적을 위해 클리어
//         }
//     }

//     io.out_valid := in_valid_next
//     io.out_last := RegNext(io.in_last)
//     io.out_id := RegNext(io.in_id)

// }

// Floating-point variant of Buffadderlight (used by FpExeUnit). Identical double-
// buffering, but accumulates with the cheap AddRecFN (align+add+round) instead of
// the generic Float.+ which is a MulAddRecFN/FMA — the latter was the synthesized
// critical path (buffadderarray/out_c_muladder/mulAddRecFNToRaw). fp-only: outputType
// must be a gemmini.Float.
class FpBuffadderlight[T <: Data : Arithmetic](inputType: T, outputType: T, max_simultaneous_matmuls: Int, num_passes: Int = 1) (implicit ev: Arithmetic[T]) extends Module {
    import ev._
    require(num_passes >= 1, "FpBuffadderlight num_passes must be >= 1")
    val io = IO(new Bundle {
        val in_d = Input(inputType)
        val in_result = Input(outputType)
        val out_c = Output(outputType)

        val in_last = Input(Bool())
        val in_valid = Input(Bool())
        val in_id = Input(UInt(log2Up(max_simultaneous_matmuls).W))

        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))
    })

    val f = outputType match {
        case fl: Float => fl
        case _ => throw new Exception("FpBuffadderlight requires a floating-point outputType")
    }

    // Register EVERY input at the module boundary (+1 pipeline stage, applied
    // uniformly so the internal relative timing is preserved). This cuts the
    // combinational path from the upstream FpAdderTree into the accumulate adder
    // for timing closure. NOTE: adds exactly 1 cycle of latency to this module —
    // must be reflected in the surrounding pipeline (WontolicWithDelays / config
    // tile_latency / mesh_output_delay) for valid/last/id to stay aligned.
    val in_d_r      = RegNext(io.in_d)
    val in_result_r = io.in_result
    val in_valid_r  = RegNext(io.in_valid, false.B)
    val in_last_r   = RegNext(io.in_last)
    val in_id_r     = RegNext(io.in_id)

    // ONE shared fp accumulate adder (AddRecFN, no FMA) used by both modes.
    val add = Module(new AddRecFN(f.expWidth, f.sigWidth))
    add.io.subOp          := false.B
    add.io.roundingMode   := consts.round_near_even
    add.io.detectTininess := consts.tininess_afterRounding

    if (num_passes <= 1) {
      // ---- original single-pass behavior: out_c = bias(in_d) + in_result ----
      val in_d_ext = in_d_r.withWidthOf(outputType)
      val c1 = RegInit(0.U.asTypeOf(outputType))
      val c2 = RegInit(0.U.asTypeOf(outputType))

      val in_d_ext_next = RegNext(in_d_ext, 0.U.asTypeOf(outputType))
      val in_valid_next = RegNext(in_valid_r, false.B)

      val toggle_reg = RegInit(false.B)
      val toggle = Wire(Bool())
      toggle := false.B
      when(in_valid_next) {
          toggle := ~toggle_reg
          toggle_reg := toggle
      }
      when(toggle) {
          c1 := in_d_ext_next
      }.otherwise {
          c2 := in_d_ext_next
      }

      val base_d = Mux(toggle, c2, c1)

      add.io.a := recFNFromFN(f.expWidth, f.sigWidth, base_d.asUInt)
      add.io.b := recFNFromFN(f.expWidth, f.sigWidth, in_result_r.asUInt)
      val out_c = fNFromRecFN(f.expWidth, f.sigWidth, add.io.out).asTypeOf(outputType)

      io.out_c := Mux(in_valid_next, out_c, 0.U.asTypeOf(outputType))

      io.out_valid := in_valid_next
      io.out_last := RegNext(in_last_r)
      io.out_id := RegNext(in_id_r)
    } else {
      // ---- multi-pass accumulate, REUSING the same AddRecFN (no extra adder) ----
      // A logical contraction split into num_passes partial sums (one `in_result`
      // per pass, fed on consecutive valid cycles — this is what lets a 128b spad
      // serve fp16: 8 fp16/read x num_passes). Compute bias + sum_passes(in_result):
      // add the bias (in_d) only on pass 0, accumulate the partials, and emit the
      // result on the final pass. II=1 (one pass/cycle); latency = +1 (input reg).
      val passCount  = RegInit(0.U(log2Up(num_passes).W))
      val isLastPass = passCount === (num_passes - 1).U
    //   when(in_valid_r) { passCount := Mux(isLastPass, 0.U, passCount + 1.U) }
      when(in_valid_r) { passCount := Mux(in_last_r || isLastPass, 0.U, passCount + 1.U) }

      val acc = Reg(UInt((f.expWidth + f.sigWidth).W))
      // pass 0 adds the bias; later passes accumulate the running sum
      val addend_a = Mux(passCount === 0.U, in_d_r.withWidthOf(outputType).asUInt, acc)

      add.io.a := recFNFromFN(f.expWidth, f.sigWidth, addend_a)
      add.io.b := recFNFromFN(f.expWidth, f.sigWidth, in_result_r.asUInt)
      val sum = fNFromRecFN(f.expWidth, f.sigWidth, add.io.out)
      when(in_valid_r) { acc := sum }

      io.out_c     := Mux(in_valid_r && isLastPass, sum.asTypeOf(outputType), 0.U.asTypeOf(outputType))
      io.out_valid := in_valid_r && isLastPass
      io.out_last  := in_last_r
      io.out_id    := in_id_r
    }
}

// class Buffadder[T <: Data : Arithmetic](inputType: T, outputType: T, max_simultaneous_matmuls: Int) (implicit ev: Arithmetic[T])  extends Module {
//     import ev._
//     val io = IO(new Bundle {
//         val in_d = Input(inputType)
//         val in_result = Input(outputType)
//         val out_c = Output(outputType)

//         val in_last = Input(Bool())
//         val in_valid = Input(Bool())
//         val in_id = Input(UInt(log2Up(max_simultaneous_matmuls).W))
//         val in_acc = Input(Bool())
//         val in_preload = Input(Bool())

//         val out_last = Output(Bool())
//         val out_valid = Output(Bool())
//         val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))
//     })

//     val in_d_ext = io.in_d.withWidthOf(outputType)
//     val c1 = RegInit(0.U.asTypeOf(outputType))
//     val c2 = RegInit(0.U.asTypeOf(outputType))
//     val c3 = RegInit(0.U.asTypeOf(outputType))

//     //internal control signal for double buffering
//     val in_d_ext_next = RegNext(in_d_ext, 0.U.asTypeOf(outputType))
//     val in_valid_next = RegNext(io.in_valid)
    
//     val toggle_reg = RegInit(false.B)
//     val toggle = Wire(Bool())
//     toggle := false.B

//     when(in_valid_next) {
//         toggle := ~toggle_reg
//         toggle_reg := toggle
//     }
    
//     when(toggle) {
//         c1 := in_d_ext_next
//     }.otherwise {
//         c2 := in_d_ext_next
//     }


//     io.out_c := 0.U.asTypeOf(outputType)

//     // ────────── 누적 / 출력 로직 ──────────
//     val base_d   = Mux(toggle, c2, c1)      // 현재 싸이클에 더할 D
//     val c3_next = c3 + io.in_result


//     when(in_valid_next){
//         when(io.in_acc){
//             c3 := c3_next                       // 누적 단계
//         } .elsewhen(io.in_preload){
//             c3 := c3
//         }.otherwise {
//             io.out_c := c3_next + base_d        // 출력 단계
//             c3 := 0.U.asTypeOf(outputType)      // 다음 누적을 위해 클리어
//         }
//     }

//     io.out_valid := in_valid_next
//     io.out_last := RegNext(io.in_last)
//     io.out_id := RegNext(io.in_id)

// }