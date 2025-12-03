package gemmini

import chisel3._
import chisel3.util._
import GemminiISA._
import Util._

class Buffadder[T <: Data : Arithmetic](inputType: T, outputType: T, max_simultaneous_matmuls: Int) (implicit ev: Arithmetic[T])  extends Module {
    import ev._
    val io = IO(new Bundle {
        val in_d = Input(inputType)
        val in_result = Input(outputType)
        val out_c = Output(outputType)

        val in_last = Input(Bool())
        val in_valid = Input(Bool())
        val in_id = Input(UInt(log2Up(max_simultaneous_matmuls).W))
        val in_acc = Input(Bool())
        val in_preload = Input(Bool())

        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))
    })

    val in_d_ext = io.in_d.withWidthOf(outputType)
    val c1 = RegInit(0.U.asTypeOf(outputType))
    val c2 = RegInit(0.U.asTypeOf(outputType))
    val c3 = RegInit(0.U.asTypeOf(outputType))

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


    io.out_c := 0.U.asTypeOf(outputType)

    // ────────── 누적 / 출력 로직 ──────────
    val base_d   = Mux(toggle, c2, c1)      // 현재 싸이클에 더할 D
    val c3_next = c3 + io.in_result


    when(in_valid_next){
        when(io.in_acc){
            c3 := c3_next                       // 누적 단계
        } .elsewhen(io.in_preload){
            c3 := c3
        }.otherwise {
            io.out_c := c3_next + base_d        // 출력 단계
            c3 := 0.U.asTypeOf(outputType)      // 다음 누적을 위해 클리어
        }
    }

    io.out_valid := in_valid_next
    io.out_last := RegNext(io.in_last)
    io.out_id := RegNext(io.in_id)

}