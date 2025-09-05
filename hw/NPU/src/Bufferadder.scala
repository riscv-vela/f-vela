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
        val in_prop = Input(Bool())
        val in_acc = Input(Bool())

        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))
        val out_prop = Output(Bool())
    })

    val in_d_ext = io.in_d.withWidthOf(outputType)
    val c1 = Reg(outputType)
    val c2 = Reg(outputType)
    val c3 = Reg(outputType)

    //internal control signal for double buffering
    val in_d_ext_next = RegNext(in_d_ext)

    val in_acc_next = ShiftRegister(io.in_acc, 2)
    
    val in_prop = RegInit(false.B)
    in_prop := Mux(io.in_valid, ~in_prop, in_prop)

    when (io.in_valid &&  in_prop) { c1 := in_d_ext_next }
    when (io.in_valid && !in_prop) { c2 := in_d_ext_next }


    io.out_c := 0.U.asTypeOf(outputType)
    // ────────── 누적 / 출력 로직 ──────────
    val base_d   = Mux(in_prop, c2, c1)      // 현재 싸이클에 더할 D
    val sum_temp = base_d + io.in_result     // PE 결과 + D

    when (in_acc_next && io.in_valid) {              // 누적 단계
        c3         := c3 + sum_temp
    } .elsewhen(!in_acc_next && io.in_valid) {               // 출력 단계
        io.out_c     := sum_temp + c3
        c3           := 0.U.asTypeOf(outputType) // 다음 누적을 위해 클리어
    }

    io.out_valid := io.in_valid
    io.out_last := io.in_last
    io.out_id := io.in_id
    io.out_prop := io.in_prop

}