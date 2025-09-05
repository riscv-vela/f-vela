package gemmini

import chisel3._
import chisel3.util._
import GemminiISA._
import Util._


class Buffvector[T <: Data : Arithmetic](inputType: T, max_simultaneous_matmuls: Int)(implicit ev: Arithmetic[T])   extends Module {
    import ev._
    val io = IO(new Bundle {
        val in_a = Input(inputType)
        val out_a = Output(inputType)

        val in_last = Input(Bool())
        val in_valid = Input(Bool())
        val in_id = Input(UInt(log2Up(max_simultaneous_matmuls).W))
        val in_prop = Input(Bool())

        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))
        val out_prop = Output(Bool())
    })

    val c1 = Reg(inputType)
    val c2 = Reg(inputType)

    //internal control signal for double buffering
    val in_prop = RegInit(false.B)
    in_prop := Mux(io.in_valid, ~in_prop, in_prop)

    when(in_prop) {
        c1 := io.in_a
        io.out_a := c2
    }
    .otherwise {
        c2 := io.in_a
        io.out_a := c1
    }

    when(!io.in_valid) {
        c1 := c1
        c2 := c2
        in_prop := false.B          // 토글 상태 리셋
    }

    def pipe[T <: Data](valid: Bool, t: T, latency: Int): T = {
        // The default "Pipe" function apparently resets the valid signals to false.B. We would like to avoid using global
        // signals in the Mesh, so over here, we make it clear that the reset signal will never be asserted
        chisel3.withReset(false.B) { Pipe(valid, t, latency).bits }
    }

    io.out_valid := ShiftRegister(io.in_valid, 1)
    io.out_last := pipe(io.in_valid, io.in_last, 1)
    io.out_id := pipe(io.in_valid, io.in_id, 1)
    io.out_prop := io.in_prop
    


    
}