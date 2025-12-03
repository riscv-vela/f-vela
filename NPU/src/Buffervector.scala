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

    val c1 = RegInit(0.U.asTypeOf(inputType))
    val c2 = RegInit(0.U.asTypeOf(inputType))

    //internal control signal for double buffering
    val toggle = RegInit(false.B)
    
    when(toggle) {
        when(io.in_valid) { c1 := io.in_a }
        io.out_a := c2
    }
    .otherwise {
        when(io.in_valid) { c2 := io.in_a }
        io.out_a := c1
    }
    
    when (io.in_valid){
        toggle := ~toggle
    }   .otherwise{
        toggle := toggle
    }

    io.out_valid := RegNext(io.in_valid)
    io.out_last := RegNext(io.in_last)
    io.out_id := RegNext(io.in_id)
    io.out_prop := RegNext(io.in_prop)


    
}