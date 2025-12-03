package gemmini

import chisel3._
import chisel3.util._

class MulUnit[T <: Data](inputType: T, outputType: T) (implicit ev: Arithmetic[T]) extends Module {
    import ev._
    val io = IO(new Bundle {
        val in_a = Input(inputType)
        val in_b = Input(inputType)
        val out_result = Output(outputType)
    })
    val product = io.in_a * io.in_b
    io.out_result  := product.clippedToWidthOf(outputType)
}

//double buffering을 써야겠네..
class PE[T <: Data :Arithmetic](inputType: T, outputType: T, max_simultaneous_matmuls: Int)(implicit ev: Arithmetic[T]) extends Module{ 
    import ev._  
    val io = IO(new Bundle {
        val in_a = Input(inputType)
        val in_b = Input(inputType)
        val out_result = Output(outputType)

        val in_last = Input(Bool())
        val in_valid = Input(Bool())
        val in_id = Input( UInt(log2Up(max_simultaneous_matmuls).W))
        val in_prop = Input(Bool())
        val in_b_fire = Input(Bool())

        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output( UInt(log2Up(max_simultaneous_matmuls).W))
    })

    val mul_unit = Module(new MulUnit(inputType, outputType)(ev))
    
    when(io.in_valid){
      mul_unit.io.in_a := io.in_a
    } .otherwise{
      mul_unit.io.in_a := 0.U.asTypeOf(inputType)
    }
    val c1 = RegInit(0.U.asTypeOf(inputType))
    val c2 = RegInit(0.U.asTypeOf(inputType))
    //in_prop이 true이면, c1이 preload, c2가 mul에 사용

    //io.in_b_fire가 true일 때만 기록

    // 현재 토글 상태에 따라 반대쪽 버퍼를 읽어 곱셈에 사용
    mul_unit.io.in_b := 0.U.asTypeOf(inputType)
    when(io.in_prop){
      when(io.in_valid){mul_unit.io.in_b := c2}
      when (io.in_b_fire){
        c1 := io.in_b
      }
    }
    .otherwise{
      when(io.in_valid){mul_unit.io.in_b := c1}
      when(io.in_b_fire){
        c2 := io.in_b
      }
    } 


    io.out_result := ShiftRegister(mul_unit.io.out_result, 1)
    io.out_valid := io.in_valid
    io.out_last := io.in_last
    io.out_id := io.in_id
    
}