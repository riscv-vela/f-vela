package gemmini

import chisel3._
import chisel3.util._

import gemmini.Util._


class MpExeUnit[T <: Data](inputType: T, weightType: T, outputType: T, ma_length: Int, ma_num: Int, max_simultaneous_matmuls: Int) (implicit ev: Arithmetic[T])  extends Module {
    import ev._

    val io = IO(new Bundle {
        val in_a = Input(Vec(ma_length, inputType))
        val in_b = Input(Vec(ma_num, weightType))
        val in_d = Input(Vec(ma_num, inputType))

        val in_last = Input(Vec(ma_length, Bool()))
        val in_prop = Input(Vec(ma_length, Bool()))
        val in_valid = Input(Vec(ma_length, Bool()))
        val in_id = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))
        val in_fire_counter = Input(UInt(log2Up(ma_length).W))
        val in_acc = Input(Bool())
        val in_preload = Input(Bool())

        val out_c = Output(Vec(ma_num, outputType))
        val out_last = Output(Vec(ma_num, Bool()))
        val out_id = Output(Vec(ma_num, UInt(log2Up(max_simultaneous_matmuls).W)))
        val out_valid = Output(Vec(ma_num, Bool()))
    })

    val buffadderarray = Seq.fill(ma_num) {
        Module{new Buffadder(inputType, outputType, max_simultaneous_matmuls)}
    }
    val buffvectorarray = Seq.fill(ma_length) {
        Module{new Buffvector(inputType, max_simultaneous_matmuls)}
    }
    val mularraybundle = Seq.fill(ma_num) {
        Module(new MpMularray(inputType, weightType, outputType, ma_length, max_simultaneous_matmuls))
    }

    

    //buffvectorarray의 in_a 값으로 mpexe의 in_a 입력
    for(i <- 0 until ma_length) {
        buffvectorarray(i).io.in_a := io.in_a(i)
        buffvectorarray(i).io.in_valid := io.in_valid(i)
        buffvectorarray(i).io.in_last := io.in_last(i)
        buffvectorarray(i).io.in_id := io.in_id(i)
        buffvectorarray(i).io.in_prop := io.in_prop(i)
    }

    //각 mularray들의 in_a 에 buffvectorarray, in_b에 mpexe의 in_b 입력 
    for(i <- 0 until ma_num) {
        mularraybundle(i).io.in_a := VecInit(buffvectorarray.map(_.io.out_a))
        //io.in_b의 i번째 element를 i번째 mularray에 입력(transpose를 안하기 위해)
        mularraybundle(i).io.in_b := io.in_b(i)
        // Create a condition that is true only for the selected mularray
        val b_fire = RegNext(io.in_valid.reduce(_||_))
        // Use a Mux to provide the vector data only to the selected mularray.
        // Others get a zero vector. 0.U.asTypeOf(...) creates a wire of the correct type with all bits set to 0.

        mularraybundle(i).io.in_b_fire :=  b_fire
        
        mularraybundle(i).io.in_fire_counter := io.in_fire_counter
        //in_valid, in_last 등의 신호 입력
        mularraybundle(i).io.in_valid :=  VecInit(buffvectorarray.map(_.io.out_valid))
        mularraybundle(i).io.in_last :=  VecInit(buffvectorarray.map(_.io.out_last))
        mularraybundle(i).io.in_id :=  VecInit(buffvectorarray.map(_.io.out_id))
        mularraybundle(i).io.in_prop :=  VecInit(buffvectorarray.map(_.io.out_prop))
    }
    
    val in_acc_next = ShiftRegister(io.in_acc, 2)
    val in_preload_next = ShiftRegister(io.in_preload, 2)

    //adder tree의 결과 값을 buffadderarray의 입력으로 연결 + buffadderarray에 in_d 연결
    for(i <- 0 until ma_num) {
        buffadderarray(i).io.in_d := io.in_d(i)
        buffadderarray(i).io.in_result := mularraybundle(i).io.out_sum
        buffadderarray(i).io.in_valid := mularraybundle(i).io.out_valid
        buffadderarray(i).io.in_last := mularraybundle(i).io.out_last
        buffadderarray(i).io.in_id := mularraybundle(i).io.out_id

        buffadderarray(i).io.in_acc := in_acc_next
        buffadderarray(i).io.in_preload := in_preload_next

        io.out_c(i) := buffadderarray(i).io.out_c
        io.out_valid(i) := buffadderarray(i).io.out_valid
        io.out_last(i) := buffadderarray(i).io.out_last
        io.out_id(i) := buffadderarray(i).io.out_id
    }

}
