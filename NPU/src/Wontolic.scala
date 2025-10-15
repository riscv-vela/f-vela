package gemmini

import chisel3._
import chisel3.util._

import gemmini.Util._

class WontolicReq[T <: Data: Arithmetic, TagT <: TagQueueTag with Data](tagType: TagT, ma_length: Int) extends Bundle {
  val in_prop = Bool()
  val in_acc = Bool()
  val in_preload = Bool()
  val total_rows = UInt(log2Up(ma_length+1).W)
  val tag = tagType
  val flush = UInt(2.W)
  val b_transpose = Bool()
  val is_mpgemm = Bool()
}

class WontolicResp[T <: Data: Arithmetic, TagT <: TagQueueTag with Data](inputType: T, weightType: T, outputType: T, ma_length: Int, ma_num: Int, tagType: TagT) extends Bundle {
  val data = Vec(ma_num*(inputType.getWidth / weightType.getWidth), outputType)
  val total_rows = UInt(log2Up(ma_length+1).W)
  val tag = tagType 
  val last = Bool()
  val is_mpgemm = Bool()
}

class Wontolic[T <: Data](inputType: T, outputType: T, ma_length: Int, ma_num: Int, max_simultaneous_matmuls: Int) (implicit ev: Arithmetic[T])  extends Module {
    import ev._

    val io = IO(new Bundle {
        val in_a = Input(Vec(ma_length, inputType))
        val in_b = Input(Vec(ma_num, inputType))
        val in_b_vec = Input(Vec(ma_length, inputType)) // B_Transpose 일 때 사용하는 io
        val in_d = Input(Vec(ma_num, inputType))

        val in_last = Input(Vec(ma_length, Bool()))
        val in_prop = Input(Vec(ma_length, Bool()))
        val in_valid = Input(Vec(ma_length, Bool()))
        val in_id = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))
        val in_fire_counter = Input(UInt(log2Up(ma_length).W))
        val in_acc = Input(Bool())
        val in_preload = Input(Bool())
        val in_b_transpose = Input(Bool())
        val in_is_mpgemm = Input(Bool())

        val out_c = Output(Vec(ma_num, outputType))
        val out_last = Output(Vec(ma_num, Bool()))
        val out_id = Output(Vec(ma_num, UInt(log2Up(max_simultaneous_matmuls).W)))
        val out_valid = Output(Vec(ma_num, Bool()))
        val out_is_mpgemm = Output(Bool())
    })

    val buffadderarray = Seq.fill(ma_num) {
        Module{new Buffadder(inputType, outputType, max_simultaneous_matmuls)}
    }
    val buffvectorarray = Seq.fill(ma_length) {
        Module{new Buffvector(inputType, max_simultaneous_matmuls)}
    }
    val mularraybundle = Seq.fill(ma_num) {
        Module(new Mularray(inputType, outputType, ma_length, max_simultaneous_matmuls))
    }

    

    //buffvectorarray의 in_a 값으로 wontolic의 in_a 입력
    for(i <- 0 until ma_length) {
        buffvectorarray(i).io.in_a := io.in_a(i)
        buffvectorarray(i).io.in_valid := io.in_valid(i)
        buffvectorarray(i).io.in_last := io.in_last(i)
        buffvectorarray(i).io.in_id := io.in_id(i)
        buffvectorarray(i).io.in_prop := io.in_prop(i)
    }

    //각 mularray들의 in_a 에 buffvectorarray, in_b에 wontolic의 in_b 입력 
    for(i <- 0 until ma_num) {
        mularraybundle(i).io.in_a := VecInit(buffvectorarray.map(_.io.out_a))
        //io.in_b의 i번째 element를 i번째 mularray에 입력(transpose를 안하기 위해)
        mularraybundle(i).io.in_b := io.in_b(i)
        // Create a condition that is true only for the selected mularray
        val is_selected_for_transpose = io.in_b_transpose && (io.in_fire_counter === i.U)
        val b_fire = RegNext(io.in_valid.reduce(_||_))
        // Use a Mux to provide the vector data only to the selected mularray.
        // Others get a zero vector. 0.U.asTypeOf(...) creates a wire of the correct type with all bits set to 0.
        mularraybundle(i).io.in_b_vec := Mux(is_selected_for_transpose, io.in_b_vec, 0.U.asTypeOf(io.in_b_vec))

        // The fire signal should also only go to the selected mularray in transpose mode.
        // In non-transpose mode, it is broadcast to all.
        mularraybundle(i).io.in_b_fire := Mux(io.in_b_transpose, Mux(is_selected_for_transpose, b_fire, false.B), b_fire)
        
        mularraybundle(i).io.in_fire_counter := io.in_fire_counter
        //in_valid, in_last 등의 신호 입력
        mularraybundle(i).io.in_valid :=  VecInit(buffvectorarray.map(_.io.out_valid))
        mularraybundle(i).io.in_last :=  VecInit(buffvectorarray.map(_.io.out_last))
        mularraybundle(i).io.in_id :=  VecInit(buffvectorarray.map(_.io.out_id))
        mularraybundle(i).io.in_prop :=  VecInit(buffvectorarray.map(_.io.out_prop))
        mularraybundle(i).io.in_b_transpose := io.in_b_transpose
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
    io.out_is_mpgemm := ShiftRegister(io.in_is_mpgemm, 3)
}
