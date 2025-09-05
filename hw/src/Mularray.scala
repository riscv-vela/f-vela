package gemmini

import chisel3._
import chisel3.util._
import GemminiISA._
import Util._

class AdderTree[T <: Data : Arithmetic](outputType: T, ma_length: Int, max_simultaneous_matmuls: Int)(implicit ev: Arithmetic[T]) extends Module {
    import ev._
    val io = IO(new Bundle {
        val in = Input(Vec(ma_length, outputType))
        val in_last = Input(Vec(ma_length, Bool()))
        val in_valid = Input(Vec(ma_length, Bool()))
        val in_id = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))
        val in_prop = Input(Vec(ma_length, Bool()))

        val out = Output(outputType)
        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output( UInt(log2Up(max_simultaneous_matmuls).W))
        val out_prop = Output(Bool())
    })

    def treeAdd(n: Seq[T]): T = {
        if (n.length == 1) {
            n.head
        } else {
            val result = n.grouped(2).map {
                case Seq(a, b) => a + b
                case Seq(a) => a
            }.toSeq

            treeAdd(result)
        }
    }

    io.out := treeAdd(io.in).clippedToWidthOf(outputType)
    io.out_valid := io.in_valid.reduce(_||_)
    io.out_last := io.in_last.reduce(_||_)
    io.out_id := io.in_id.head
    io.out_prop := io.in_prop.reduce(_||_)

}

//한 사이클에 B matrix 값 하나씩 받도록 FSM 로직 설정해야함. Transpose를 안하기 위해서.
class Mularray[T <: Data](inputType: T, outputType: T, ma_length: Int, max_simultaneous_matmuls: Int) (implicit ev: Arithmetic[T]) extends Module {
    import ev._
    val io = IO(new Bundle {
        val in_a = Input(Vec(ma_length, inputType))
        val in_b = Input(inputType)
         

        val in_last = Input(Vec(ma_length, Bool()))
        val in_valid = Input(Vec(ma_length, Bool()))
        val in_id = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))
        val in_prop = Input(Vec(ma_length, Bool()))
        val in_fire_counter = Input(UInt(log2Up(ma_length).W))
        val in_valid_b = Input(Vec(ma_length, Bool()))
        val in_b_fire = Input(Bool())

        val out_sum = Output(outputType)
        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output( UInt(log2Up(max_simultaneous_matmuls).W))
        val out_prop = Output(Bool())
    })

    val adderTree = Module(new AdderTree(outputType, ma_length, max_simultaneous_matmuls))
    val pe_array = Seq.fill(ma_length) {Module(new PE(inputType, outputType, max_simultaneous_matmuls))}


    //각 PE에 in_a 입력 연결
    pe_array.zipWithIndex.foreach { case (pe, i) =>
        pe.io.in_a := io.in_a(i)
        pe.io.in_valid := io.in_valid(i)
        pe.io.in_last := io.in_last(i)
        pe.io.in_id := io.in_id(i)
        pe.io.in_prop := io.in_prop(i)
        pe.io.in_valid_b := io.in_valid_b(i)
    }

    // 선택된 PE 만 진짜 입력 연결

    for ((pe, idx) <- pe_array.zipWithIndex) {
        val sel = io.in_fire_counter === idx.U
        pe.io.in_b      := Mux(sel, io.in_b,      0.U.asTypeOf(inputType))
        pe.io.in_b_fire := Mux(sel, io.in_b_fire, false.B)
    }

    //각 pe의 결과 값을 adderTree의 입력으로 연결
    adderTree.io.in := VecInit(pe_array.map(_.io.out_result))
    adderTree.io.in_valid := VecInit(pe_array.map(_.io.out_valid))
    adderTree.io.in_last := VecInit(pe_array.map(_.io.out_last))
    adderTree.io.in_id := VecInit(pe_array.map(_.io.out_id))
    adderTree.io.in_prop := VecInit(pe_array.map(_.io.out_prop))


    //adder Tree 결과값 연결
    io.out_sum := adderTree.io.out
    io.out_valid := adderTree.io.out_valid
    io.out_last := adderTree.io.out_last
    io.out_id := adderTree.io.out_id
    io.out_prop := adderTree.io.out_prop

}

