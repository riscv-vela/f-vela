package gemmini

import chisel3._
import chisel3.util._

class TernaryMulUnit(val inWidth: Int) extends Module {
  val io = IO(new Bundle {
    val in_a = Input(SInt(inWidth.W))
    val in_b = Input(SInt(2.W))
    val out_result = Output(SInt(inWidth.W))
  })
  val b1 = io.in_b(1)
  val b0 = io.in_b(0)
  val inverted_a = io.in_a ^ Fill(inWidth, b1).asSInt
  val sum = (inverted_a.asUInt + b1).asSInt
  io.out_result := sum & Fill(inWidth, b0).asSInt
}

class MpMulUnit[T <: Data](inputType: T, weightType: T) (implicit ev: Arithmetic[T]) extends Module {
  import ev._
  val io = IO(new Bundle {
    val in_a = Input(inputType)
    val in_b = Input(weightType)
    val out_result = Output(inputType)
  })

  // 타입 확인: 입력이 SInt이고 가중치가 SInt(2.W)인지 확인
  val isTernary = inputType.isInstanceOf[SInt] && weightType.isInstanceOf[SInt] && weightType.getWidth == 2

  if (isTernary) {
    // === 조건이 맞으면, 최적화된 유닛을 인스턴스화하여 연결 ===
    val optimized_unit = Module(new TernaryMulUnit(inputType.getWidth))
    
    // 타입 캐스팅: 제네릭 T를 구체적인 SInt로 변환하여 연결
    optimized_unit.io.in_a := io.in_a.asTypeOf(SInt(inputType.getWidth.W))
    optimized_unit.io.in_b := io.in_b.asTypeOf(SInt(2.W))
    io.out_result := optimized_unit.io.out_result.asTypeOf(inputType)

  } else {
    // === 조건이 맞지 않으면, 일반적인 곱셈으로 안전하게 동작 (Fallback) ===
    // weightType을 inputType과 같은 T 타입으로 변환 후 곱셈
    // TODO: 
    val product = io.in_a * io.in_b.asTypeOf(inputType)
    io.out_result := product.clippedToWidthOf(inputType)
  }
}

class MpPE[T <: Data :Arithmetic](inputType: T, weightType: T, max_simultaneous_matmuls: Int)(implicit ev: Arithmetic[T]) extends Module{ 
    import ev._  
    val io = IO(new Bundle {
        val in_a = Input(inputType)
        val in_b = Input(weightType)
        val out_result = Output(inputType)

        val in_last = Input(Bool())
        val in_valid = Input(Bool())
        val in_id = Input( UInt(log2Up(max_simultaneous_matmuls).W))
        val in_prop = Input(Bool())
        val in_b_fire = Input(Bool())

        val out_last = Output(Bool())
        val out_valid = Output(Bool())
        val out_id = Output( UInt(log2Up(max_simultaneous_matmuls).W))
    })

    val mul_unit = Module(new MpMulUnit(inputType, weightType)(ev))
    
    when(io.in_valid){
      mul_unit.io.in_a := io.in_a
    } .otherwise{
      mul_unit.io.in_a := 0.U.asTypeOf(inputType)
    }
    val c1 = RegInit(0.U.asTypeOf(weightType))
    val c2 = RegInit(0.U.asTypeOf(weightType))
    //in_prop이 true이면, c1이 preload, c2가 mul에 사용

    //io.in_b_fire가 true일 때만 기록

    // 현재 토글 상태에 따라 반대쪽 버퍼를 읽어 곱셈에 사용
    mul_unit.io.in_b := 0.U.asTypeOf(weightType)
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