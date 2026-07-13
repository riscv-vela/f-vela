package f_vela_saturn.exu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.tile.HasCoreParameters
import f_vela_saturn.common._
import f_vela_saturn.insns._
// import chisel3.util.experimental.loadMemoryFromFileInline

// -----------------------------------------------------------------------------
// RoPEUnit: vfrope.fvx 전용 실행 유닛 (8 요소, SEW=16 가정)
// Saturn의 PipelinedFunctionalUnit 인터페이스에 맞춰 stage valid는 io.pipe(k).valid 사용
// -----------------------------------------------------------------------------

object RoPEUnitFactory extends FunctionalUnitFactory {
  // woojin modify: val insns = Seq(VROPE) --> val insns = Seq(VROPE.VX.pipelined(4))로 변경 (RoPEUnit(depth=4)와 일치)
  val insns = Seq(VROPE.VX.pipelined(4)) // RoPEUnit(depth=4)와 일치: PipelinedExecution + PipelineStagesMinus1(3) 등록
  def generate(implicit p: Parameters): FunctionalUnit = new RoPEUnit
}

class RoPEUnit(implicit p: Parameters)
  extends PipelinedFunctionalUnit(depth = 4)
  with HasVectorParams
  with HasCoreParameters {

  // ==== LUT 규약 ==============================================================
  private val A_I_MAJOR     = false   // LUT_A는 q-major (addrA = q*64 + i)
  private val B_I_MAJOR     = false   // LUT_B는 r-major (addrB = r*64 + i)
  private val B_SIGNED_Q1P7 = true    // B는 signed INT8(Q1.7) 가정


  // ---------------------------------------------------------------------------
  // 공통 출력 기본값
  // ---------------------------------------------------------------------------
  io.stall  := false.B
  io.scalar_write.valid := false.B
  io.scalar_write.bits  := 0.U.asTypeOf(io.scalar_write.bits)
  io.set_vxsat          := false.B
  io.set_fflags.valid   := false.B
  io.set_fflags.bits    := 0.U

  // 내부 stage valid
  val v0 = io.pipe(0).valid
  val v1 = RegNext(v0, false.B)
  val v2 = RegNext(v1, false.B)
  val v3 = RegNext(v2, false.B)

  // 내부 stage bits 전달
  val u0 = io.pipe(0).bits
  val u1 = RegEnable(u0, 0.U.asTypeOf(u0), v0)
  val u2 = RegEnable(u1, 0.U.asTypeOf(u1), v1)
  val u3 = RegEnable(u2, 0.U.asTypeOf(u2), v2)

  // ---------------------------------------------------------------------------
  // S0: 입력 슬라이스 추출
  // ---------------------------------------------------------------------------
  val rs1_val = u0.scalar(31, 0)
  val s0_idx  = rs1_val(15, 0)      // idx
  val s0_m    = rs1_val(31, 16)     // m
  val s0_vd   = u0.rd
  val s0_vm   = u0.vm
  val s0_vl   = u0.vl
  val s0_sew  = u0.sew

  // vs2에서 8 elements x_idx..x_idx+7 추출
  val s0_xSlice = Wire(Vec(8, SInt(16.W)))
  for (i <- 0 until 8) {
    s0_xSlice(i) := u0.rvs2_data(16*(i+1)-1, 16*i).asSInt
  }

  // ---------------------------------------------------------------------------
  // S1: LUT index 계산 & 메모리 read 요청
  // q = m/64, r = m%64, i0 = (idx%128)/2 → [i0..i0+3]
  // A: addrA = q*64 + i, B: addrB = r*64 + i
  // ---------------------------------------------------------------------------
  val s1_q   = RegEnable(s0_m(11,6), 0.U, v0)                       // 0..63
  val s1_r   = RegEnable(s0_m( 5,0), 0.U, v0)                       // 0..63
  val s1_i0  = RegEnable(((s0_idx & 127.U) >> 1).asUInt, 0.U, v0)   // 0..63
  val s1_x   = RegEnable(s0_xSlice, VecInit(Seq.fill(8)(0.S(16.W))), v0)

  // 파이프라인 레지스터
  val s1_vd  = RegEnable(s0_vd,  0.U.asTypeOf(s0_vd),  v0)
  val s1_vm  = RegEnable(s0_vm,  false.B,              v0)
  val s1_vl  = RegEnable(s0_vl,  0.U.asTypeOf(s0_vl),  v0)
  val s1_sew = RegEnable(s0_sew, 0.U.asTypeOf(s0_sew), v0)
  val s1_idx = RegEnable(s0_idx, 0.U, v0)

  val s1_i = Wire(Vec(4, UInt(6.W)))
  s1_i(0) := s1_i0
  s1_i(1) := (s1_i0 + 1.U)(5,0)
  s1_i(2) := (s1_i0 + 2.U)(5,0)
  s1_i(3) := (s1_i0 + 3.U)(5,0)

  // 주소 생성 (order 스위치 반영)
  def addrA(q: UInt, i: UInt): UInt = if (A_I_MAJOR) Cat(i, q)(11,0) else Cat(q, i)(11,0)
  def addrB(r: UInt, i: UInt): UInt = if (B_I_MAJOR) Cat(i, r)(11,0) else Cat(r, i)(11,0)

  val lutDepth = 4096
  // A는 FP16(half) 저장, B는 INT8 저장 (기존 코드 + import chisel3.util.experimental.loadMemoryFromFileInline)
  // val lutA = SyncReadMem(lutDepth, UInt(32.W)) // {sinA, cosA}
  // val lutB = SyncReadMem(lutDepth, UInt(16.W)) // {sinB, cosB}
  // loadMemoryFromFileInline(lutA, "lutA.hex")
  // loadMemoryFromFileInline(lutB, "lutB.hex")
  
  // woojin modify (debugging용)
  // ---------------------------------------------------------------------------
  // LUT를 elaboration(빌드) 시점에 hex에서 읽어 상수 ROM(VecInit)으로 굽는다.
  // 이유: loadMemoryFromFileInline은 firtool(CIRCT)의 메모리 매크로 추출
  //   (split_lutA_ext) 과정에서 $readmemh 초기화가 누락되어, Verilator가
  //   RANDOMIZE_MEM_INIT로 초기화된 SRAM을 읽어 LUT 값이 전부 쓰레기가 됐음.
  //   ROM으로 구우면 값이 넷리스트에 리터럴로 박혀 런타임 파일/매크로 의존이 사라진다.
  // A는 FP16(half) {sinA[31:16], cosA[15:0]}, B는 INT8 {sinB[15:8], cosB[7:0]}
  // ---------------------------------------------------------------------------
  private def loadLutHex(fname: String, width: Int): Seq[UInt] = {
    val candidates = Seq(
      fname,
      s"generators/_f_vela/software/test/rv_rope_test/$fname",
      s"../../generators/_f_vela/software/test/rv_rope_test/$fname",
      s"/home/woojin/_risc_vela/chipyard/generators/_f_vela/software/test/rv_rope_test/$fname"
    )
    val path = candidates.find(p => new java.io.File(p).exists).getOrElse(
      throw new RuntimeException(
        s"[RoPEUnit] LUT hex not found: $fname (tried: ${candidates.mkString(", ")})"))
    val vals = scala.io.Source.fromFile(path).getLines()
      .map(_.trim).filter(_.nonEmpty).map(s => BigInt(s, 16).U(width.W)).toSeq
    require(vals.length == lutDepth,
      s"[RoPEUnit] $path has ${vals.length} entries, expected $lutDepth")
    vals
  }

  val lutA_rom = VecInit(loadLutHex("lutA.hex", 32))
  val lutB_rom = VecInit(loadLutHex("lutB.hex", 16))
  // ---------------------------------------------------------------------------

  val s1_addrA  = Wire(Vec(4, UInt(12.W)))
  val s1_addrB  = Wire(Vec(4, UInt(12.W)))
  val rdA       = Wire(Vec(4, UInt(32.W)))
  val rdB       = Wire(Vec(4, UInt(16.W)))

  for (k <- 0 until 4) {
    s1_addrA(k) := addrA(s1_q, s1_i(k)) // q*64 + i
    s1_addrB(k) := addrB(s1_r, s1_i(k)) // r*64 + i

    // SyncReadMem과 동일한 1-cycle read latency 유지 (S1 주소 → S2 데이터)
    rdA(k) := RegEnable(lutA_rom(s1_addrA(k)), v1)
    rdB(k) := RegEnable(lutB_rom(s1_addrB(k)), v1)
  }

  // ---------------------------------------------------------------------------
  // S2: LUT 데이터 변환 (고정소수점 유틸 & 포맷 변환)
  // ---------------------------------------------------------------------------
  val s2_x   = RegEnable(s1_x,   VecInit(Seq.fill(8)(0.S(16.W))), v1)
  val s2_vd  = RegEnable(s1_vd,  0.U.asTypeOf(s1_vd),             v1)
  val s2_vm  = RegEnable(s1_vm,  false.B,                         v1)
  val s2_vl  = RegEnable(s1_vl,  0.U.asTypeOf(s1_vl),             v1)
  val s2_sew = RegEnable(s1_sew, 0.U.asTypeOf(s1_sew),            v1)
  val s2_idx = RegEnable(s1_idx, 0.U,                             v1)

  def q15SatWithFlag(x: SInt): (SInt, Bool) = {
    val max = ( 32767).S(16.W); val min = (-32768).S(16.W)
    val gt = x > max; val lt = x < min
    val y  = Mux(gt, max, Mux(lt, min, x))
    (y, gt || lt)
  }

  def mulQ15(a: SInt, b: SInt): SInt = {
    val a32 = a.pad(32)
    val b32 = b.pad(32)
    val p   = a32 * b32
    val rnd = (1.S(p.getWidth.W) << 14)
    val pAdj = Mux(p >= 0.S, p + rnd, p - rnd)
    (pAdj >> 15).asSInt
  }

  // FP16 → Q1.15 변환
  def fp16_to_q15(h: UInt): SInt = {
    val sign = h(15)
    val exp  = h(14,10)   // 0..31
    val frac = h(9,0)     // 10b

    val isZero    = exp === 0.U && frac === 0.U
    val isSubNorm = exp === 0.U && frac =/= 0.U
    val isInfNaN  = exp === 31.U

    val mag32 = Wire(UInt(32.W))
    when (isZero) { mag32 := 0.U }
    .elsewhen (isSubNorm) { mag32 := ((frac + 256.U) >> 9).asUInt }
    .elsewhen (isInfNaN) { mag32 := 32768.U }
    .otherwise {
      val mant = 1024.U + frac
      val e    = exp
      val val32 = Wire(UInt(32.W))
      when (e >= 10.U) { val32 := (mant << (e - 10.U))(31,0) }
      .otherwise {
        val shift = (10.U - e)(4,0)
        val add   = (1.U << (shift - 1.U))(31,0)
        val32 := ((mant + add) >> shift)(31,0)
      }
      mag32 := val32
    }
    val pos = Mux(mag32 >= 32767.U, 32767.U, mag32)(15,0).asSInt
    val neg = Mux(mag32 >= 32768.U, 32768.U, mag32)(15,0).asSInt
    Mux(sign, -neg, pos)
  }

  // INT8 → Q1.15 변환
  def b8_to_q15(x: UInt): SInt = {
    val s8: SInt = if (B_SIGNED_Q1P7) x.asSInt else (x.asSInt - 128.S(9.W)).asSInt
    (s8 << 8).asSInt
  }

  // 합성
  val s2_cos_w = Wire(Vec(4, SInt(16.W)))
  val s2_sin_w = Wire(Vec(4, SInt(16.W)))
  val s2_sat_w = Wire(Bool())

  for (k <- 0 until 4) {
    s2_cos_w(k) := 0.S(16.W)
    s2_sin_w(k) := 0.S(16.W)
  }
  s2_sat_w := false.B

  when (v2) {
    var anySat = false.B
    for (k <- 0 until 4) {
      val cosA_q15 = fp16_to_q15(rdA(k)(15, 0))
      val sinA_q15 = fp16_to_q15(rdA(k)(31,16))
      val cosB_q15 = b8_to_q15 (rdB(k)(7, 0))
      val sinB_q15 = b8_to_q15 (rdB(k)(15,8))

      val cRaw = mulQ15(cosA_q15, cosB_q15) - mulQ15(sinA_q15, sinB_q15)
      val sRaw = mulQ15(sinA_q15, cosB_q15) + mulQ15(cosA_q15, sinB_q15)
      val (cSat, cF) = q15SatWithFlag(cRaw)
      val (sSat, sF) = q15SatWithFlag(sRaw)

      s2_cos_w(k) := cSat
      s2_sin_w(k) := sSat
      anySat = anySat || cF || sF
    }
    s2_sat_w := anySat
  }
  
  // ---------------------------------------------------------------------------
  // S3: RoPE 회전 & Writeback
  // cos(mθᵢ) = cosA·cosB − sinA·sinB
  // sin(mθᵢ) = sinA·cosB + cosA·sinB
  // ---------------------------------------------------------------------------
  val s3_x   = RegEnable(s2_x,   VecInit(Seq.fill(8)(0.S(16.W))), v2)
  val s3_vd  = RegEnable(s2_vd,  0.U.asTypeOf(s2_vd),             v2)
  val s3_vm  = RegEnable(s2_vm,  false.B,                         v2)
  val s3_vl  = RegEnable(s2_vl,  0.U.asTypeOf(s2_vl),             v2)
  val s3_sew = RegEnable(s2_sew, 0.U.asTypeOf(s2_sew),            v2)
  val s3_idx = RegEnable(s2_idx, 0.U,                             v2)

  val s3_cos      = RegEnable(s2_cos_w, VecInit(Seq.fill(4)(0.S(16.W))), v2)
  val s3_sin      = RegEnable(s2_sin_w, VecInit(Seq.fill(4)(0.S(16.W))), v2)
  val s3_sat_trig = RegEnable(s2_sat_w, false.B, v2)

  val s3_y = Wire(Vec(8, SInt(16.W)))
  var anySatXY = false.B

  for (i <- 0 until 4) {
    val x0 = s3_x(2*i)
    val x1 = s3_x(2*i+1)
    val c  = s3_cos(i)
    val s  = s3_sin(i)

    val y0Raw = mulQ15(x0, c) - mulQ15(x1, s)
    val y1Raw = mulQ15(x1, c) + mulQ15(x0, s)
    val (y0, f0) = q15SatWithFlag(y0Raw)
    val (y1, f1) = q15SatWithFlag(y1Raw)

    s3_y(2*i)   := y0
    s3_y(2*i+1) := y1
    anySatXY = anySatXY || f0 || f1
  }

  // tail/마스크 처리
  val diff = Mux(s3_vl > s3_idx, s3_vl - s3_idx, 0.U)
  val lanesActive = Mux(diff >= 8.U, 8.U, diff(2,0))

  val s3_vmask = Mux(s3_vm, "hFF".U(8.W), u3.rmask(7,0))

  val s3_y_final = Wire(Vec(8, SInt(16.W)))
  for (i <- 0 until 8) {
    val active = (i.U < lanesActive) && s3_vmask(i).asBool
    s3_y_final(i) := Mux(active, s3_y(i), s3_x(i))
  }

  val s3_anySat = s3_sat_trig || anySatXY
  io.set_vxsat := v3 && s3_anySat

  val wbData = Cat(s3_y_final.reverse.map(_.asUInt)) // 128b

  io.write.valid     := v3
  io.write.bits.data := wbData
  // Woojin modify: s3_idx >> 3.U 대신 u3.wvd_eg 사용 (wvd_eg는 vd의 element-group index를 나타냄)
  io.write.bits.eg   := u3.wvd_eg       // 목적지 vd의 물리 element-group (idx가 아니라 wvd_eg 사용)
  io.write.bits.mask := Fill(128, 1.U)
  
  // ---------------------------------------------------------------------------
  // 디버그 로그: 각 파이프 스테이지가 유효할 때 출력
  // (+verbose 추가 후 verilator 실행 시 build/*.log 및 콘솔 stdout으로 나감)
  // ---------------------------------------------------------------------------
  when (v0) {
    printf(p"[RoPE S0] m=${s0_m} idx=${s0_idx} vd=${s0_vd} vl=${s0_vl} sew=${s0_sew}\n")
    printf("[RoPE S0] x0=0x%x x1=0x%x x2=0x%x x3=0x%x\n",
      s0_xSlice(0).asUInt, s0_xSlice(1).asUInt, s0_xSlice(2).asUInt, s0_xSlice(3).asUInt)
  }
  when (v1) {
    printf(p"[RoPE S1] q=${s1_q} r=${s1_r} i0=${s1_i0} addrA0=${s1_addrA(0)} addrB0=${s1_addrB(0)}\n")
  }
  when (v2) {
    printf("[RoPE S2] rdA0=0x%x rdB0=0x%x cos0=0x%x sin0=0x%x sat=%d\n",
      rdA(0), rdB(0), s2_cos_w(0).asUInt, s2_sin_w(0).asUInt, s2_sat_w)
  }
  when (v3) {
    printf("[RoPE S3] y0=0x%x y1=0x%x y2=0x%x y3=0x%x\n",
      s3_y_final(0).asUInt, s3_y_final(1).asUInt, s3_y_final(2).asUInt, s3_y_final(3).asUInt)
    printf("[RoPE WB] valid=%d vd=%d eg=%d data=0x%x\n",
      io.write.valid, s3_vd, io.write.bits.eg, wbData)
  }
}
