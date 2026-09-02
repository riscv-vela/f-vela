package velaVPU.exu

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.tile.HasCoreParameters
import velaVPU.common._
import velaVPU.insns._
// import chisel3.util.experimental.loadMemoryFromFileInline

// -----------------------------------------------------------------------------
// RoPEUnit: vfrope.fvx 전용 실행 유닛 (8 요소, SEW=16 가정)
// Saturn의 PipelinedFunctionalUnit 인터페이스에 맞춰 stage valid는 io.pipe(k).valid 사용
// -----------------------------------------------------------------------------

object RoPEUnitFactory extends FunctionalUnitFactory {
  // woojin modify: val insns = Seq(VROPE) --> val insns = Seq(VROPE.VX.pipelined(4))로 변경 (RoPEUnit(depth=4)와 일치)
  // 2단(64x64) LUT 분할로 파이프 1단 추가 (depth 4 -> 5): row-select 레지스터 컷을 넣어
  // FireSim MidasTransforms(legacy SFC)의 ConstantPropagation이 4096-way 단일 mux tree를
  // 순회하다 StackOverflowError 나던 문제를 회피한다.
  val insns = Seq(VROPE.VX.pipelined(5)) // RoPEUnit(depth=5)와 일치: PipelinedExecution + PipelineStagesMinus1(4) 등록
  def generate(implicit p: Parameters): FunctionalUnit = new RoPEUnit
}

class RoPEUnit(implicit p: Parameters)
  extends PipelinedFunctionalUnit(depth = 5)
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

  // 내부 stage valid (LUT 2단 분할로 1단 추가: v0..v4, 총 5단)
  val v0 = io.pipe(0).valid
  val v1 = RegNext(v0, false.B)
  val v2 = RegNext(v1, false.B)
  val v3 = RegNext(v2, false.B)
  val v4 = RegNext(v3, false.B)

  // 내부 stage bits 전달
  val u0 = io.pipe(0).bits
  val u1 = RegEnable(u0, 0.U.asTypeOf(u0), v0)
  val u2 = RegEnable(u1, 0.U.asTypeOf(u1), v1)
  val u3 = RegEnable(u2, 0.U.asTypeOf(u2), v2)
  val u4 = RegEnable(u3, 0.U.asTypeOf(u3), v3)

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
  // S1: LUT row/col index 계산
  // q = m/64, r = m%64, i0 = (idx%128)/2 → [i0..i0+3]
  // row(q or r)로 64개 row 중 하나를 고르고, col(i)로 그 row 안에서 하나를 고른다
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

  val lutDepth = 4096
  val lutCols  = 64   // q, r, i 각각 0..63 (addr = row*64 + i, row-major 주소 체계 가정)
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

  // ---------------------------------------------------------------------------
  // 2단(64x64) LUT: 예전엔 4096-entry flat Vec을 동적 인덱스 8번(rdA/rdB x 4) 읽었는데,
  // FireSim MidasTransforms의 legacy SFC ConstantPropagation(regConstant$1)이 그 4096-way
  // mux tree를 하나의 레지스터 fan-in으로 재귀 순회하다가 StackOverflowError를 냈다.
  // row(64-way) select를 레지스터로 한 번 끊고 col(64-way) select를 다음 사이클에 하도록
  // 나눠서 fan-in을 4096 -> 64+64로 줄인다. (S1: row select 준비 → S2: row 레지스터 →
  // S3: row 안에서 col select = 실제 LUT 데이터 확정)
  // requires A_I_MAJOR=false, B_I_MAJOR=false (addr = row*64 + i, row-major)
  // ---------------------------------------------------------------------------
  require(!A_I_MAJOR && !B_I_MAJOR,
    "[RoPEUnit] 2-level LUT split assumes row-major addressing (addr = row*64 + i)")
  val lutA_2d = VecInit(loadLutHex("lutA.hex", 32).grouped(lutCols).map(VecInit(_)).toSeq)
  val lutB_2d = VecInit(loadLutHex("lutB.hex", 16).grouped(lutCols).map(VecInit(_)).toSeq)
  // ---------------------------------------------------------------------------

  // ---------------------------------------------------------------------------
  // S2 (신규): row select 레지스터 컷.
  // q/r로 64개 row 중 하나만 골라 레지스터에 담아, S3에서 그 64개짜리 row 안에서만
  // col(i) select 하도록 함 (SyncReadMem과 동일한 1-cycle read latency 유지).
  // ---------------------------------------------------------------------------
  val s2_rowA = RegEnable(lutA_2d(s1_q), VecInit(Seq.fill(lutCols)(0.U(32.W))), v1)
  val s2_rowB = RegEnable(lutB_2d(s1_r), VecInit(Seq.fill(lutCols)(0.U(16.W))), v1)
  val s2_i    = RegEnable(s1_i,          VecInit(Seq.fill(4)(0.U(6.W))),       v1)

  // 파이프라인 레지스터 (v1 -> v2)
  val s2_x   = RegEnable(s1_x,   VecInit(Seq.fill(8)(0.S(16.W))), v1)
  val s2_vd  = RegEnable(s1_vd,  0.U.asTypeOf(s1_vd),  v1)
  val s2_vm  = RegEnable(s1_vm,  false.B,              v1)
  val s2_vl  = RegEnable(s1_vl,  0.U.asTypeOf(s1_vl),  v1)
  val s2_sew = RegEnable(s1_sew, 0.U.asTypeOf(s1_sew), v1)
  val s2_idx = RegEnable(s1_idx, 0.U,                  v1)

  // ---------------------------------------------------------------------------
  // S3: row 안에서 col(i) select → 실제 LUT 데이터 확정 (rdA/rdB) + 데이터 변환
  // ---------------------------------------------------------------------------
  val rdA = Wire(Vec(4, UInt(32.W)))
  val rdB = Wire(Vec(4, UInt(16.W)))
  for (k <- 0 until 4) {
    rdA(k) := RegEnable(s2_rowA(s2_i(k)), v2)
    rdB(k) := RegEnable(s2_rowB(s2_i(k)), v2)
  }

  // 파이프라인 레지스터 (v2 -> v3)
  val s3_x   = RegEnable(s2_x,   VecInit(Seq.fill(8)(0.S(16.W))), v2)
  val s3_vd  = RegEnable(s2_vd,  0.U.asTypeOf(s2_vd),             v2)
  val s3_vm  = RegEnable(s2_vm,  false.B,                         v2)
  val s3_vl  = RegEnable(s2_vl,  0.U.asTypeOf(s2_vl),             v2)
  val s3_sew = RegEnable(s2_sew, 0.U.asTypeOf(s2_sew),            v2)
  val s3_idx = RegEnable(s2_idx, 0.U,                             v2)

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

  // 합성 (S3: rdA/rdB가 확정된 사이클에 바로 변환)
  val s3_cos_w = Wire(Vec(4, SInt(16.W)))
  val s3_sin_w = Wire(Vec(4, SInt(16.W)))
  val s3_sat_w = Wire(Bool())

  for (k <- 0 until 4) {
    s3_cos_w(k) := 0.S(16.W)
    s3_sin_w(k) := 0.S(16.W)
  }
  s3_sat_w := false.B

  when (v3) {
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

      s3_cos_w(k) := cSat
      s3_sin_w(k) := sSat
      anySat = anySat || cF || sF
    }
    s3_sat_w := anySat
  }

  // ---------------------------------------------------------------------------
  // S4: RoPE 회전 & Writeback
  // cos(mθᵢ) = cosA·cosB − sinA·sinB
  // sin(mθᵢ) = sinA·cosB + cosA·sinB
  // ---------------------------------------------------------------------------
  val s4_x   = RegEnable(s3_x,   VecInit(Seq.fill(8)(0.S(16.W))), v3)
  val s4_vd  = RegEnable(s3_vd,  0.U.asTypeOf(s3_vd),             v3)
  val s4_vm  = RegEnable(s3_vm,  false.B,                         v3)
  val s4_vl  = RegEnable(s3_vl,  0.U.asTypeOf(s3_vl),             v3)
  val s4_sew = RegEnable(s3_sew, 0.U.asTypeOf(s3_sew),            v3)
  val s4_idx = RegEnable(s3_idx, 0.U,                             v3)

  val s4_cos      = RegEnable(s3_cos_w, VecInit(Seq.fill(4)(0.S(16.W))), v3)
  val s4_sin      = RegEnable(s3_sin_w, VecInit(Seq.fill(4)(0.S(16.W))), v3)
  val s4_sat_trig = RegEnable(s3_sat_w, false.B, v3)

  val s4_y = Wire(Vec(8, SInt(16.W)))
  var anySatXY = false.B

  for (i <- 0 until 4) {
    val x0 = s4_x(2*i)
    val x1 = s4_x(2*i+1)
    val c  = s4_cos(i)
    val s  = s4_sin(i)

    val y0Raw = mulQ15(x0, c) - mulQ15(x1, s)
    val y1Raw = mulQ15(x1, c) + mulQ15(x0, s)
    val (y0, f0) = q15SatWithFlag(y0Raw)
    val (y1, f1) = q15SatWithFlag(y1Raw)

    s4_y(2*i)   := y0
    s4_y(2*i+1) := y1
    anySatXY = anySatXY || f0 || f1
  }

  // tail/마스크 처리
  val diff = Mux(s4_vl > s4_idx, s4_vl - s4_idx, 0.U)
  val lanesActive = Mux(diff >= 8.U, 8.U, diff(2,0))

  val s4_vmask = Mux(s4_vm, "hFF".U(8.W), u4.rmask(7,0))

  val s4_y_final = Wire(Vec(8, SInt(16.W)))
  for (i <- 0 until 8) {
    val active = (i.U < lanesActive) && s4_vmask(i).asBool
    s4_y_final(i) := Mux(active, s4_y(i), s4_x(i))
  }

  val s4_anySat = s4_sat_trig || anySatXY
  io.set_vxsat := v4 && s4_anySat

  val wbData = Cat(s4_y_final.reverse.map(_.asUInt)) // 128b

  io.write.valid     := v4
  io.write.bits.data := wbData
  // Woojin modify: idx >> 3.U 대신 wvd_eg 사용 (wvd_eg는 vd의 element-group index를 나타냄)
  io.write.bits.eg   := u4.wvd_eg       // 목적지 vd의 물리 element-group (idx가 아니라 wvd_eg 사용)
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
    printf(p"[RoPE S1] q=${s1_q} r=${s1_r} i0=${s1_i0}\n")
  }
  when (v3) {
    printf("[RoPE S3] rdA0=0x%x rdB0=0x%x cos0=0x%x sin0=0x%x sat=%d\n",
      rdA(0), rdB(0), s3_cos_w(0).asUInt, s3_sin_w(0).asUInt, s3_sat_w)
  }
  when (v4) {
    printf("[RoPE S4] y0=0x%x y1=0x%x y2=0x%x y3=0x%x\n",
      s4_y_final(0).asUInt, s4_y_final(1).asUInt, s4_y_final(2).asUInt, s4_y_final(3).asUInt)
    printf("[RoPE WB] valid=%d vd=%d eg=%d data=0x%x\n",
      io.write.valid, s4_vd, io.write.bits.eg, wbData)
  }
}
