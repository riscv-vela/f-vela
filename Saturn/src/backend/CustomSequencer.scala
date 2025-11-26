package saturn.backend

import chisel3._
import chisel3.util._
import chisel3.experimental.dataview._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import saturn.common._
import saturn.insns._
import freechips.rocketchip.regmapper.RRTestRequest.busy
import hardfloat._
import svsim.Backend
import hardfloat._
import freechips.rocketchip.tile.FType
import dataclass.data
import saturn.common.FloatCasts._
import saturn.common.FloatCasts.{FP64U => FP64, FP32U => FP32}

class CustomExecuteMicroOp(implicit p: Parameters) extends CoreBundle()(p) with HasVectorParams{
    
    val eg = UInt(log2Ceil(egsTotal).W) // eg id
    val eidx = UInt(log2Ceil(maxVLMax).W) // element index
    val dt = UInt(64.W)
    val dt_inv = UInt(64.W)

    // for clearVat
    val tail = Bool()                  // 이 마이크로옵이 이 명령의 "마지막" 발행인지
    val vat  = UInt(vParams.vatSz.W)   // age tag
    val isF32 = Bool()               // f32 모드인지 여부
}

class CustomSequencerIO(maxDepth: Int)(implicit p: Parameters) extends SequencerIO(new CustomExecuteMicroOp){
    val read_req = Decoupled(new VectorReadReq)
    val pipe_write_req = new VectorPipeWriteReqIO(maxDepth)
    /*
    // From issue queue
    val dis = Flipped(Decoupled(new BackendIssueInst))
    val dis_stall = Input(Bool()) // used to disable OOO

    // Emits pending reads/writes + age
    val seq_hazard = Output(Valid(new SequencerHazard))
    val vat = Output(UInt(vParams.vatSz.W))

    // Consumes older reads/writes
    val older_writes = Input(UInt(egsTotal.W))
    val older_reads  = Input(UInt(egsTotal.W))

    // Used to determine when this is the oldest insn
    val vat_head = Input(UInt(vParams.vatSz.W))

    val busy = Output(Bool())
    val head = Output(Bool())

    // Issued operation
    val iss = Decoupled(issType)
    */
}

class CustomSequencer(
    supported_insns: Seq[VectorInstruction],
    maxPipeDepth:Int,
    exuPipeDepth: Int = 7
    )(implicit p: Parameters)
    extends Sequencer[CustomExecuteMicroOp]()(p) with HasVectorParams {
    // 이 시퀀서가 받을 명령인지 (커스텀 명령만 등록해서 매치)
    def accepts(inst: VectorIssueInst): Bool = !inst.vmu && new VectorDecoder(inst, supported_insns, Nil).matched

    val io = IO(new CustomSequencerIO(maxPipeDepth))

    // 필요 reg들
    val inst = Reg(new BackendIssueInst)
    val haveInst = RegInit(false.B)
    val curReg = Reg(UInt(5.W)) // 현재 레지스터
    val maxReg = Reg(UInt(5.W)) // 최대 레지스터
    val head = Reg(Bool()) // 현재 명령이 head인지 여부
    val eidx = Reg(UInt(log2Ceil(maxVLMax).W)) // 현재 eg의 첫 element index (0 또는 4임)
    val rvs2_mask = Reg(UInt(egsTotal.W))
    val wvd_mask = Reg(UInt(egsTotal.W))
    val isF32_reg = RegInit(false.B) // 현재 명령이 f32 모드인지 여부


    // 필요 변수들
    val next_eidx = get_next_eidx(inst.vconfig.vl, eidx, 3.U, false.B, false.B, false.B, inst.dLen)
    val tail = (curReg === maxReg) && (next_eidx === inst.vconfig.vl) // 현재 명령의 tail 여부
    val curEg = getEgId(curReg, eidx, 3.U, false.B) // 현재 eg idx
    val pipe_stages = (exuPipeDepth - 1).U
    val isF32 = (io.dis.bits.vconfig.vtype.vsew === 2.U) // 현재 명령이 f32 모드인지 여부

    // exu 파이프 스케줄러????
    val exu_scheduler = Module(new PipeScheduler(1, maxPipeDepth))
    exu_scheduler.io.reqs(0).request := haveInst
    exu_scheduler.io.reqs(0).fire := io.iss.fire
    exu_scheduler.io.reqs(0).depth := pipe_stages

    // 헬퍼
    def regMin(a: UInt, b: UInt) = Mux(a <= b, a, b) // vs2 뽑아내기
    def regMax(a: UInt, b: UInt) = Mux(a >= b, a, b) // vd 뽑아내기
    def custom_get_arch_mask(lo: UInt, hi: UInt): UInt = {
        // VecInit을 사용하여 조합 논리 루프 방지
        val mask_bits = VecInit((0 until 32).map { i =>
            (i.U >= lo) && (i.U <= hi)
        })
        
        // 비트들을 결합하여 최종 마스크 생성
        mask_bits.asUInt
    }






    // ===================== Dispatch (input 들어왔을 때) =====================
    io.dis.ready := (!haveInst || (tail && io.iss.fire)) && !io.dis_stall

    when(io.dis.fire){
        inst := io.dis.bits
        haveInst := true.B
        val lo = regMin(io.dis.bits.rs2, io.dis.bits.rd) // vs2
        val hi = regMax(io.dis.bits.rs2, io.dis.bits.rd) // vd
        curReg := lo
        maxReg := hi
        head := true.B
        eidx := 0.U
    }.elsewhen(io.iss.fire){
        head := false.B
        haveInst := !tail
    }

    when(io.dis.fire){
        isF32_reg := isF32
    }.elsewhen(io.iss.fire && tail){
        isF32_reg := false.B
    }


    // ===================== dt_inv 계산 =====================
    val F = FType.D
    val div = Module(new DivSqrtRecFN_small(F.exp, F.sig, 0))
    val dt_inv_valid = RegInit(false.B)
    div.io.detectTininess := consts.tininess_afterRounding
    div.io.roundingMode   := 0.U // RNE
    div.io.sqrtOp         := false.B
    div.io.a := F.recode("h3FF0000000000000".U(64.W)) // 1.0 recoded

    // dt_inv 패스트 매칭
    def u64(x: String) = x.U(64.W)
    val DT_0001 = u64("h3F50624DD2F1A9FC") // 0.001
    val DT_0002 = u64("h3F60624DD2F1A9FC") // 0.002
    val DT_0004 = u64("h3F70624DD2F1A9FC") // 0.004
    val DT_0008 = u64("h3F80624DD2F1A9FC") // 0.008
    val DT_0010 = u64("h3F847AE147AE147B") // 0.01
    val DT_0100 = u64("h3FB999999999999A") // 0.1
    val DT_1000 = u64("h3FF0000000000000") // 1.0
    val INV_0001 = u64("h408F400000000000") // 1000.0
    val INV_0002 = u64("h407F400000000000") // 500.0
    val INV_0004 = u64("h406F400000000000") // 250.0
    val INV_0008 = u64("h405F400000000000") // 125.0
    val INV_0010 = u64("h4059000000000000") // 100.0
    val INV_0100 = u64("h4024000000000000") // 10.0
    val INV_1000 = u64("h3FF0000000000000") // 1.0

    // ==== F32 dt constants (IEEE-32) ====
    def u32(x: String) = x.U(32.W)
    val DT32_0001 = u32("h3A83126F") // 0.001f
    val DT32_0002 = u32("h3B03126F") // 0.002f
    val DT32_0004 = u32("h3B83126F") // 0.004f
    val DT32_0008 = u32("h3C03126F") // 0.008f
    val DT32_0010 = u32("h3C23D70A") // 0.01f
    val DT32_0100 = u32("h3DCCCCCD") // 0.1f
    val DT32_1000 = u32("h3F800000") // 1.0f

    // dt 래치 (새 명령 디스패치 시)
    val dt = io.dis.bits.rs1_data
    val dt32 = Mux(isF32, dt(31, 0), 0.U(32.W))
    val dt32_reg = RegInit(0.U(32.W))
    val dt64 = Mux(isF32, f32_to_f64_ieee(dt32), dt)
    val dt64_reg = RegInit(0.U(64.W))
    val dt_reg = RegInit(0.U(64.W))
    when (io.dis.fire) {
      dt_reg := dt64
      dt32_reg := dt32
      dt64_reg := dt64
    }
    // 패스트 매치 검출
    // val fast_hit_next =
    //     (dt === DT_0001) || (dt === DT_0002) || (dt === DT_0004) || (dt === DT_0008) ||
    //     (dt === DT_0010) || (dt === DT_0100) || (dt === DT_1000) ||
    //     (dt32 === DT32_0001) || (dt32 === DT32_0002) || (dt32 === DT32_0004) || (dt32 === DT32_0008) ||
    //     (dt32 === DT32_0010) || (dt32 === DT32_0100) || (dt32 === DT32_1000)
    // val fast_hit = RegInit(false.B)
    // val fast_inv_ieee = Mux1H(Seq(
    // (Mux(isF32_reg, dt32_reg === DT32_0001, dt64_reg === DT_0001)) -> INV_0001,
    // (Mux(isF32_reg, dt32_reg === DT32_0002, dt64_reg === DT_0002)) -> INV_0002,
    // (Mux(isF32_reg, dt32_reg === DT32_0004, dt64_reg === DT_0004)) -> INV_0004,
    // (Mux(isF32_reg, dt32_reg === DT32_0008, dt64_reg === DT_0008)) -> INV_0008,
    // (Mux(isF32_reg, dt32_reg === DT32_0010, dt64_reg === DT_0010)) -> INV_0010,
    // (Mux(isF32_reg, dt32_reg === DT32_0100, dt64_reg === DT_0100)) -> INV_0100,
    // (Mux(isF32_reg, dt32_reg === DT32_1000, dt64_reg === DT_1000)) -> INV_1000
    // ))

    val fast_hit_dt0001 = Mux(isF32, dt32 === DT32_0001, dt64 === DT_0001)
    val fast_hit_dt0002 = Mux(isF32, dt32 === DT32_0002, dt64 === DT_0002)
    val fast_hit_dt0004 = Mux(isF32, dt32 === DT32_0004, dt64 === DT_0004)
    val fast_hit_dt0008 = Mux(isF32, dt32 === DT32_0008, dt64 === DT_0008)
    val fast_hit_dt0010 = Mux(isF32, dt32 === DT32_0010, dt64 === DT_0010)
    val fast_hit_dt0100 = Mux(isF32, dt32 === DT32_0100, dt64 === DT_0100)
    val fast_hit_dt1000 = Mux(isF32, dt32 === DT32_1000, dt64 === DT_1000)
    val fast_hit_dt0001_reg = RegInit(false.B)
    val fast_hit_dt0002_reg = RegInit(false.B)
    val fast_hit_dt0004_reg = RegInit(false.B)
    val fast_hit_dt0008_reg = RegInit(false.B)
    val fast_hit_dt0010_reg = RegInit(false.B)
    val fast_hit_dt0100_reg = RegInit(false.B)
    val fast_hit_dt1000_reg = RegInit(false.B)
    when (io.dis.fire) {
      fast_hit_dt0001_reg := fast_hit_dt0001
      fast_hit_dt0002_reg := fast_hit_dt0002
      fast_hit_dt0004_reg := fast_hit_dt0004
      fast_hit_dt0008_reg := fast_hit_dt0008
      fast_hit_dt0010_reg := fast_hit_dt0010
      fast_hit_dt0100_reg := fast_hit_dt0100
      fast_hit_dt1000_reg := fast_hit_dt1000
    }
    val fast_inv_ieee = Mux1H(Seq(
      fast_hit_dt0001_reg -> INV_0001,
      fast_hit_dt0002_reg -> INV_0002,
      fast_hit_dt0004_reg -> INV_0004,
      fast_hit_dt0008_reg -> INV_0008,
      fast_hit_dt0010_reg -> INV_0010,
      fast_hit_dt0100_reg -> INV_0100,
      fast_hit_dt1000_reg -> INV_1000
    ))
    val fast_hit = fast_hit_dt0001_reg || fast_hit_dt0002_reg ||
                   fast_hit_dt0004_reg || fast_hit_dt0008_reg ||
                   fast_hit_dt0010_reg || fast_hit_dt0100_reg ||
                   fast_hit_dt1000_reg
    val fast_inv_recoded = FP64.F.recode(fast_inv_ieee)
    
    // ============항상 래치된 dt를 b로 recode-->이거는 fast 안되었을 때 div에 넣음===========
    div.io.b := FP64.F.recode(dt_reg)

    // launch -> inReady 시점 start 펄스 → divider 시작
    val launch = RegInit(false.B)
    when (io.dis.fire) { 
      launch := true.B
    }
    val start = launch && div.io.inReady && !fast_hit // 패스트 매치 아닐때 div 연산
    div.io.inValid := start
    when (start || fast_hit) {
      launch := false.B
    }

    // 결과 래치 (recoded 65b)
    val dt_inv_recoded = RegInit(0.U(65.W))
    val dt_inv_ready   = RegInit(false.B)
    when (io.dis.fire) {
      dt_inv_ready := false.B
    }.elsewhen(fast_hit){
      dt_inv_recoded := fast_inv_recoded
      dt_inv_ready   := true.B
    }.elsewhen(div.io.outValid_div) {
      dt_inv_recoded := div.io.out
      dt_inv_ready   := true.B
    }
    val dt_inv_ieee = FP64.F.ieee(dt_inv_recoded)

    // ===================== output =====================
    io.busy := haveInst
    io.head := head
    io.vat := inst.vat

    // ===================== Hazard =====================
    io.seq_hazard.valid := haveInst
    when(io.dis.fire){
        rvs2_mask := FillInterleaved(egsPerVReg, custom_get_arch_mask(io.dis.bits.rs2, io.dis.bits.rd)) // emul 1이 일단 0.U로 하는 걸로 아는데 틀리면 수정 필요
        wvd_mask := FillInterleaved(egsPerVReg, custom_get_arch_mask(io.dis.bits.rs2, io.dis.bits.rd)) // emul 1이 일단 0.U로 하는 걸로 아는데 틀리면 수정 필요
    }
    val curEgOH = UIntToOH(curEg, egsTotal)
    when(io.read_req.fire){
        rvs2_mask := rvs2_mask & ~curEgOH
    }
    when(io.pipe_write_req.fire){
        wvd_mask := wvd_mask & ~curEgOH
    }

    val vs2_read_oh = curEgOH                      // 이 시퀀서는 항상 vs2를 읽음(설계상 renv2=true)
    val vd_write_oh = curEgOH                      // 목적지도 curEg에 씀(설계상 elementwise)

    // === older_* 와의 충돌: RAW/WAW/WAR ===
    val raw_hazard = (vs2_read_oh & io.older_writes) =/= 0.U
    val waw_hazard = (vd_write_oh & io.older_writes) =/= 0.U
    val war_hazard = (vd_write_oh & io.older_reads ) =/= 0.U
    val data_hazard = raw_hazard || waw_hazard || war_hazard
    
    io.seq_hazard.bits.vat := inst.vat
    io.seq_hazard.bits.rintent := rvs2_mask
    io.seq_hazard.bits.wintent := wvd_mask

    // ===================== Read_req to VRF =====================
    io.read_req.valid := haveInst && dt_inv_ready
    io.read_req.bits.eg := curEg
    io.read_req.bits.oldest := inst.vat === io.vat_head // 해당 inst의 vat가 head인지 아닌지 여부

    // ===================== Write_req to VRF =====================
    io.pipe_write_req.request := haveInst // 추가적으로 exu랑 맞추는 시그널 필요
    io.pipe_write_req.bank_sel := (if (vrfBankBits == 0) 1.U else UIntToOH(curEg(vrfBankBits-1,0)))
    io.pipe_write_req.pipe_depth := pipe_stages
    io.pipe_write_req.oldest := inst.vat === io.vat_head
    io.pipe_write_req.fire := io.iss.fire

    // ===================== Data to Exu 매 사이클 =====================
    io.iss.valid := haveInst && io.read_req.ready && dt_inv_ready && !data_hazard &&
                    (inst.bits(31, 0) =/= "b000001".U || (curEg(0) === 0.U))
    io.iss.bits.dt := dt_reg
    io.iss.bits.dt_inv := dt_inv_ieee
    io.iss.bits.eg := curEg
    io.iss.bits.eidx := eidx
    io.iss.bits.tail := tail
    io.iss.bits.vat := inst.vat
    io.iss.bits.isF32 := isF32_reg
    
    when(io.iss.fire && !tail){
        val next_next_eidx = get_next_eidx(inst.vconfig.vl, eidx, 3.U, false.B, false.B, false.B, inst.dLen)
        eidx := Mux(next_next_eidx === inst.vconfig.vl, 0.U, next_next_eidx)
        val next_reg = Mux(next_next_eidx === inst.vconfig.vl, curReg + 1.U, curReg)
        curReg := next_reg
    }



    // 설정해줘야 할 변수
    
    //seq에서
    // eidx

    // backend에서
    // io.read_req.ready

}