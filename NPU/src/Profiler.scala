
package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
import chisel3.reflect.DataMirror
import freechips.rocketchip.tile.RoCCCommand
import freechips.rocketchip.util.PlusArg
import GemminiISA._
import Util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy.{IdRange, LazyModule, LazyModuleImp}
import freechips.rocketchip.tile.{CoreBundle, HasCoreParameters}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.rocket.MStatus
import freechips.rocketchip.rocket.constants.MemoryOpConstants

class PStreamWriteRequest(val dataWidth: Int, val maxBytes: Int)(implicit p: Parameters) extends CoreBundle {
  val vaddr = UInt(coreMaxAddrBits.W)
  val data = UInt(dataWidth.W)
  val len = UInt(log2Up((dataWidth/8 max maxBytes)+1).W) // The number of bytes to write
  val block = UInt(8.W) // TODO magic number
  val status = new MStatus

  val store_en = Bool()
}

class PStreamWriter[T <: Data: Arithmetic](nXacts: Int, beatBits: Int, maxBytes: Int, dataWidth: Int, aligned_to: Int)
                  (implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "stream-writer", sourceId = IdRange(0, nXacts))))))

  require(isPow2(aligned_to))

  lazy val module = new Impl
  class Impl extends LazyModuleImp(this) with HasCoreParameters with MemoryOpConstants {
    val (tl, edge) = node.out(0)
    val dataBytes = dataWidth / 8
    val beatBytes = beatBits / 8
    val lgBeatBytes = log2Ceil(beatBytes)
    val maxBeatsPerReq = maxBytes / beatBytes
    val inputTypeRowBytes = dataBytes
    val maxBlocks = maxBytes / inputTypeRowBytes

    require(beatBytes > 0)

    val io = IO(new Bundle {
      val req = Flipped(Decoupled(new PStreamWriteRequest(dataWidth, maxBytes)))
      val tlb = new FrontendTLBIO
      val busy = Output(Bool())
    })

    val (s_idle :: s_writing_new_block :: s_writing_beats :: Nil) = Enum(3)
    val state = RegInit(s_idle)

    val req = Reg(new PStreamWriteRequest(dataWidth, maxBytes))

    // TODO use the same register to hold data_blocks and data_single_block, so that this Mux here is not necessary
    val data_blocks = Reg(Vec(maxBlocks, UInt((inputTypeRowBytes * 8).W)))
    val data_single_block = Reg(UInt(dataWidth.W)) // For data that's just one-block-wide
    val data = Mux(req.block === 0.U, data_single_block, data_blocks.asUInt)

    val bytesSent = Reg(UInt(log2Ceil((dataBytes max maxBytes)+1).W))  // TODO this only needs to count up to (dataBytes/aligned_to), right?
    val bytesLeft = req.len - bytesSent

    val xactBusy = RegInit(0.U(nXacts.W))
    val xactOnehot = PriorityEncoderOH(~xactBusy)
    val xactId = OHToUInt(xactOnehot)

    val xactBusy_fire = WireInit(false.B)
    val xactBusy_add = Mux(xactBusy_fire, (1.U << xactId).asUInt, 0.U)
    val xactBusy_remove = ~Mux(tl.d.fire, (1.U << tl.d.bits.source).asUInt, 0.U)
    xactBusy := (xactBusy | xactBusy_add) & xactBusy_remove.asUInt

    val state_machine_ready_for_req = WireInit(state === s_idle)
    io.req.ready := state_machine_ready_for_req
    io.busy := xactBusy.orR || (state =/= s_idle)

    val vaddr = req.vaddr

    // Select the size and mask of the TileLink request
    class Packet extends Bundle {
      val size = UInt(log2Ceil(maxBytes+1).W)
      val lg_size = UInt(log2Ceil(log2Ceil(maxBytes+1)+1).W)
      val mask = Vec(maxBeatsPerReq, Vec(beatBytes, Bool()))
      val vaddr = UInt(vaddrBits.W)
      val is_full = Bool()

      val bytes_written = UInt(log2Up(maxBytes+1).W)
      val bytes_written_per_beat = Vec(maxBeatsPerReq, UInt(log2Up(beatBytes+1).W))

      def total_beats(dummy: Int = 0) = Mux(size < beatBytes.U, 1.U, size / beatBytes.U(size.getWidth.W)) // The width expansion is added here solely to satsify Verilator's linter
    }

    val smallest_write_size = aligned_to max beatBytes
    // 여기서부터 핵심
    val write_sizes = (smallest_write_size to maxBytes by aligned_to).
      filter(s => isPow2(s)).
      filter(s => s % beatBytes == 0) /*.
      filter(s => s <= dataBytes*2 || s == smallest_write_size)*/
    val write_packets = write_sizes.map { s =>
      val lg_s = log2Ceil(s)
      val vaddr_aligned_to_size = if (s == 1) vaddr else Cat(vaddr(vaddrBits-1, lg_s), 0.U(lg_s.W))
      val vaddr_offset = if (s > 1) vaddr(lg_s - 1, 0) else 0.U

      val mask = (0 until maxBytes).map { i => i.U >= vaddr_offset && i.U < vaddr_offset +& bytesLeft && (i < s).B }

      val bytes_written = {
        Mux(vaddr_offset +& bytesLeft > s.U, s.U - vaddr_offset, bytesLeft)
      }

      val packet = Wire(new Packet())
      packet.size := s.U
      packet.lg_size := lg_s.U
      packet.mask := VecInit(mask.grouped(beatBytes).map(v => VecInit(v)).toSeq)
      packet.vaddr := vaddr_aligned_to_size
      packet.is_full := mask.take(s).reduce(_ && _)

      packet.bytes_written := bytes_written
      packet.bytes_written_per_beat.zipWithIndex.foreach { case (b, i) =>
        val start_of_beat = i * beatBytes
        val end_of_beat = (i+1) * beatBytes

        val left_shift = Mux(vaddr_offset >= start_of_beat.U && vaddr_offset < end_of_beat.U,
          vaddr_offset - start_of_beat.U,
          0.U)

        val right_shift = Mux(vaddr_offset +& bytesLeft >= start_of_beat.U && vaddr_offset +& bytesLeft < end_of_beat.U,
          end_of_beat.U - (vaddr_offset +& bytesLeft),
          0.U)

        val too_early = vaddr_offset >= end_of_beat.U
        val too_late = vaddr_offset +& bytesLeft <= start_of_beat.U

        b := Mux(too_early || too_late, 0.U, beatBytes.U - (left_shift +& right_shift))
      }

      packet
    }
    val best_write_packet = write_packets.reduce { (acc, p) =>
      Mux(p.bytes_written > acc.bytes_written, p, acc)
    }
    val write_packet = RegEnableThru(best_write_packet, state === s_writing_new_block)
    val write_size = write_packet.size
    val lg_write_size = write_packet.lg_size
    val write_beats = write_packet.total_beats()
    val write_vaddr = write_packet.vaddr
    val write_full = write_packet.is_full

    val beatsLeft = Reg(UInt(log2Up(maxBytes/aligned_to).W))
    val beatsSent = Mux(state === s_writing_new_block, 0.U, write_beats - beatsLeft)

    val write_mask = write_packet.mask(beatsSent)
    val write_shift = PriorityEncoder(write_mask)

    val bytes_written_this_beat = write_packet.bytes_written_per_beat(beatsSent)

    // Firing off TileLink write requests
    val putFull = edge.Put(
      fromSource = RegEnableThru(xactId, state === s_writing_new_block),
      toAddress = 0.U,
      lgSize = lg_write_size,
      data = (data >> (bytesSent * 8.U)).asUInt
    )._2

    val putPartial = edge.Put(
      fromSource = RegEnableThru(xactId, state === s_writing_new_block),
      toAddress = 0.U,
      lgSize = lg_write_size,
      data = ((data >> (bytesSent * 8.U)) << (write_shift * 8.U)).asUInt,
      mask = write_mask.asUInt
    )._2

    class TLBundleAWithInfo extends Bundle {
      val tl_a = DataMirror.internal.chiselTypeClone[TLBundleA](tl.a.bits)
      val vaddr = Output(UInt(vaddrBits.W))
      val status = Output(new MStatus)
    }

    val untranslated_a = Wire(Decoupled(new TLBundleAWithInfo))
    xactBusy_fire := untranslated_a.fire && state === s_writing_new_block
    untranslated_a.valid := (state === s_writing_new_block || state === s_writing_beats) && !xactBusy.andR
    untranslated_a.bits.tl_a := Mux(write_full, putFull, putPartial)
    untranslated_a.bits.vaddr := write_vaddr
    untranslated_a.bits.status := req.status

    val retry_a = Wire(Decoupled(new TLBundleAWithInfo))
    val shadow_retry_a = Module(new Queue(new TLBundleAWithInfo, 1))
    shadow_retry_a.io.enq.valid := false.B
    shadow_retry_a.io.enq.bits := DontCare
    val tlb_arb = Module(new Arbiter(new TLBundleAWithInfo, 3))
    tlb_arb.io.in(0) <> retry_a
    tlb_arb.io.in(1) <> shadow_retry_a.io.deq
    tlb_arb.io.in(2) <> untranslated_a

    val tlb_q = Module(new Queue(new TLBundleAWithInfo, 1, pipe=true))
    tlb_q.io.enq <> tlb_arb.io.out

    io.tlb.req.valid := tlb_q.io.deq.fire
    io.tlb.req.bits := DontCare
    io.tlb.req.bits.tlb_req.vaddr := tlb_q.io.deq.bits.vaddr
    io.tlb.req.bits.tlb_req.passthrough := false.B
    io.tlb.req.bits.tlb_req.size := 0.U
    io.tlb.req.bits.tlb_req.cmd := M_XWR
    io.tlb.req.bits.status := tlb_q.io.deq.bits.status

    val translate_q = Module(new Queue(new TLBundleAWithInfo, 1, pipe=true))
    translate_q.io.enq <> tlb_q.io.deq
    when (retry_a.valid) {
      translate_q.io.enq.valid := false.B
      shadow_retry_a.io.enq.valid := tlb_q.io.deq.valid
      shadow_retry_a.io.enq.bits := tlb_q.io.deq.bits
    }
    translate_q.io.deq.ready := tl.a.ready || io.tlb.resp.miss

    retry_a.valid := translate_q.io.deq.valid && io.tlb.resp.miss
    retry_a.bits := translate_q.io.deq.bits
    assert(!(retry_a.valid && !retry_a.ready))

    tl.a.valid := translate_q.io.deq.valid && !io.tlb.resp.miss
    tl.a.bits := translate_q.io.deq.bits.tl_a
    tl.a.bits.address := RegEnableThru(io.tlb.resp.paddr, RegNext(io.tlb.req.fire))

    tl.d.ready := xactBusy.orR

    when (untranslated_a.fire) {
      when (state === s_writing_new_block) {
        beatsLeft := write_beats - 1.U

        val next_vaddr = req.vaddr + write_packet.bytes_written
        req.vaddr := next_vaddr

        bytesSent := bytesSent + bytes_written_this_beat

        when (write_beats === 1.U) {
          when (bytes_written_this_beat >= bytesLeft) {
            // We're done with this request at this point
            state_machine_ready_for_req := true.B
            state := s_idle
          }
        }.otherwise {
          state := s_writing_beats
        }
      }.elsewhen(state === s_writing_beats) {
        beatsLeft := beatsLeft - 1.U
        bytesSent := bytesSent + bytes_written_this_beat

        assert(beatsLeft > 0.U)

        when (beatsLeft === 1.U) {
          when (bytes_written_this_beat >= bytesLeft) {
            // We're done with this request at this point
            state_machine_ready_for_req := true.B
            state := s_idle
          }.otherwise {
            state := s_writing_new_block
          }
        }
      }
    }
    // Accepting requests to kick-start the state machine
    when (io.req.fire) {
      req := io.req.bits
      req.len := io.req.bits.block * inputTypeRowBytes.U + io.req.bits.len

      data_single_block := io.req.bits.data
      data_blocks(io.req.bits.block) := io.req.bits.data

      bytesSent := 0.U

      state := Mux(io.req.bits.store_en, s_writing_new_block, s_idle)

      assert(io.req.bits.len <= (inputTypeRowBytes).U || io.req.bits.block === 0.U, "DMA can't write multiple blocks to main memory when writing full accumulator output")
    }
  }
}

object ProfileEvent{
    val DISABLE = 0

    val ROB_ALLOC = 1 // controller - rs의 rob에 배치

    val ROB_ISSUE_LD = 2 // controller - rob -> ld컨트롤러로 issue
    val ROB_ISSUE_EX = 3 // controller - rob -> ex컨트롤러로 issue 
    val ROB_ISSUE_ST = 4 // controller - rob -> st컨트롤러로 issue
    
    val ROB_COMPLETE = 5 // controller - 
    
    val ENTER_LD_CTRL = 6 // controller - rob에서 issue되어 ld컨트롤러 진입
    val ENTER_EX_CTRL = 7 // controller - rob에서 issue되어 ex컨트롤러 진입
    val ENTER_ST_CTRL = 8 // controller - rob에서 issue되어 st컨트롤러 진입

    val LEAVE_LD_CTRL = 9 // controller - ld 컨트롤러 작업 끝나서 나감
    val LEAVE_EX_CTRL = 10 // controller - ex 컨트롤러 작업 끝나서 나감
    val LEAVE_ST_CTRL = 11 // controller - st 컨트롤러 작업 끝나서 나감

    val ENTER_DMA_READ  = 12 // controller에서 주석처리 - 구현 X
    val ENTER_DMA_WRITE = 13 // controller에서 주석처리 - 구현 X
    val LEAVE_DMA_READ  = 14 // controller에서 주석처리 - 구현 X
    val LEAVE_DMA_WRITE = 15 // write DMA TileLink D 응답

    val ENTER_SPAD_READ = 16 // 구현 X
    val ENTER_SPAD_WRITE= 17 // 구현 X
    val LEAVE_SPAD_READ = 18 // 구현 X
    val LEAVE_SPAD_WRITE= 19 // 구현 X

    val ENTER_MESH_CTRL = 20 // 구현 X
    val LEAVE_MESH_CTRL = 21 // 구현 X

    val ENTER_DEL_MESH  = 22 // 구현 X
    val LEAVE_DEL_MESH  = 23 // 구현 X

    val LD_CTRL_EXECUTE = 24 // load controller
    val EX_CTRL_EXECUTE = 25 // execution controller
    val ST_CTRL_EXECUTE = 26 // store controller
    
    val n = 27

    val STORE_PROFILE_TOKEN_WIDTH = 16
    def storeProfileIdWidth(robIdWidth: Int): Int = STORE_PROFILE_TOKEN_WIDTH + robIdWidth
}

class ProfileEventIO(EVENT_ID_WIDTH : Int) extends Bundle{
    val event_signal = Output(Vec(ProfileEvent.n, Bool()))
    val event_id     = Output(Vec(ProfileEvent.n, UInt(EVENT_ID_WIDTH.W)))

    private var connected = Array.fill(ProfileEvent.n)(false)

    def connectEventSignal(addr: Int, sig: UInt, id: UInt) = {
        event_signal(addr) := sig
        event_id(addr) := id
        connected(addr) = true
    }

    def collect[T <: Data](io: ProfileEventIO) = {
        for(i <-0 until ProfileEvent.n)
            if(io.connected(i)){
                if(connected(i)){
                    throw new IllegalStateException("Port " + i + " is already connected in another IO")
                }
                else{
                    connected(i) = true
                    event_signal(i) := io.event_signal(i)
                    event_id(i) := io.event_id(i)
                }
            }
    }
}

object ProfileEventIO {
    def init (io: ProfileEventIO) = {
        io.event_signal := 0.U.asTypeOf(io.event_signal.cloneType)
        io.event_id := 0.U.asTypeOf(io.event_id.cloneType)
    }
}

class ProfileIO [T <: Data](cmd_t: T, ROB_ID_WIDTH: Int, EVENT_ID_WIDTH: Int = ROB_ID_WIDTH) extends Bundle{
    val issue_cmd = new ReservationStationIssue(cmd_t, ROB_ID_WIDTH)
    val event_io = new ProfileEventIO(EVENT_ID_WIDTH)
}

class Profiler [T <: Data : Arithmetic, U <: Data, V <: Data](config: GemminiArrayConfig[T, U, V], cmd_t: GemminiCmd)
    (implicit p: Parameters) extends LazyModule {

    import config._

    val maxBytes = dma_maxbytes
    val dataBits = dma_buswidth
    val num_set = 1
    val data_w = 64 * num_set

    val id_node = TLIdentityNode()
    val xbar_node = TLXbar()

    val writer = LazyModule(new PStreamWriter(max_in_flight_mem_reqs, dataBits, maxBytes, data_w, aligned_to))
    val profile_event_id_width = ProfileEvent.storeProfileIdWidth(ROB_ID_WIDTH)

    xbar_node := TLBuffer() := writer.node
    id_node := TLWidthWidget(dma_buswidth/8) := TLBuffer() := xbar_node

    lazy val module = new Impl
    class Impl extends LazyModuleImp(this) with HasCoreParameters {
      val io = IO(new Bundle {
          val profile_io = Flipped(new ProfileIO(cmd_t, ROB_ID_WIDTH, profile_event_id_width))
          val profiler_vaddr_valid = Input(Bool())
          val profiler_vaddr = Input(UInt(coreMaxAddrBits.W))
          val profiler_status = Input(new MStatus)
          val tlb = new FrontendTLBIO
          val busy = Output(Bool())
      })

      writer.module.io.req.bits.len := 8.U
      writer.module.io.req.bits.block := 0.U
      writer.module.io.req.bits.store_en := true.B
      writer.module.io.tlb <> io.tlb
    
      val ldq :: exq :: stq :: Nil = Enum(3)
      val q_t = ldq.cloneType // 큐 타입 저장(ld?ex?st?)

      class ProfileDataWCMD extends Bundle {
          val tag = UInt(30.W)
          val start_time = UInt(31.W) // ROB_ALLOC ~ ROB_COMPLETE
          val cmd = cmd_t.cloneType
      }

      class PendingStoreProfile extends Bundle {
          val valid = Bool()
          val profile_id = UInt(profile_event_id_width.W)
          val tag = UInt(30.W)
          val start_time = UInt(31.W)
          val cmd = cmd_t.cloneType
      }

      val profiler_vaddr = RegInit(0.U(coreMaxAddrBits.W))
      val profiler_vaddr_valid = RegInit(false.B)
      val profiler_status = RegInit(0.U.asTypeOf(new MStatus))

      val write_q = Module(new Queue(UInt(data_w.W), res_max_per_type, true, true))
      write_q.io.enq.valid := false.B
      write_q.io.enq.bits := 0.U
      write_q.io.deq.ready := writer.module.io.req.ready
      writer.module.io.req.valid := write_q.io.deq.valid
      writer.module.io.req.bits.data := write_q.io.deq.bits
      writer.module.io.req.bits.vaddr := profiler_vaddr
      writer.module.io.req.bits.status := profiler_status

      io.busy := writer.module.io.busy || write_q.io.deq.valid

      when (io.profiler_vaddr_valid) {
          assert(!io.busy, "Profiler address can only be configured while profiler is idle")
          profiler_vaddr_valid := io.profiler_vaddr =/= 0.U
          profiler_vaddr := io.profiler_vaddr
          profiler_status := io.profiler_status
          printf("vaddr: %d\n", io.profiler_vaddr)
      }. elsewhen (writer.module.io.req.fire) {
        val next_vaddr = profiler_vaddr + 8.U
        profiler_vaddr := next_vaddr
      }

      val tag = RegInit(0.U(30.W))
      val clk_cnt = RegInit(0.U(31.W))
      clk_cnt := clk_cnt + 1.U

      val entries_ld = Reg(Vec(reservation_station_entries_ld, new ProfileDataWCMD))
      val entries_ex = Reg(Vec(reservation_station_entries_ex, new ProfileDataWCMD))
      val entries_st = Reg(Vec(reservation_station_entries_st, new ProfileDataWCMD))
      val entries = entries_ld ++ entries_ex ++ entries_st

      // Store profiles stay live until the final write DMA TL-D response, which can lag the ROB completion.
      val scratchpad_write_pipeline_entries =
        2 + (spad_read_delay + 2) + (spad_read_delay + 2) + (spad_read_delay + 1) + 1
      val completed_store_write_window_entries = scratchpad_write_pipeline_entries + max_in_flight_mem_reqs
      val pending_store_entries = reservation_station_entries_st + completed_store_write_window_entries
      val pending_stores = RegInit(VecInit(Seq.fill(pending_store_entries)(0.U.asTypeOf(new PendingStoreProfile))))
      val pending_store_free_vec = VecInit(pending_stores.map(p => !p.valid))
      val pending_store_free = pending_store_free_vec.asUInt.orR
      val pending_store_free_idx = OHToUInt(PriorityEncoderOH(pending_store_free_vec))

      io.profile_io.issue_cmd.ready := true.B
      
      val signal = io.profile_io.event_io.event_signal
      val id = io.profile_io.event_io.event_id
      
      val new_entry = Wire(new ProfileDataWCMD)
      new_entry.tag := tag
      new_entry.start_time := 0.U
      new_entry.cmd := io.profile_io.issue_cmd.cmd

      val mvNcomList = Seq(1.U, 2.U, 3.U, 4.U, 5.U, 14.U)

      when(io.profile_io.issue_cmd.valid && signal(ProfileEvent.ROB_ALLOC)){
          val type_width = log2Up(res_max_per_type)
          val queue_type = io.profile_io.issue_cmd.rob_id(type_width + 1, type_width)
          val issue_id = io.profile_io.issue_cmd.rob_id(type_width - 1, 0)
          val cmd = io.profile_io.issue_cmd.cmd.cmd

          // new_entry.start_time := clk_cnt
          
          printf("0x%x/%d-%d/%d-%d-%d\n",
                  new_entry.tag, 1.U, clk_cnt, cmd.inst.asUInt, cmd.rs1, cmd.rs2)

          Seq((ldq, entries_ld, reservation_station_entries_ld),
              (exq, entries_ex, reservation_station_entries_ex),
              (stq, entries_st, reservation_station_entries_st))
              .foreach { case(q, entries_type, entries_count) =>
                  when(queue_type===q){
                      entries_type(issue_id) := new_entry
                  }
              }
          tag := tag + 1.U
      }

      for(i <- 2 until ProfileEvent.n){
          when(signal(i)){
              val type_width = log2Up(res_max_per_type)
              val event_rob_id = id(i)(ROB_ID_WIDTH - 1, 0)
              val queue_type = event_rob_id(type_width + 1, type_width)
              val issue_id = event_rob_id(type_width - 1, 0)
              Seq((ldq, entries_ld, reservation_station_entries_ld),
              (exq, entries_ex, reservation_station_entries_ex),
              (stq, entries_st, reservation_station_entries_st))
              .foreach { case(q, entries_type, entries_count) =>
                  when(queue_type===q){
                      val entry = entries_type(issue_id)
                      val cmd = entry.cmd.cmd
                      val is_profiled_cmd = mvNcomList.map(_ === cmd.inst.funct).reduce((a: Bool, b: Bool) => a || b)
                      val is_store_entry = q === stq
                      val has_start_time = entry.start_time =/= 0.U

                      when(i.U =/= ProfileEvent.LEAVE_DMA_WRITE.U) {
                        printf("0x%x/%d-%d/%d-%d-%d\n",
                                entry.tag, i.U, clk_cnt, cmd.inst.asUInt, cmd.rs1, cmd.rs2)
                      }

                      when(i.U === ProfileEvent.LD_CTRL_EXECUTE.U || i.U === ProfileEvent.EX_CTRL_EXECUTE.U){
                        entry.start_time := clk_cnt
                      }
                      .elsewhen(i.U === ProfileEvent.ST_CTRL_EXECUTE.U){
                          entry.start_time := clk_cnt
                          when (is_profiled_cmd && is_store_entry) {
                            assert(pending_store_free, "Profiler pending store table overflow")
                            pending_stores(pending_store_free_idx).valid := true.B
                            pending_stores(pending_store_free_idx).profile_id := id(i)
                            pending_stores(pending_store_free_idx).tag := entry.tag
                            pending_stores(pending_store_free_idx).start_time := clk_cnt
                            pending_stores(pending_store_free_idx).cmd := entry.cmd
                          }
                      }
                      .elsewhen(i.U === ProfileEvent.ROB_COMPLETE.U){
                          when (is_profiled_cmd && !is_store_entry && has_start_time) {
                            printf("%d, %d, %d\n", q, entry.start_time, clk_cnt)
                            when (profiler_vaddr_valid) {
                              write_q.io.enq.valid := true.B
                              write_q.io.enq.bits := Cat(q, entry.start_time, clk_cnt)
                              when (write_q.io.enq.fire) {
                                entry.start_time := 0.U
                              }
                            }
                          }
                      }
                      .elsewhen(i.U === ProfileEvent.LEAVE_DMA_WRITE.U){
                          val pending_store_match_vec = VecInit(pending_stores.map(p => p.valid && p.profile_id === id(i)))
                          val pending_store_match = pending_store_match_vec.asUInt.orR
                          val pending_store_match_idx = PriorityEncoder(pending_store_match_vec)
                          val pending_store = pending_stores(pending_store_match_idx)
                          val pending_cmd = pending_store.cmd.cmd
                          val is_profiled_store_cmd = mvNcomList.map(_ === pending_cmd.inst.funct).reduce((a: Bool, b: Bool) => a || b)
                          val pending_store_has_start_time = pending_store.start_time =/= 0.U

                          when (pending_store_match) {
                            printf("0x%x/%d-%d/%d-%d-%d\n",
                                    pending_store.tag, i.U, clk_cnt, pending_cmd.inst.asUInt, pending_cmd.rs1, pending_cmd.rs2)

                            when (is_profiled_store_cmd && pending_store_has_start_time) {
                              printf("%d, %d, %d\n", stq, pending_store.start_time, clk_cnt)
                              when (profiler_vaddr_valid) {
                                assert(write_q.io.enq.ready, "Profiler write queue is full on store DMA completion")
                                write_q.io.enq.valid := true.B
                                write_q.io.enq.bits := Cat(stq, pending_store.start_time, clk_cnt)
                              }
                            }

                            when (!profiler_vaddr_valid || !is_profiled_store_cmd || !pending_store_has_start_time || write_q.io.enq.ready) {
                              pending_store.valid := false.B
                            }
                          }
                      }
                  }
              }
          }
      }
    }
}
