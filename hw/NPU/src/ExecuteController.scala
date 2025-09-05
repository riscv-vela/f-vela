package gemmini

import chisel3._
import chisel3.util._
import GemminiISA._
import Util._
import org.chipsalliance.cde.config.Parameters
import midas.targetutils.PerfCounter

class ExecuteController[T <: Data, U <: Data, V <: Data](xLen: Int, tagWidth: Int, config: GemminiArrayConfig[T, U, V])
                                  (implicit p: Parameters, ev: Arithmetic[T]) extends Module {
  import config._
  import ev._
  
  val io = IO(new Bundle {
      val cmd = Flipped(Decoupled(new GemminiCmd(reservation_station_entries)))

      val srams = new Bundle {
          val read = Vec(sp_banks, new ScratchpadReadIO(sp_bank_entries, sp_width))
          val write = Vec(sp_banks, new ScratchpadWriteIO(sp_bank_entries, sp_width, (sp_width / (aligned_to * 8)) max 1))
      }
      val acc = new Bundle {
          val read_req = Vec(acc_banks, Decoupled(new AccumulatorReadReq(
              acc_bank_entries, accType, acc_scale_t
          )))

          val read_resp = Flipped(Vec(acc_banks, Decoupled(new AccumulatorScaleResp(
              Vec(meshColumns, Vec(tileColumns, inputType)),
              Vec(meshColumns, Vec(tileColumns, accType))
          ))))

          val write = Vec(acc_banks, Decoupled(new AccumulatorWriteReq(acc_bank_entries, Vec(meshColumns, Vec(tileColumns, accType)))))
      }
      val completed = Valid(UInt(log2Up(reservation_station_entries).W))
      val busy = Output(Bool())

      val counter = new CounterEventIO()
  })

  val block_size = meshRows*tileRows // = ma_length

  //TODO: Do we really need "val mesh_tag"?
  //we need it to identify each matmul's id
  val mesh_tag = new Bundle with TagQueueTag {
    val rob_id = UDValid(UInt(log2Up(reservation_station_entries).W))
    val addr = local_addr_t.cloneType
    val rows = UInt(log2Up(block_size + 1).W)
    val cols = UInt(log2Up(block_size + 1).W)

    override def make_this_garbage(dummy: Int = 0): Unit = {
      rob_id.valid := false.B
      addr.make_this_garbage()
    }
  }


  //b_transpose할 경우 preload + compute_and_flip을 3단계의 명령어로 분할-> 할 경우 없으므로 우선 제거
  // val unrolled_cmd = TransposePreloadUnroller(io.cmd, config, io.counter)

  val cmd_q_heads = 3
  assert(ex_queue_length >= cmd_q_heads)  
  val (cmd, _) = MultiHeadedQueue(io.cmd, ex_queue_length, cmd_q_heads)
  cmd.pop := 0.U

  // STATE defines
  val waiting_for_cmd :: compute :: flush :: flushing :: Nil = Enum(4)
  val control_state = RegInit(waiting_for_cmd)

  val functs = cmd.bits.map(_.cmd.inst.funct)
  val rs1s = VecInit(cmd.bits.map(_.cmd.rs1))
  val rs2s = VecInit(cmd.bits.map(_.cmd.rs2))


  val DoConfig = functs(0) === CONFIG_CMD
  val DoComputes = functs.map(f => f === COMPUTE_AND_FLIP_CMD || f === COMPUTE_AND_STAY_CMD)
  val DoPreloads = functs.map(_ === PRELOAD_CMD)

  val in_acc = rs1s(0)(rs1s(0).getWidth - 1)
  val in_prop = functs(0) === COMPUTE_AND_FLIP_CMD

  val in_acc_buf = RegInit(false.B)
  val cmd_pop_next = RegNext(cmd.pop =/= 0.U)
  when(cmd_pop_next && DoComputes(0)){
    in_acc_buf := in_acc
  }
  val in_acc_delay = ShiftRegister(in_acc_buf, spad_read_delay+1)

  val in_prop_flush = Reg(Bool())
  in_prop_flush := false.B

  val acc_scale = Reg(acc_scale_t)
  val activation = if (has_nonlinear_activations) Reg(UInt(Activation.bitwidth.W)) else Activation.NONE // TODO magic number
  val config_initialized = RegInit(false.B)

  val bc_address_place = Mux(DoPreloads(0), 0.U, 1.U)
  val ad_address_place = Mux(DoPreloads(0), 1.U, 0.U)

// SRAM addresses of matmul operands
  val a_address_rs1 = rs1s(ad_address_place).asTypeOf(local_addr_t)
  val b_address_rs1 = rs1s(bc_address_place).asTypeOf(local_addr_t)
  val d_address_rs2 = rs2s(ad_address_place).asTypeOf(local_addr_t)
  val c_address_rs2 = rs2s(bc_address_place).asTypeOf(local_addr_t)

  if (hardcode_d_to_garbage_addr) {
    d_address_rs2.make_this_garbage()
  }
  val multiply_garbage = a_address_rs1.is_garbage()
  val accumulate_zeros = d_address_rs2.is_garbage()
  val preload_zeros = b_address_rs1.is_garbage()

  val a_cols = rs1s(ad_address_place)(32 + log2Up(block_size + 1) - 1, 32)
  val a_rows = rs1s(ad_address_place)(48 + log2Up(block_size + 1) - 1, 48)
  val b_cols = rs1s(bc_address_place)(32 + log2Up(block_size + 1) - 1, 32)
  val b_rows = rs1s(bc_address_place)(48 + log2Up(block_size + 1) - 1, 48)
  val d_cols = rs2s(ad_address_place)(32 + log2Up(block_size + 1) - 1, 32)
  val d_rows = rs2s(ad_address_place)(48 + log2Up(block_size + 1) - 1, 48)
  val c_cols = rs2s(bc_address_place)(32 + log2Up(block_size + 1) - 1, 32)
  val c_rows = rs2s(bc_address_place)(48 + log2Up(block_size + 1) - 1, 48)
  

  // Dependency stuff
  io.completed.valid := false.B
  io.completed.bits := DontCare

  // val pending_completed_rob_id = Reg(UDValid(UInt(log2Up(rob_entries).W)))
  val pending_completed_rob_ids = Reg(Vec(2, UDValid(UInt(log2Up(reservation_station_entries).W))))

  // Instantiate a queue which queues up signals which must be fed into the mesh
  val mesh_cntl_signals_q = Module(new Queue(new ComputeCntlSignals, spad_read_delay+1,
    pipe=true))

  val cntl_ready = mesh_cntl_signals_q.io.enq.ready
  val cntl_valid = mesh_cntl_signals_q.io.deq.valid
  val cntl = mesh_cntl_signals_q.io.deq.bits

  val wontolic = Module(new WontolicWithDelays(inputType, spatialArrayOutputType, accType, mesh_tag, dataflow, tree_reduction, tile_latency, mesh_output_delay,
    tileRows, tileColumns, meshRows, meshColumns, shifter_banks, shifter_banks))

  wontolic.io.a.valid := false.B
  wontolic.io.b.valid := false.B
  wontolic.io.d.valid := false.B
  wontolic.io.req.valid := control_state === flush

  wontolic.io.a.bits := DontCare
  wontolic.io.b.bits := DontCare
  wontolic.io.d.bits := DontCare
  wontolic.io.req.bits.tag := DontCare
  wontolic.io.req.bits.tag.cols := cntl.c_cols
  wontolic.io.req.bits.tag.rows := cntl.c_rows
  wontolic.io.req.bits.total_rows := block_size.U
  wontolic.io.req.bits.flush := Mux(control_state === flush && !cntl_valid, 1.U, 0.U) // We want to make sure that the mesh has absorbed all inputs before flushing
  wontolic.io.req.bits.tag.rob_id := cntl.rob_id
  wontolic.io.req.bits.in_prop := Mux(control_state === flush, in_prop_flush, cntl.prop)
  wontolic.io.req.bits.in_acc := in_acc_delay

//Hazards
  val raw_hazards_are_impossible = !ex_read_from_acc && !ex_write_to_spad // Special case where RAW hazards are impossible

  val raw_hazard_pre = wontolic.io.tags_in_progress.map { t =>
    val is_garbage = t.addr.is_garbage()
    val pre_raw_haz = t.addr.is_same_address(rs1s(0))
    val mul_raw_haz = t.addr.is_same_address(rs1s(1)) || t.addr.is_same_address(rs2s(1))

    !is_garbage && (pre_raw_haz || mul_raw_haz) && !raw_hazards_are_impossible.B
  }.reduce(_ || _)

  val raw_hazard_mulpre = wontolic.io.tags_in_progress.map { t =>
    val is_garbage = t.addr.is_garbage()
    val pre_raw_haz = t.addr.is_same_address(rs1s(1))
    val mul_raw_haz = t.addr.is_same_address(rs1s(2)) || t.addr.is_same_address(rs2s(2))

    !is_garbage && (mul_raw_haz || pre_raw_haz) && !raw_hazards_are_impossible.B
  }.reduce(_ || _)

  // stall이 발생하면 세번째 명령어까지 받아서 처리
  val third_instruction_needed = !raw_hazards_are_impossible.B //ad_address_place > 1.U || bc_address_place > 1.U

  val matmul_in_progress = wontolic.io.tags_in_progress.map(_.rob_id.valid).reduce(_ || _)

  io.busy := cmd.valid(0) || matmul_in_progress

  val a_fire_counter = Reg(UInt(log2Up(block_size).W))
  val b_fire_counter = Reg(UInt(log2Up(block_size).W))
  val d_fire_counter = Reg(UInt(log2Up(block_size).W))

  val a_fire_started = RegInit(false.B)
  val b_fire_started = RegInit(false.B)
  val d_fire_started = RegInit(false.B)

  val a_addr_offset = Reg(UInt((16 + log2Up(block_size)).W))
  val a_addr_stride = Reg(UInt(16.W))
  val c_addr_stride = Reg(UInt(16.W)) // TODO magic numbers

  val a_address = a_address_rs1 + a_addr_offset
  val b_address = b_address_rs1 + b_fire_counter
  val d_address = d_address_rs2 + d_fire_counter

  val dataAbank = a_address.sp_bank()
  val dataBbank = b_address.sp_bank()
  val dataDbank = d_address.sp_bank()

  val dataABankAcc = a_address.acc_bank()
  val dataBBankAcc = b_address.acc_bank()
  val dataDBankAcc = d_address.acc_bank()

  val a_read_from_acc = ex_read_from_acc.B && a_address_rs1.is_acc_addr
  val b_read_from_acc = ex_read_from_acc.B && b_address_rs1.is_acc_addr
  val d_read_from_acc = ex_read_from_acc.B && d_address_rs2.is_acc_addr

  val start_inputting_a = WireInit(false.B)
  val start_inputting_b = WireInit(false.B)
  val start_inputting_d = WireInit(false.B)
  val start_array_outputting = WireInit(false.B)

  val a_garbage = a_address_rs1.is_garbage() || !start_inputting_a
  val b_garbage = b_address_rs1.is_garbage() || !start_inputting_b
  val d_garbage = d_address_rs2.is_garbage() || !start_inputting_d

  val perform_single_preload = RegInit(false.B)
  val perform_single_mul = RegInit(false.B)
  val perform_mul_pre = RegInit(false.B)
  
  // TODO merge these into one enum
  val performing_single_preload = WireInit(perform_single_preload && control_state === compute)
  val performing_single_mul = WireInit(perform_single_mul && control_state === compute)
  val performing_mul_pre = WireInit(perform_mul_pre && control_state === compute)

  val total_rows = WireInit(block_size.U)

  // TODO Also reduce the number of rows when "perform_single_preload === true.B"
  when (b_garbage) {
    val rows_a = Mux(a_garbage, 1.U, a_rows)
    val rows_b = Mux(b_garbage, 1.U, b_rows)

    /* We can only retire one ROB instruction per cycle (max), but if total_rows == 1, then we would be trying to retire
      2 ROB instructions per cycle (one for the preload, and one for the compute). Therefore, to prevent ROB
      instructions from being lost, we set a minimum floor for total_rows of 2.

      Furthermore, two writes to the same accumulator address must occur at least 4 cycles apart to allow the write to
      fully propagate through. Therefore, we raise the minimum floor for total_rows to 4.
      TODO: add a WAW check to the ROB so that we can lower the floor back to 2
    */

    //TODO: total row 제약이 Wontolic에서도 필요한지 고민해보기, 4가 아닌 2로도 해보기.
    total_rows := maxOf(maxOf(rows_a, rows_b), 4.U)
  }

  //mul_pre sync가 필요한지 생각해보기

  // These variables determine whether or not the row that is currently being read should be completely padded with 0
  // 실제 행렬의 유효 범위를 초과할 경우.
  val a_row_is_not_all_zeros = a_fire_counter < a_rows
  val b_row_is_not_all_zeros = b_fire_counter < b_rows
  val d_row_is_not_all_zeros = d_fire_counter < d_rows

  // scratch pad 혹은 accumulator의 같은 뱅크에서 데이터를 가져오는 경우.
  def same_bank(addr1: LocalAddr, addr2: LocalAddr, is_garbage1: Bool, is_garbage2: Bool, start_inputting1: Bool, start_inputting2: Bool): Bool = {
    val addr1_read_from_acc = addr1.is_acc_addr
    val addr2_read_from_acc = addr2.is_acc_addr
    val is_garbage = is_garbage1 || is_garbage2 || !start_inputting1 || !start_inputting2

    !is_garbage && ((addr1_read_from_acc && addr2_read_from_acc) ||  
    (!addr1_read_from_acc && !addr2_read_from_acc && addr1.sp_bank() === addr2.sp_bank()))
  }

  val a_ready = WireInit(true.B)
  val b_ready = WireInit(true.B)
  val d_ready = WireInit(true.B)

  //Same bank일 때, 충돌 해결 로직(같은 뱅크에서 읽어오는 행위가 한사이클 차이만 난다면, stall -> 과연 필요한가?)
  case class Operand(addr: LocalAddr, is_garbage: Bool, start_inputting: Bool, counter: UInt, started: Bool, priority: Int) {
    val done = counter === 0.U && started
  }
  val a_operand = Operand(a_address, a_address_rs1.is_garbage(), start_inputting_a, a_fire_counter, a_fire_started, 0)
  val b_operand = Operand(b_address, b_address_rs1.is_garbage(), start_inputting_b, b_fire_counter, b_fire_started, 1)
  val d_operand = Operand(d_address, d_address_rs2.is_garbage(), start_inputting_d, d_fire_counter, d_fire_started, 2)
  val operands = Seq(a_operand, b_operand, d_operand)

  val Seq(a_valid, b_valid, d_valid) = operands.map { case Operand(addr, is_garbage, start_inputting, counter, started, priority) =>
    val others = operands.filter(_.priority != priority)

    val same_banks = others.map(o => same_bank(addr, o.addr, is_garbage, o.is_garbage, start_inputting, o.start_inputting))
    val same_counter = others.map(o => started === o.started && counter === o.counter)

    val one_ahead = others.map(o => started && counter === wrappingAdd(o.counter, 1.U, total_rows))

    val higher_priorities = others.map(o => (o.priority < priority).B)

    val must_wait_for = ((same_banks zip same_counter) zip (one_ahead zip higher_priorities)).map {
      case ((sb, sc), (oa, hp)) =>
        (sb && hp && sc) || oa
    }

    !must_wait_for.reduce(_ || _)
  }

  //a, b, d matrix 몇번째 줄까지 진행되었는지 추적하기 위한 로직
  val a_fire = a_valid && a_ready
  val b_fire = b_valid && b_ready
  val d_fire = d_valid && d_ready

  val firing = start_inputting_a || start_inputting_b || start_inputting_d

  when (!firing) {
    a_fire_counter := 0.U
    a_addr_offset := 0.U
  }.elsewhen (firing && a_fire && cntl_ready) {
    a_fire_counter := wrappingAdd(a_fire_counter, 1.U, total_rows)
    a_addr_offset := Mux(a_fire_counter === (total_rows-1.U), 0.U, a_addr_offset + a_addr_stride)
    a_fire_started := true.B
  }

  when (!firing) {
    b_fire_counter := 0.U
  }.elsewhen (firing && b_fire && cntl_ready) {
    b_fire_counter := wrappingAdd(b_fire_counter, 1.U, total_rows)
    b_fire_started := true.B
  }

  when (!firing) {
    d_fire_counter := 0.U
  }.elsewhen (firing && d_fire && cntl_ready) {
    d_fire_counter := wrappingAdd(d_fire_counter, 1.U, total_rows)
    d_fire_started := true.B
  }

/* 현재 코드에서는 저장해 놓은 mul_pre_counter_count가 사용되지 않고 있음
  //cntl_ready가 되지 않아 stall이 발생한다면 b_fire_counter+1 저장
  when(performing_mul_pre && !cntl_ready && !mul_pre_counter_lock){
    mul_pre_counter_count := b_fire_counter //store 2
  }.elsewhen(!performing_mul_pre){
    mul_pre_counter_count := 0.U
    mul_pre_counter_lock := false.B
  }.elsewhen(!cntl_ready){
    mul_pre_counter_lock := true.B
  }
*/
  // The last line in this (long) Boolean is just to make sure that we don't think we're done as soon as we begin firing
  // TODO change when square requirement lifted
  val about_to_fire_all_rows = ((a_fire_counter === (total_rows-1.U) && a_fire) || a_fire_counter === 0.U) &&
    ((b_fire_counter === (total_rows-1.U) && b_fire) || b_fire_counter === 0.U) &&
    ((d_fire_counter === (total_rows-1.U) && d_fire) || d_fire_counter === 0.U) &&
    (a_fire_started || b_fire_started || d_fire_started) &&
    cntl_ready

  when (about_to_fire_all_rows) {
    a_fire_started := false.B
    b_fire_started := false.B
    d_fire_started := false.B
  }


  // Scratchpad reads   
    for (i <- 0 until sp_banks) {
    val read_a = a_valid && !a_read_from_acc && dataAbank === i.U && start_inputting_a && !multiply_garbage && a_row_is_not_all_zeros
    val read_b = b_valid && !b_read_from_acc && dataBbank === i.U && start_inputting_b && !preload_zeros && b_row_is_not_all_zeros
    val read_d = d_valid && !d_read_from_acc && dataDbank === i.U && start_inputting_d && !accumulate_zeros && d_row_is_not_all_zeros

    Seq((read_a, a_ready), (read_b, b_ready), (read_d, d_ready)).foreach { case (rd, r) =>
      when (rd && !io.srams.read(i).req.ready) {
        r := false.B
      }
    }

    if (ex_read_from_spad) {
      io.srams.read(i).req.valid := (read_a || read_b || read_d) && cntl_ready
      io.srams.read(i).req.bits.fromDMA := false.B
      io.srams.read(i).req.bits.addr := MuxCase(a_address_rs1.sp_row() + a_fire_counter,
        Seq(read_b -> (b_address_rs1.sp_row() + b_fire_counter),
          read_d -> (d_address_rs2.sp_row() + d_fire_counter)))

    } else {
      io.srams.read(i).req.valid := false.B
      io.srams.read(i).req.bits.fromDMA := false.B
      io.srams.read(i).req.bits.addr := DontCare
    }

    io.srams.read(i).resp.ready := false.B
  }

  // Accumulator read
  for (i <- 0 until acc_banks) {
    val read_a_from_acc = a_valid && a_read_from_acc && dataABankAcc === i.U && start_inputting_a && !multiply_garbage && a_row_is_not_all_zeros
    val read_b_from_acc = b_valid && b_read_from_acc && dataBBankAcc === i.U && start_inputting_b && !preload_zeros && b_row_is_not_all_zeros
    val read_d_from_acc = d_valid && d_read_from_acc && dataDBankAcc === i.U && start_inputting_d && !accumulate_zeros && d_row_is_not_all_zeros

    Seq((read_a_from_acc, a_ready), (read_b_from_acc, b_ready), (read_d_from_acc, d_ready)).foreach { case (rd, r) =>
      when(rd && !io.acc.read_req(i).ready) {
        r := false.B
      }
    }

    if (ex_read_from_acc) {
      io.acc.read_req(i).valid := read_a_from_acc || read_b_from_acc || read_d_from_acc
      io.acc.read_req(i).bits.scale := acc_scale
      io.acc.read_req(i).bits.full := false.B
      io.acc.read_req(i).bits.igelu_qb := DontCare
      io.acc.read_req(i).bits.igelu_qc := DontCare
      io.acc.read_req(i).bits.iexp_qln2 := DontCare
      io.acc.read_req(i).bits.iexp_qln2_inv := DontCare
      io.acc.read_req(i).bits.act := activation
      io.acc.read_req(i).bits.fromDMA := false.B
      io.acc.read_req(i).bits.addr := MuxCase(a_address_rs1.acc_row() + a_fire_counter,
        Seq(read_b_from_acc -> (b_address_rs1.acc_row() + b_fire_counter),
          read_d_from_acc -> (d_address_rs2.acc_row() + d_fire_counter)))


    } else {
      io.acc.read_req(i).valid := false.B
      io.acc.read_req(i).bits.scale := DontCare
      io.acc.read_req(i).bits.full := false.B
      io.acc.read_req(i).bits.igelu_qb := DontCare
      io.acc.read_req(i).bits.igelu_qc := DontCare
      io.acc.read_req(i).bits.iexp_qln2 := DontCare
      io.acc.read_req(i).bits.iexp_qln2_inv := DontCare
      io.acc.read_req(i).bits.act := DontCare
      io.acc.read_req(i).bits.fromDMA := false.B
      io.acc.read_req(i).bits.addr := DontCare
    }

    io.acc.read_resp(i).ready := false.B
  }


  // FSM logic
  switch (control_state) {
    is(waiting_for_cmd) {
      // Default state
      perform_single_preload := false.B
      perform_mul_pre := false.B
      perform_single_mul := false.B

      when(cmd.valid(0))
      {
        when(DoConfig && !matmul_in_progress && !pending_completed_rob_ids.map(_.valid).reduce(_ || _)) {
          val config_ex_rs1 = rs1s(0).asTypeOf(new ConfigExRs1(acc_scale_t_bits))
          val config_ex_rs2 = rs2s(0).asTypeOf(new ConfigExRs2)

          val config_cmd_type = rs1s(0)(1,0) // TODO magic numbers

          when (config_cmd_type === CONFIG_EX) {
            val set_only_strides = config_ex_rs1.set_only_strides

            when (!set_only_strides) {
              if (has_nonlinear_activations) {
                activation := config_ex_rs1.activation
              }
              //WS에서는 기본적으로 shift 안쓰임(OS에서 값을 축적할 때, 정밀도 손실을 줄이기 위해 사용되는 것으로 추정)
              //in_shift := config_ex_rs2.in_shift
              acc_scale := rs1s(0)(xLen - 1, 32).asTypeOf(acc_scale_t) // TODO magic number
              //Transpose 기능 삭제
              //a_transpose := config_ex_rs1.a_transpose
              //bd_transpose := config_ex_rs1.b_transpose
              /* dataflow도 ws만 지원
              if (dataflow == Dataflow.BOTH) {
                current_dataflow := config_ex_rs1.dataflow
              }
              */
            }

            a_addr_stride := config_ex_rs1.a_stride // TODO this needs to be kept in sync with ROB.scala
            c_addr_stride := config_ex_rs2.c_stride // TODO this needs to be kept in sync with ROB.scala
            config_initialized := true.B
          }

          io.completed := cmd.bits(0).rob_id

          cmd.pop := 1.U
        }

        // Preload
        .elsewhen(DoPreloads(0) && cmd.valid(1) && (raw_hazards_are_impossible.B || !raw_hazard_pre)) {
          perform_single_preload := true.B
          performing_single_preload := true.B

          start_inputting_a := false.B
          start_inputting_b := true.B
          start_inputting_d := false.B

          control_state := compute
        }

        // Overlap compute and preload
        .elsewhen(DoComputes(0) && cmd.valid(1) && DoPreloads(1) && (!third_instruction_needed || (cmd.valid(2) && !raw_hazard_mulpre)))
        {
          perform_mul_pre := true.B
          performing_mul_pre := true.B

          start_inputting_a := true.B
          start_inputting_b := true.B
          start_inputting_d := true.B

          control_state := compute
        }

        // Single mul
        .elsewhen(DoComputes(0)) {
          perform_single_mul := true.B
          performing_single_mul := true.B

          start_inputting_a := true.B
          start_inputting_d := true.B

          control_state := compute
        }

        // Flush , TODO: flush를 굳이 해야하나? 생각해보기
        // .elsewhen(matmul_in_progress && (DoConfig)) {
        //   control_state := flush
        // }
      // }.elsewhen(matmul_in_progress && current_dataflow === Dataflow.OS.id.U) {
      //   control_state := flush
      }
    }
    is(compute) {
      // Only preloading
      when(perform_single_preload) {
        start_inputting_a := false.B
        start_inputting_b := true.B
        start_inputting_d := false.B

        when(about_to_fire_all_rows) {
          cmd.pop := 1.U
          control_state := waiting_for_cmd

          pending_completed_rob_ids(0).valid := cmd.bits(0).rob_id.valid && c_address_rs2.is_garbage()
          pending_completed_rob_ids(0).bits := cmd.bits(0).rob_id.bits

        }
      }
      // Overlapping
      .elsewhen(perform_mul_pre) {
        start_inputting_a := true.B
        start_inputting_b := true.B
        start_inputting_d := true.B

        when(about_to_fire_all_rows) {
          cmd.pop := 2.U
          control_state := waiting_for_cmd

          pending_completed_rob_ids(0) := cmd.bits(0).rob_id
          pending_completed_rob_ids(1).valid := cmd.bits(1).rob_id.valid && c_address_rs2.is_garbage()
          pending_completed_rob_ids(1).bits := cmd.bits(1).rob_id.bits

        }
      }
      // Only compute
      .elsewhen(perform_single_mul) {
        start_inputting_a := true.B
        start_inputting_d := true.B

        when(about_to_fire_all_rows) {
          cmd.pop := 1.U
          control_state := waiting_for_cmd
          pending_completed_rob_ids(0) := cmd.bits(0).rob_id
        }
      }
    }
    is(flush) {
      when(wontolic.io.req.fire) {
        control_state := flushing
      }
    }
    is(flushing) {
      when(wontolic.io.req.ready) {
        // TODO we waste a cycle here if it was better to continue with the flush
        control_state := waiting_for_cmd
      }
    }
  }


  val computing = performing_mul_pre || performing_single_mul || performing_single_preload

  class ComputeCntlSignals extends Bundle {
    val perform_mul_pre = Bool()
    val perform_single_mul = Bool()
    val perform_single_preload = Bool()

    val a_bank = UInt(log2Up(sp_banks).W)
    val b_bank = UInt(log2Up(sp_banks).W)
    val d_bank = UInt(log2Up(sp_banks).W)

    val a_bank_acc = UInt(log2Up(acc_banks).W)
    val b_bank_acc = UInt(log2Up(acc_banks).W)
    val d_bank_acc = UInt(log2Up(acc_banks).W)

    val a_read_from_acc = Bool()
    val b_read_from_acc = Bool()
    val d_read_from_acc = Bool()

    val a_garbage = Bool()
    val b_garbage = Bool()
    val d_garbage = Bool()

    val accumulate_zeros = Bool()
    val preload_zeros = Bool()

    val a_fire = Bool()
    val b_fire = Bool()
    val d_fire = Bool()

    val a_unpadded_cols = UInt(log2Up(block_size + 1).W)
    val b_unpadded_cols = UInt(log2Up(block_size + 1).W)
    val d_unpadded_cols = UInt(log2Up(block_size + 1).W)

    val c_addr = local_addr_t.cloneType
    val c_rows = UInt(log2Up(block_size + 1).W)
    val c_cols = UInt(log2Up(block_size + 1).W)

    val total_rows = UInt(log2Up(block_size + 1).W)

    val rob_id = UDValid(UInt(log2Up(reservation_station_entries).W))

    val dataflow = UInt(1.W)
    val prop = UInt(1.W)

    val first = Bool()
  }

  mesh_cntl_signals_q.io.enq.valid := computing

  mesh_cntl_signals_q.io.enq.bits.perform_mul_pre := performing_mul_pre
  mesh_cntl_signals_q.io.enq.bits.perform_single_mul := performing_single_mul
  mesh_cntl_signals_q.io.enq.bits.perform_single_preload := performing_single_preload

  mesh_cntl_signals_q.io.enq.bits.a_bank := dataAbank
  mesh_cntl_signals_q.io.enq.bits.b_bank := dataBbank
  mesh_cntl_signals_q.io.enq.bits.d_bank := dataDbank

  mesh_cntl_signals_q.io.enq.bits.a_bank_acc := dataABankAcc
  mesh_cntl_signals_q.io.enq.bits.b_bank_acc := dataBBankAcc
  mesh_cntl_signals_q.io.enq.bits.d_bank_acc := dataDBankAcc

  mesh_cntl_signals_q.io.enq.bits.a_garbage := a_garbage
  mesh_cntl_signals_q.io.enq.bits.b_garbage := b_garbage
  mesh_cntl_signals_q.io.enq.bits.d_garbage := d_garbage

  mesh_cntl_signals_q.io.enq.bits.a_read_from_acc := a_read_from_acc
  mesh_cntl_signals_q.io.enq.bits.b_read_from_acc := b_read_from_acc
  mesh_cntl_signals_q.io.enq.bits.d_read_from_acc := d_read_from_acc

  mesh_cntl_signals_q.io.enq.bits.accumulate_zeros := accumulate_zeros
  mesh_cntl_signals_q.io.enq.bits.preload_zeros := preload_zeros //&& (in_shift(19) =/= 1.U)) //fixed for negative shift?

  mesh_cntl_signals_q.io.enq.bits.a_unpadded_cols := Mux(a_row_is_not_all_zeros, a_cols, 0.U)
  mesh_cntl_signals_q.io.enq.bits.b_unpadded_cols := Mux(b_row_is_not_all_zeros, b_cols, 0.U)
  mesh_cntl_signals_q.io.enq.bits.d_unpadded_cols := Mux(d_row_is_not_all_zeros, d_cols, 0.U)

  mesh_cntl_signals_q.io.enq.bits.total_rows := total_rows

  mesh_cntl_signals_q.io.enq.bits.a_fire := a_fire
  mesh_cntl_signals_q.io.enq.bits.b_fire := b_fire
  mesh_cntl_signals_q.io.enq.bits.d_fire := d_fire

  mesh_cntl_signals_q.io.enq.bits.c_addr := c_address_rs2
  mesh_cntl_signals_q.io.enq.bits.c_rows := c_rows
  mesh_cntl_signals_q.io.enq.bits.c_cols := c_cols

  mesh_cntl_signals_q.io.enq.bits.rob_id.valid := !performing_single_mul && !c_address_rs2.is_garbage()
  mesh_cntl_signals_q.io.enq.bits.rob_id.bits := cmd.bits(bc_address_place).rob_id.bits

  mesh_cntl_signals_q.io.enq.bits.dataflow := dataflow.id.U
  mesh_cntl_signals_q.io.enq.bits.prop := Mux(performing_single_preload, in_prop_flush, in_prop)//prop) //available propagate or not?

  mesh_cntl_signals_q.io.enq.bits.first := !a_fire_started && !b_fire_started && !d_fire_started

  val readData = VecInit(io.srams.read.map(_.resp.bits.data))
  val accReadData = if (ex_read_from_acc) VecInit(io.acc.read_resp.map(_.bits.data.asUInt)) else readData

  val readValid = VecInit(io.srams.read.map(bank => ex_read_from_spad.B && bank.resp.valid && !bank.resp.bits.fromDMA))
  val accReadValid = VecInit(io.acc.read_resp.map(bank => ex_read_from_acc.B && bank.valid && !bank.bits.fromDMA))

  mesh_cntl_signals_q.io.deq.ready := (!cntl.a_fire || wontolic.io.a.fire || !wontolic.io.a.ready) &&
    (!cntl.b_fire || wontolic.io.b.fire || !wontolic.io.b.ready) &&
    (!cntl.d_fire || wontolic.io.d.fire || !wontolic.io.d.ready) &&
    (!cntl.first || wontolic.io.req.ready)

  val dataA_valid = cntl.a_garbage || cntl.a_unpadded_cols === 0.U || Mux(cntl.a_read_from_acc, accReadValid(cntl.a_bank_acc), readValid(cntl.a_bank))

  val dataB_valid = cntl.b_garbage || cntl.b_unpadded_cols === 0.U || MuxCase(readValid(cntl.b_bank), Seq(
    cntl.preload_zeros -> false.B,
    cntl.b_read_from_acc -> accReadValid(cntl.b_bank_acc)
  ))
  val dataD_valid = cntl.d_garbage || cntl.d_unpadded_cols === 0.U || MuxCase(readValid(cntl.d_bank), Seq(
    cntl.accumulate_zeros -> false.B,
    cntl.d_read_from_acc -> accReadValid(cntl.d_bank_acc)
  ))

//preload_zero일 때 padding 추가해서 pipeline 맞춰주기
  val preload_zero_counter = RegInit(0.U(5.W))
  preload_zero_counter := wrappingAdd(preload_zero_counter, 1.U, block_size.U, dataA_valid && dataB_valid && cntl.preload_zeros && (cntl.perform_single_preload || cntl.perform_mul_pre))

  val dataA_unpadded = Mux(cntl.a_read_from_acc, accReadData(cntl.a_bank_acc), readData(cntl.a_bank))
  val dataB_unpadded = MuxCase(readData(cntl.b_bank), Seq(cntl.preload_zeros -> 0.U, cntl.b_read_from_acc -> accReadData(cntl.b_bank_acc)))
  val dataD_unpadded = MuxCase(readData(cntl.d_bank), Seq(cntl.accumulate_zeros -> 0.U, cntl.d_read_from_acc -> accReadData(cntl.d_bank_acc)))

  val dataA = VecInit(dataA_unpadded.asTypeOf(Vec(block_size, inputType)).zipWithIndex.map { case (d, i) => Mux(i.U < cntl.a_unpadded_cols, d, inputType.zero)})
  val dataB = VecInit(dataB_unpadded.asTypeOf(Vec(block_size, inputType)).zipWithIndex.map { case (d, i) => Mux(i.U < cntl.b_unpadded_cols, d, inputType.zero)})
  val dataD = VecInit(dataD_unpadded.asTypeOf(Vec(block_size, inputType)).zipWithIndex.map { case (d, i) => Mux(i.U < cntl.d_unpadded_cols, d, inputType.zero)})

  // Pop responses off the scratchpad io ports
  when (mesh_cntl_signals_q.io.deq.fire) {
    when (cntl.a_fire && wontolic.io.a.fire && !cntl.a_garbage && cntl.a_unpadded_cols > 0.U ) {
      when (cntl.a_read_from_acc) {
        io.acc.read_resp(cntl.a_bank_acc).ready := !io.acc.read_resp(cntl.a_bank_acc).bits.fromDMA
      }.otherwise {
        io.srams.read(cntl.a_bank).resp.ready := !io.srams.read(cntl.a_bank).resp.bits.fromDMA
      }
    }

    when (cntl.b_fire && wontolic.io.b.fire && !cntl.b_garbage && !cntl.preload_zeros && cntl.b_unpadded_cols > 0.U) {
      when (cntl.b_read_from_acc) {
        io.acc.read_resp(cntl.b_bank_acc).ready := !io.acc.read_resp(cntl.b_bank_acc).bits.fromDMA
      }.otherwise {
        io.srams.read(cntl.b_bank).resp.ready := !io.srams.read(cntl.b_bank).resp.bits.fromDMA
      }
    }

    when (cntl.d_fire && wontolic.io.d.fire && !cntl.d_garbage && !cntl.accumulate_zeros && cntl.d_unpadded_cols > 0.U) {
      when (cntl.d_read_from_acc) {
        io.acc.read_resp(cntl.d_bank_acc).ready := !io.acc.read_resp(cntl.d_bank_acc).bits.fromDMA
      }.otherwise {
        io.srams.read(cntl.d_bank).resp.ready := !io.srams.read(cntl.d_bank).resp.bits.fromDMA
      }
    }
  }

  if (!ex_read_from_acc) {
    for (acc_r <- io.acc.read_resp) {
      acc_r.ready := true.B
    }
  }

  when (cntl_valid) {
    // Default inputs
    wontolic.io.a.valid := cntl.a_fire && dataA_valid
    wontolic.io.b.valid := cntl.b_fire && dataB_valid
    wontolic.io.d.valid := cntl.d_fire && dataD_valid

    wontolic.io.a.bits := dataA.asTypeOf(Vec(meshColumns, inputType))
    wontolic.io.b.bits := dataB.asTypeOf(Vec(meshRows, inputType))
    wontolic.io.d.bits := dataD.asTypeOf(Vec(meshRows, inputType))

    wontolic.io.req.valid := mesh_cntl_signals_q.io.deq.fire && (cntl.a_fire || cntl.b_fire || cntl.d_fire)

    wontolic.io.req.bits.tag.addr := cntl.c_addr

    wontolic.io.req.bits.total_rows := cntl.total_rows
  }

  when (cntl_valid && cntl.perform_single_preload) {
    wontolic.io.a.bits := 0.U.asTypeOf(Vec(meshColumns, inputType))
    wontolic.io.b.bits := dataB.asUInt.asTypeOf(Vec(meshRows, inputType))
  }

  when (cntl_valid && cntl.perform_single_mul) {
    wontolic.io.a.bits := dataA.asUInt.asTypeOf(Vec(meshColumns, inputType))
    wontolic.io.b.bits := 0.U.asTypeOf(Vec(meshRows, inputType))
    wontolic.io.req.bits.tag.addr.make_this_garbage()
  }

  // Scratchpad writes
  // val output_counter = new Counter(block_size)
  val output_counter = RegInit(0.U(log2Up(block_size).W))

  val w_total_output_rows = wontolic.io.resp.bits.total_rows

  val w_address = wontolic.io.resp.bits.tag.addr + output_counter * c_addr_stride
  val write_to_acc = w_address.is_acc_addr

  val w_bank = Mux(write_to_acc, w_address.acc_bank(), w_address.sp_bank())
  val w_row = Mux(write_to_acc, w_address.acc_row(), w_address.sp_row())

  val is_garbage_addr = wontolic.io.resp.bits.tag.addr.is_garbage()

  val w_matrix_rows = wontolic.io.resp.bits.tag.rows
  val w_matrix_cols = wontolic.io.resp.bits.tag.cols

  val write_this_row = output_counter < w_matrix_rows
  val w_mask = (0 until block_size).map(_.U < w_matrix_cols) // This is an element-wise mask, rather than a byte-wise mask

  // Write to normal scratchpad
  for(i <- 0 until sp_banks) {
    val activated_wdata = VecInit(wontolic.io.resp.bits.data.map{ e =>
      val e_clipped = e.clippedToWidthOf(inputType)
      val e_act = MuxCase(e_clipped, Seq(
        (activation === Activation.RELU) -> e_clipped.relu))

      e_act
    })

    if (ex_write_to_spad) {
      io.srams.write(i).en := start_array_outputting && w_bank === i.U && !write_to_acc && !is_garbage_addr && write_this_row
      io.srams.write(i).addr := w_row
      io.srams.write(i).data := activated_wdata.asUInt
      io.srams.write(i).mask := w_mask.flatMap(b => Seq.fill(inputType.getWidth / (aligned_to * 8))(b))
    } else {
      io.srams.write(i).en := false.B
      io.srams.write(i).addr := DontCare
      io.srams.write(i).data := DontCare
      io.srams.write(i).mask := DontCare
    }
  }

  // Write to accumulator
  for (i <- 0 until acc_banks) {
    if (ex_write_to_acc) {
      io.acc.write(i).valid := start_array_outputting && w_bank === i.U && write_to_acc && !is_garbage_addr && write_this_row
      io.acc.write(i).bits.addr := w_row
      //TODO: 수정필요
      val flatData = VecInit(wontolic.io.resp.bits.data.map(_.withWidthOf(accType)).toList)
      io.acc.write(i).bits.data :=  VecInit(flatData.map(e => VecInit(Seq(e))))
      io.acc.write(i).bits.acc := w_address.accumulate
      io.acc.write(i).bits.mask := w_mask.flatMap(b => Seq.fill(accType.getWidth / (aligned_to * 8))(b))
    } else {
      io.acc.write(i).valid := false.B
      io.acc.write(i).bits.addr := DontCare
      io.acc.write(i).bits.data := DontCare
      io.acc.write(i).bits.acc := DontCare
      io.acc.write(i).bits.mask := DontCare
    }

    assert(!(io.acc.write(i).valid && !io.acc.write(i).ready), "Execute controller write to AccumulatorMem was skipped")
  }

  // Handle dependencies and turn off outputs for garbage addresses
  val mesh_completed_rob_id_fire = WireInit(false.B)
  //val complete_lock = RegInit(false.B)

  //Seah: added for WS accumulator
  when(wontolic.io.resp.fire && wontolic.io.resp.bits.tag.rob_id.valid) {
    output_counter := wrappingAdd(output_counter, 1.U, w_total_output_rows)
    val last = wontolic.io.resp.bits.last

    when(last) {
      mesh_completed_rob_id_fire := true.B
      io.completed.valid := true.B
      io.completed.bits := wontolic.io.resp.bits.tag.rob_id.bits
    }
    start_array_outputting :=  !is_garbage_addr
  }

  when (!mesh_completed_rob_id_fire) {
    when(pending_completed_rob_ids(0).valid) {
      io.completed.valid := true.B
      io.completed.bits := pending_completed_rob_ids(0).pop()
    }.elsewhen(pending_completed_rob_ids(1).valid) {
      io.completed.valid := true.B
      io.completed.bits := pending_completed_rob_ids(1).pop()
    }
  }
  val complete_bits_count = RegInit(0.U(15.W))
  when(io.completed.valid) {
    complete_bits_count := complete_bits_count + 1.U
  }

  when (reset.asBool) {
    // pending_completed_rob_id.valid := false.B
    pending_completed_rob_ids.foreach(_.valid := false.B)
  }

  // Performance counter
  CounterEventIO.init(io.counter)
  io.counter.connectEventSignal(CounterEvent.EXE_ACTIVE_CYCLE, firing || matmul_in_progress)
  io.counter.connectEventSignal(CounterEvent.EXE_FLUSH_CYCLE,
    control_state === flushing || control_state === flush)
  io.counter.connectEventSignal(CounterEvent.EXE_CONTROL_Q_BLOCK_CYCLE,
    !mesh_cntl_signals_q.io.enq.ready && mesh_cntl_signals_q.io.enq.valid)
  io.counter.connectEventSignal(CounterEvent.EXE_PRELOAD_HAZ_CYCLE,
    cmd.valid(0) && DoPreloads(0) && cmd.valid(1) && raw_hazard_pre)
  io.counter.connectEventSignal(CounterEvent.EXE_OVERLAP_HAZ_CYCLE,
    cmd.valid(0) && DoPreloads(1) && cmd.valid(1) && DoComputes(0) && cmd.valid(2) && raw_hazard_mulpre)
  io.counter.connectEventSignal(CounterEvent.A_GARBAGE_CYCLES, cntl.a_garbage)
  io.counter.connectEventSignal(CounterEvent.B_GARBAGE_CYCLES, cntl.b_garbage)
  io.counter.connectEventSignal(CounterEvent.D_GARBAGE_CYCLES, cntl.d_garbage)
  io.counter.connectEventSignal(CounterEvent.ACC_A_WAIT_CYCLE,
    !(!cntl.a_fire || wontolic.io.a.fire || !wontolic.io.a.ready) && cntl.a_read_from_acc)
  io.counter.connectEventSignal(CounterEvent.ACC_B_WAIT_CYCLE,
    !(!cntl.b_fire || wontolic.io.b.fire || !wontolic.io.b.ready) && cntl.b_read_from_acc)
  io.counter.connectEventSignal(CounterEvent.ACC_D_WAIT_CYCLE,
    !(!cntl.d_fire || wontolic.io.d.fire || !wontolic.io.d.ready) && cntl.d_read_from_acc)
  io.counter.connectEventSignal(CounterEvent.SCRATCHPAD_A_WAIT_CYCLE,
    !(!cntl.a_fire || wontolic.io.a.fire || !wontolic.io.a.ready) && !cntl.a_read_from_acc)
  io.counter.connectEventSignal(CounterEvent.SCRATCHPAD_B_WAIT_CYCLE,
    !(!cntl.b_fire || wontolic.io.b.fire || !wontolic.io.b.ready) && !cntl.b_read_from_acc)
  io.counter.connectEventSignal(CounterEvent.SCRATCHPAD_D_WAIT_CYCLE,
    !(!cntl.d_fire || wontolic.io.d.fire || !wontolic.io.d.ready) && !cntl.d_read_from_acc)

  if (use_firesim_simulation_counters) {
    val ex_flush_cycle = control_state === flushing || control_state === flush
    val ex_preload_haz_cycle = cmd.valid(0) && DoPreloads(0) && cmd.valid(1) && raw_hazard_pre
    val ex_mulpre_haz_cycle = cmd.valid(0) && DoPreloads(1) && cmd.valid(1) && DoComputes(0) && cmd.valid(2) && raw_hazard_mulpre

    PerfCounter(ex_flush_cycle, "ex_flush_cycle", "cycles during which the ex controller is flushing the spatial array")
    PerfCounter(ex_preload_haz_cycle, "ex_preload_haz_cycle", "cycles during which the execute controller is stalling preloads due to hazards")
    PerfCounter(ex_mulpre_haz_cycle, "ex_mulpre_haz_cycle", "cycles during which the execute controller is stalling matmuls due to hazards")
  }

 // Performance counter
  io.counter.connectEventSignal(CounterEvent.IM2COL_ACTIVE_CYCLES, false.B)
  io.counter.connectEventSignal(CounterEvent.IM2COL_MEM_CYCLES, false.B)
  io.counter.connectEventSignal(CounterEvent.IM2COL_TRANSPOSER_WAIT_CYCLE, false.B)
  io.counter.connectEventSignal(CounterEvent.TRANSPOSE_PRELOAD_UNROLLER_ACTIVE_CYCLES, false.B)  


}