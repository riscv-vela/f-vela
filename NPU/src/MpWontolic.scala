package gemmini

import chisel3._
import chisel3.util._

import gemmini.Util._


//Wontolic과 ExecuteController와의 인터페이스, a를 한 row씩, b를 한 column씩 입력
class WontolicWithDelays[T <: Data: Arithmetic, U <: TagQueueTag with Data]
  (inputType: T, weightType: T, val outputType: T, accType: T,
   tagType: U, df: Dataflow.Value, tree_reduction: Boolean, tile_latency: Int, output_delay: Int,
   tileRows: Int, tileColumns: Int, meshRows: Int, meshColumns: Int,
   leftBanks: Int, upBanks: Int, outBanks: Int = 1, n_simultaneous_matmuls: Int = 3)
  extends Module {

    val ma_length = meshColumns
    val ma_num = meshRows
    val mp_ma_num = meshRows * (inputType.getWidth / weightType.getWidth - 1)

    val B_MAX_SIZE = scala.math.max(ma_length, ma_num)

    val A_TYPE = Vec(ma_length, inputType)
    val B_TYPE = Vec(B_MAX_SIZE, inputType)
    val B_TW_TYPE = Vec(mp_ma_num, weightType)
    val B_W_TYPE = Vec(B_MAX_SIZE, weightType)
    val C_TYPE = Vec(ma_num, inputType)
    val D_TYPE = Vec(ma_num, inputType)

    //TODO: 한번에 실행 가능한 matrix 연산의 개수 => 실험적으로 설정해볼 것.
    val max_simultaneous_matmuls = n_simultaneous_matmuls
    val tagqlen = max_simultaneous_matmuls+1


    val io = IO(new Bundle {
        val a = Flipped(Decoupled(A_TYPE))
        val b = Flipped(Decoupled(B_TYPE))
        val d = Flipped(Decoupled(D_TYPE))
        //TODO: argument 수정
        val req = Flipped(Decoupled(new WontolicReq(tagType.cloneType, ma_length)))
        val resp = Valid(new WontolicResp(inputType, weightType, outputType, ma_length,ma_num ,tagType.cloneType))

        val tags_in_progress = Output(Vec(tagqlen, tagType))
    })

     //입력된 req 저장해놓음.pop을 통해 가져오면 valid false됨.push하면 valid true TODO: argument 수정할 것
    val req = RegInit(0.U.asTypeOf(UDValid(new WontolicReq(tagType, ma_length))))

    //PE의 double buffering control logic
    val in_prop = RegInit(false.B)
    val total_fires = req.bits.total_rows
    val fire_counter = RegInit(0.U(log2Up(ma_length).W))

    val a_written = RegInit(false.B)
    val b_written = RegInit(false.B)
    val d_written = RegInit(false.B)
    //flush 모드이거나, req가 valid하고 abd 모두 로드했다면, 다음 행 로드 가능
    val input_next_row_into_spatial_array = req.valid && ((a_written && b_written && d_written) || req.bits.flush > 0.U)

    val last_fire = fire_counter === total_fires - 1.U && input_next_row_into_spatial_array
    val matmul_id = RegInit(0.U(log2Up(max_simultaneous_matmuls).W))


    //last_fire면 다음 cycle에 io.req.ready 1되어서 fire true됨
    when (io.req.fire) {
        req.push(io.req.bits)
        //gemmini_compute_preloaded => COMPUTE_AND_FLIP이면 propagate = 1 => in_prop = 1
        in_prop := io.req.bits.in_prop ^ in_prop
        matmul_id := wrappingAdd(matmul_id, 1.U, max_simultaneous_matmuls)
    }.elsewhen (last_fire) {
        req.valid := req.bits.flush > 1.U
        req.bits.flush := req.bits.flush - 1.U
    }
    // Tags
    class TagWithIdAndTotalRows extends Bundle with TagQueueTag {
        val tag = tagType.cloneType
        val id = UInt(log2Up(max_simultaneous_matmuls).W)
        val total_rows = UInt(log2Up(ma_num+1).W)

        override def make_this_garbage(dummy: Int=0): Unit = {
        total_rows := ma_num.U
        tag.make_this_garbage()
        }

    }
    //TODO : 실험적으로 정해보기
    val matmul_id_of_output = wrappingAdd(matmul_id, 2.U, max_simultaneous_matmuls)
    val matmul_id_of_current = wrappingAdd(matmul_id, 1.U, max_simultaneous_matmuls)


    val tagq = Module(new TagQueue(new TagWithIdAndTotalRows, tagqlen))
    tagq.io.enq.valid := io.req.fire && io.req.bits.flush === 0.U
    tagq.io.enq.bits.tag := io.req.bits.tag
    tagq.io.enq.bits.total_rows := DontCare
    tagq.io.enq.bits.id := matmul_id_of_output

    val tag_garbage = Wire(tagType.cloneType)
    tag_garbage := DontCare
    tag_garbage.make_this_garbage()

    //wontolic, mpexeunit에 a, b, d 입력
    val wontolic = Module(new Wontolic(inputType, outputType, ma_length, ma_num, max_simultaneous_matmuls))
    val mpexeunit = Module(new MpExeUnit(inputType, weightType ,outputType, ma_length, mp_ma_num, max_simultaneous_matmuls))

    val a_buf = RegEnable(io.a.bits, io.a.fire)   // fire 때만 io.a.bits → a_buf
    val b_buf_0 = RegEnable(io.b.bits, io.b.fire)   // (Decoupled ⇒ ready & valid)
    val b_buf = RegNext(b_buf_0) 
    val d_buf = RegEnable(io.d.bits, io.d.fire)

    val b_buf_upper_96 = b_buf.asUInt(127, 32).asTypeOf(B_TW_TYPE) // mpexeunit 입력 타입에 맞춰야 함
    val b_buf_lower_32 = b_buf.asUInt(31, 0).asTypeOf(B_W_TYPE) // wontolic 입력 타입에 맞춰야 함
    val extended_b_buf_lower_32 = VecInit(b_buf_lower_32.map { chunk =>
        val chunk_asUInt = chunk.asUInt
        Cat(Fill(6, chunk_asUInt(1)), chunk_asUInt).asSInt
    })

    val is_mpgemm = req.bits.is_mpgemm

    // mpexeunit의 Input
    mpexeunit.io.in_a := Mux(is_mpgemm, a_buf, 0.U.asTypeOf(A_TYPE))
    mpexeunit.io.in_b := Mux(is_mpgemm, b_buf_upper_96, 0.U.asTypeOf(B_TW_TYPE))
    mpexeunit.io.in_d := 0.U.asTypeOf(chiselTypeOf(mpexeunit.io.in_d))

    // wontolic의 Input
    // wontolic.io.in_a := Mux(is_mpgemm, 0.U.asTypeOf(A_TYPE), a_buf)
    wontolic.io.in_a := a_buf
    when(is_mpgemm){
        wontolic.io.in_b := extended_b_buf_lower_32.asTypeOf(B_TYPE)
        wontolic.io.in_b_vec := extended_b_buf_lower_32.asTypeOf(B_TYPE)
    } .otherwise {
        wontolic.io.in_b := b_buf
        wontolic.io.in_b_vec := b_buf
    }
    
    wontolic.io.in_d := Mux(is_mpgemm, 0.U.asTypeOf(D_TYPE), d_buf)

    wontolic.io.in_acc := io.req.bits.in_acc
    wontolic.io.in_preload := io.req.bits.in_preload

    wontolic.io.in_prop.foreach(_ := in_prop)

    wontolic.io.in_b_transpose := RegNext(req.bits.b_transpose)
    wontolic.io.in_is_mpgemm := is_mpgemm

    val out_matmul_id: UInt = wontolic.io.out_id.head

    tagq.io.deq.ready := io.resp.valid && io.resp.bits.last && out_matmul_id === tagq.io.deq.bits.id

    //row의 수와 id 저장
    val total_rows_q = Module(new Queue(new TagWithIdAndTotalRows, tagqlen))
    total_rows_q.io.enq.valid := io.req.fire && io.req.bits.flush === 0.U
    total_rows_q.io.enq.bits.tag := DontCare
    total_rows_q.io.enq.bits.total_rows := io.req.bits.total_rows
    total_rows_q.io.enq.bits.id := matmul_id_of_current


    io.req.ready := (!req.valid || last_fire) && tagq.io.enq.ready && total_rows_q.io.enq.ready

    // 다음 줄 로드 할때는 모두 false로 바꾸고, fire counter 1증가
    when (input_next_row_into_spatial_array) {
        a_written := false.B
        b_written := false.B
        d_written := false.B

        fire_counter := wrappingAdd(fire_counter, 1.U, total_fires)
    }
    //a, b, d ready, valid되면 written true
    when (io.a.fire) {
        a_written := true.B
    }

    when (io.b.fire) {
        b_written := true.B
    }

    when (io.d.fire) {
        d_written := true.B
    }

    io.a.ready := !a_written || input_next_row_into_spatial_array || io.req.ready
    io.b.ready := !b_written || input_next_row_into_spatial_array || io.req.ready
    io.d.ready := !d_written || input_next_row_into_spatial_array || io.req.ready

    wontolic.io.in_fire_counter := RegNext(fire_counter)

    //pause 로 valid신호 만들어서 wontolic의 각 pe에 전파
    val pause = !req.valid || !input_next_row_into_spatial_array
    val not_paused_vec = VecInit(Seq.fill(ma_length)(!pause))
    wontolic.io.in_valid := not_paused_vec

    val matmul_last_vec = VecInit(Seq.fill(ma_length)(last_fire))
    wontolic.io.in_last := matmul_last_vec

    val matmul_id_vec = VecInit(Seq.fill(meshColumns)(matmul_id))
    wontolic.io.in_id := matmul_id_vec

    mpexeunit.io.in_valid := Mux(is_mpgemm, not_paused_vec, VecInit(Seq.fill(ma_length)(false.B)))
    mpexeunit.io.in_last := Mux(is_mpgemm, matmul_last_vec, VecInit(Seq.fill(ma_length)(false.B)))
    mpexeunit.io.in_id := matmul_id_vec
    mpexeunit.io.in_acc := io.req.bits.in_acc
    mpexeunit.io.in_preload := io.req.bits.in_preload
    mpexeunit.io.in_prop.foreach(_ := in_prop)
    mpexeunit.io.in_fire_counter :=  RegNext(fire_counter)

    io.resp.bits.total_rows := Mux(total_rows_q.io.deq.valid && out_matmul_id === total_rows_q.io.deq.bits.id,
        total_rows_q.io.deq.bits.total_rows, ma_length.U)

    total_rows_q.io.deq.ready := io.resp.valid && io.resp.bits.last && out_matmul_id === total_rows_q.io.deq.bits.id


    //output 연결
    io.resp.bits.data := Cat(mpexeunit.io.out_c.asUInt, wontolic.io.out_c.asUInt).asTypeOf(io.resp.bits.data.cloneType)
    io.resp.valid := wontolic.io.out_valid.head
    io.resp.bits.last := wontolic.io.out_last.head
    io.resp.bits.tag := Mux(tagq.io.deq.valid && out_matmul_id === tagq.io.deq.bits.id, tagq.io.deq.bits.tag, tag_garbage)
    io.tags_in_progress := VecInit(tagq.io.all.map(_.tag))
    io.resp.bits.is_mpgemm := wontolic.io.out_is_mpgemm


    when (reset.asBool) {
        req.valid := false.B
    }

  }
