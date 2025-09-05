package gemmini

import chisel3._
import chisel3.util._

import gemmini.Util._

class WontolicReq[T <: Data: Arithmetic, TagT <: TagQueueTag with Data](tagType: TagT, ma_length: Int) extends Bundle {
  val in_prop = Bool()
  val in_acc = Bool()
  val total_rows = UInt(log2Up(ma_length+1).W)
  val tag = tagType
  val flush = UInt(2.W)
}

class WontolicResp[T <: Data: Arithmetic, TagT <: TagQueueTag with Data](outputType: T, ma_length: Int, ma_num: Int, tagType: TagT) extends Bundle {
  val data = Vec(ma_num, outputType)
  val total_rows = UInt(log2Up(ma_length+1).W)
  val tag = tagType 
  val last = Bool()
}

class Wontolic[T <: Data](inputType: T, outputType: T, ma_length: Int, ma_num: Int, max_simultaneous_matmuls: Int) (implicit ev: Arithmetic[T])  extends Module {
    import ev._

    val io = IO(new Bundle {
        val in_a = Input(Vec(ma_length, inputType))
        val in_b = Input(Vec(ma_num, inputType))
        val in_d = Input(Vec(ma_num, inputType))

        val in_last = Input(Vec(ma_length, Bool()))
        val in_prop = Input(Vec(ma_length, Bool()))
        val in_valid = Input(Vec(ma_length, Bool()))
        val in_id = Input(Vec(ma_length, UInt(log2Up(max_simultaneous_matmuls).W)))
        val in_fire_counter = Input(UInt(log2Up(ma_length).W))
        val in_b_fire = Input(Bool())
        val in_acc = Input(Bool())

        val out_c = Output(Vec(ma_num, outputType))
        val out_last = Output(Vec(ma_num, Bool()))
        val out_id = Output(Vec(ma_num, UInt(log2Up(max_simultaneous_matmuls).W)))
        val out_valid = Output(Vec(ma_num, Bool()))
        val out_prop = Output(Vec(ma_num, Bool()))
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
        mularraybundle(i).io.in_fire_counter := io.in_fire_counter
        //in_valid, in_last 등의 신호 입력
        mularraybundle(i).io.in_valid :=  VecInit(buffvectorarray.map(_.io.out_valid))
        mularraybundle(i).io.in_last :=  VecInit(buffvectorarray.map(_.io.out_last))
        mularraybundle(i).io.in_id :=  VecInit(buffvectorarray.map(_.io.out_id))
        mularraybundle(i).io.in_prop :=  VecInit(buffvectorarray.map(_.io.out_prop))
        mularraybundle(i).io.in_valid_b := io.in_valid
        mularraybundle(i).io.in_b_fire := io.in_b_fire
    }
    


    //adder tree의 결과 값을 buffadderarray의 입력으로 연결 + buffadderarray에 in_d 연결
    for(i <- 0 until ma_num) {
        buffadderarray(i).io.in_d := io.in_d(i)
        buffadderarray(i).io.in_result := mularraybundle(i).io.out_sum
        buffadderarray(i).io.in_valid := mularraybundle(i).io.out_valid
        buffadderarray(i).io.in_last := mularraybundle(i).io.out_last
        buffadderarray(i).io.in_id := mularraybundle(i).io.out_id
        buffadderarray(i).io.in_prop := mularraybundle(i).io.out_prop

        buffadderarray(i).io.in_acc := io.in_acc

        io.out_c(i) := buffadderarray(i).io.out_c
        io.out_valid(i) := buffadderarray(i).io.out_valid
        io.out_last(i) := buffadderarray(i).io.out_last
        io.out_id(i) := buffadderarray(i).io.out_id
        io.out_prop(i) := buffadderarray(i).io.out_prop
    }

}

//Wontolic과 ExecuteController와의 인터페이스, a를 한 row씩, b를 한 column씩 입력
class WontolicWithDelays[T <: Data: Arithmetic, U <: TagQueueTag with Data]
  (inputType: T, val outputType: T, accType: T,
   tagType: U, df: Dataflow.Value, tree_reduction: Boolean, tile_latency: Int, output_delay: Int,
   tileRows: Int, tileColumns: Int, meshRows: Int, meshColumns: Int,
   leftBanks: Int, upBanks: Int, outBanks: Int = 1, n_simultaneous_matmuls: Int = 3)
  extends Module {

    val ma_length = meshColumns
    val ma_num = meshRows

    val A_TYPE = Vec(ma_length, inputType)
    val B_TYPE = Vec(ma_num, inputType)
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
        val resp = Valid(new WontolicResp(outputType, ma_length,ma_num ,tagType.cloneType))

        val tags_in_progress = Output(Vec(tagqlen, tagType))
    })

     //입력된 req 저장해놓음.pop을 통해 가져오면 valid false됨.push하면 valid true TODO: argument 수정할 것
    val req = Reg(UDValid(new WontolicReq( tagType, ma_length)))

    //PE의 double buffering control logic
    val in_prop = Reg(Bool())
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

    //wontolic에 a, b, d 입력
    val wontolic = Module(new Wontolic(inputType, outputType, ma_length, ma_num, max_simultaneous_matmuls))

    val a_buf = RegEnable(io.a.bits, io.a.fire)   // fire 때만 io.a.bits → a_buf
    val b_buf = RegEnable(io.b.bits, io.b.fire)   // (Decoupled ⇒ ready & valid)
    val d_buf = RegEnable(io.d.bits, io.d.fire)


    wontolic.io.in_a := a_buf
    wontolic.io.in_b := b_buf
    wontolic.io.in_d := d_buf

    wontolic.io.in_b_fire := RegNext(io.b.fire)
    wontolic.io.in_acc := io.req.bits.in_acc

    wontolic.io.in_prop.foreach(_ := in_prop)


    val out_matmul_id: UInt = wontolic.io.out_id.reduce(_ | _)

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

    wontolic.io.in_fire_counter := fire_counter

    //pause 로 valid신호 만들어서 wontolic의 각 pe에 전파
    val pause = !req.valid || !input_next_row_into_spatial_array
    val not_paused_vec = VecInit(Seq.fill(ma_num)(!pause))
    wontolic.io.in_valid := not_paused_vec


    val matmul_last_vec = VecInit(Seq.fill(ma_num)(last_fire))
    wontolic.io.in_last := matmul_last_vec

    val matmul_id_vec = VecInit(Seq.fill(meshColumns)(matmul_id))
    wontolic.io.in_id := matmul_id_vec

    io.resp.bits.total_rows := Mux(total_rows_q.io.deq.valid && out_matmul_id === total_rows_q.io.deq.bits.id,
        total_rows_q.io.deq.bits.total_rows, ma_length.U)

    total_rows_q.io.deq.ready := io.resp.valid && io.resp.bits.last && out_matmul_id === total_rows_q.io.deq.bits.id


    //output 연결
    io.resp.bits.data := wontolic.io.out_c
    io.resp.valid := wontolic.io.out_valid.reduce(_ && _)
    io.resp.bits.last := wontolic.io.out_last.reduce(_ && _)
    io.resp.bits.tag := Mux(tagq.io.deq.valid && out_matmul_id === tagq.io.deq.bits.id, tagq.io.deq.bits.tag, tag_garbage)
    io.tags_in_progress := VecInit(tagq.io.all.map(_.tag))


    when (reset.asBool) {
        req.valid := false.B
    }

  }
