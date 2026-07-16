package f_vela_saturn.frontend

import chisel3._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import f_vela_saturn.common._
import f_vela_saturn.insns.{VectorInstruction, VectorDecoder}

class EarlyVectorDecode(supported_ex_insns: Seq[VectorInstruction])(implicit p: Parameters) extends RocketVectorDecoder()(p) with HasVectorConsts {

  // io.vector := false.B
  io.legal := false.B
  io.fp := false.B
  io.read_rs1 := false.B
  io.read_rs2 := false.B
  io.read_frs1 := false.B
  io.write_rd := false.B
  io.write_frd := false.B

  val opcode = io.inst(6,0)
  val width = io.inst(14,12)
  val lumop = io.inst(24,20)
  val sumop = lumop
  val vm = io.inst(25)
  val mop = io.inst(27,26)
  val mew = io.inst(28)
  val nf = io.inst(31,29)
  val funct3 = io.inst(14,12)
  val funct6 = io.inst(31,26)
  val rs1 = io.inst(19,15)
  val rs2 = io.inst(24,20)

  val v_load = opcode === opcLoad && !width.isOneOf(1.U, 2.U, 3.U, 4.U)
  val v_store = opcode === opcStore && !width.isOneOf(1.U, 2.U, 3.U, 4.U)
  val v_arith_maybe = opcode === opcVector && funct3 =/= 7.U
  val v_arith = v_arith_maybe && new VectorDecoder(rs1, rs2, funct3, funct6, io.vconfig.vtype.vsew, supported_ex_insns, Nil).matched
  // @@@@ Drive RocketVectorDecoder.io.vector (added by the f-vela rocket-patch). This reports
  //   "is a vector op" independent of legality/vconfig, so RocketCore sets id_ctrl.vec (and thus
  //   applies the vconfig hazard stall) even in the shadow of a not-yet-retired vsetvl. The real
  //   legality is then re-checked in EX against the settled vconfig (patch's ex_vec_valid).
  io.vector := v_load || v_store || v_arith_maybe

  when (v_load || v_store) {
    val unit = mop === 0.U
    // @@@@ Do NOT gate legality on vtype.vill here. With the chipyard-1.13.0 RocketVectorDecoder
    //   interface (no io.vector), Rocket only treats an op as a vector op — and thus only applies
    //   the vconfig hazard stall — when this decoder reports io.legal. A vector load/store decoded
    //   in the shadow of a not-yet-retired vsetvl would see a stale vill=1, be marked illegal, and
    //   (never being recognized as vector) trap instead of stalling. Report legal purely from the
    //   encoding; the real vtype.vill illegality is still enforced downstream in RocketCore
    //   (id_illegal_insn: id_ctrl.vec && vconfig.vtype.vill). Mirrors the known-good 1.13.0 ref.
    io.legal := mew === 0.U && width.isOneOf(0.U, 5.U, 6.U, 7.U)
    when (unit) {
      when (v_load && !lumop.isOneOf(lumopUnit, lumopWhole, lumopMask, lumopFF)) { io.legal := false.B }
      when (v_store && !sumop.isOneOf(sumopUnit, sumopWhole, sumopMask)) { io.legal := false.B }
    }
    when (mew === 1.U) { io.legal := false.B }
    io.read_rs1 := true.B
    io.read_rs2 := mop === mopStrided
  } .elsewhen (v_arith) {
    // @@@@ See the load/store note above: report legal from the decode match alone, not vill.
    //   RocketCore still traps on a real vtype.vill for id_ctrl.vec ops. Mirrors the 1.13.0 ref.
    io.legal := true.B
    io.read_rs1 := funct3.isOneOf(OPIVX, OPMVX)
    io.read_frs1 := funct3 === OPFVF
    io.write_rd := funct3 === OPMVV && OPMFunct6(funct6) === OPMFunct6.wrxunary0
    io.write_frd := funct3 === OPFVV && OPFFunct6(funct6) === OPFFunct6.wrfunary0
    io.fp := funct3 === OPFVF
  }
}
