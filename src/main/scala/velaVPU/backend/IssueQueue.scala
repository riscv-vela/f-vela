package velaVPU.backend

import chisel3._
import chisel3.util._
import chisel3.experimental.dataview._
import org.chipsalliance.cde.config._
import freechips.rocketchip.rocket._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import velaVPU.common._

class IssueQueue(depth: Int, nSeqs: Int)(implicit p: Parameters) extends CoreModule()(p) with HasVectorParams {

  val io = IO(new Bundle {
    val enq = Flipped(Decoupled(new IssueQueueInst(nSeqs)))
    val deq = Decoupled(new IssueQueueInst(nSeqs))
    val hazards = Output(Vec(depth, Valid(new InstructionHazard)))
  })

  // @@@@ Custom: precompute per-vreg architectural masks (used by the PID span logic).
  private val all_vreg_arch_mask = VecInit((0 until 32).map { i => (1.U(32.W) << i.U(5.W)) })
  private val ARCH_W = all_vreg_arch_mask.head.getWidth.W

  if (depth > 0) {
    val q = Module(new DCEQueue(new IssueQueueInst(nSeqs), depth, pipe=true))
    q.io.enq <> io.enq
    io.deq <> q.io.deq

    q.io.peek.zip(io.hazards).foreach { case (e,h) =>
      h.valid    := e.valid
      h.bits.vat := e.bits.vat
      val rs2 = Mux(e.bits.rs1_is_rs2, e.bits.rs1, e.bits.rs2)
      val only_writes_vd0 = e.bits.scalar_to_vd0 || e.bits.reduction
      val vd_lmul  = Mux(only_writes_vd0      , 0.U, e.bits.emul +& e.bits.wide_vd +& e.bits.nf_log2)
      val vs1_lmul = Mux(e.bits.reads_vs1_mask, 0.U, e.bits.emul)
      val vs2_lmul = Mux(e.bits.reads_vs2_mask, 0.U, e.bits.emul +& e.bits.wide_vs2 +& e.bits.nf_log2)
      val vd_arch_mask  = get_arch_mask(e.bits.rd , vd_lmul )
      val vs1_arch_mask = get_arch_mask(e.bits.rs1, vs1_lmul)
      val vs2_arch_mask = get_arch_mask(rs2       , vs2_lmul)

      // @@@@ Custom: the PID instruction spans the whole register range [min(rs2,rd), max(rs2,rd)]
      //   (in-place read/modify/write of the actuator vregs). When pid_flag is set, reserve the
      //   entire span for both reads and writes so younger insns cannot race the multi-vreg op.
      val lo = Mux(rs2 <= e.bits.rd, rs2, e.bits.rd)
      val hi = Mux(rs2 <= e.bits.rd, e.bits.rd, rs2)
      var span_mask = 0.U(ARCH_W)
      for (i <- 0 until 32) {
        val inRange = (i.U >= lo) && (i.U <= hi)
        span_mask = span_mask | Mux(inRange, all_vreg_arch_mask(i), 0.U)
      }
      val pid_flag = e.bits.pid_flag

      val rintent_base = Seq(
        (e.bits.renv1, vs1_arch_mask),
        (e.bits.renv2, vs2_arch_mask),
        (e.bits.renvd, vd_arch_mask),
        (e.bits.renvm, 1.U)
      ).map(t => Mux(t._1, t._2, 0.U)).reduce(_|_)
      val wintent_base = Mux(e.bits.wvd, vd_arch_mask, 0.U)

      h.bits.rintent := Mux(pid_flag, rintent_base | span_mask, rintent_base)
      h.bits.wintent := Mux(pid_flag, span_mask, wintent_base)
    }
  } else {
    io.deq <> io.enq
  }
}
