package gemmini

import chisel3._
import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.diplomacy.LazyModule
import freechips.rocketchip.tile.{BuildRoCC, OpcodeSet}

// ===========================================================================
// fp16 matmul engine: one Gemmini that runs int8/int2 (ternary) matmul/conv as
// before and, when a LOOP_WS carries is_fp, fp16 x fp16 matmul on the fp16 exe
// unit (FpExeUnit). The accumulator stays 32b per cell: int ops accumulate
// int32, fp ops accumulate raw fp32 (shared AddRecFN adder). The mvout either
// saturates acc*scale to int8 or writes the fp32 bits (output_as_float).
//
//   - DIM = 16 (128b spad row = 16 int8 = 8 fp16). An fp16 logical row spans 2
//     spad rows (lo / hi half); the fp unit reads 8 fp16 x 2 passes.
//   - spatialArrayOutputType = SInt(32) so fp32 results pass without truncation.
// ===========================================================================
object GemminiFp16Configs {
  val fp16MatmulConfig = GemminiConfigs.defaultConfig.copy(
    opcodes = OpcodeSet.custom3,
    dataflow = Dataflow.WS,
    tileRows = 1, tileColumns = 1,
    meshRows = 16, meshColumns = 16,
    spatialArrayOutputType = SInt(32.W),
    support_fp = true,
  )
}

class Fp16MatmulGemminiConfig extends Config((site, here, up) => {
  case BuildRoCC => up(BuildRoCC) ++ Seq(
    (p: Parameters) => {
      implicit val q = p
      LazyModule(new Gemmini(GemminiFp16Configs.fp16MatmulConfig))
    }
  )
})
