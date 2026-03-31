/** 
 * @author Woojin
 * 
 * @brief FVelaGemminiConfigTest
 * 
 * @note 
 * - Build : make -C sims/verilator CONFIG=FVelaGemminiConfigTest
 * - Test
 * 		1) Verilator 시뮬레이터: ./scripts/sims/verilator_sim.sh FVelaGemminiConfigTest simple_test.elf  or  ./scripts/sims/verilator_sim.sh CryptGemminiRocketConfig npu_matmul_test.elf
 * 		2) Spike 시뮬레이터: ./scripts/sims/run_spike.sh build/simple_test.elf
 * */

package chipyard
import org.chipsalliance.cde.config.Config
import freechips.rocketchip.rocket._
import f_vela_gemmini._  

class FVelaGemminiConfigTest extends Config(
	new f_vela_gemmini.GemminiCustomConfig() ++ 
	// new f_vela_gemmini.DefaultGemminiConfig ++ 
	// new f_vela_gemmini.GemminiParamsDSE1 ++ 
	new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  	new chipyard.config.WithSystemBusWidth(128) ++
  	new chipyard.config.AbstractConfig)







// package chipyard.config
// import org.chipsalliance.cde.config.Config
// import chipyard._              // 메인 패키지 참조

// ------------------------------
// Configs with Gemmini RoCC
// ------------------------------
// DOC include start: GemminiRocketConfig
// class GemminiRocketConfig extends Config(
//   new f_vela_gemmini.DefaultGemminiConfig ++                            // use Gemmini systolic array GEMM accelerator
//   new freechips.rocketchip.rocket.WithNHugeCores(1) ++
//   new chipyard.config.WithSystemBusWidth(128) ++
//   new chipyard.config.AbstractConfig)
// DOC include end: GemminiRocketConfig

