package chipyard
import org.chipsalliance.cde.config.Config

import freechips.rocketchip.rocket._
import f_vela_saturn.common.{VectorParams}
import f_vela_gemmini._ 

class FVelaSoCConfigTest extends Config(
    new f_vela_saturn.rocket.WithRocketVectorUnit(512, 256, VectorParams.refParams) ++ //vlen minimum 512, dlen minimum 256 (PID needs egsPerVReg=2 -> vLen=2*dLen)
    new f_vela_gemmini.GemminiCustomConfig() ++ 
    new freechips.rocketchip.rocket.WithNHugeCores(1) ++
    // new shuttle.common.WithNShuttleCores(1) ++
    new chipyard.config.WithSystemBusWidth(128) ++
	new chipyard.config.AbstractConfig
)

class FVelaGemminiOnlyConfig extends Config(
    new f_vela_gemmini.GemminiCustomConfig() ++
    new freechips.rocketchip.rocket.WithNHugeCores(1) ++
    new chipyard.config.WithSystemBusWidth(128) ++
    new chipyard.config.AbstractConfig
)

class FVelaSaturnOnlyConfig extends Config(
    new f_vela_saturn.rocket.WithRocketVectorUnit(
        512,
        256,
        VectorParams.refParams
    ) ++
    new freechips.rocketchip.rocket.WithNHugeCores(1) ++
    new chipyard.config.WithSystemBusWidth(128) ++
    new chipyard.config.AbstractConfig
)

class FVelaSaturnMinConfig extends Config(
    new f_vela_saturn.rocket.WithRocketVectorUnit(
        512,
        128,
        VectorParams.minParams
    ) ++
    new freechips.rocketchip.rocket.WithNHugeCores(1) ++
    new chipyard.config.WithSystemBusWidth(128) ++
    new chipyard.config.AbstractConfig
)


// legacy verilator version 
// class SaturnGemminiConfig extends Config(
//     new gemmini.DefaultGemminiConfig ++                            // use Gemmini systolic array GEMM accelerator
//     new saturn.rocket.WithRocketVectorUnit(256, 128, VectorParams.refParams) ++
//     new freechips.rocketchip.rocket.WithNHugeCores(1) ++
//     new chipyard.config.WithSystemBusWidth(128) ++
//     new chipyard.config.AbstractConfig)

// firesim version
// class FireSimSaturnNPU16interruptConfig extends Config(
//     new freechips.rocketchip.subsystem.WithExtMemSize((1L << 30) * 8) ++
//     new WithDefaultFireSimBridges ++
//     new WithFireSimConfigTweaks ++
//     new saturn.rocket.WithRocketVectorUnit(256, 128, VectorParams.refParams) ++
//     new chipyard.GemminiRocketConfig  
// )