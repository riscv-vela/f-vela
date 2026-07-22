package firechip.chip

import org.chipsalliance.cde.config.Config

class FireSimFVelaSoCConfig extends Config(
    new freechips.rocketchip.subsystem.WithExtMemSize(8L << 30) ++
    new WithDefaultFireSimBridges ++
    new WithFireSimConfigTweaks ++
    new chipyard.FVelaSoCConfigTest
)

class FireSimFVelaSaturnOnlyConfig extends Config(
    new freechips.rocketchip.subsystem.WithExtMemSize(8L << 30) ++
    new WithDefaultFireSimBridges ++
    new WithFireSimConfigTweaks ++
    new chipyard.FVelaSaturnOnlyConfig
)

class FireSimFVelaGemminiOnlyConfig extends Config(
    new freechips.rocketchip.subsystem.WithExtMemSize(8L << 30) ++
    new WithDefaultFireSimBridges ++
    new WithFireSimConfigTweaks ++
    new chipyard.FVelaGemminiOnlyConfig
)

class FireSimFVelaSaturnMinConfig extends Config(
    new freechips.rocketchip.subsystem.WithExtMemSize(8L << 30) ++
    new WithDefaultFireSimBridges ++
    new WithFireSimConfigTweaks ++
    new chipyard.FVelaSaturnMinConfig
)
// class FireSimFVelaSoCConfigTest extends Config(
//   new freechips.rocketchip.subsystem.WithExtMemSize(8L << 30) ++
//   new WithDefaultFireSimBridges ++
//   new WithFireSimConfigTweaks ++
//   new f_vela_saturn.rocket.WithRocketVectorUnit(
//     512,
//     256,
//     VectorParams.refParams
//   ) ++
//   new f_vela_gemmini.GemminiCustomConfig() ++
//   new freechips.rocketchip.rocket.WithNHugeCores(1) ++
//   new chipyard.config.WithSystemBusWidth(128) ++
//   new chipyard.config.AbstractConfig
// )