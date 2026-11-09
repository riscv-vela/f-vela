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