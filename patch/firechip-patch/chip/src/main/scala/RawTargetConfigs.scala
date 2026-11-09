package firechip.chip

import org.chipsalliance.cde.config.Config

/**
 * FireSim wrapper for the stock/upstream Gemmini + Saturn target.
 *
 * This Config intentionally does not depend on generators/_f_vela and must
 * remain compilable while the _f_vela SBT project is temporarily disabled.
 */
class FireSimNoCustomFvelaTest extends Config(
  new freechips.rocketchip.subsystem.WithExtMemSize(8L << 30) ++
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new chipyard.NoCustomFvelaTest
)

class FireSimOnlyRocketCoreTest extends Config(
  new freechips.rocketchip.subsystem.WithExtMemSize(8L << 30) ++
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new chipyard.OnlyRocketCoreTest
)
// Upstream Gemmini + Rocket, no vector unit (see chipyard.NoCustomGemminiOnlyTest)
class FireSimNoCustomGemminiOnlyTest extends Config(
  new freechips.rocketchip.subsystem.WithExtMemSize(8L << 30) ++
  new WithDefaultFireSimBridges ++
  new WithFireSimConfigTweaks ++
  new chipyard.NoCustomGemminiOnlyTest
)
