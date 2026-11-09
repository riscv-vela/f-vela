package chipyard

import org.chipsalliance.cde.config.Config
import saturn.common.{VectorParams}

// Stock chipyard SoC combining the upstream (non-F-Vela) Gemmini RoCC accelerator
// with the upstream Saturn vector unit on a Rocket core. Nothing F-Vela-specific added.
class NoCustomFvelaTest extends Config(
  new gemmini.DefaultGemminiConfig ++                                  // Gemmini systolic-array RoCC accelerator
  new saturn.rocket.WithRocketVectorUnit(512, 256, VectorParams.refParams) ++  // Saturn vector unit on the Rocket core
  new chipyard.config.WithSystemBusWidth(128) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

class OnlyRocketCoreTest extends Config(
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)
  
// Upstream (unmodified) Gemmini on a Rocket core, no vector unit: the stock-Gemmini counterpart of
// FVelaGemminiOnlyConfig. Without a vector unit none of the Rocket <-> vector glue is generated, so
// it builds with the current (F-Vela-patched) rocket-chip and does not depend on Saturn.
class NoCustomGemminiOnlyTest extends Config(
  new gemmini.DefaultGemminiConfig ++                                  // Gemmini systolic-array RoCC accelerator
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)
