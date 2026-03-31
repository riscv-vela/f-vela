package chipyard
import org.chipsalliance.cde.config.Config

import freechips.rocketchip.rocket._
import f_vela_saturn.common.{VectorParams}
import f_vela_gemmini._ 

class REFV256D128RocketConfig extends Config(
	new f_vela_saturn.rocket.WithRocketVectorUnit(256, 128, VectorParams.refParams) ++
	new chipyard.config.WithSystemBusWidth(128) ++
	new freechips.rocketchip.rocket.WithNHugeCores(1) ++
	new chipyard.config.AbstractConfig)