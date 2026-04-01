package chipyard
import org.chipsalliance.cde.config.Config

import freechips.rocketchip.rocket._
import f_vela_saturn.common.{VectorParams}

class REFV256D256RocketConfig extends Config(
	new f_vela_saturn.rocket.WithRocketVectorUnit(256, 256, VectorParams.refParams) ++
	new chipyard.config.WithSystemBusWidth(256) ++
	new freechips.rocketchip.rocket.WithNHugeCores(1) ++
	new chipyard.config.AbstractConfig)