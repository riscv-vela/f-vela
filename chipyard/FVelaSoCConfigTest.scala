package chipyard
import org.chipsalliance.cde.config.Config

import freechips.rocketchip.rocket._
import f_vela_saturn.common.{VectorParams}
import f_vela_gemmini._ 

class FVelaSoCConfigTest extends Config(
    new f_vela_saturn.rocket.WithRocketVectorUnit(256, 256, VectorParams.refParams) ++
    new f_vela_gemmini.GemminiCustomConfig() ++ 
    new freechips.rocketchip.rocket.WithNHugeCores(1) ++
    new chipyard.config.WithSystemBusWidth(128) ++
	new chipyard.config.AbstractConfig
)

