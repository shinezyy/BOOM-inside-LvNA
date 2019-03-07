//******************************************************************************
// Copyright (c) 2018 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------
// Author: Christopher Celio
//------------------------------------------------------------------------------

package boom.lsu

import scala.collection.mutable.ListBuffer

import chisel3._

import freechips.rocketchip.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.diplomacy.{AddressSet, LazyModule}
import freechips.rocketchip.rocket.{DCache, HellaCache, HellaCacheArbiter, HellaCacheIO, NonBlockingDCache, PTW}
import freechips.rocketchip.subsystem.RocketCrossingKey
import freechips.rocketchip.tile.{BaseTile, HasTileParameters}
import freechips.rocketchip.tilelink.TLIdentityNode

/**
 * Top level mixin to construct a tile with a BOOM HellaCache.
 */
trait HasBoomHellaCache { this: BaseTile =>
  val module: HasBoomHellaCacheModule
  implicit val p: Parameters
  def findScratchpadFromICache: Option[AddressSet]
  var nDCachePorts = 0
  val dcache: HellaCache = LazyModule(
    if (tileParams.dcache.get.nMSHRs == 0)
    {
      new DCache(hartId, findScratchpadFromICache _, p(RocketCrossingKey).head.knownRatio)

    }
    else
    {
      new BoomNonBlockingDCache(hartId)
    })

  println(s"Boom nMSHRs: ${tileParams.dcache.get.nMSHRs}")

  //tlMasterXbar.node := dcache.node
  val dCacheTap = TLIdentityNode()
  tlMasterXbar.node := dCacheTap := dcache.node
}

/**
 * Mixin to construct a tile with a BOOM HellaCache.
 */
trait HasBoomHellaCacheModule
{
  val outer: HasBoomHellaCache
  val dcachePorts = ListBuffer[HellaCacheIO]()
  val dcacheArb = Module(new HellaCacheArbiter(outer.nDCachePorts)(outer.p))
  outer.dcache.module.io.cpu <> dcacheArb.io.mem
}

/**
 * Top level mixin to construct a tile with a BOOM PTW.
 */
trait CanHaveBoomPTW extends HasTileParameters with HasBoomHellaCache { this: BaseTile =>
  val module: CanHaveBoomPTWModule
  var nPTWPorts = 1
  println(s"2 nDCachePorts before: $nDCachePorts")
  nDCachePorts += (if (usingPTW) 1 else 0)
  println(s"2 nDCachePorts after: $nDCachePorts")
}

/**
 * Mixin to construct a tile with a BOOM PTW.
 */
trait CanHaveBoomPTWModule extends HasBoomHellaCacheModule
{
  val outer: CanHaveBoomPTW
  val ptwPorts = ListBuffer(outer.dcache.module.io.ptw)
  val ptw: Option[PTW] = (outer.usingPTW).option(Module(new PTW(outer.nPTWPorts)(outer.dcache.node.edges.out(0), outer.p)))
  if (outer.usingPTW)
  {
    dcachePorts += ptw.get.io.mem
  }
}

/**
 * Bundle to monitor cache data writebacks/releases for memory ordering.
 */
class ReleaseInfo(implicit p: Parameters) extends boom.common.BoomBundle()(p)
{
   val address = UInt(corePAddrBits.W)
}
