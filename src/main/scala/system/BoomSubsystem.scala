//******************************************************************************
// Copyright (c) 2018 - 2019, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------
// Author: Christopher Celio
//------------------------------------------------------------------------------

package boom.system

import chisel3._
import chisel3.internal.sourceinfo.{SourceInfo}

import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.devices.debug.{HasPeripheryDebug, HasPeripheryDebugModuleImp}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.diplomaticobjectmodel.model.{OMComponent, OMInterruptTarget}
import freechips.rocketchip.tile._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.util._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.amba.axi4._
import lvna.TokenBucketNode
import boom.common._
import ila.{BoomCSRILABundle, FPGATraceBaseBundle}

case object BoomTilesKey extends Field[Seq[boom.common.BoomTileParams]](Nil)

trait HasBoomTiles extends HasTiles
  with CanHavePeripheryPLIC
  with CanHavePeripheryCLINT
  with HasPeripheryDebug
{ this: BaseSubsystem =>

  val module: HasBoomTilesModuleImp

  protected val boomTileParams = p(BoomTilesKey)
  private val crossings = perTileOrGlobalSetting(p(RocketCrossingKey), boomTileParams.size)

  // Make a tile and wire its nodes into the system,
  //   according to the specified type of clock crossing.
  //   Note that we also inject new nodes into the tile itself,
  //   also based on the crossing type.
  val tokenBuckets = Seq.fill(p(NBoomTiles)){ LazyModule(new TokenBucketNode()) }
  println(s"n boom tiles: ${p(NBoomTiles)}")
  val boomTiles = boomTileParams.zip(crossings).zip(tokenBuckets).map { case ((tp, crossing), tokenBucket) =>
    val boomCore = LazyModule(
      new boom.common.BoomTile(tp, crossing.crossingType)(augmentedTileParameters(tp))).suggestName(tp.name)

//    println("####Connecting interupts for boom core")
    connectMasterPortsToSBus(boomCore, crossing, tokenBucket)
    connectSlavePortsToCBus(boomCore, crossing)
    connectInterrupts(boomCore, Some(debug), clintOpt, plicOpt)

    boomCore
  }

  def coreMonitorBundles = (boomTiles map { t =>
    t.module.core.coreMonitorBundle
  }).toList

  def getOMRocketInterruptTargets(): Seq[OMInterruptTarget] =
    boomTiles.flatMap(c => c.cpuDevice.getInterruptTargets())

  def getOMRocketCores(resourceBindingsMap: ResourceBindingsMap): Seq[OMComponent] =
    boomTiles.flatMap(c => c.cpuDevice.getOMComponents(resourceBindingsMap))
}

trait HasBoomTilesModuleImp extends HasTilesModuleImp
  with HasPeripheryDebugModuleImp
{
  val outer: HasBoomTiles

  outer.boomTiles.zip(outer.debug.module.io.zid).foreach {
    case (tile, zid) =>
      tile.module.dmiDebugInterruptIO <> zid
  }

  outer.boomTiles.zipWithIndex.foreach{ case(tile, i) =>
    println(s"i=$i, tile=$tile")
    when (outer.debug.module.io.zid(i).dmiInterrupt) {
      dprintf(DEBUG_ETHER, "debug.m.io.zid.dit(%d) asserted\n", i.U)
    }
  }
}

class BoomSubsystem(implicit p: Parameters) extends BaseSubsystem
  with HasBoomTiles
{
  val tiles = boomTiles
  override lazy val module = new BoomSubsystemModule(this)
}

class BoomSubsystemModule[+L <: BoomSubsystem](_outer: L) extends BaseSubsystemModuleImp(_outer)
  with HasBoomTilesModuleImp
{
  tile_inputs.zip(outer.hartIdList).foreach { case(wire, i) =>
    wire.clock := clock
    wire.reset := reset
    wire.hartid := i.U
    wire.reset_vector := global_reset_vector
  }

  // head: single core only
  val ila = IO(new BoomCSRILABundle())
  ila := _outer.tiles.head.module.ila

  val fpga_trace = IO(new FPGATraceBaseBundle(_outer.tiles.head.module.core.retireWidth))
  fpga_trace := _outer.tiles.head.module.fpga_trace
}

///// Adds a port to the system intended to master an AXI4 DRAM controller that supports a large physical address size

trait CanHaveMisalignedMasterAXI4MemPort
{ this: BaseSubsystem =>

  val module: CanHaveMisalignedMasterAXI4MemPortModuleImp

  val memAXI4Node = p(ExtMem).map { case MemoryPortParams(memPortParams, nMemoryChannels) =>
    val portName = "misaligned_axi4"
    val device = new MemoryDevice

    val memAXI4Node = AXI4SlaveNode(Seq.tabulate(nMemoryChannels) { channel =>
      AXI4SlavePortParameters(
        slaves = Seq(AXI4SlaveParameters(
          address       = AddressSet.misaligned(memPortParams.base, memPortParams.size),
          resources     = device.reg,
          regionType    = RegionType.UNCACHED, // cacheable
          executable    = true,
          supportsWrite = TransferSizes(1, mbus.blockBytes),
          supportsRead  = TransferSizes(1, mbus.blockBytes),
          interleavedId = Some(0))), // slave does not interleave read responses
        beatBytes = memPortParams.beatBytes)
    })

    memAXI4Node := mbus.toDRAMController(Some(portName)) {
      AXI4UserYanker() := AXI4IdIndexer(memPortParams.idBits) := TLToAXI4()
    }

    memAXI4Node
  }
}

/** Actually generates the corresponding IO in the concrete Module */
trait CanHaveMisalignedMasterAXI4MemPortModuleImp extends LazyModuleImp
{
  val outer: CanHaveMisalignedMasterAXI4MemPort

  val mem_axi4 = outer.memAXI4Node.map(x => IO(HeterogeneousBag.fromNode(x.in)))
  (mem_axi4 zip outer.memAXI4Node) foreach { case (io, node) =>
    (io zip node.in).foreach { case (io, (bundle, _)) => io <> bundle }
  }

  def connectSimAXIMem() {
    (mem_axi4 zip outer.memAXI4Node).foreach { case (io, node) =>
      (io zip node.in).foreach { case (io, (_, edge)) =>
        // setting the max size for simulated memory to be 256MB
        val mem = LazyModule(new SimAXIMem(edge, size = 0x10000000))
        Module(mem.module).io.axi4.head <> io
      }
    }
  }
}
