//******************************************************************************
// Copyright (c) 2017 - 2018, The Regents of the University of California (Regents).
// All Rights Reserved. See LICENSE and LICENSE.SiFive for license details.
//------------------------------------------------------------------------------
// Author: Christopher Celio
//------------------------------------------------------------------------------

package boom.common

import chisel3._

import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.devices.tilelink._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.interrupts._
import freechips.rocketchip.util._
import freechips.rocketchip.tile._
import boom.exu._
import boom.ifu._
import boom.lsu.{CanHaveBoomPTW, CanHaveBoomPTWModule, HasBoomHellaCache, HasBoomHellaCacheModule}
import boom.lsu.pref.{TPCPrefetcher, UsePrefetcher}
import freechips.rocketchip.debug.DebugCSRIntIO
import ila.{BoomCSRILABundle, FPGATraceBaseBundle}
import lvna.HasControlPlaneParameters


/**
 * BOOM tile parameter class used in configurations
 *
 * @param core BOOM core params
 * @param icache i$ params
 * @param dcache d$ params
 * @param btb btb params
 * @param dataScratchpadBytes ...
 * @param trace ...
 * @param hcfOnUncorrectable ...
 * @param name name of tile
 * @param hartId hardware thread id
 * @param blockerCtrlAddr ...
 * @param boundaryBuffers ...
 */
case class BoomTileParams(
    core: BoomCoreParams = BoomCoreParams(),
    icache: Option[ICacheParams] = Some(ICacheParams()),
    dcache: Option[DCacheParams] = Some(DCacheParams()),
    btb: Option[BTBParams] = Some(BTBParams()),
    dataScratchpadBytes: Int = 0,
    trace: Boolean = false,
    hcfOnUncorrectable: Boolean = false,
    name: Option[String] = Some("tile"),
    hartId: Int = 0,
    blockerCtrlAddr: Option[BigInt] = None,
    boundaryBuffers: Boolean = false // if synthesized with hierarchical PnR, cut feed-throughs?
    ) extends TileParams
{
  require(icache.isDefined)
  require(dcache.isDefined)
}

/**
 * BOOM tile
 *
 * @param boomParams BOOM tile params
 * @param crossing ...
 */
class BoomTile(
    val boomParams: BoomTileParams,
    crossing: ClockCrossingType)
  (implicit p: Parameters) extends BaseTile(boomParams, crossing)(p)
    with HasExternalInterrupts
    //with HasLazyRoCC  // implies CanHaveSharedFPU with CanHavePTW with HasHellaCache
    with CanHaveBoomPTW
    with HasBoomHellaCache
    with HasBoomICacheFrontend
{

  val intOutwardNode = IntIdentityNode()
  val slaveNode = TLIdentityNode()
  val masterNode = TLIdentityNode()

  val dtim_adapter = tileParams.dcache.flatMap { d => d.scratch.map(s =>
    LazyModule(new ScratchpadSlavePort(AddressSet(s, d.dataScratchpadBytes-1),
                                       xBytes,
                                       tileParams.core.useAtomics && !tileParams.core.useAtomicsOnlyForIO)))
  }
  dtim_adapter.foreach(lm => connectTLSlave(lm.node, xBytes))

  val bus_error_unit = tileParams.core.tileControlAddr map { a =>
    val beu = LazyModule(new BusErrorUnit(new L1BusErrors, BusErrorUnitParams(a)))
    intOutwardNode := beu.intNode
    connectTLSlave(beu.node, xBytes)
    beu
  }

  val tile_master_blocker =
    tileParams.blockerCtrlAddr
      .map(BasicBusBlockerParams(_, xBytes, masterPortBeatBytes, deadlock = true))
      .map(bp => LazyModule(new BasicBusBlocker(bp)))

  tile_master_blocker.foreach(lm => connectTLSlave(lm.controlNode, xBytes))

  // TODO: this doesn't block other masters, e.g. RoCCs
  tlOtherMastersNode := tile_master_blocker.map { _.node := tlMasterXbar.node } getOrElse { tlMasterXbar.node }
  masterNode :=* tlOtherMastersNode
  DisableMonitors { implicit p => tlSlaveXbar.node :*= slaveNode }

  def findScratchpadFromICache: Option[AddressSet] = dtim_adapter.map { s =>
    val finalNode = frontend.masterNode.edges.out.head.manager.managers.find(_.nodePath.last == s.node)
    require (finalNode.isDefined, "Could not find the scratch pad; not reachable via icache?")
    require (finalNode.get.address.size == 1, "Scratchpad address space was fragmented!")
    finalNode.get.address(0)
  }

  println(s"1 nDCachePorts before: $nDCachePorts")
  nDCachePorts += 1 /*core */ + (dtim_adapter.isDefined).toInt + p(UsePrefetcher).toInt
  println(s"1 nDCachePorts after: $nDCachePorts")

  val dtimProperty = dtim_adapter.map(d => Map(
    "sifive,dtim" -> d.device.asProperty)).getOrElse(Nil)

  val itimProperty = tileParams.icache.flatMap(_.itimAddr.map(i => Map(
    "sifive,itim" -> frontend.icache.device.asProperty))).getOrElse(Nil)

  val cpuDevice = new SimpleDevice("cpu", Seq("ucb-bar,boom0", "riscv")) {
    override def parent = Some(ResourceAnchors.cpus)
    override def describe(resources: ResourceBindings): Description = {
      val Description(name, mapping) = super.describe(resources)
      Description(name, mapping ++
                        cpuProperties ++
                        nextLevelCacheProperty ++
                        tileProperties ++
                        dtimProperty ++
                        itimProperty)
    }
  }

  ResourceBinding {
    Resource(cpuDevice, "reg").bind(ResourceAddress(hartId))
  }

  override lazy val module = new BoomTileModuleImp(this)

  override def makeMasterBoundaryBuffers(implicit p: Parameters) = {
    if (!boomParams.boundaryBuffers) super.makeMasterBoundaryBuffers
    else TLBuffer(BufferParams.none, BufferParams.flow, BufferParams.none, BufferParams.flow, BufferParams(1))
  }

  override def makeSlaveBoundaryBuffers(implicit p: Parameters) = {
    if (!boomParams.boundaryBuffers) super.makeSlaveBoundaryBuffers
    else TLBuffer(BufferParams.flow, BufferParams.none, BufferParams.none, BufferParams.none, BufferParams.none)
  }
}

/**
 * BOOM tile implicit
 *
 * @param outer top level BOOM tile
 */
class BoomTileModuleImp(outer: BoomTile) extends BaseTileModuleImp(outer)
    //with HasLazyRoCCModule[BoomTile]
    with CanHaveBoomPTWModule
    with HasControlPlaneParameters
    with HasBoomHellaCacheModule
    with HasBoomICacheFrontendModule
{
  Annotated.params(this, outer.boomParams)

  val core = Module(new BoomCore()(outer.p, outer.dcache.module.edge))

  //val fpuOpt = outer.tileParams.core.fpu.map(params => Module(new FPU(params)(outer.p))) //RocketFpu - not needed

  // Observe the Tilelink Channel C traffic leaving the L1D (writeback/releases).
  val tl_c = outer.dCacheTap.out(0)._1.c
  core.io.release.valid := tl_c.fire()
  core.io.release.bits.address := tl_c.bits.address

  val uncorrectable = RegInit(false.B)
  val halt_and_catch_fire = outer.boomParams.hcfOnUncorrectable.option(IO(Output(Bool())))

  outer.dtim_adapter.foreach { lm => dcachePorts += lm.module.io.dmem }

  outer.bus_error_unit.foreach { lm =>
    lm.module.io.errors.dcache := outer.dcache.module.io.errors
    lm.module.io.errors.icache := outer.frontend.module.io.errors
  }

  val memBase = IO(Input(UInt(p(XLen).W)))
  val memMask = IO(Input(UInt(p(XLen).W)))

  outer.dcache.module.memBase := memBase
  outer.dcache.module.memMask := memMask
  outer.frontend.module.io.memBase := memBase
  outer.frontend.module.io.memMask := memMask

  val dsid = IO(Input(UInt(ldomDSidWidth.W)))
  val pc = IO(Output(UInt(vaddrBitsExtended.W)))
  pc := core.io.ifu.get_pc.fetch_pc

  val dmiDebugInterruptIO = IO(new DebugCSRIntIO())

  core.io.dmi_debug_interrupt_io <> dmiDebugInterruptIO
  when (dmiDebugInterruptIO.dmiInterrupt) {
    dprintf(DEBUG_ETHER, "tile.io.dmi_int asserted\n")
  }

  val ila = IO(new BoomCSRILABundle())
  ila := core.io.ila

  val fpga_trace = IO(new FPGATraceBaseBundle(core.retireWidth))
  fpga_trace := core.io.fpga_trace

  outer.decodeCoreInterrupts(core.io.interrupts) // Decode the interrupt vector
  outer.bus_error_unit.foreach { beu => core.io.interrupts.buserror.get := beu.module.io.interrupt }
  core.io.hartid := constants.hartid // Pass through the hartid
  trace.foreach { _ := core.io.trace }
  halt_and_catch_fire.foreach { _ := uncorrectable }
  outer.frontend.module.io.cpu <> core.io.ifu
  outer.frontend.module.io.reset_vector := constants.reset_vector
  outer.frontend.module.io.hartid := constants.hartid
  outer.dcache.module.io.hartid := constants.hartid
  dcachePorts += core.io.dmem // TODO outer.dcachePorts += () => module.core.io.dmem ??
  //fpuOpt foreach { fpu => core.io.fpu <> fpu.io } RocketFpu - not needed in boom
  core.io.ptw := DontCare
  if (usingPTW)
  {
    core.io.ptw <> ptw.get.io.dpath
  }
  core.io.rocc := DontCare
  core.io.fpu := DontCare
  core.io.reset_vector := DontCare

  //roccCore.cmd <> core.io.rocc.cmd
  //roccCore.exception := core.io.rocc.exception
  //core.io.rocc.resp <> roccCore.resp
  //core.io.rocc.busy := roccCore.busy
  //core.io.rocc.interrupt := roccCore.interrupt

  when(!uncorrectable)
  {
    uncorrectable :=
    List(outer.frontend.module.io.errors, outer.dcache.module.io.errors)
      .flatMap { e => e.uncorrectable.map(_.valid) }
      .reduceOption(_||_)
      .getOrElse(false.B)
  }

  // add prefetcher

  // NOTE: always put it last to indicate lowest priority
  if (p(UsePrefetcher)) {
    println("Adding prefetcher")
    val prefetcher = Module(new TPCPrefetcher()(outer.p))
    dcachePorts += prefetcher.io.l1d
    prefetcher.io.l2 := DontCare
    prefetcher.io.cf := core.io.cf
    prefetcher.io.df := core.io.df
  }

  // TODO eliminate this redundancy
  val h = dcachePorts.size
  val c = core.dcacheArbPorts
  val o = outer.nDCachePorts
  require(h == c, s"port list size was $h, core expected $c")
  require(h == o, s"port list size was $h, outer counted $o")

  // TODO figure out how to move the below into their respective mix-ins
  dcacheArb.io.requestor <> dcachePorts
  ptwPorts += core.io.ptw_tlb
  core.io.ptw_tlb := DontCare
  if (usingPTW)
  {
    ptw.get.io.requestor <> ptwPorts
  }
  val frontendStr = outer.frontend.module.toString
  ElaborationArtefacts.add(
    """core.config""",
    frontendStr + core.toString + "\n"
  )
  print(outer.frontend.module.toString + core.toString + "\n")
}
