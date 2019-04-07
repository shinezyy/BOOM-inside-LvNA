package boom.lsu.pref

import boom.common.{BoomBundle, BoomModule}
import boom.lsu.{HasBoomHellaCache, HasBoomHellaCacheModule}
import chisel3._
import chisel3.util.{DecoupledIO, Valid}
import freechips.rocketchip.config.{Config, Field, Parameters}
import freechips.rocketchip.rocket.HellaCacheIO
import freechips.rocketchip.rocket.constants._
import freechips.rocketchip.tile.{BaseTile, HasTileParameters}

case object UsePrefetcher extends Field[Boolean](false)

class WithPrefetcher extends Config ((site, here, up) => {
  case UsePrefetcher => true
})


trait PrefetcherConstants {
  val TemporalityBits = 6
  val ConfidenceBits = 6
  val ConfidenceThreshold = 32

  val AddrDeltaWidth = 8
  val Addrbits = 48

  val LoopCounterMax = 16
  val MinLoopSize = 4
  val NLPCTSize = 16
}

class ControlFlowInfo(implicit p: Parameters) extends BoomBundle()(p)
  with ControlOpConstants
  with PrefetcherConstants
{
  val inst_addr = UInt(Addrbits.W)
  val last_callsite_addr = UInt(Addrbits.W)
  val target_addr = UInt(Addrbits.W)
  val typ = Bits(CT_SZ.W)
}

class DataflowInfo(implicit p: Parameters) extends BoomBundle()(p)
  with MemoryOpConstants
  with ScalarOpConstants
  with PrefetcherConstants
{
  val pc   = UInt(Addrbits.W)
  val addr = UInt(Addrbits.W)
  val cmd  = Bits(M_SZ.W)
  val typ  = Bits(MT_SZ.W)
}

class PrefetcherCacheSide(implicit p: Parameters) extends BoomBundle()(p)
  with MemoryOpConstants
  with ScalarOpConstants
  with PrefetcherConstants
{
  val addr = UInt(Addrbits.W)
  val cmd  = Bits(M_SZ.W)
}

class CommonSubPrefetcherOut(implicit p: Parameters) extends BoomBundle()(p)
  with PrefetcherConstants
{
  val addr = UInt(Addrbits.W)
  val temporality = UInt(TemporalityBits.W)
  val confidence = UInt(ConfidenceBits.W)
}

class TPCPrefetcher(implicit p: Parameters) extends BoomModule()(p)
  with MemoryOpConstants
  with PrefetcherConstants
  with ScalarOpConstants
{
  val io = IO(new Bundle() {
    val cf = Flipped(new Valid(new ControlFlowInfo()))
    val df = Flipped(new Valid(new DataflowInfo()))
    val l1d = new HellaCacheIO()
    val l2 = new HellaCacheIO()
  })

  val loopPred = Module(new LoopPred())
  loopPred.io.cf := io.cf

  def set_default_hello_req(dc_port: HellaCacheIO): Unit = {
    dc_port.req.valid := false.B
    dc_port.req.bits.typ := MT_W  // usually prefetch a word?
    dc_port.req.bits.addr := 0.U  // set by sub prefetcher
    dc_port.req.bits.tag := 0.U  // prefetch ignore tag (tag is used for keep resp order?)
    dc_port.req.bits.cmd := M_PFR  // prefetcher read
    dc_port.req.bits.phys := false.B  // currently only vaddr is used

    dc_port.s1_data.data := 0.U   // prefetcher writes nothing
    dc_port.s1_data.mask := 0.U

    dc_port.s1_kill := false.B  // prefetch kills nothing
    dc_port.s2_kill := false.B
  }

  set_default_hello_req(io.l1d)
  set_default_hello_req(io.l2)

  val T2 = Module(new StreamPrefetcher())
  T2.io.df := io.df
  T2.io.cf := io.cf
  T2.io.s2_miss := io.l1d.s2_primary_miss
  T2.io.isInLoop := loopPred.io.loop_info.inLoop


  when (T2.io.pred.valid) {
    when (T2.io.pred.bits.confidence > ConfidenceThreshold.U) {
      io.l1d.req.valid := true.B
      io.l1d.req.bits.addr := T2.io.pred.bits.addr
    }.otherwise {
      io.l2.req.valid := true.B
      io.l2.req.bits.addr := T2.io.pred.bits.addr
    }
  }
}

