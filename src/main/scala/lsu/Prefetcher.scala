package boom.src.main.scala.lsu

import boom.common.{BoomBundle, BoomModule}
import chisel3._
import chisel3.util.DecoupledIO
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.constants._

trait PrefetcherConstants {
  val TemporalityBits = 6
  val ConfidenceBits = 6
  val ConfidenceThreshold = 32

  val AddrDeltaWidth = 8
  val PCbits = 48
}

class ControlFlowInfo(implicit p: Parameters) extends BoomBundle()(p)
  with ControlOpConstants
  with PrefetcherConstants
{
  val inst_addr = UInt(PCbits.W)
  val last_callsit_addr = UInt(PCbits.W)
  val target_addr = UInt(PCbits.W)
  val typ = Bits(CT_SZ.W)
}

class DataflowInfo(implicit p: Parameters) extends BoomBundle()(p)
  with MemoryOpConstants
  with ScalarOpConstants
  with PrefetcherConstants
{
  val addr = UInt(PCbits.W)
  val cmd  = Bits(M_SZ.W)
  val typ  = Bits(MT_SZ.W)
}

class PrefetcherCacheSide(implicit p: Parameters) extends BoomBundle()(p)
  with MemoryOpConstants
  with ScalarOpConstants
  with PrefetcherConstants
{
  val addr = UInt(PCbits.W)
  val cmd  = Bits(M_SZ.W)
}

class CommonSubPrefetcherOut(implicit p: Parameters) extends BoomBundle()(p)
  with PrefetcherConstants
{
  val addr = UInt(PCbits.W)
  val temporality = UInt(TemporalityBits.W)
  val confidence = UInt(ConfidenceBits.W)
}

class Prefetcher (implicit p: Parameters) extends BoomModule()(p)
  with MemoryOpConstants
  with PrefetcherConstants
{
  val io = IO(new Bundle() {
    val cf = Flipped(new DecoupledIO(new ControlFlowInfo()))
    val df = Flipped(new DecoupledIO(new DataflowInfo()))
    val l1d = new DecoupledIO(new PrefetcherCacheSide())
    val l2 = new DecoupledIO(new PrefetcherCacheSide())
    val s2_miss = Input(Bool())
  })

  io.l1d.valid := false.B
  io.l1d.bits.cmd := M_PFR

  io.l2.valid := false.B
  io.l2.bits.cmd := M_PFR

  val T2 = new StreamPrefetcher()
  T2.io.df := io.df
  T2.io.cf := io.cf
  T2.io.s2_miss := io.s2_miss

  when (T2.io.pred.valid) {
    when (T2.io.pred.bits.confidence > ConfidenceThreshold.U) {
      io.l1d.valid := true.B
      io.l1d.bits.addr := T2.io.pred.bits.addr
    }.otherwise {
      io.l2.valid := true.B
      io.l2.bits.addr := T2.io.pred.bits.addr
    }
  }
}
