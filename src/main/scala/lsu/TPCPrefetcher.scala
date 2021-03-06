package boom.lsu.pref

import boom.common._
import chisel3._
import chisel3.util.{DecoupledIO, Queue, Valid}
import freechips.rocketchip.config.{Config, Field, Parameters}
import freechips.rocketchip.rocket.HellaCacheIO
import freechips.rocketchip.rocket.constants._
import freechips.rocketchip.tile.{BaseTile, HasTileParameters}
import freechips.rocketchip.util.GTimer

case object UsePrefetcher extends Field[Boolean](false)

class WithPrefetcher extends Config ((site, here, up) => {
  case UsePrefetcher => true
})


trait PrefetcherConstants {
  val TemporalityBits = 6
  val ConfidenceBits = 6
  val ConfidenceThreshold = 32

  val AddrDeltaWidth = 16
  val Addrbits = 40

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
  with MemoryOpConstants
  with PrefetcherConstants
{
  val addr = UInt(Addrbits.W)
  val temporality = UInt(TemporalityBits.W)
  val confidence = UInt(ConfidenceBits.W)
  val cmd  = Bits(M_SZ.W)
}

class TPCPrefetcher(implicit p: Parameters) extends BoomModule()(p)
  with MemoryOpConstants
  with PrefetcherConstants
  with ScalarOpConstants
{
  val io = IO(new Bundle() {
    val cf = Flipped(Valid(new ControlFlowInfo()))
    val df = Flipped(Valid(new DataflowInfo()))
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
//    dc_port.req.bits.cmd := M_PFR  // prefetcher read
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

  val T2_wrapper = Module(new PrefReqMaintainer())
  T2_wrapper.io.pred_in := T2.io.pred
  T2_wrapper.io.pred_out.ready := io.l1d.req.ready
  T2_wrapper.io.s2_rejected := io.l1d.s2_nack

  when (T2_wrapper.io.pred_out.valid) {
    io.l1d.req.valid := true.B
    io.l1d.req.bits.addr := T2_wrapper.io.pred_out.bits.addr
    io.l1d.req.bits.cmd := T2_wrapper.io.pred_out.bits.cmd
  }
}

class PrefReqMaintainer(implicit p:Parameters) extends BoomModule()(p)
  with PrefetcherConstants
  with MemoryOpConstants
{

  class InternalReq (implicit p: Parameters) extends BoomBundle ()(p)
    with MemoryOpConstants
  {
    val persistent_req = Bool()
    val temporality = UInt(TemporalityBits.W)
    val addr = UInt(Addrbits.W)
    val cmd = UInt(M_SZ.W)
  }
  val io = IO(new Bundle(){
    val pred_in = Flipped(Valid(new CommonSubPrefetcherOut()))
    val pred_out = DecoupledIO(new InternalReq())
    val s2_rejected = Input(Bool())
  })


  val req_fired_two_cycle_ago = RegNext(RegNext(io.pred_out.fire()))

  val reqQueue = Module(new Queue(new InternalReq(), entries = 4))
  reqQueue.io.deq.ready := req_fired_two_cycle_ago && !io.s2_rejected

  when(req_fired_two_cycle_ago ) {
    when (!io.s2_rejected) {
      dprintf(D_T2_2,
        "[%d] deq because of pref issued to addr 0x%x\n",
        GTimer(), reqQueue.io.deq.bits.addr)
    }.otherwise{
      dprintf(D_T2_2,
        "[%d] req 2 cycles ago to 0x%x rejected\n",
        GTimer(), reqQueue.io.deq.bits.addr)
    }
  }

  io.pred_out.valid := false.B
  when (reqQueue.io.deq.valid) {
    io.pred_out.valid := true.B
    io.pred_out.bits := reqQueue.io.deq.bits
    dprintf(D_T2_2, "[%d] try to send req to addr 0x%x\n",
      GTimer(), reqQueue.io.deq.bits.addr)
  }

  reqQueue.io.enq.valid := false.B
  reqQueue.io.enq.bits := DontCare
  // new req override old one
  when (io.pred_in.valid) {
    reqQueue.io.enq.valid := true.B
    dprintf(D_T2_2, "[%d] received req from sub pref, ", GTimer())

    when (io.pred_in.bits.temporality === 0.U) {
      dprintf(D_T2_2, "persistent req, ")
      reqQueue.io.enq.bits.persistent_req := true.B

    }.otherwise{
      dprintf(D_T2_2, "temporal req, ")
      reqQueue.io.enq.bits.persistent_req := false.B
      reqQueue.io.enq.bits.temporality := io.pred_in.bits.temporality
    }

    when (io.pred_in.bits.cmd === M_PFR) {
      dprintf(D_T2_3, "M_PFR, ")
    }.elsewhen(io.pred_in.bits.cmd === M_PFW) {
      dprintf(D_T2_3, "M_PFW, ")
    }.otherwise {
      dprintf(D_T2_3, "Unknown cmd!! ")
    }

    dprintf(D_T2_2, "to address 0x%x\n", io.pred_in.bits.addr)
    reqQueue.io.enq.bits.addr := io.pred_in.bits.addr
    reqQueue.io.enq.bits.cmd := io.pred_in.bits.cmd

    when (reqQueue.io.enq.ready) {
      dprintf(D_T2_2, "enqueued!\n")
    }.otherwise{
      dprintf(D_T2_2, "dropped because of full!\n")
    }
  }
}

