package boom.lsu.pref

import boom.common.{BoomBundle, BoomModule}
import chisel3._
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.constants._

trait T2StateConstants {
  val TS_SZ   = 3
  def TS_X    = BitPat("b???")
  def TS_UNK  = 0.U(3.W)
  def TS_OB   = 1.U(3.W)
  def TS_STRD = 2.U(3.W)
  def TS_N    = 3.U(3.W)
  def TS_INV  = 4.U(3.W)
}

trait T2Parameters {
  val SIT_SIZE = 32
  val StrideCounterWidth = 6
  val StrideCounterThreshold = 4
}

class SITEntry  // Stride identifier table
(implicit p: Parameters) extends BoomBundle()(p)
  with PrefetcherConstants
  with T2StateConstants
  with T2Parameters
{
  val lastAddr      = UInt(coreMaxAddrBits.W)
  val lastPrefAddr  = UInt(coreMaxAddrBits.W)
  val delta         = UInt(AddrDeltaWidth.W)
  val state         = UInt(TS_SZ.W)
  val counter       = UInt(StrideCounterWidth.W)
}

class MPCEntry (implicit p: Parameters) extends BoomModule()(p)
  with T2Parameters
  with PrefetcherConstants
{
  val io = IO(new Bundle{
    val write = Input(Bool())
    val r_addr = Input(UInt(Addrbits.W))
    val w_addr = Input(UInt(Addrbits.W))
    val eq = Output(Bool())
  })
  val valid = RegInit(false.B)

  val cell = RegInit(0.U(Addrbits.W))
  io.eq := valid && (io.r_addr === cell)
  when (io.write) {
    cell := io.w_addr
    valid := true.B
  }
}

class ExpandedSITSignals (implicit p: Parameters) extends BoomBundle()(p)
  with T2Parameters
{
  val hit = Bool()
  val miss = Bool()
  val sit_ptr = UInt(log2Ceil(SIT_SIZE).W)
  val addr = UInt(log2Ceil(coreMaxAddrBits).W)
}

class StreamPrefetcher (implicit p: Parameters) extends BoomModule()(p)
  with T2Parameters
  with T2StateConstants
  with PrefetcherConstants
{
  val io = IO(new Bundle() {
    val isInLoop = Input(Bool())
    val pred = new Valid(new CommonSubPrefetcherOut())
    val cf = Flipped(new Valid(new ControlFlowInfo()))
    val df = Flipped(new Valid(new DataflowInfo()))
    val s2_miss = Input(Bool())
  })

  private val mPC = RegInit(0.U(Addrbits.W))

  val data_array = SyncReadMem(SIT_SIZE, new SITEntry())

  val mPCEntries = for (i <- 0 until SIT_SIZE) yield {val slot = Module(new MPCEntry()); slot}
  val mPCEntryPorts = VecInit(mPCEntries.map(_.io))

  mPCEntryPorts.foreach{e => e.r_addr := io.df.bits.pc}

  val s1_hit_wire = io.df.valid && mPCEntryPorts.map(_.eq).reduce(_|_) // any match
  val s1_hit_ptr_w = OHToUInt(mPCEntryPorts.map(_.eq))
  val s1_hit_ptr = RegNext(s1_hit_ptr_w)
  val s1_miss_wire = io.df.valid && !s1_hit_wire

  val s1_info = Reg(new Valid(new ExpandedSITSignals()))

  s1_info.valid := io.df.valid
  s1_info.bits.hit := s1_hit_wire
  s1_info.bits.miss := s1_miss_wire
  s1_info.bits.addr := io.df.bits.addr

  val s1_read_sit_entry = Wire(new SITEntry())
  s1_read_sit_entry := data_array.read(s1_hit_ptr_w, s1_hit_wire)

  // s2
  val (sit_alloc_ptr, sit_ptr_wrap) = Counter(s1_info.bits.miss, SIT_SIZE)

  val sit_w_ptr_oh = UIntToOH(sit_alloc_ptr)

  mPCEntryPorts.zipWithIndex.foreach{case(e, i) => e.write := s1_info.bits.miss && sit_w_ptr_oh(i)}

  mPCEntryPorts.foreach{e => e.w_addr := s1_info.bits.addr}

  val s2_info_w = WireInit(s1_info)
  s2_info_w.bits.sit_ptr := Mux(s1_info.bits.miss, sit_alloc_ptr, s1_hit_ptr)
  val s2_info = RegNext(s2_info_w)

  val s2_read_sit_entry = RegNext(s1_read_sit_entry)

  // s3
  val s3_wb_entry_w = Wire(new SITEntry())
  s3_wb_entry_w.state := TS_UNK
  s3_wb_entry_w.lastAddr := s2_info.bits.addr
  s3_wb_entry_w.lastPrefAddr := 0.U
  s3_wb_entry_w.delta := 0.U
  s3_wb_entry_w.counter := 1.U

  val new_delta = s2_info.bits.addr - s2_read_sit_entry.lastAddr

  when (s2_info.valid && s2_info.bits.hit) {
    // keep most states by default
    s3_wb_entry_w := s2_read_sit_entry

    when(s2_read_sit_entry.state === TS_UNK && io.s2_miss) {
      s3_wb_entry_w.state := TS_OB
    }

    when(new_delta === s2_read_sit_entry.delta) {
      when(s2_read_sit_entry.state === TS_OB || s2_read_sit_entry.state === TS_N) {
        s3_wb_entry_w.state := TS_STRD
        s3_wb_entry_w.counter := 1.U
      }.elsewhen (s2_read_sit_entry.state === TS_STRD &&
        s2_read_sit_entry.counter =/= ((1 << StrideCounterWidth) - 1).U(StrideCounterWidth.W)) { // saturating counter
        s3_wb_entry_w.counter := s2_read_sit_entry.counter + 1.U
      }
    }
  }
  val pref_addr = WireInit(0.U(Addrbits.W))
  io.pred.valid := false.B
  when (s2_read_sit_entry.state === TS_STRD &&
    s3_wb_entry_w.counter > StrideCounterThreshold.U) {
    io.pred.valid := true.B
    io.pred.bits.addr := pref_addr
    io.pred.bits.confidence := s3_wb_entry_w.counter
    io.pred.bits.temporality := 0.U
    s3_wb_entry_w.lastPrefAddr := pref_addr
  }

  data_array.write(s2_info.bits.sit_ptr, s3_wb_entry_w)

}



