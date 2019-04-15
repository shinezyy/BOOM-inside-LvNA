package boom.lsu.pref

import boom.common._
import boom.common.util.{RoundRobinCAM, SaturatingCounter}
import chisel3._
import chisel3.internal.naming.chiselName
import chisel3.util._
import freechips.rocketchip.config.Parameters
import freechips.rocketchip.rocket.constants.MemoryOpConstants
import freechips.rocketchip.tile.HasL1CacheParameters
import freechips.rocketchip.util.GTimer

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
  def CounterMax = ((1 << StrideCounterWidth) - 1).U(StrideCounterWidth.W)
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
  val ever_written  = Bool()
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

    // debug
    val cell = Output(UInt(Addrbits.W))
  })
  val valid = RegInit(false.B)

  val cell = RegInit(0.U(Addrbits.W))
  io.eq := valid && (io.r_addr === cell)
  when (io.write) {
    cell := io.w_addr
    valid := true.B
  }
  io.cell := cell
}

class ExpandedSITSignals (implicit p: Parameters) extends BoomBundle()(p)
  with T2Parameters
{
  val hit = Bool()
  val miss = Bool()
  val sit_ptr = UInt(log2Ceil(SIT_SIZE).W)
  val pc = UInt(coreMaxAddrBits.W)
  val addr = UInt(coreMaxAddrBits.W)
  val is_store = Bool()
}

class StreamPrefetcher (implicit p: Parameters) extends BoomModule ()(p)
  with T2Parameters
  with T2StateConstants
  with PrefetcherConstants
  with MemoryOpConstants
{
  val io = IO(new Bundle() {
    val isInLoop = Input(Bool())
    val pred = Valid(new CommonSubPrefetcherOut())
    val cf = Flipped(Valid(new ControlFlowInfo()))
    val df = Flipped(Valid(new DataflowInfo()))
    val s2_miss = Input(Bool())
  })

  ////////////////////// s1
  val data_array = SyncReadMem(SIT_SIZE, new SITEntry())

  val mPCEntries = for (i <- 0 until SIT_SIZE) yield {val slot = Module(new MPCEntry()); slot}
  val mPCEntryPorts = VecInit(mPCEntries.map(_.io))

  mPCEntryPorts.foreach{e => e.r_addr := io.df.bits.pc}

  val s1_valid_masked = io.df.valid && io.isInLoop
  val s1_hit_wire = s1_valid_masked && mPCEntryPorts.map(_.eq).reduce(_|_) // any match

  val blockMask_1 = ((1L << Addrbits) - 1) ^ ((1 << 6) - 1)
  println(f"block mask: 0x$blockMask_1%x")
  val blockMask = (((1L << Addrbits) - 1) ^ ((1 << 6) - 1)).U(Addrbits.W)

  val masked_addr = (io.df.bits.addr & blockMask).asUInt()


  val s1_hit_ptr_w = OHToUInt(mPCEntryPorts.map(_.eq))
  val s1_hit_ptr = RegNext(s1_hit_ptr_w)
  val s1_miss_wire = s1_valid_masked && !s1_hit_wire

  val s1_info = Reg(Valid(new ExpandedSITSignals()))

  s1_info.valid := s1_valid_masked
  s1_info.bits.hit := s1_hit_wire
  s1_info.bits.miss := s1_miss_wire
  s1_info.bits.addr := masked_addr
  s1_info.bits.pc := io.df.bits.pc
  s1_info.bits.is_store := isWrite(io.df.bits.cmd)

  //  val s1_kill_s2 = s1_miss_wire && s1_info.valid && s1_info.bits.miss &&
  //    s1_info.bits.addr === masked_addr
  //               |      s2           |              s1                 |      s2     |
  val s1_kill_s2 = s1_info.bits.addr === masked_addr && s1_valid_masked && s1_info.valid

  mPCEntryPorts.foreach{
    e => dprintf(D_T2, s1_info.valid, "cell: 0x%x\n", e.cell)
  }

  val s1_read_sit_entry = Wire(new SITEntry())
  s1_read_sit_entry := data_array.read(s1_hit_ptr_w, s1_hit_wire)

  ////////////////////// s2
  val sit_alloc_ptr = RegInit(0.U(log2Ceil(SIT_SIZE).W))
  val sit_w_ptr_oh = UIntToOH(sit_alloc_ptr)

  val s2_miss_masked = !s1_kill_s2 && s1_info.bits.miss

  mPCEntryPorts.zipWithIndex.foreach{case(e, i) =>
    e.write := s2_miss_masked && sit_w_ptr_oh(i)}
  dprintf(D_T2_3, s2_miss_masked,
    "[%d] req pc0x%x addr[0x%x] writing to mPC table [%d]\n",
    GTimer(), s1_info.bits.pc, s1_info.bits.addr, sit_alloc_ptr)
  dprintf(D_T2_3, RegNext(s2_miss_masked),
    "[%d] mPC table [%d]: pc0x%x\n",
    GTimer(), RegNext(sit_alloc_ptr), mPCEntryPorts(RegNext(sit_alloc_ptr)).cell)


  mPCEntryPorts.foreach{e => e.w_addr := s1_info.bits.pc}

  val s2_info_w = WireInit(s1_info)
  when (s1_kill_s2) {
    s2_info_w.valid := false.B
    s2_info_w.bits.hit := false.B
    s2_info_w.bits.miss := false.B
  }
  s2_info_w.bits.sit_ptr := Mux(s1_info.bits.miss, sit_alloc_ptr, s1_hit_ptr)

  when (s2_miss_masked) {
    sit_alloc_ptr := sit_alloc_ptr + 1.U
  }

  val s2_info = RegNext(s2_info_w)

  val s2_read_sit_entry = RegNext(s1_read_sit_entry)

  ////////////////////// s3
  val s3_wb = Wire(Bool())
  s3_wb := s2_info.bits.miss
  val s3_wb_entry_w = Wire(new SITEntry())

  // default values to write back on miss
  s3_wb_entry_w.lastAddr := s2_info.bits.addr  // always "updated"
  s3_wb_entry_w.lastPrefAddr := 0.U  // kept in most cases
  s3_wb_entry_w.state := TS_UNK
  s3_wb_entry_w.counter := 0.U(StrideCounterWidth.W)
  s3_wb_entry_w.delta := 0.U(AddrDeltaWidth.W)
  s3_wb_entry_w.ever_written := s2_info.bits.is_store

  val new_delta = Wire(UInt(AddrDeltaWidth.W))
  new_delta := s2_info.bits.addr - s2_read_sit_entry.lastAddr

  io.pred.valid := false.B
  io.pred.bits := DontCare
  val pref_addr = WireInit(0.U(Addrbits.W))

  when (s2_info.bits.hit) {
    s3_wb_entry_w.ever_written := s2_read_sit_entry.ever_written || s2_info.bits.is_store

    dprintf(D_T2_3, "[%d] req pc0x%x addr[0x%x] state: %d\n",
      GTimer(), s1_info.bits.pc, s1_info.bits.addr, s2_read_sit_entry.state)

    when(s2_read_sit_entry.state === TS_UNK) {
      when(io.s2_miss) {
        dprintf(D_T2_3, "[%d] set to OB because of primary miss\n",
          GTimer())
        s3_wb := true.B
        s3_wb_entry_w.delta := new_delta
        s3_wb_entry_w.state := TS_OB
        s3_wb_entry_w.counter := 0.U(StrideCounterWidth.W)
      }
    }
    when(s2_read_sit_entry.state === TS_OB) {
      when(new_delta === 0.U) {
        // keep them with no modification
        dprintf(D_T2_3, "[%d] skip from TS_OB because of 0 delta\n", GTimer())

      }.otherwise {

        s3_wb := true.B
        s3_wb_entry_w.delta := new_delta

        when(new_delta === s2_read_sit_entry.delta) {
          s3_wb_entry_w.state := TS_STRD
          s3_wb_entry_w.counter := 1.U(StrideCounterWidth.W)
          dprintf(D_T2_3,
            "[%d] set to striding with old addr 0x%x, old delta %d, new delta %d\n",
            GTimer(), s2_read_sit_entry.lastAddr, s2_read_sit_entry.delta, new_delta)

        }.otherwise {
          s3_wb_entry_w.state := TS_OB
          s3_wb_entry_w.counter := 0.U(StrideCounterWidth.W)
        }
      }
    }
    when(s2_read_sit_entry.state === TS_STRD) {
      when(new_delta === 0.U) {
        // keep them with no modification
        dprintf(D_T2_3, "[%d] skip from TS_STRD because of 0 delta\n", GTimer())

      }.otherwise {

        s3_wb := true.B
        s3_wb_entry_w.delta := new_delta

        when(new_delta === s2_read_sit_entry.delta) {
          dprintf(D_T2_3,
            "[%d] inc striding with old addr 0x%x, old delta %d, new delta %d\n",
            GTimer(), s2_read_sit_entry.lastAddr, s2_read_sit_entry.delta, new_delta)

          s3_wb_entry_w.state := TS_STRD
          when (s2_read_sit_entry.counter =/= CounterMax) {
            s3_wb_entry_w.counter :=
              s2_read_sit_entry.counter + 1.U
          }.otherwise {
            s3_wb_entry_w.counter := s2_read_sit_entry.counter
          }

          when (s3_wb_entry_w.counter > StrideCounterThreshold.U) {
            s3_wb_entry_w.lastPrefAddr := pref_addr

            io.pred.valid := true.B

            pref_addr := s2_info.bits.addr + s3_wb_entry_w.delta
            io.pred.bits.addr := pref_addr

            io.pred.bits.confidence := s3_wb_entry_w.counter
            io.pred.bits.temporality := 0.U
            io.pred.bits.cmd := Mux(s3_wb_entry_w.ever_written, M_PFW, M_PFR)

            dprintf(D_T2_3, "[%d] create prefetch to addr 0x%x\n", GTimer(), pref_addr)
          }
        }.otherwise {
          s3_wb_entry_w.state := TS_OB
          s3_wb_entry_w.counter := 0.U(StrideCounterWidth.W)
          dprintf(D_T2_3,
            "[%d] revoke striding with old addr 0x%x, old delta %d, new delta %d\n",
            GTimer(), s2_read_sit_entry.lastAddr, s2_read_sit_entry.delta, new_delta)
        }
      }
    }
  }
  when (s3_wb) {
    data_array.write(s2_info.bits.sit_ptr, s3_wb_entry_w)
  }
}

class LoopInfo(implicit p: Parameters) extends BoomBundle()(p)
  with PrefetcherConstants
{
  val inLoop = Bool()
  val branchAddr = UInt(Addrbits.W)
  val targetAddr = UInt(Addrbits.W)
}

class LoopPred (implicit p: Parameters) extends BoomModule()(p)
  with T2Parameters
  with T2StateConstants
  with PrefetcherConstants
{
  val io = IO(new Bundle() {
    val cf = Flipped(Valid(new ControlFlowInfo()))
    val loop_info = Output(new LoopInfo())
  })

  // structures
  val loopInstPC = RegInit(0.U(Addrbits.W))
  val loopTargetPC = RegInit(0.U(Addrbits.W))

  val inLoop = RegInit(false.B)

  io.loop_info.inLoop := inLoop
  io.loop_info.branchAddr := loopInstPC
  io.loop_info.targetAddr := loopTargetPC

  val loopCount = SaturatingCounter(LoopCounterMax)
  val nlpct = Module(new RoundRobinCAM(UInt(Addrbits.W), NLPCTSize))

  // logic:
  private val pc = io.cf.bits.inst_addr
  private val target = io.cf.bits.target_addr

  nlpct.io.search_data.valid := io.cf.valid
  nlpct.io.search_data.bits := pc

  nlpct.io.insert_data.valid := false.B
  nlpct.io.insert_data.bits := 0.U

  private val is_backward = pc > target
  private val found_in_nlpct = nlpct.io.found

  dprintf(D_LoopPred, is_backward, "LoopPred: received backward branch!\n")

  when (is_backward && !found_in_nlpct) {
    when (io.cf.bits.inst_addr === loopInstPC) {
      loopCount.inc()
      inLoop := true.B
      dprintf(D_LoopPred, "LoopPred: loop found!\n")

    }.otherwise{
      when (!inLoop || (pc =/= loopInstPC && target =/= loopTargetPC &&
        pc =/= loopTargetPC && target =/= loopInstPC)) {
        when (loopCount.value < MinLoopSize.U) {
          nlpct.io.insert_data.valid := true.B
          nlpct.io.insert_data.bits := loopInstPC
        }
        loopInstPC := pc
        loopTargetPC := target
        loopCount.clear()
        inLoop := false.B
      }
    }
  }
}
