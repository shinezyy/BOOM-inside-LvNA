package boom.lsu.pref

import boom.common._
import boom.common.util.{RoundRobinCAM, SaturatingCounter}
import chisel3._
import chisel3.internal.naming.chiselName
import chisel3.util._
import freechips.rocketchip.config.Parameters
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
}

class StreamPrefetcher (implicit p: Parameters) extends BoomModule ()(p)
  with T2Parameters
  with T2StateConstants
  with PrefetcherConstants
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

  val s1_hit_wire = io.df.valid && mPCEntryPorts.map(_.eq).reduce(_|_) // any match

  val blockMask_1 = (((1L << Addrbits) - 1) ^ ((1 << 6) - 1))
  val blockMask = (((1L << Addrbits) - 1) ^ ((1 << 6) - 1)).U(Addrbits.W)
  println(f"block mask: 0x$blockMask_1%x")

  val masked_addr = (io.df.bits.addr & blockMask).asUInt()

  dprintf(D_T2_2, s1_hit_wire, "[%d] access hit in mPCEntries with pc0x%x, addr 0x%x\n",
    GTimer(), io.df.bits.pc, masked_addr)

  val s1_hit_ptr_w = OHToUInt(mPCEntryPorts.map(_.eq))
  val s1_hit_ptr = RegNext(s1_hit_ptr_w)
  val s1_miss_wire = io.df.valid && !s1_hit_wire

  val s1_info = Reg(Valid(new ExpandedSITSignals()))

  s1_info.valid := io.df.valid && io.isInLoop
  s1_info.bits.hit := s1_hit_wire
  s1_info.bits.miss := s1_miss_wire
  s1_info.bits.addr := masked_addr
  s1_info.bits.pc := io.df.bits.pc

  //              |     s1      |                 s2                |
  val s1_kill_s2 = s1_miss_wire && s1_info.valid && s1_info.bits.miss &&
    s1_info.bits.addr === masked_addr

  dprintf(D_T2, io.df.valid && io.isInLoop,
    "[%d] s1_miss wire: %d, s1_hit wire: %d, s1_pc wire: 0x%x\n",
    GTimer(), s1_miss_wire, s1_hit_wire, io.df.bits.pc)

  dprintf(D_T2, s1_info.valid,
    "[%d] s1_miss reg: %d, s1_hit reg: %d, s1_pc reg: 0x%x\n",
    GTimer(), s1_info.bits.miss, s1_info.bits.hit, s1_info.bits.pc)


  dprintf(D_T2, s1_info.valid, "[%d] valid mem access in loop: pc 0x%x, addr 0x%x\n",
    GTimer(), s1_info.bits.pc, s1_info.bits.addr)

  mPCEntryPorts.foreach{
    e => dprintf(D_T2, s1_info.valid, "cell: 0x%x\n", e.cell)
  }

  val s1_read_sit_entry = Wire(new SITEntry())
  s1_read_sit_entry := data_array.read(s1_hit_ptr_w, s1_hit_wire)

  ////////////////////// s2
  val (sit_alloc_ptr, sit_ptr_wrap) = Counter(s1_info.bits.miss, SIT_SIZE)

  val sit_w_ptr_oh = UIntToOH(sit_alloc_ptr)

  mPCEntryPorts.zipWithIndex.foreach{case(e, i) =>
    e.write := !s1_kill_s2 && s1_info.valid && s1_info.bits.miss && sit_w_ptr_oh(i)}
  when (s1_info.valid && s1_info.bits.miss) {
    dprintf(D_T2, "[%d] w_ptr: %d\n", GTimer(), sit_alloc_ptr)
  }

  mPCEntryPorts.foreach{e => e.w_addr := s1_info.bits.pc}

  val s2_info_w = WireInit(s1_info)
  when (s1_kill_s2) {
    s2_info_w.valid := false.B
  }
  s2_info_w.bits.sit_ptr := Mux(s1_info.bits.miss, sit_alloc_ptr, s1_hit_ptr)

  val s2_info = RegNext(s2_info_w)

  val s2_read_sit_entry = RegNext(s1_read_sit_entry)

  ////////////////////// s3
  val s3_wb_entry_w = Wire(new SITEntry())
  s3_wb_entry_w.state := TS_UNK
  s3_wb_entry_w.lastPrefAddr := 0.U
  s3_wb_entry_w.counter := 1.U(StrideCounterWidth.U)
  s3_wb_entry_w.lastAddr := s2_info.bits.addr

  val new_delta = Wire(UInt(AddrDeltaWidth.W))
  new_delta := s2_info.bits.addr - s2_read_sit_entry.lastAddr

  s3_wb_entry_w.delta := new_delta

  when (s2_info.valid) {
    s3_wb_entry_w := s2_read_sit_entry
    s3_wb_entry_w.lastAddr := s2_info.bits.addr
    val changed = WireInit(false.B)
    when(io.s2_miss) {
      changed := true.B
      dprintf(D_T2_1, s2_info.valid && s2_info.bits.hit,
        "[%d] s2 processing pc0x%x addr: 0x%x!\n",
        GTimer(), s2_info.bits.pc, s2_info.bits.addr)

      dprintf(D_T2_1, s2_info.valid && s2_info.bits.hit,
        "[%d] state: %d, s2_miss: %d\n",
        GTimer(), s2_read_sit_entry.state, io.s2_miss)

      when(s2_read_sit_entry.state === TS_UNK) {
        dprintf(D_T2_1, "[%d] primary miss triggered by pc0x%x addr[0x%x]," +
          "set to OB\n",
          GTimer(), s2_info.bits.pc, s2_info.bits.addr)
        s3_wb_entry_w.state := TS_OB
        s3_wb_entry_w.delta := new_delta
      }

      dprintf(D_T2_2, "[%d] l1d miss by pc0x%x addr[0x%x], delta old: %d, new:%d\n",
        GTimer(), s2_info.bits.pc, s2_info.bits.addr, s2_read_sit_entry.delta, new_delta)

    }.otherwise {
      dprintf(D_T2_2, "[%d] pc0x%x addr: 0x%x hit in SIT but not miss!\n",
        GTimer(), s2_info.bits.pc, s2_info.bits.addr)
    }


    when (s2_info.bits.hit) {
      // keep most states by default
      s3_wb_entry_w.delta := s2_read_sit_entry.delta

      when(new_delta =/= 0.U) {
        s3_wb_entry_w.delta := new_delta
        when(new_delta === s2_read_sit_entry.delta) {
          changed := true.B
          dprintf(D_T2_1, "[%d] sit ptr: %d\n", GTimer(), s2_info.bits.sit_ptr)

          when(s2_read_sit_entry.state === TS_OB && new_delta =/= 0.U) {
            dprintf(D_T2_1, "[%d] set to striding\n", GTimer())
            s3_wb_entry_w.state := TS_STRD
            s3_wb_entry_w.counter := 1.U(StrideCounterWidth.W)

          }.elsewhen(s2_read_sit_entry.state === TS_STRD &&
            s2_read_sit_entry.counter =/= ((1 << StrideCounterWidth) - 1).U(StrideCounterWidth.W)) { // saturating counter

            dprintf(D_T2_1, "[%d] inc striding\n", GTimer())
            s3_wb_entry_w.state := TS_STRD
            s3_wb_entry_w.counter := (s2_read_sit_entry.counter + 1.U) (StrideCounterWidth - 1, 0)

            for (i <- 0 until SIT_SIZE) {
              dprintf(D_T2_1, "SIT[%d]: 0x%x\n",
                i.U, data_array.read(i.U).lastAddr)
            }
          }
        }.elsewhen(s2_read_sit_entry.state === TS_STRD) { // not equal
          changed := true.B
          s3_wb_entry_w.state := TS_OB
          s3_wb_entry_w.counter := 0.U(StrideCounterWidth - 1, 0)

          dprintf(D_T2_1, "[%d] Set striding back to OB\n", GTimer())
          dprintf(D_T2_1, "[%d] sit ptr: %d\n", GTimer(), s2_info.bits.sit_ptr)
        }
      }
    }
    when (changed) {
      for (i <- 0 until SIT_SIZE) {
        val e = data_array.read(i.U)
        dprintf(D_T2_1, "SIT[%d], pc: 0x%x, addr: 0x%x, state: %d, delta: %d\n",
          i.U, mPCEntryPorts(i).cell, e.lastAddr, e.state, e.delta)
      }
    }
  }
  val pref_addr = s2_info.bits.addr + s3_wb_entry_w.delta
//  val pref_addr = WireInit(0.U(Addrbits.W))

  io.pred.valid := false.B
  when (s2_info.valid && s2_info.bits.hit && io.s2_miss &&
    s3_wb_entry_w.state === TS_STRD &&
    s3_wb_entry_w.counter > StrideCounterThreshold.U) {
    dprintf(D_T2_2, "[%d] Stride prefetch issued to fetch pc0x%x addr[0x%x]\n",
      GTimer(), s2_info.bits.pc, pref_addr)
    io.pred.valid := true.B
    io.pred.bits.addr := pref_addr
    io.pred.bits.confidence := s3_wb_entry_w.counter
    io.pred.bits.temporality := 0.U
    s3_wb_entry_w.lastPrefAddr := pref_addr
  }

  val s3_wb_next = RegNext(s3_wb_entry_w)
  val s2_sit_ptr = RegNext(s2_info.bits.sit_ptr)

  val last_cycle_wb = RegInit(false.B)
  when (s2_info.valid && s2_info.bits.hit && io.s2_miss) {
    data_array.write(s2_info.bits.sit_ptr, s3_wb_entry_w)
    last_cycle_wb := true.B
  }.otherwise {
    last_cycle_wb := false.B
  }
  dprintf(D_T2_1, last_cycle_wb, "[%d] last wb ptr: %d, wb addr: 0x%x, state: %d\n",
    GTimer(), s2_sit_ptr, s3_wb_next.lastAddr, s3_wb_next.state)
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
