package boom.common.util

import chisel3._
import chisel3.core.IO
import chisel3.util._

class SaturatingCounter(val n: Int) {

  require(n >= 2)
  val value = RegInit(0.U(log2Ceil(n).W))

  def inc(): Bool = {
    val will_wrap = value === (n-1).asUInt
    when (!will_wrap) {
      value := value + 1.U
    }
    !will_wrap
  }
  def clear(): Unit = {
    value := 0.U
  }
}
object SaturatingCounter{
  /** Instantiate a [[SaturatingCounter! counter]] with the specified number of counts.
    */
  def apply(n: Int): SaturatingCounter = new SaturatingCounter(n)
}

class CAMCell[T <: UInt](gen: T) extends Module{
  val io = IO(new Bundle{
    val write = Input(Bool())
    val search_data = Input(gen)
    val insert_data = Input(gen)
    val eq = Output(Bool())
  })
  val valid = RegInit(false.B)

  val cell = RegInit(0.U(gen.getWidth.W))
  io.eq := valid && (io.search_data === cell)
  when (io.write) {
    cell := io.insert_data
    valid := true.B
  }
}
object CAMCell {
  def apply[T <: UInt](gen: T): CAMCell[T] = new CAMCell(gen)
}


class RoundRobinCAM[T <: UInt](gen: T, n: Int) extends Module{
  class RRCAMIO extends Bundle {
    val search_data = Flipped(Valid(gen))

    val found = Output(Bool())
    val match_addr = Output(UInt(log2Ceil(n).W))

    val insert_data = Flipped(Valid(gen))
  }
  val io = IO(new RRCAMIO)

  val entries = for (i <- 0 until n) yield {Module(new CAMCell(gen))}
  val ports = VecInit(entries.map(_.io))
  ports.foreach{e => e.search_data := io.search_data.bits}

  val (insert_addr, wrap) = Counter(io.insert_data.valid, n)

  val insert_addr_oh = UIntToOH(insert_addr)
  ports.zipWithIndex.foreach{case(e, i) => e.write := insert_addr_oh(i) && io.insert_data.valid}
  ports.foreach{e => e.insert_data := io.insert_data.bits}

  io.found := io.search_data.valid && ports.map(_.eq).reduce(_|_)
  io.match_addr := OHToUInt(ports.map(_.eq))
}

object RoundRobinCAM {
  def apply[T <: UInt](gen: T, n: Int): RoundRobinCAM[T] = new RoundRobinCAM(gen, n)
}

