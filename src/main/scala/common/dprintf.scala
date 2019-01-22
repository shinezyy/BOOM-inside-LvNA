
package boom.common

import chisel3.{core, _}
import chisel3.core._
import freechips.rocketchip.tile



object dprintf{

  def apply(cond: Boolean, cond2: Bool, fmt: String, data: Bits*): Unit = {
    core.when (cond2) {
      apply(cond, fmt, data: _*)
    }
  }

  def apply(cond: Boolean,  fmt: String, data: Bits*): Unit = {
    if (cond) {
      core.printf(fmt, data: _*)
    }
  }

  def apply(cond: Boolean, fmt: String): Unit = {
    if (cond) {
      core.printf(fmt)
    }
  }
}
