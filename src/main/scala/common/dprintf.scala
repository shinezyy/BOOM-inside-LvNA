
package boom.common

import chisel3._
import chisel3.core._



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
