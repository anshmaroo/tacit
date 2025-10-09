
package tacit

import chisel3._
import chisel3.util._
import scala.math.min
import freechips.rocketchip.trace._
import org.chipsalliance.cde.config.Parameters


class PacketQueue(depth: Int, params: TraceCoreParams) extends Module {
    val io = IO(new Bundle {
        val entry = Input(new TraceCoreInterface(params))
        val entry_valid = Input(Bool())

        val invalidate_current_entry_idx = Input(UInt(log2Ceil(params.nGroups).W))
        val invalidate_current_entry_insn = Input(Bool())

        val dequeue = Input(Bool())

        val stall = Output(Bool())
        
        val current_entry = Output(new TraceCoreInterface(params))
        val current_entry_valid = Output(Bool())

        val next_entry = Output(new TraceCoreInterface(params))
        val next_entry_valid = Output(Bool())
    })

    val IDX_WIDTH = log2Ceil(depth)

    // fifo
    val mem = SyncReadMem(depth, new TraceCoreInterface(params))

    // pointers
    val head = RegInit(0.U((IDX_WIDTH + 1).W))
    val tail = RegInit(0.U((IDX_WIDTH + 1).W))

    // status
    // val full = Wire(Bool())
    val count = RegInit(0.U((IDX_WIDTH + 1).W))
    val stall = RegInit(0.U(1.W))

    // outputs
    io.current_entry := mem(tail)
    io.current_entry_valid := count > 0.U
    io.next_entry := mem(tail + 1.U)
    io.next_entry_valid := count > 1.U

    stall := false.B
    io.stall := stall

    when (io.invalidate_current_entry_insn) {
        mem(tail).group(io.invalidate_current_entry_idx).iretire := 0.U
    }

    when (io.entry_valid) {
        when (count < depth.U) {
            mem(head) := io.entry
            head := head + 1.U
            count := count + 1.U
        } .otherwise {
            stall := true.B
        }
    }

    when (io.dequeue && count > 0.U) {
        tail := tail + 1.U;
        count := count - 1.U;   
    }

}