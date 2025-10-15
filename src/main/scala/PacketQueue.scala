package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.trace._
import org.chipsalliance.cde.config.Parameters

class PacketQueueIO(depth: Int, coreParams: TraceCoreParams) extends Bundle {
  val entry = Input(new TraceCoreInterface(coreParams))
  val entry_valid = Input(Bool())
  val dequeue = Input(Bool())

  val invalidate_current_entry_insn = Input(Bool())
  val invalidate_current_entry_idx = Input(UInt(log2Ceil(coreParams.nGroups).W))

  val current_entry = Output(new TraceCoreInterface(coreParams))
  val current_entry_valid = Output(Bool())
  val next_entry = Output(new TraceCoreInterface(coreParams))
  val next_entry_valid = Output(Bool())

  val stall = Output(Bool())
  val count = Output(UInt(log2Ceil(depth + 1).W))
}

class PacketQueue(val depth: Int, val coreParams: TraceCoreParams) extends Module {
  val io = IO(new PacketQueueIO(depth, coreParams))

  val queue = RegInit(VecInit(Seq.fill(depth)(0.U.asTypeOf(new TraceCoreInterface(coreParams)))))

  val head = RegInit(0.U(log2Ceil(depth).W))
  val tail = RegInit(0.U(log2Ceil(depth).W))
  val count = RegInit(0.U(log2Ceil(depth + 1).W))

  io.count := count

  val full = count === depth.U
  val empty = count === 0.U
  io.stall := full

  val current_entry = queue(head)
  val next_head = Mux(head === (depth.U - 1.U), 0.U, head + 1.U)
  val next_entry = queue(next_head)

  io.current_entry := current_entry
  io.current_entry_valid := count > 0.U
  io.next_entry := next_entry
  io.next_entry_valid := count > 1.U

  val do_enq = io.entry_valid && !full
  val do_deq = io.dequeue && !empty

  switch(Cat(do_enq, do_deq)) {
    is("b10".U) { // enqueue only
      queue(tail) := io.entry
      tail := Mux(tail === (depth - 1).U, 0.U, tail + 1.U)
      count := count + 1.U
    }
    is("b01".U) { // dequeue only
      head := Mux(head === (depth - 1).U, 0.U, head + 1.U)
      count := count - 1.U
    }
    is("b11".U) { // simultaneous enqueue & dequeue
      queue(tail) := io.entry
      tail := Mux(tail === (depth - 1).U, 0.U, tail + 1.U)
      head := Mux(head === (depth - 1).U, 0.U, head + 1.U)
      // count stays the same
    }
  }

  // selective invalidation of one instruction within current entry
  when(io.invalidate_current_entry_insn) {
    val modified_entry = WireInit(current_entry)
    modified_entry.group(io.invalidate_current_entry_idx).iretire := 0.U
    queue(head) := modified_entry
  }
  
  printf("number of packets in queue: %x\n", count)

  assert(!(count === 0.U && head =/= tail), "FIFO pointers out of sync when empty!")
}