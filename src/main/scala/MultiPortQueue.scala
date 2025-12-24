package tacit

import chisel3._
import chisel3.util._

class MultiPortQueueDecoupled[T <: Data](gen: T, val nPorts: Int, val depth: Int) extends Module {
  val io = IO(new Bundle {
    val enq = Flipped(Vec(nPorts, Decoupled(gen)))
    val deq = Decoupled(gen)
    // expose helper signals
    val enqFire = Output(Vec(nPorts, Bool()))
    val deqFire = Output(Bool())
    val count   = Output(UInt(log2Ceil(depth + 1).W))
  })

  // internal single-port queue
  val queue = Module(new Queue(gen, depth))
  queue.io.deq <> io.deq

  val validMask = io.enq.map(_.valid)
  val enqIdx = PriorityEncoder(validMask)

  for (i <- 0 until nPorts) { io.enq(i).ready := false.B }

  val enq_bits = WireInit(0.U.asTypeOf(io.enq(0).bits))

    // only override the chosen port
    when (queue.io.enq.ready && validMask.reduce(_||_)) {
    enq_bits := io.enq(enqIdx).bits
    io.enq(enqIdx).ready := true.B
    }

    // connect to internal queue
    queue.io.enq.bits := enq_bits
    queue.io.enq.valid := queue.io.enq.ready && validMask.reduce(_||_)

  when (queue.io.enq.ready && validMask.reduce(_||_)) {
    queue.io.enq.bits := io.enq(enqIdx).bits
    queue.io.enq.valid := true.B
    io.enq(enqIdx).ready := true.B
  } .otherwise {
    queue.io.enq.valid := false.B
  }
  // fire signals (initialize to zero)
  val enq_fire = WireInit(VecInit(Seq.fill(nPorts)(false.B)))
  val deq_fire = WireInit(false.B)

  for (i <- 0 until nPorts) { enq_fire(i) := io.enq(i).valid && io.enq(i).ready }
  deq_fire := queue.io.deq.valid && queue.io.deq.ready

  // count (initialize to zero)
  val countReg = RegInit(0.U(log2Ceil(depth + 1).W))
  when (enq_fire.reduce(_||_) && !deq_fire) { countReg := countReg + 1.U }
  .elsewhen (!enq_fire.reduce(_||_) && deq_fire) { countReg := countReg - 1.U }

  // cnnect to IO (all outputs driven)
  io.enqFire := enq_fire
  io.deqFire := deq_fire
  io.count   := countReg
}