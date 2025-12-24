package tacit

import chisel3._
import chisel3.util._

class MultiPortQueue[T <: Data](gen: T, val nPorts: Int, val depth: Int) extends Module {
  require(depth > 0, "Queue depth must be positive")
  require(nPorts > 0, "Number of enqueue ports must be positive")

  val io = IO(new Bundle {
    val enq = Flipped(Vec(nPorts, Decoupled(gen)))
    val deq = Decoupled(gen)
    val enq_fire = Output(Vec(nPorts, Bool()))
    val deq_fire = Output(Bool())
    val count = Output(UInt(log2Ceil(depth + 1).W))
  })

  for (i <- 0 until nPorts) {
    io.enq(i).ready := false.B
  }

  val queue = Reg(Vec(depth, gen))
  val enq_ptr = RegInit(0.U(log2Ceil(depth).W))
  val deq_ptr = RegInit(0.U(log2Ceil(depth).W))
  val occupancy = RegInit(0.U((log2Ceil(depth + 1)).W))

  // determine which enqueue ports can actually fire
  val enq_fire_vec = Wire(Vec(nPorts, Bool()))
  for (i <- 0 until nPorts) {
    enq_fire_vec(i) := io.enq(i).valid && (occupancy + (i + 1).U <= depth.U)
    io.enq_fire(i) := enq_fire_vec(i)
  }

  // compute how many enqueues actually happen
  val enqCount = enq_fire_vec.map(b => b.asUInt).reduce(_ +& _)

  // write to queue for each firing enqueue port
  var tempenq_ptr = enq_ptr
  for (i <- 0 until nPorts) {
    when(enq_fire_vec(i)) {
      queue(tempenq_ptr) := io.enq(i).bits
    }
    // update temp pointer combinationally
    tempenq_ptr = Mux(enq_fire_vec(i), (tempenq_ptr + 1.U) % depth.U, tempenq_ptr)
  }
  enq_ptr := tempenq_ptr

  // dequeue logic
  val deqValid = occupancy =/= 0.U
  io.deq.valid := deqValid
  io.deq.bits := queue(deq_ptr)
  io.deq_fire := io.deq.fire

  when(io.deq.fire) {
    deq_ptr := (deq_ptr + 1.U) % depth.U
  }

  // update occupancy
  occupancy := occupancy + enqCount - io.deq.fire.asUInt

  // output count
  io.count := occupancy
}