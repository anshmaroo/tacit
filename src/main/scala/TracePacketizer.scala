package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.trace.TraceCoreParams

// slice packets into bytes TODO: is this efficient?
class TracePacketizer(val coreParams: TraceCoreParams, val out_bytes: Int) extends Module with MetaDataWidthHelper {

  val io = IO(new Bundle {
    // val target_addr = Flipped(Decoupled(Vec(addrMaxNumBytes, UInt(8.W))))
    // val trap_addr = Flipped(Decoupled(Vec(addrMaxNumBytes, UInt(8.W))))
    // val time = Flipped(Decoupled(Vec(timeMaxNumBytes, UInt(8.W))))
    // val prv = Flipped(Decoupled(UInt(8.W)))
    // val ctx = Flipped(Decoupled(Vec(ctxMaxNumBytes, UInt(8.W))))

    val message = Flipped(Decoupled(new MessageBundle(coreParams)))

    val byte = Flipped(Decoupled(UInt(8.W)))
    val metadata = Flipped(Decoupled(new MetaDataBundle(coreParams)))

    val out = Decoupled(Vec(out_bytes, UInt(8.W)))
    val out_count = Decoupled(UInt(log2Ceil(out_bytes + 1).W))
  })

  val pIdle :: pComp :: pFull :: Nil = Enum(3)
  val state = RegInit(pIdle)

  val trap_addr_num_bytes = Reg(UInt(log2Ceil(addrMaxNumBytes).W))
  val trap_addr_index = Reg(UInt(log2Ceil(addrMaxNumBytes).W))
  val target_addr_num_bytes = Reg(UInt(log2Ceil(addrMaxNumBytes).W))
  val target_addr_index = Reg(UInt(log2Ceil(addrMaxNumBytes).W))
  val time_num_bytes = Reg(UInt(log2Ceil(timeMaxNumBytes).W))
  val time_index = Reg(UInt(log2Ceil(timeMaxNumBytes).W))
  val prv_num_bytes = Reg(UInt(1.W))
  val prv_index = Reg(UInt(1.W))
  val ctx_num_bytes = Reg(UInt(log2Ceil(ctxMaxNumBytes).W))
  val ctx_index = Reg(UInt(log2Ceil(ctxMaxNumBytes).W))
  val header_num_bytes = Reg(UInt(1.W))
  val header_index = Reg(UInt(1.W))
  
  // default values
  io.out.valid := false.B
  io.out_count.valid := false.B
  io.metadata.ready := false.B
  io.message.ready := false.B
  io.byte.ready := false.B
  for (i <- 0 until out_bytes) {
    io.out.bits(i) := 0.U
  }
  // io.out.bits := 0.U
  io.out_count.bits := 0.U

  def prep_next_state(): Unit = {
    trap_addr_index := 0.U
    trap_addr_num_bytes := Mux(io.metadata.fire, io.metadata.bits.trap_addr, 0.U)
    target_addr_index := 0.U
    target_addr_num_bytes := Mux(io.metadata.fire, io.metadata.bits.target_addr, 0.U)
    time_index := 0.U
    time_num_bytes := Mux(io.metadata.fire, io.metadata.bits.time, 0.U)
    prv_index := 0.U
    prv_num_bytes := Mux(io.metadata.fire, io.metadata.bits.prv, 0.U)
    header_index := 0.U
    header_num_bytes := Mux(io.metadata.fire, ~io.metadata.bits.is_compressed, 0.U)
    state := Mux(io.metadata.fire, 
      Mux(io.metadata.bits.is_compressed.asBool, pComp, pFull),
      pIdle
    )
  }
  
  switch (state) {
    is (pIdle) {
      io.metadata.ready := true.B
      when (io.metadata.fire) {
        trap_addr_num_bytes := io.metadata.bits.trap_addr
        trap_addr_index := 0.U
        target_addr_num_bytes := io.metadata.bits.target_addr
        target_addr_index := 0.U
        time_num_bytes := io.metadata.bits.time
        time_index := 0.U
        header_num_bytes := ~io.metadata.bits.is_compressed
        header_index := 0.U
        prv_num_bytes := io.metadata.bits.prv
        prv_index := 0.U
        ctx_num_bytes := io.metadata.bits.ctx
        ctx_index := 0.U
        state := Mux(io.metadata.bits.is_compressed.asBool, pComp, pFull)
      }
    }
    is (pComp) {
      // transmit a byte from byte buffer
      io.byte.ready := io.out.ready
      io.out.valid := io.byte.valid
      io.out.bits(0) := io.byte.bits
      io.out_count.bits := 1.U
      io.out_count.valid := io.byte.valid
      when (io.byte.fire) {
        // metadata runs ahead by 1 cycle for performance optimization
        io.metadata.ready := true.B
        prep_next_state()
      }
    }
    is (pFull) {
      val BytesPerCycle = out_bytes 

      val rem_header = Mux(header_num_bytes > header_index, 1.U, 0.U)
      val rem_prv = Mux(prv_num_bytes > prv_index,1.U, 0.U)
      val rem_ctx = Mux(ctx_num_bytes > ctx_index, ctx_num_bytes - ctx_index, 0.U)
      val rem_trap = Mux(trap_addr_num_bytes > trap_addr_index, trap_addr_num_bytes - trap_addr_index, 0.U)
      val rem_target = Mux(target_addr_num_bytes > target_addr_index, target_addr_num_bytes - target_addr_index, 0.U)
      val rem_time = Mux(time_num_bytes > time_index, time_num_bytes - time_index, 0.U)

      val total_struct_remaining = rem_header + rem_prv + rem_ctx + rem_trap + rem_target + rem_time
      val header_is_blocking = rem_header > 0.U && !io.byte.valid
      
      
      val any_msg_needed  = rem_prv > 0.U || rem_ctx > 0.U || rem_trap > 0.U || rem_target > 0.U || rem_time > 0.U
      val msg_is_blocking = any_msg_needed && !io.message.valid

      val stall = header_is_blocking || msg_is_blocking

      
      val send_header = Mux(header_is_blocking, 0.U, rem_header)
      val send_prv = Mux(stall, 0.U, rem_prv)
      
      val send_ctx = Mux(stall, 0.U, rem_ctx)
      val send_trap = Mux(stall, 0.U, rem_trap)
      val send_target = Mux(stall, 0.U, rem_target)
      val send_time = Mux(stall, 0.U, rem_time)

     
      val end_header = Wire(UInt(log2Ceil(out_bytes + 1).W))
      val end_prv = Wire(UInt(log2Ceil(out_bytes + 1).W))
      end_header := send_header
      end_prv := end_header + send_prv
      val end_ctx = end_prv + send_ctx
      val end_trap = end_ctx + send_trap
      val end_target = end_trap + send_target
      val end_time = end_target + send_time
      
      val total_ready = end_time

      when (total_struct_remaining === 0.U) {
        io.out.valid := false.B
        io.out_count.valid := false.B
        
        io.byte.ready := true.B
        // release message only if we are done with all its fields
        io.message.ready := true.B 
        io.metadata.ready := true.B
        
        prep_next_state()
      }
      .otherwise {
        io.out.valid := total_ready > 0.U
        val valid_count = Mux(total_ready > BytesPerCycle.U, BytesPerCycle.U, total_ready)
        io.out_count.bits  := valid_count
        io.out_count.valid := true.B
        dontTouch(end_header)
        dontTouch(end_prv)
        for (i <- 0 until BytesPerCycle) { 
            val laneIdx = i.U
            io.out.bits(i) := 0.U 
            when (laneIdx < valid_count) {
              when (laneIdx < end_header) {
                  io.out.bits(i) := io.byte.bits 
              } .elsewhen (laneIdx < end_prv) {
                  io.out.bits(i) := io.message.bits.prv_encoder_output_byte
              } .elsewhen (laneIdx < end_ctx) {
                  io.out.bits(i) := io.message.bits.ctx_encoder_output_bytes(ctx_index + (laneIdx - end_prv))
              } .elsewhen (laneIdx < end_trap) {
                  io.out.bits(i) := io.message.bits.trap_addr_encoder_output_bytes(trap_addr_index + (laneIdx - end_ctx))
              } .elsewhen (laneIdx < end_target) {
                  io.out.bits(i) := io.message.bits.target_addr_encoder_output_bytes(target_addr_index + (laneIdx - end_trap))
              } .elsewhen (laneIdx < end_time) {
                  io.out.bits(i) := io.message.bits.time_encoder_output_bytes(time_index + (laneIdx - end_target))
              }
            }
        }

        when (io.out.fire) {
            when (valid_count >= end_header && send_header > 0.U) {
              header_index := header_index + 1.U
            }
            when (valid_count >= end_prv && send_prv > 0.U) {
              prv_index := prv_index + 1.U
            }
            val bytes_for_ctx = Mux(valid_count > end_prv, valid_count - end_prv, 0.U)
            val consumed_ctx  = Mux(bytes_for_ctx > rem_ctx, rem_ctx, bytes_for_ctx)
            ctx_index := ctx_index + consumed_ctx

            val bytes_for_trap = Mux(valid_count > end_ctx, valid_count - end_ctx, 0.U)
            val consumed_trap  = Mux(bytes_for_trap > rem_trap, rem_trap, bytes_for_trap)
            trap_addr_index := trap_addr_index + consumed_trap

            val bytes_for_target = Mux(valid_count > end_trap, valid_count - end_trap, 0.U)
            val consumed_target  = Mux(bytes_for_target > rem_target, rem_target, bytes_for_target)
            target_addr_index := target_addr_index + consumed_target

            val bytes_for_time = Mux(valid_count > end_target, valid_count - end_target, 0.U)
            val consumed_time  = Mux(bytes_for_time > rem_time, rem_time, bytes_for_time)
            time_index := time_index + consumed_time
        }
      }
    }
  }
}