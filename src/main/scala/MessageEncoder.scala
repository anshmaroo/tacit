package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.trace._

class MessageFields extends Bundle {
  val has_trap   = Bool()
  val has_target = Bool()
  val has_prv    = Bool()
  val has_ctx    = Bool()
}

class MessageBundle(val coreParams: TraceCoreParams) extends Bundle with MetaDataWidthHelper {
    // outputs
    val maxNumBytes = coreParams.iaddrWidth/(8-1) + 1
    val trap_addr_encoder_output_bytes = Vec(maxNumBytes, UInt(8.W))
    val target_addr_encoder_output_bytes = Vec(maxNumBytes, UInt(8.W))
    val time_encoder_output_bytes = Vec(maxNumBytes, UInt(8.W))
    val prv_encoder_output_byte = UInt(8.W)
    val ctx_encoder_output_bytes = Vec(ctxMaxNumBytes, UInt(8.W))
}   

class MessageEncoder(val coreParams: TraceCoreParams) extends Module with MetaDataWidthHelper {
    val maxNumBytes = coreParams.iaddrWidth/(8-1) + 1
    val io = IO(new Bundle {
        // valid bits
        // val encode_trap_addr_valid = Input(Bool())
        // val encode_target_addr_valid = Input(Bool())
        // val encode_prv_valid = Input(Bool())
        // val encode_ctx_valid = Input(Bool())

        val msg_type = Input(MessageType())
        val is_compressed = Input(Bool())
        val packet_valid = Input(Bool())

        val trap_addr_encoder_input = Input(UInt(coreParams.iaddrWidth.W))
        val target_addr_encoder_input = Input(UInt(coreParams.iaddrWidth.W))
        val time_encoder_input = Input(UInt(coreParams.xlen.W))
        val prv_encoder_from_priv_input = Input(UInt(3.W))
        val prv_encoder_to_priv_input = Input(UInt(3.W))
        val ctx_encoder_input = Input(UInt(maxASIdBits.W))

        val message = Output(new MessageBundle(coreParams))
        val trap_addr_encoder_output_num_bytes = Output(UInt(log2Ceil(maxNumBytes).W))
        val target_addr_encoder_output_num_bytes = Output(UInt(log2Ceil(maxNumBytes).W))
        val time_encoder_output_num_bytes = Output(UInt(log2Ceil(maxNumBytes).W))
        val ctx_encoder_output_num_bytes = Output(UInt(log2Ceil(ctxMaxNumBytes).W))
        val prv_encoder_output_valid = Output(Bool())
        val fields = Output(new MessageFields())
    })

    val fields = Wire(new MessageFields)
    fields := 0.U.asTypeOf(new MessageFields)
    io.fields := fields
    switch(io.msg_type) {
        is(MessageType.Trap) {
            fields.has_trap := true.B
            fields.has_target := true.B
            fields.has_prv := true.B
        }
        is(MessageType.Return) {
            fields.has_trap := true.B
            fields.has_target := true.B
            fields.has_prv := true.B
            fields.has_ctx := true.B
        }
        is(MessageType.Sync) {
            fields.has_trap := true.B
            fields.has_target := true.B
            fields.has_prv := true.B
            fields.has_ctx := true.B
        }
        is(MessageType.UninfJump) {
            fields.has_target := true.B
        }
    }   

    val trap_addr_encoder = Module(new VarLenEncoder(coreParams.iaddrWidth))
    val target_addr_encoder = Module(new VarLenEncoder(coreParams.iaddrWidth))
    val time_encoder = Module(new VarLenEncoder(coreParams.xlen))
    val prv_encoder = Module(new PrvEncoder)
    val ctx_encoder = Module(new VarLenEncoder(maxASIdBits))

    val base_valid = io.packet_valid && !io.is_compressed
    trap_addr_encoder.io.input_valid := base_valid && fields.has_trap
    target_addr_encoder.io.input_valid := base_valid && fields.has_target
    prv_encoder.io.input_valid := base_valid && fields.has_prv
    ctx_encoder.io.input_valid := base_valid && fields.has_ctx
    time_encoder.io.input_valid := base_valid


    trap_addr_encoder.io.input_value := io.trap_addr_encoder_input
    target_addr_encoder.io.input_value := io.target_addr_encoder_input
    time_encoder.io.input_value := io.time_encoder_input
    ctx_encoder.io.input_value := io.ctx_encoder_input
    prv_encoder.io.from_priv := io.prv_encoder_from_priv_input
    prv_encoder.io.to_priv := io.prv_encoder_to_priv_input

    io.trap_addr_encoder_output_num_bytes := Mux(fields.has_trap, trap_addr_encoder.io.output_num_bytes, 0.U)
    io.message.trap_addr_encoder_output_bytes := trap_addr_encoder.io.output_bytes

    io.target_addr_encoder_output_num_bytes := Mux(fields.has_target, target_addr_encoder.io.output_num_bytes, 0.U)
    io.message.target_addr_encoder_output_bytes := target_addr_encoder.io.output_bytes

    io.time_encoder_output_num_bytes := Mux(io.packet_valid && !io.is_compressed, time_encoder.io.output_num_bytes, 0.U)
    io.message.time_encoder_output_bytes := time_encoder.io.output_bytes

    io.prv_encoder_output_valid := (io.packet_valid && !io.is_compressed) && fields.has_prv
    io.message.prv_encoder_output_byte := prv_encoder.io.output_byte
    
    io.ctx_encoder_output_num_bytes := Mux(fields.has_ctx, ctx_encoder.io.output_num_bytes, 0.U)
    io.message.ctx_encoder_output_bytes := ctx_encoder.io.output_bytes
}
