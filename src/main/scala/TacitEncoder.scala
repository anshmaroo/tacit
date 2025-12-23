// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.trace._

import org.chipsalliance.cde.config.Parameters

object FullHeaderType extends ChiselEnum {
  val FTakenBranch    = Value(0x0.U) // 000
  val FNotTakenBranch = Value(0x1.U) // 001
  val FUninfJump      = Value(0x2.U) // 010
  val FInfJump        = Value(0x3.U) // 011
  val FTrap           = Value(0x4.U) // 100
  val FSync           = Value(0x5.U) // 101
  val FValue          = Value(0x6.U) // 110
  val FReserved       = Value(0x7.U) // 111
}

object CompressedHeaderType extends ChiselEnum {
  val CTB = Value(0x0.U) // 00, taken branch
  val CNT = Value(0x1.U) // 01, not taken branch
  val CNA = Value(0x2.U) // 10, not a compressed packet
  val CIJ = Value(0x3.U) // 11, is a jump
}

object TrapType extends ChiselEnum {
  val TNone      = Value(0x0.U)
  val TException = Value(0x1.U)
  val TInterrupt = Value(0x2.U)
  val TReturn    = Value(0x4.U)
}

object SyncType extends ChiselEnum {
  val SyncNone = Value(0b000.U)
  val SyncStart = Value(0b001.U)
  val SyncPeriodic = Value(0b010.U)
  val SyncEnd = Value(0b011.U)
}

object MessageType extends ChiselEnum {
  val Branch     = Value(0x0.U)
  val InfJump    = Value(0x1.U)
  val UninfJump  = Value(0x2.U)
  val Trap       = Value(0x3.U)
  val Return     = Value(0x4.U)
  val Sync       = Value(0x5.U)
  val BPHit      = Value(0x6.U)
  val BPMiss     = Value(0x7.U)
}

object HeaderByte {
  def from_trap_type(header_type: FullHeaderType.Type, trap_type: TrapType.Type): UInt = {
    Cat(
      trap_type.asUInt,
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }

  def from_sync_type(header_type: FullHeaderType.Type, sync_type: SyncType.Type): UInt = {
    Cat(
      sync_type.asUInt,
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }

  def apply(header_type: FullHeaderType.Type): UInt = {
    Cat(
      0.U(3.W),
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }
}

trait MetaDataWidthHelper {
  // abstract parameter
  val coreParams: TraceCoreParams
  def getMaxNumBytes(width: Int): Int = { width/(8-1) + 1 }
  lazy val maxASIdBits = coreParams.xlen match {
    case 32 => 9
    case 64 => 16
  }
  lazy val addrMaxNumBytes = getMaxNumBytes(coreParams.iaddrWidth)
  lazy val timeMaxNumBytes = getMaxNumBytes(coreParams.xlen)
  lazy val ctxMaxNumBytes = getMaxNumBytes(maxASIdBits)
}

class MetaDataBundle(val coreParams: TraceCoreParams) extends Bundle with MetaDataWidthHelper {
  val prv = UInt(1.W)
  val ctx = UInt(ctxMaxNumBytes.W)
  val target_addr = UInt(addrMaxNumBytes.W)
  val trap_addr = UInt(addrMaxNumBytes.W)
  val time = UInt(timeMaxNumBytes.W)
  val is_compressed = UInt(1.W)
}


class TacitEncoder(override val coreParams: TraceCoreParams, val bufferDepth: Int, val coreStages: Int, val bpParams: TacitBPParams)(implicit p: Parameters) 
    extends LazyTraceEncoder(coreParams)(p) {
  override lazy val module = new TacitEncoderModule(this)
}

class TacitEncoderModule(outer: TacitEncoder) extends LazyTraceEncoderModule(outer) with MetaDataWidthHelper {

  val coreParams = outer.coreParams

  val MAX_DELTA_TIME_COMP = 0x3F // 63, 6 bits
  def stallThreshold(count: UInt) = count >= (outer.bufferDepth - outer.coreStages).U

  // mode of operation
  // 0: branch target only
  // 1: branch prediction and skip jump
  // 2: branch prediction and don't skip jump
  def is_bt_mode = io.control.bp_mode === 0.U
  def is_bp_mode = io.control.bp_mode === 2.U 

  // states
  val sIdle :: sSync :: sData :: Nil = Enum(3)
  val state = RegInit(sIdle)
  val sync_type = RegInit(SyncType.SyncNone)
  val enabled = RegInit(false.B)
  val stall = Wire(Bool())
  val prev_time = Reg(UInt(coreParams.xlen.W))

  // pipeline of ingress data
  val ingress_0 = RegInit(0.U.asTypeOf(new TraceCoreInterface(coreParams)))
  val ingress_1 = RegInit(0.U.asTypeOf(new TraceCoreInterface(coreParams)))

  // shift every cycle, if not stalled
  val pipeline_advance = Wire(Bool())
  pipeline_advance := io.in.group(0).iretire === 1.U
  when (pipeline_advance) {
    ingress_0 := io.in
    ingress_1 := ingress_0
  }

  // encoders
  val trap_addr_encoder_max_num_bytes = coreParams.iaddrWidth / (8 - 1) + 1
  val target_addr_encoder_max_num_bytes = coreParams.iaddrWidth / (8 - 1) + 1
  val time_encoder_max_num_bytes = coreParams.xlen / (8 - 1) + 1
  val ctx_encoder_max_num_bytes = maxASIdBits / (8 - 1) + 1

  val metadataWidth = log2Ceil(trap_addr_encoder_max_num_bytes) + log2Ceil(target_addr_encoder_max_num_bytes) + log2Ceil(time_encoder_max_num_bytes) + 1
  val message_encoder = Seq.fill(coreParams.nGroups) (Module(new MessageEncoder(coreParams)))

  // buffers
  val message_buffer = Module(new Queue(new MessageBundle(coreParams), outer.bufferDepth))
  val byte_buffer = Module(new Queue(UInt(8.W), outer.bufferDepth)) // buffer compressed packet or full header
  val metadata_buffer = Module(new Queue(new MetaDataBundle(coreParams), outer.bufferDepth))
  
  // intermediate packet signals
  val is_compressed = Wire(Vec(coreParams.nGroups, Bool()))
  val delta_time    = ingress_1.time - prev_time
  val packet_valid  = Wire(Vec(coreParams.nGroups, Bool()))
  val header_byte   = Wire(Vec(coreParams.nGroups, UInt(8.W))) // full header

  val comp_packet   = Wire(UInt(8.W)) // compressed packet
  val comp_header   = Wire(Vec(coreParams.nGroups, UInt(CompressedHeaderType.getWidth.W))) // compressed header
  
  // branch predictor
  val bp = Module(new DSCBranchPredictor(outer.bpParams))
  val bp_hit_count_next = Wire(UInt(32.W))
  val bp_inference_valid = Wire(Bool())
  val bp_hit_count_en = Wire(Bool())
  val bp_hit_count = RegEnable(bp_hit_count_next, 0.U, bp_hit_count_en)
  bp_hit_count_next := bp_hit_count // default behavior is to hold the value
  val bp_miss_flag_next = Wire(Bool())
  val bp_miss_flag_en = Wire(Bool())
  val bp_miss_flag = RegEnable(bp_miss_flag_next, false.B, bp_miss_flag_en)
  bp_miss_flag_next := false.B // default behavior is to set to false
  val bp_flush_hit = Wire(Bool())
  bp_flush_hit := false.B
  
  val bp_hit_packet = Cat(bp_hit_count(5, 0), comp_header(0))
  comp_packet := Cat(delta_time(5, 0), comp_header(0))

  // packetization of buffered message
  val trace_packetizer = Module(new TracePacketizer(coreParams))
  trace_packetizer.io.message <> message_buffer.io.deq
  trace_packetizer.io.metadata <> metadata_buffer.io.deq
  trace_packetizer.io.byte <> byte_buffer.io.deq
  trace_packetizer.io.out <> io.out

  // metadata packing
  val metadata = Wire(Vec(coreParams.nGroups, new MetaDataBundle(coreParams)))
  for (i <- 0 until coreParams.nGroups) {
    metadata(i).prv := message_encoder(i).io.prv_encoder_output_valid
    metadata(i).ctx := message_encoder(i).io.ctx_encoder_output_num_bytes
    metadata(i).trap_addr := message_encoder(i).io.trap_addr_encoder_output_num_bytes
    metadata(i).target_addr := message_encoder(i).io.target_addr_encoder_output_num_bytes
    metadata(i).time := message_encoder(i).io.time_encoder_output_num_bytes
    metadata(i).is_compressed := is_compressed(i)
  }

  // default values
  for (i <- 0 until coreParams.nGroups) {
    is_compressed(i) := false.B
    packet_valid(i) := false.B
  }

  val message_type = Wire(Vec(coreParams.nGroups, MessageType()))
  for (i <- 0 until coreParams.nGroups) {
    message_type(i) := MessageType.Branch
  }

  for (i <- 0 until coreParams.nGroups) {
    message_encoder(i).io.trap_addr_encoder_input := 0.U
    message_encoder(i).io.target_addr_encoder_input := 0.U
    message_encoder(i).io.time_encoder_input := 0.U
    message_encoder(i).io.prv_encoder_from_priv_input := 0.U
    message_encoder(i).io.prv_encoder_to_priv_input := 0.U
    message_encoder(i).io.ctx_encoder_input := 0.U
    message_encoder(i).io.msg_type := message_type(i)
    message_encoder(i).io.is_compressed := is_compressed(i)
    message_encoder(i).io.packet_valid := packet_valid(i)
  }
   
  for (i <- 0 until coreParams.nGroups) {
    comp_header(i) := CompressedHeaderType.CNA.asUInt
    header_byte(i) := HeaderByte(FullHeaderType.FReserved)
  }

  /* 
  - oldest instruction -
    ingress_1_group_0
    ingress_1_group_n
    ingress_0_group_0
    ingress_0_group_n
  - youngest instruction -
  */



  val ingress_0_has_message = ingress_0.group.map(g => g.itype =/= TraceItype.ITNothing && g.iretire === 1.U).reduce(_ || _)
  val ingress_0_has_branch = ingress_0.group.map(g => (g.itype === TraceItype.ITBrTaken || g.itype === TraceItype.ITBrNTaken) && g.iretire === 1.U).reduce(_ || _)
  val ingress_0_has_ij = ingress_0.group.map(g => (g.itype === TraceItype.ITInJump) && g.iretire === 1.U).reduce(_ || _)
  val ingress_0_has_flush = ingress_0_has_message && !ingress_0_has_branch && !ingress_0_has_ij
  val ingress_0_msg_idx = PriorityEncoder(ingress_0.group.map(g => g.itype =/= TraceItype.ITNothing && g.iretire === 1.U))
  
  val ingress_1_has_message = ingress_1.group.map(g => g.itype =/= TraceItype.ITNothing && g.iretire === 1.U).reduce(_ || _)
  val ingress_1_has_branch = ingress_1.group.map(g => (g.itype === TraceItype.ITBrTaken || g.itype === TraceItype.ITBrNTaken) && g.iretire === 1.U).reduce(_ || _)
  val ingress_1_has_packet = Mux(is_bp_mode, ingress_1_has_message && !ingress_1_has_branch, ingress_1_has_message)
  val ingress_1_msg_idx = PriorityEncoder(ingress_1.group.map(g => g.itype =/= TraceItype.ITNothing && g.iretire === 1.U))

  val ingress_1_valid_count = PopCount(ingress_1.group.map(g => g.iretire === 1.U))

  val target_addr_msg = Mux(ingress_1_msg_idx === (ingress_1_valid_count - 1.U), // am I the last message?
                            (ingress_1.group(ingress_1_msg_idx).iaddr ^ ingress_0.group(0).iaddr) >> 1.U,
                            (ingress_1.group(ingress_1_msg_idx).iaddr ^ ingress_1.group(ingress_1_msg_idx + 1.U).iaddr) >> 1.U)
  

  metadata_buffer.io.enq.bits := metadata(ingress_1_msg_idx) 
  metadata_buffer.io.enq.valid := packet_valid(ingress_1_msg_idx)

  // buffering compressed packet or full header depending on is_compressed
  byte_buffer.io.enq.bits := Mux(is_compressed(ingress_1_msg_idx), 
                                  Mux(bp_flush_hit, bp_hit_packet, comp_packet),
                                  header_byte(0))
  byte_buffer.io.enq.valid := packet_valid(ingress_1_msg_idx)

  /* message buffering (replaces separated buffers) */
  
  val encoder_outputs = VecInit(message_encoder.map(_.io.message))
  message_buffer.io.enq.bits := encoder_outputs(ingress_1_msg_idx)
  message_buffer.io.enq.valid := !is_compressed(ingress_1_msg_idx) && packet_valid(ingress_1_msg_idx)

  // stall if any buffer is almost full 
  // technically it should always the byte buffer, but just to be safe
  stall := stallThreshold(message_buffer.io.count) // || stallThreshold(target_addr_buffer.io.count) || stallThreshold(time_buffer.io.count) || stallThreshold(byte_buffer.io.count)
  io.stall := stall
  
  val sent = RegInit(false.B)
  // reset takes priority over enqueue
  when (pipeline_advance) {
    sent := false.B
  } .elsewhen (byte_buffer.io.enq.fire) {
    sent := true.B
  }

  // driving branch predictor signals
  bp.io.req_pc := ingress_0.group(ingress_0_msg_idx).iaddr
  bp_inference_valid := ingress_0.group(ingress_0_msg_idx).iretire === 1.U && 
                        (ingress_0.group(ingress_0_msg_idx).itype === TraceItype.ITBrTaken || ingress_0.group(ingress_0_msg_idx).itype === TraceItype.ITBrNTaken) &&
                        pipeline_advance && io.control.enable
  bp.io.update_valid := bp_inference_valid
  bp.io.update_taken := ingress_0.group(ingress_0_msg_idx).itype === TraceItype.ITBrTaken
  bp_hit_count_en := pipeline_advance && io.control.enable
  bp_miss_flag_en := pipeline_advance && io.control.enable

  
  // state machine
  switch (state) {
    is (sIdle) {
      when (io.control.enable) { 
        state := sSync 
        sync_type := SyncType.SyncStart
      }
    }
    is (sSync) {
      message_type(0) := MessageType.Sync
      header_byte(0):= HeaderByte.from_sync_type(FullHeaderType.FSync, sync_type)
      message_encoder(0).io.time_encoder_input := ingress_0.time
      prev_time := ingress_0.time
      // target address
      message_encoder(0).io.target_addr_encoder_input := ingress_0.group(0).iaddr >> 1.U // last bit is always 0
      // prv
      message_encoder(0).io.prv_encoder_from_priv_input := 0b00.U
      message_encoder(0).io.prv_encoder_to_priv_input := ingress_0.priv
      // reuse trap address for runtime_cfg
      val runtime_cfg = Wire(UInt(7.W))
      // 2 bits for bp mode, 6 bits for n_entries
      runtime_cfg := Cat(log2Ceil(outer.bpParams.n_entries/64).U, io.control.bp_mode(1,0))
      message_encoder(0).io.trap_addr_encoder_input := runtime_cfg
      // context
      message_encoder(0).io.ctx_encoder_input := ingress_0.ctx
      is_compressed(0) := false.B
      packet_valid(0) := !sent
      // state transition: wait for message to go in
      state := Mux(pipeline_advance && (sent || byte_buffer.io.enq.fire), Mux(io.control.enable, sData, sIdle), sSync)
    }
    is (sData) {
      when (!io.control.enable) {
        state := sSync
        sync_type := SyncType.SyncEnd
      } .otherwise {
        // ingress0 logic - branch resolution
        when (ingress_0_has_branch) {
          val taken = ingress_0.group(ingress_0_msg_idx).itype === TraceItype.ITBrTaken
          bp_hit_count_next := Mux((bp.io.resp === taken) && is_bp_mode, 
                              bp_hit_count + 1.U, 
                              0.U) // reset if responded with miss
          bp_miss_flag_next := (bp.io.resp =/= taken) && is_bp_mode 
          bp_flush_hit := (bp.io.resp =/= taken) && is_bp_mode && bp_hit_count > 0.U 
        }
        .elsewhen (ingress_0_has_flush) { // these two conditions are mutually exclusive
          bp_flush_hit := is_bp_mode && bp_hit_count > 0.U
          bp_hit_count_next := 0.U
        }
        // ingress1 logic - message encoding
        when (bp_flush_hit && is_bp_mode) {
          // encode hit packet
          header_byte(0) := HeaderByte(FullHeaderType.FTakenBranch)
          comp_header(0) := CompressedHeaderType.CTB.asUInt
          message_encoder(0).io.time_encoder_input := bp_hit_count
          is_compressed(0) := bp_hit_count <= MAX_DELTA_TIME_COMP.U
          packet_valid(0) := !sent && is_bp_mode
          message_type(0) := MessageType.BPHit
        }
        .elsewhen (bp_miss_flag && is_bp_mode) {
          // encode miss packet
          header_byte(0) := HeaderByte(FullHeaderType.FNotTakenBranch)
          comp_header(0) := CompressedHeaderType.CNT.asUInt
          message_encoder(0).io.time_encoder_input := delta_time
          prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
          is_compressed(0) := delta_time <= MAX_DELTA_TIME_COMP.U
          packet_valid(0) := !sent && is_bp_mode
          message_type(0) := MessageType.BPMiss
        }
        .elsewhen (ingress_1_has_message) {
          for (i <- 0 until coreParams.nGroups) {
            switch (ingress_1.group(i).itype) {
              is (TraceItype.ITNothing) {
                packet_valid(i) := false.B
                message_type(i) := DontCare
              }
              is (TraceItype.ITBrTaken) {
                header_byte(i) := HeaderByte(FullHeaderType.FTakenBranch)
                comp_header(i) := CompressedHeaderType.CTB.asUInt
                message_encoder(i).io.time_encoder_input := delta_time
                prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
                is_compressed(i) := delta_time <= MAX_DELTA_TIME_COMP.U
                packet_valid(i) := !sent && is_bt_mode
                message_type(i) := MessageType.Branch
              }
              is (TraceItype.ITBrNTaken) {
                header_byte(i) := HeaderByte(FullHeaderType.FNotTakenBranch)
                comp_header(i) := CompressedHeaderType.CNT.asUInt
                message_encoder(i).io.time_encoder_input := delta_time
                prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
                is_compressed(i) := delta_time <= MAX_DELTA_TIME_COMP.U
                packet_valid(i) := !sent && is_bt_mode
                message_type(i) := MessageType.Branch
              }
              is (TraceItype.ITInJump) {
                header_byte(i) := HeaderByte(FullHeaderType.FInfJump)
                comp_header(i) := CompressedHeaderType.CIJ.asUInt
                message_encoder(i).io.time_encoder_input := delta_time
                prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
                is_compressed(i) := delta_time <= MAX_DELTA_TIME_COMP.U
                packet_valid(i) := !sent && is_bt_mode
                message_type(i) := MessageType.InfJump
              }
              is (TraceItype.ITUnJump) {
                header_byte(i) := HeaderByte(FullHeaderType.FUninfJump)
                message_encoder(i).io.time_encoder_input := delta_time 
                prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
                message_encoder(i).io.target_addr_encoder_input := target_addr_msg
                is_compressed(i) := false.B
                packet_valid(i) := !sent
                message_type(i) := MessageType.UninfJump
              }
              is (TraceItype.ITException) {
                header_byte(i) := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TException)
                comp_header(i) := CompressedHeaderType.CNA.asUInt
                message_encoder(i).io.time_encoder_input := delta_time
                prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
                message_encoder(i).io.target_addr_encoder_input := target_addr_msg
                message_encoder(i).io.trap_addr_encoder_input := ingress_1.group(ingress_1_msg_idx).iaddr >> 1.U
                message_encoder(i).io.prv_encoder_from_priv_input := ingress_1.priv
                message_encoder(i).io.prv_encoder_to_priv_input := ingress_0.priv
                is_compressed(i) := false.B
                packet_valid(i) := !sent
                message_type(i) := MessageType.Trap
              }
              is (TraceItype.ITInterrupt) {
                header_byte(i) := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TInterrupt)
                comp_header(i) := CompressedHeaderType.CNA.asUInt
                message_encoder(i).io.time_encoder_input := delta_time
                prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
                message_encoder(i).io.target_addr_encoder_input := target_addr_msg
                message_encoder(i).io.trap_addr_encoder_input := ingress_1.group(ingress_1_msg_idx).iaddr >> 1.U
                message_encoder(i).io.prv_encoder_from_priv_input := ingress_1.priv
                message_encoder(i).io.prv_encoder_to_priv_input := ingress_0.priv
                is_compressed(i) := false.B
                packet_valid(i) := !sent
                message_type(i) := MessageType.Trap
              }
              is (TraceItype.ITReturn) {
                header_byte(i) := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TReturn)
                comp_header(i) := CompressedHeaderType.CNA.asUInt
                message_encoder(i).io.time_encoder_input := delta_time
                prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
                message_encoder(i).io.target_addr_encoder_input := target_addr_msg
                message_encoder(i).io.trap_addr_encoder_input := ingress_1.group(ingress_1_msg_idx).iaddr >> 1.U
                message_encoder(i).io.prv_encoder_from_priv_input := ingress_1.priv
                message_encoder(i).io.prv_encoder_to_priv_input := ingress_0.priv
                message_encoder(i).io.ctx_encoder_input := ingress_1.ctx
                is_compressed(i) := false.B
                packet_valid(i) := !sent
                message_type(i) := MessageType.Return
              }
            }
          }
          
        }
      }
    }
  }
}