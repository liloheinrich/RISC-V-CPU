`timescale 1ns/1ps
`default_nettype none

`include "alu_types.sv"
`include "rv32i_defines.sv"

module rv32i_multicycle_core(
  clk, rst, ena,
  mem_addr, mem_rd_data, mem_wr_data, mem_wr_ena,
  PC
);

parameter PC_START_ADDRESS=0;

// Standard control signals.
input  wire clk, rst, ena; // <- worry about implementing the ena signal last.

// Memory interface.
output logic [31:0] mem_addr, mem_wr_data;
input   wire [31:0] mem_rd_data;
output logic mem_wr_ena;

logic [31:0] ir;

// Program Counter
output wire [31:0] PC;
wire [31:0] PC_old;
logic PC_ena;
logic [31:0] PC_next; 

// Program Counter Registers
register #(.N(32), .RESET(PC_START_ADDRESS)) PC_REGISTER (
  .clk(clk), .rst(rst), .ena(PC_ena), .d(PC_next), .q(PC)
);
register #(.N(32)) PC_OLD_REGISTER(
  .clk(clk), .rst(rst), .ena(PC_ena), .d(PC), .q(PC_old)
);

// Register file
logic reg_write;
logic [4:0] rd, rs1, rs2;
logic [31:0] rfile_wr_data;
wire [31:0] reg_data1, reg_data2;
register_file REGISTER_FILE(
  .clk(clk), 
  .wr_ena(reg_write), .wr_addr(rd), .wr_data(rfile_wr_data),
  .rd_addr0(rs1), .rd_addr1(rs2),
  .rd_data0(reg_data1), .rd_data1(reg_data2)
);

// ALU and related control signals
// Feel free to replace with your ALU from the homework.
logic [31:0] src_a, src_b;
alu_control_t alu_control;
wire [31:0] alu_result;
wire overflow, zero, equal;
alu_behavioural ALU (
  .a(src_a), .b(src_b), .result(alu_result),
  .control(alu_control),
  .overflow(overflow), .zero(zero), .equal(equal)
);

// Implement your multicycle rv32i CPU here!


//  an example of how to make named inputs for a mux:
/*
    enum logic {MEM_SRC_PC, MEM_SRC_RESULT} mem_src;
    always_comb begin : memory_read_address_mux
      case(mem_src)
        MEM_SRC_RESULT : mem_addr = alu_result;
        MEM_SRC_PC : mem_addr = PC;
        default: mem_addr = 0;
    end
*/
always_comb begin
  mem_addr = PC; // TODO: not hardwire PC = mem_addr so we can do reads and writes other than INST_RAM
end

enum logic [2:0] {FETCH, MEM_ADDR, EXECUTE_R, EXECUTE_I, ALU_WRITEBACK} state;

always_ff @(posedge clk) begin
  if (rst) begin
    // TODO: probably shouldn't do enables in rst
    PC_ena = 1'b1;
    reg_write <= 1'b1; // write enable for register file
    PC_next <= 32'b0;
    state <= FETCH;
  end

  // TODO 11/28: skipping addi 7 again and weird behaviour on non-add commands
  // theory: error with defaults
  // last thing changed: just restructured FSM so look at state routing

  case(state)
    FETCH : begin
      $display("FETCH: mem_rd_data %b", mem_rd_data);
      ir <= mem_rd_data; // ir = instruction register

      $display("FETCH: PC=%d", PC);
      PC_next <= PC + 4;

      state <= MEM_ADDR;
    end
    MEM_ADDR : begin
      case(ir[6:0]) // ir[6:0] is the op code
        OP_RTYPE : begin
          $display("MEM_ADDR OP_RTYPE: rd=%d, rs1=%d, rs2=%d", ir[11:7], ir[19:15], ir[24:20]);
          rs1 <= ir[19:15];
          rs2 <= ir[24:20];
          state <= EXECUTE_R;
        end
        OP_ITYPE : begin
          $display("MEM_ADDR OP_ITYPE: rd=%d, rs1=%d, imm=%d", ir[11:7], ir[19:15], ir[31:20]);
          rs1 <= ir[19:15];
          state <= EXECUTE_I;
        end
        default : begin
          $display("MEM_ADDR optype case: default");
        end
      endcase
    end
    EXECUTE_R : begin
      $display("EXECUTE_R: register rs1 data=%d, rs2 data=%d", reg_data1, reg_data2);
      src_a <= reg_data1;
      src_b <= reg_data2;

      case(ir[14:12]) // ir[14:12] is the funct3 code
        FUNCT3_ADD : begin
          $display("EXECUTE_R case: ADD");
          alu_control <= ALU_ADD;
        end
        FUNCT3_SLL : begin
          $display("EXECUTE_R case: SLL");
          alu_control <= ALU_SLL;
        end
        FUNCT3_SLT : begin
          $display("EXECUTE_R case: SLT");
          alu_control <= ALU_SLT;
        end
        FUNCT3_SLTU : begin
          $display("EXECUTE_R case: SLTU");
          alu_control <= ALU_SLTU;
        end
        FUNCT3_XOR : begin
          $display("EXECUTE_R case: XOR");
          alu_control <= ALU_XOR;
        end
        FUNCT3_SHIFT_RIGHT : begin // Needs a funct7 bit to determine!
          $display("EXECUTE_R case: SHIFT_RIGHT");
          alu_control <= ALU_SRA; // TODO: should this be SRA? SRL? something else?
        end
        FUNCT3_OR : begin
          $display("EXECUTE_R case: OR");
          alu_control <= ALU_OR;
        end
        FUNCT3_AND : begin
          $display("EXECUTE_R case: AND");
          alu_control <= ALU_AND;
        end
        default : begin
          $display("EXECUTE_R case: default");
          alu_control <= ALU_INVALID;
        end
      endcase

      state <= ALU_WRITEBACK;
    end
    EXECUTE_I : begin
      $display("EXECUTE_I: register rs1 data=%d imm=%d", reg_data1, ir[31:20]);
      src_a <= reg_data1;
      src_b <= ir[31:20]; // the immediate imm 
      
      case(ir[14:12]) // ir[14:12] is the funct3 code
        FUNCT3_ADD : begin
          $display("EXECUTE_I case: ADD");
          alu_control <= ALU_ADD;
        end
        FUNCT3_SLL : begin
          $display("EXECUTE_I case: SLL");
          alu_control <= ALU_SLL;
        end
        FUNCT3_SLT : begin
          $display("EXECUTE_I case: SLT");
          alu_control <= ALU_SLT;
        end
        FUNCT3_SLTU : begin
          $display("EXECUTE_I case: SLTU");
          alu_control <= ALU_SLTU;
        end
        FUNCT3_XOR : begin
          $display("EXECUTE_I case: XOR");
          alu_control <= ALU_XOR;
        end
        FUNCT3_SHIFT_RIGHT : begin // Needs a funct7 bit to determine!
          $display("EXECUTE_I case: SHIFT_RIGHT");
          alu_control <= ALU_SRA; // TODO: should this be SRA? SRL? something else?
        end
        FUNCT3_OR : begin
          $display("EXECUTE_I case: OR");
          alu_control <= ALU_OR;
        end
        FUNCT3_AND : begin
          $display("EXECUTE_I case: AND");
          alu_control <= ALU_AND;
        end
        default : begin
          $display("EXECUTE_I case: default");
          alu_control <= ALU_INVALID;
        end
      endcase

      state <= ALU_WRITEBACK;
    end
    ALU_WRITEBACK : begin
      $display("ALU_WRITEBACK: alu_result=%d", alu_result);
      rd <= ir[11:7];
      rfile_wr_data <= alu_result;
      state <= FETCH;
    end
    default : begin
      $display("state case: default");
    end
  endcase

end

endmodule
