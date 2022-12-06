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

// memory mux:
enum logic {MEM_SRC_PC, MEM_SRC_RESULT} mem_src;
always_comb begin : memory_read_address_mux
  case(mem_src)
    MEM_SRC_RESULT : mem_addr = alu_result;
    MEM_SRC_PC : mem_addr = PC;
    default: mem_addr = 0;
  endcase
end

enum logic [3:0] {FETCH, MEM_ADDR, EXECUTE_R, EXECUTE_I, EXECUTE_L, EXECUTE_S, TURN_OFF_WRITE_S, EXECUTE_JAL, ALU_WRITEBACK} state;

always_ff @(posedge clk) begin
  if (rst) begin
    // TODO: probably shouldn't do enables in rst
    PC_ena = 1'b1;
    reg_write <= 1'b1; // write enable for register file
    PC_next <= 32'b0;
    mem_src <= MEM_SRC_PC;
    state <= FETCH;
  end

  case(state)
    FETCH : begin
      $display("");
      $display("FETCH: mem_rd_data %b", mem_rd_data);
      ir <= mem_rd_data; // ir = instruction register

      $display("FETCH: PC=%d", PC);
      PC_next <= PC + 4;

      case(mem_rd_data[6:0]) // mem_rd_data[6:0] is the op code
        OP_RTYPE : begin
          $display("OP_RTYPE: rd=%d, rs1=%d, rs2=%d", mem_rd_data[11:7], mem_rd_data[19:15], mem_rd_data[24:20]);
          rs1 <= mem_rd_data[19:15];
          rs2 <= mem_rd_data[24:20];
          state <= EXECUTE_R;
        end
        OP_ITYPE : begin
          $display("OP_ITYPE: rd=%d, rs1=%d, imm=%d", mem_rd_data[11:7], mem_rd_data[19:15], mem_rd_data[31:20]);
          rs1 <= mem_rd_data[19:15];
          state <= EXECUTE_I;
        end
        OP_LTYPE : begin
          $display("OP_LTYPE: rd=%d, rs1=%d, imm=%d", mem_rd_data[11:7], mem_rd_data[19:15], mem_rd_data[31:20]);
          case(mem_rd_data[14:12]) // case on funct3 code
            FUNCT3_LOAD_LW : begin
              src_a <= mem_rd_data[19:15];
              src_b <= mem_rd_data[31:20];
              alu_control <= ALU_ADD;
            end
          endcase
          mem_src <= MEM_SRC_RESULT;
          state <= EXECUTE_L;
        end
        OP_STYPE : begin
          $display("OP_STYPE: rs2=%d, rs1=%d, imm=%d", mem_rd_data[24:20], mem_rd_data[19:15], {mem_rd_data[31:25], mem_rd_data[11:7]});
          src_a <= mem_rd_data[19:15];
          src_b <= {mem_rd_data[31:25], mem_rd_data[11:7]};
          alu_control <= ALU_ADD;
          rs2 <= mem_rd_data[24:20]; 
          state <= EXECUTE_S;
        end
        OP_JAL : begin
          $display("OP_JAL: rd=%d, imm=%d", mem_rd_data[11:7], mem_rd_data[31:12]);
          // calculate address to jump to
          src_a <= PC;
          // src_b <= mem_rd_data[31:12];
          src_b <= {mem_rd_data[31], mem_rd_data[19:12], mem_rd_data[20], mem_rd_data[30:21]};
          alu_control <= ALU_ADD;

          // write address + 4 to return
          rd <= mem_rd_data[11:7];
          rfile_wr_data <= PC + 4;

          state <= EXECUTE_JAL;
        end
        default : begin
          $display("optype: default (ERROR)");
        end
      endcase
 
      if (mem_rd_data[6:0] == OP_RTYPE | mem_rd_data[6:0] == OP_ITYPE) begin
        case(mem_rd_data[14:12]) // ir[14:12] is the funct3 code
          FUNCT3_ADD : begin
            alu_control <= ALU_ADD; // TODO: fix subtract sub which uses funct7 to determine
          end
          FUNCT3_SLL : begin
            alu_control <= ALU_SLL;
          end
          FUNCT3_SLT : begin
            alu_control <= ALU_SLT;
          end
          FUNCT3_SLTU : begin
            alu_control <= ALU_SLTU;
          end
          FUNCT3_XOR : begin
            alu_control <= ALU_XOR;
          end
          FUNCT3_SHIFT_RIGHT : begin // TODO: fix Shift_right which uses funct7 to determine
            alu_control <= ALU_SRA;
          end
          FUNCT3_OR : begin
            alu_control <= ALU_OR;
          end
          FUNCT3_AND : begin
            alu_control <= ALU_AND;
          end
          default : begin
            alu_control <= ALU_INVALID;
          end
        endcase
      end
    end
    EXECUTE_R : begin
      $display("EXECUTE_R: alu_control=%s", alu_control_name(alu_control));
      $display("EXECUTE_R: register rs1 data=%d, rs2 data=%d", reg_data1, reg_data2);
      src_a <= reg_data1;
      src_b <= reg_data2;
      state <= ALU_WRITEBACK;
    end
    EXECUTE_I : begin
      $display("EXECUTE_I: alu_control=%s", alu_control_name(alu_control));
      $display("EXECUTE_I: register rs1 data=%d imm=%d", reg_data1, ir[31:20]);
      src_a <= reg_data1;
      src_b <= ir[31:20]; // the immediate imm 
      state <= ALU_WRITEBACK;
    end
    EXECUTE_L : begin
      $display("EXECUTE_L: imm(rs1) data=%d", mem_rd_data);
      rd <= ir[11:7]; 
      rfile_wr_data <= mem_rd_data;
      mem_src <= MEM_SRC_PC; // switch mem_src back to PC preemptively so that we can read the next instruction
      state <= FETCH;
    end
    EXECUTE_S : begin
      mem_src <= MEM_SRC_RESULT;
      mem_wr_data <= reg_data2;
      mem_wr_ena <= 1'b1; // write enable needs to be turned off after write to be able to read again
      state <= TURN_OFF_WRITE_S;
    end
    TURN_OFF_WRITE_S : begin
      mem_src <= MEM_SRC_PC; // switch mem_src back to PC preemptively so that we can read the next instruction
      mem_wr_ena <= 1'b0; // write enable needs to be turned off after write to be able to read again
      state <= FETCH;
    end
    EXECUTE_JAL : begin
      PC_next <= alu_result;
      state <= FETCH;
    end
    ALU_WRITEBACK : begin
      $display("ALU_WRITEBACK: alu_result=%d", alu_result);
      rd <= ir[11:7];
      rfile_wr_data <= alu_result;
      state <= FETCH;
    end
    default : begin
      $display("state case: default (ERROR)");
    end
  endcase

end

endmodule
