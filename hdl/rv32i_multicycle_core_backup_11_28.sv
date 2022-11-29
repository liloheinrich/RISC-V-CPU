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

enum logic [2:0] {READ_PARSE, ALU_INPUT, WRITE} state;

always_ff @(posedge clk) begin
  if (rst) begin
    PC_ena = 1'b1;
    PC_next <= 32'b0;
    state <= READ_PARSE;
  end

  case(mem_rd_data[6:0]) // mem_rd_data[6:0] is the op code
    OP_RTYPE : begin
      case(mem_rd_data[14:12]) // mem_rd_data[14:12] is the funct3 code
        FUNCT3_ADD : begin

          case(state)
            READ_PARSE : begin
              $display("READ_PARSE: mem_rd_data %b", mem_rd_data);
              ir <= mem_rd_data;

              $display("READ_PARSE: add rd=%b, rs1=%b, rs2=%b", mem_rd_data[11:7], mem_rd_data[19:15], mem_rd_data[24:20]);
              rs1 <= mem_rd_data[19:15];
              rs2 <= mem_rd_data[24:20];
              state <= ALU_INPUT;

              $display("INC_PC: PC=%b", PC);
              PC_next <= PC + 4;
            end
            ALU_INPUT : begin
              $display("ALU_INPUT: register rs1=%b, rs2=%b", reg_data1, reg_data2);
              src_a <= reg_data1;
              src_b <= reg_data2;
              alu_control <= ALU_ADD;
              state <= WRITE;
            end
            WRITE : begin
              $display("WRITE: alu_result=%b", alu_result);
              rd <= ir[11:7];
              rfile_wr_data <= alu_result;
              reg_write <= 1'b1;
              state <= READ_PARSE;
            end
            default : begin
              $display("case: default");
            end
          endcase

        end
        default : begin
          $display("mem_rd_data[14:12] case: default");
        end
      endcase
    end
    OP_ITYPE : begin

      case(mem_rd_data[14:12]) // mem_rd_data[14:12] is the funct3 code
        FUNCT3_ADD : begin

          case(state)
            READ_PARSE : begin
              $display("READ_PARSE: mem_rd_data %b", mem_rd_data);
              ir <= mem_rd_data;

              $display("READ_PARSE: addi rd=%b, rs1=%b, imm=%b", mem_rd_data[11:7], mem_rd_data[19:15], mem_rd_data[31:20]);
              rs1 <= mem_rd_data[19:15];
              state <= ALU_INPUT;

              $display("INC_PC: PC=%b", PC);
              PC_next <= PC + 4;

            end
            ALU_INPUT : begin
              $display("ALU_INPUT: register rs1=%b", reg_data1);
              src_a <= reg_data1;
              src_b <= mem_rd_data[31:20]; // the immediate imm 
              alu_control <= ALU_ADD;
              state <= WRITE;
            end
            WRITE : begin
              $display("WRITE: alu_result=%b", alu_result);
              rd <= ir[11:7];
              rfile_wr_data <= alu_result;
              reg_write <= 1'b1;
              state <= READ_PARSE;
            end
            default : begin
              $display("case: default");
            end
          endcase

        end
        default : begin
          $display("mem_rd_data[14:12] case: default");
        end
      endcase
    end
    default : begin
      $display("mem_rd_data[6:0] case: default");
    end
  endcase
end

// mem_rd_data, mem_wr_data, mem_wr_ena
// output logic [31:0] mem_addr, mem_wr_data;
// input   wire [31:0] mem_rd_data;
// output logic mem_wr_ena;

endmodule
