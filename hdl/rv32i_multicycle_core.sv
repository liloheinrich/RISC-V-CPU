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

//  an example of how to make named inputs for a mux:
/*
    enum logic {MEM_SRC_PC, MEM_SRC_RESULT} mem_src;
    always_comb begin : memory_read_address_mux
      case(mem_src)
        MEM_SRC_RESULT : mem_rd_addr = alu_result;
        MEM_SRC_PC : mem_rd_addr = PC;
        default: mem_rd_addr = 0;
    end
*/

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


always_ff @(posedge clk) begin
  if (rst) begin
    PC_next <= 32'b0;
  end
end

// TODO: set PC. was having an l-value error. Apparently we're not allowed to store state (from an ff) on a wire
// this is why PC_next exists, we may need to wire PC to PC_next?
// always_comb begin
//   PC = PC_next;
// end

always_comb begin
  mem_addr = PC_next; // TODO: not hardwire PC = mem_addr so we can do reads and writes other than INST_RAM
  $display("mem_rd_data %b", mem_rd_data);
end

always_ff @(posedge clk) begin
  case(mem_rd_data[6:0]) 
    OP_RTYPE : begin
      case(mem_rd_data[14:12]) 
        FUNCT3_ADD : begin

          // TODO: make a FSM such that reading and writing from the MMU and ALU are not on the same clock cycle
          // see textbook on multicycle 

          $display("add %b, %b, %b", mem_rd_data[11:7], mem_rd_data[19:15], mem_rd_data[24:20]);

          rs1 <= mem_rd_data[19:15];
          rs2 <= mem_rd_data[24:20];
          $display("register vals %b, %b", reg_data1, reg_data2);

          src_a <= reg_data1;
          src_b <= reg_data2;
          alu_control <= ALU_ADD;
          $display("alu_result %b", alu_result);

          // rd <= mem_rd_data[11:7];
          // rfile_wr_data <= alu_result;
          // reg_write <= 1'b1;

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
