`timescale 1ns/1ps
`default_nettype none

module sign_extend(in, out);

input wire signed [N-1:0] in;
output logic signed [N-1:0] out;

always_comb begin : sign_ext
  
end

endmodule
