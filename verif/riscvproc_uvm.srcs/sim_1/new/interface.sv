interface alu_interface(input logic clock); 
    logic [31:0] a;
    logic [31:0] b;
    logic [2:0]  alu_control;
    logic [31:0] result;
    logic        zero;

endinterface: alu_interface
