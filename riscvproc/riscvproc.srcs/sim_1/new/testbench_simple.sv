`timescale 1ns/1ns

module testbench(
);
    logic        clk;
    logic        rst_n;
    logic [31:0] write_data; 
    logic [31:0] mem_addr; 
    logic        mem_write;
    logic [7:0]  cycles;

    // instantiate device to be tested
    top dut(clk, rst_n, write_data, mem_addr, mem_write);

    // initialize test 
    initial begin
        cycles <= 0;
        rst_n <= 1; # 5; rst_n <= 0; 
    end

    // generate clock to sequence tests
    always begin
        clk <= 1; # 5; clk <= 0; # 5;
        cycles <= cycles + 1; 
    end

    // check results 
    always @(negedge clk) begin
        if(mem_write) begin
            if(mem_addr === 100 & write_data === 32'hAAAAA02E) begin
                $display("Simulation success");
                $stop; 
            end
        end 
        if (cycles >= 50) begin
            $display("Simulation failed, max cycles reached");
            $stop;
        end
    end
endmodule