// ALU Verification
`timescale 1ns/1ns

// Remove FILE and LINE info from UVM prints
`define UVM_REPORT_DISABLE_FILE_LINE 

import uvm_pkg::*;
`include "uvm_macros.svh"

//
//Include Files
//
`include "interface.sv"
`include "sequence_item.sv"
`include "sequence.sv"
`include "sequencer.sv"
`include "driver.sv"
`include "monitor.sv"
`include "agent.sv"
`include "scoreboard.sv"
`include "env.sv"
`include "test.sv"

module top_tb;

    //
    //Instantiation
    //
    logic clock;
    
    alu_interface intf(.clock(clock));

    alu dut(.a(intf.a),
                    .b(intf.b),
                    .alu_control(intf.alu_control),
                    .result(intf.result),
                    .zero(intf.zero)
                    );
    
    //
    //Interface Setting
    //
    initial begin
        uvm_config_db #(virtual alu_interface)::set(null, "*", "vif", intf );
        //https://www.synopsys.com/content/dam/synopsys/services/whitepapers/hierarchical-testbench-configuration-using-uvm.pdf
    end
    
    //
    //Start The Test
    //
    initial begin
        run_test("alu_test");
    end 
    
    //
    //Clock Generation
    //
    initial begin
        clock = 0;
        #10;
        forever begin
            clock = ~clock;
            #5;
        end
    end
    
    //
    //Maximum Simulation Time
    //
    initial begin
        #5000;
        $display("Err - ran out of clock cycles!");
        $finish();
    end
    
    //
    //Generate Waveforms
    //
    initial begin
        $dumpfile("d.vcd");
        $dumpvars();
    end
    
endmodule: top_tb
