class alu_scoreboard extends uvm_test;
    `uvm_component_utils(alu_scoreboard)
    
    uvm_analysis_imp #(alu_sequence_item, alu_scoreboard) scoreboard_port;
    alu_sequence_item transactions[$];
    
    //
    //Constructor
    //
    function new(string name = "alu_scoreboard", uvm_component parent);
        super.new(name, parent);
        `uvm_info("SCB_CLASS", "Inside Constructor!", UVM_HIGH)
    endfunction: new
    
    //
    //Build Phase
    //
    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("SCB_CLASS", "Build Phase!", UVM_HIGH)
     
        scoreboard_port = new("scoreboard_port", this);
    endfunction: build_phase
    
    
    //
    //Connect Phase
    //
    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("SCB_CLASS", "Connect Phase!", UVM_HIGH)
    endfunction: connect_phase
    
    //
    //Write Method
    //
    function void write(alu_sequence_item item);
        transactions.push_back(item);
    endfunction: write 
    
    //
    //Run Phase
    //
    task run_phase (uvm_phase phase);
        super.run_phase(phase);
        `uvm_info("SCB_CLASS", "Run Phase!", UVM_HIGH)
     
        forever begin
            // get the packet
            // generate expected value
            // compare it with actual value
            // score the transactions
            alu_sequence_item curr_trans;
            wait((transactions.size() != 0));
            curr_trans = transactions.pop_front();
            compare(curr_trans);
            
        end
        
    endtask: run_phase
    
    //
    //Compare : Generate Expected Result and Compare with Actual
    //
    task compare(alu_sequence_item curr_trans);
        logic [31:0] expected;
        
        case(curr_trans.alu_control)
            3'b000: begin // add
                expected = curr_trans.a + curr_trans.b;
            end
            3'b001: begin // subtract
                expected = curr_trans.a - curr_trans.b;
            end
            3'b010: begin // and
                expected = curr_trans.a & curr_trans.b;
            end
            3'b011: begin // or
                expected = curr_trans.a | curr_trans.b;
            end
            3'b100: begin // xor
                expected = curr_trans.a ^ curr_trans.b;
            end 
            3'b101: begin // slt
                // Generates an error intentionally
                expected = 32'd99999999;
            end
            3'b110: begin // sll
                expected = curr_trans.a << curr_trans.b[4:0];
            end 
            3'b111: begin // srl
                expected = curr_trans.a >> curr_trans.b[4:0];
            end 
        endcase
        
        if(curr_trans.result != expected) begin
            `uvm_error("COMPARE", 
                       $sformatf("Transaction Failed! ALUControl %b, a(%0d), b(%0d), got=%0d, expected=%0d",
                       curr_trans.alu_control, curr_trans.a, curr_trans.b, curr_trans.result, expected))
        end
        else begin
            `uvm_info("COMPARE",
                      $sformatf("Transaction Passed! ALUControl %b, a(%0d), b(%0d), got=%0d, expected=%0d",
                      curr_trans.alu_control, curr_trans.a, curr_trans.b, curr_trans.result, expected), UVM_LOW)
        end
        
    endtask: compare
    
endclass: alu_scoreboard
