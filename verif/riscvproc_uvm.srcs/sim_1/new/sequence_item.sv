//Object class
class alu_sequence_item extends uvm_sequence_item;
    `uvm_object_utils(alu_sequence_item)

    //
    //Instantiation
    //
    rand logic [31:0] a;
    rand logic [31:0] b;
    rand logic [2:0]  alu_control;

    logic [31:0]      result;
    logic             zero;
    
    //
    //Default Constraints
    //
    constraint input1_c {a inside {[10:20]};} // Set range between 10 to 20
    constraint input2_c {b inside {[0:10]};}
    constraint alu_control_c {alu_control inside {[3'b000:3'b111]};}
    
    //
    //Constructor
    //
    function new(string name = "alu_sequence_item");
        super.new(name);
    endfunction: new

endclass: alu_sequence_item
