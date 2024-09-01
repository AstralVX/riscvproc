`timescale 1ns/1ns

// Top module with processor and simulated memories
module top(
    input  logic        clk_i, 
    input  logic        rst_ni,
    output logic [31:0] write_data_m, 
    output logic [31:0] mem_addr_m, 
    output logic        mem_write_m
);
    logic [31:0] pc_f;        // Current PC in fetch stage
    logic [31:0] instr_f;     // Instruction in fetch stage
    logic [31:0] read_data_m; // Data memory result that has been read

    // Instantiate processor and memories
    riscv riscv(clk_i, rst_ni, pc_f, instr_f, mem_write_m, mem_addr_m, write_data_m, read_data_m);
    imem imem(pc_f, instr_f);
    dmem dmem(clk_i, mem_write_m, mem_addr_m, write_data_m, read_data_m); 
endmodule

// Pipelined RISC-V processor
module riscv(   
    input  logic        clk_i, 
    input  logic        rst_ni, 
    output logic [31:0] pc_f,
    input  logic [31:0] instr_f, 
    output logic        mem_write_m,
    output logic [31:0] alu_result_m,
    output logic [31:0] write_data_m,
    input  logic [31:0] read_data_m
);
    logic [6:0] op_d;
    logic [2:0] funct3_d; 
    logic       funct7b5_d; 
    logic [2:0] imm_src_d; 
    logic       zero_e;
    logic       pc_src_e; 
    logic [2:0] alu_control_e;
    logic       alu_src_a_e, alu_src_b_e;
    logic       result_src_b0_e;
    logic       reg_write_m; 
    logic [1:0] result_src_w; 
    logic       reg_write_w;
    logic [1:0] forward_a_e, forward_b_e;
    logic       stall_f, stall_d, flush_d, flush_e;
    logic [4:0] rs1_d, rs2_d, rs1_e, rs2_e, rd_e, rd_m, rd_w;

    controller c(clk_i, rst_ni, op_d, funct3_d, funct7b5_d, imm_src_d, 
                 flush_e, zero_e, pc_src_e, alu_control_e, alu_src_a_e, alu_src_b_e, result_src_b0_e, 
                 mem_write_m, reg_write_m, 
                 reg_write_w, result_src_w);

    datapath dp(clk_i, rst_ni, stall_f, instr_f, pc_f, 
                stall_d, flush_d, imm_src_d, op_d, funct3_d, funct7b5_d, 
                flush_e, forward_a_e, forward_b_e, pc_src_e, alu_control_e, alu_src_a_e, alu_src_b_e, zero_e,
                mem_write_m, read_data_m, write_data_m, alu_result_m, 
                reg_write_w, result_src_w, 
                rs1_d, rs2_d, rs1_e, rs2_e, rd_e, rd_m, rd_w);

    hazard hu(rs1_d, rs2_d, rs1_e, rs2_e, rd_e, rd_m, rd_w,
              pc_src_e, result_src_b0_e, 
              reg_write_m, reg_write_w, 
              forward_a_e, forward_b_e, 
              stall_f, stall_d, flush_d, flush_e);
endmodule

// Control path - path of logic that manages the flow of data through the datapath using pipeline control registers
module controller(
    input  logic       clk_i,
    input  logic       rst_ni,
    // Decode stage control signals 
    input  logic [6:0] op_d,            // 7-bit opcode field from the instruction
    input  logic [2:0] funct3_d,        // Determines ALU operation
    input  logic       funct7b5_d,      // Determines ALU operation
    output logic [2:0] imm_src_d,       // Determines width of immediate to extend
    // Execute stage control signals 
    input  logic       flush_e,         // Flush registerE
    input  logic       zero_e,          // ALU flag if zero result
    output logic       pc_src_e,        // Determines the next PC source, from branch taken or PC+4
    output logic [2:0] alu_control_e,   // Determines ALU operation
    output logic       alu_src_a_e,     // 
    output logic       alu_src_b_e,     //
    output logic       result_src_b0_e, // Bit 0 of resultSrcE, indicates if its a Load operation
    // Memory stage control signals
    output logic       mem_write_m,     // Write enable signal for Data memory
    output logic       reg_write_m,     // Write enable signal for Register file
    // Writeback stage control signals
    output logic       reg_write_w,     // Write enable signal for Register file
    output logic [1:0] result_src_w     // Determines result of Writeback stage
);
    // Pipelined control signals
    logic       reg_write_d;
    logic       reg_write_e;
    logic [1:0] result_src_d; 
    logic [1:0] result_src_e; 
    logic [1:0] result_src_m; 
    logic       mem_write_d;
    logic       mem_write_e;
    logic       jump_d;
    logic       jump_e;
    logic       branch_d; 
    logic       branch_e; 
    logic [1:0] alu_op_d;
    logic [2:0] alu_control_d; 
    logic       alu_src_a_d;
    logic       alu_src_b_d;

    // Decode stage logic in control unit
    maindec md(op_d, result_src_d, mem_write_d, branch_d, alu_src_a_d, alu_src_b_d, reg_write_d, jump_d, imm_src_d, alu_op_d); 
    aludec ad(op_d[5], funct3_d, funct7b5_d, alu_op_d, alu_control_d);

    // Execute stage pipeline control register and logic 
    floprc #(11) controlregE(clk_i, rst_ni, flush_e,
                             {reg_write_d, result_src_d, mem_write_d, jump_d, branch_d, alu_control_d, alu_src_a_d, alu_src_b_d},
                             {reg_write_e, result_src_e, mem_write_e, jump_e, branch_e, alu_control_e, alu_src_a_e, alu_src_b_e});

    assign pc_src_e = (branch_e & zero_e) | jump_e; 
    assign result_src_b0_e = result_src_e[0];

    // Memory stage pipeline control register 
    flopr #(4) controlregM(clk_i, rst_ni,
                           {reg_write_e, result_src_e, mem_write_e},
                           {reg_write_m, result_src_m, mem_write_m});

    // Writeback stage pipeline control register 
    flopr #(3) controlregW(clk_i, rst_ni,
                           {reg_write_m, result_src_m},
                           {reg_write_w, result_src_w});
endmodule

// Decodes the opcode into a series of signals to control the datapath
module maindec(
    input  logic [6:0]  op,         // 7-bit opcode field from the instruction
    output logic [1:0]  result_src, // Controls result src mux in stage Writeback
    output logic        mem_write,  // Controls Write Enable for data memory
    output logic        branch,     // Indicates a branch related instruction
    output logic        alu_src_a,  // Controls ALU source A
    output logic        alu_src_b,  // Controls ALU source B
    output logic        reg_write,  // Controls WE3 in register file from stage Writeback
    output logic        jump,       // Forces the PC mux to take jump addr
    output logic [2:0]  imm_src,    // Controls width of sign extensions
    output logic [1:0]  alu_op      // Conntrols ALU operation: 00=LW/SW/AUIPC, 01=branch, 10=Rtype/Itype, 11=JAL/LUI
);
    // Controls are the 13 internal signals
    logic [12:0] controls;

    // Combinationally assign all internal control signals out to controller
    assign {reg_write, imm_src, alu_src_a, alu_src_b, mem_write, result_src, branch, alu_op, jump} = controls;

    // Set control signals based on opcode
    always_comb
        case(op)
            7'b0000011: controls = 13'b1_000_0_1_0_01_0_00_0; // lw
            7'b0100011: controls = 13'b0_001_0_1_1_00_0_00_0; // sw 
            7'b0110011: controls = 13'b1_xxx_0_0_0_00_0_10_0; // R-type: add, sub, sll, slt, sltu, xor, srl, sra, or, and
            7'b1100011: controls = 13'b0_010_0_0_0_00_1_01_0; // beq
            7'b0010011: controls = 13'b1_000_0_1_0_00_0_10_0; // I-type ALU
            7'b1101111: controls = 13'b1_011_0_0_0_10_0_00_1; // jal
            7'b0110111: controls = 13'b1_100_1_1_0_00_0_00_0; // lui
            7'b0000000: controls = 13'b0_000_0_0_0_00_0_00_0; // Reset values
            default:    controls = 13'bx_xxx_x_x_x_xx_x_xx_x; // Non-implemented instruction
        endcase 
endmodule

// Decodes the operation and other inputs to a ALU control value used to drive ALU
module aludec(
    input  logic       opb5,        // Opcode bit 5
    input  logic [2:0] funct3,      // Determines if R-type or I-type ALU operation
    input  logic       funct7b5,    // Determines if R-type subtract instruction
    input  logic [1:0] alu_op,      // 00 = add used for address calculation, 01 = sub of numbers, 10 = R-type ALU operation
    output logic [2:0] alu_control  // Signal used for ALU
);
    logic r_type_sub;
    
    assign r_type_sub = funct7b5 & opb5; // TRUE for R-type subtract instruction

    // Create signal to drive ALU
    always_comb 
        case(alu_op)
            2'b00:                  alu_control = 3'b000; // Addition
            2'b01:                  alu_control = 3'b001; // Subtraction 
            default: case(funct3)                         // R-type or I-type ALU
                        3'b000: if (r_type_sub)
                                    alu_control = 3'b001; // sub 
                                else
                                    alu_control = 3'b000; // add, addi 
                        3'b010:     alu_control = 3'b101; // slt, slti 
                        3'b100:     alu_control = 3'b100; // xor 
                        3'b110:     alu_control = 3'b011; // or, ori 
                        3'b111:     alu_control = 3'b010; // and, andi 
                        default:    alu_control = 3'bxxx; // ???
                     endcase
        endcase 
endmodule

// ALU in stage Execute
module alu(
    input  logic [31:0] a,           // Source A
    input  logic [31:0] b,           // Source B
    input  logic [2:0]  alu_control, // Control signal that drives calculation
    output logic [31:0] result,      // Result of ALU operation
    output logic        zero         // Zero flag
);
    logic [31:0] cond_inv_b;    // Post-processed b, conditionally inverted
    logic [31:0] sum;           // 2's complement result
    logic        v;             // Overflow flag
    logic        is_add_sub;    // True when is add or subtract operation

    assign cond_inv_b = alu_control[0] ? ~b : b;    // alu_control[Bit 0] if set is subtraction operation by aludec
    assign sum = a + cond_inv_b + alu_control[0];   // Sum via 2's complement + operation, if B is inverted (alu_control[0] used as +1) resulting in a subtraction
    assign is_add_sub = ~alu_control[2] & ~alu_control[1] | ~alu_control[1] & alu_control[0];

    // Do ALU calculation
    always_comb
        case (alu_control)
            3'b000: result = sum;         // add
            3'b001: result = sum;         // subtract 
            3'b010: result = a & b;       // and 
            3'b011: result = a | b;       // or 
            3'b100: result = a ^ b;       // xor 
            3'b101: result = sum[31] ^ v; // slt 
            3'b110: result = a << b[4:0]; // sll 
            3'b111: result = a >> b[4:0]; // srl 
            default: result = 32'bx;
        endcase

    assign zero = (result == 32'b0); // Set zero flag
    assign v = ~(alu_control[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & is_add_sub; // Set overflow flag
endmodule

// Datapath - path on which computations occurs; contains internal buses, ALU, pipeline registers, and mux's to control data flow
module datapath(
    input  logic        clk_i,         // Clock
    input  logic        rst_ni,        // Reset async
    // Fetch stage signals
    input  logic        stall_f,       // Stall the fetch stage
    input  logic [31:0] instr_f,       // Instruction fetched instruction memory based on PC address 
    output logic [31:0] pc_f,          // Current pipeline PC address to execute
    // Decode stage signals            // 
    input  logic        stall_d,       // Stall the decode stage
    input  logic        flush_d,       // Flush the decode stage
    input  logic [2:0]  imm_src_d,     // Select sign extension width
    output logic [6:0]  op_d,          // Opcode extracted from instruction
    output logic [2:0]  funct3_d,      // Selects ALU operation type
    output logic        funct7b5_d,    // Selects ALU operation type
    // Execute stage signals
    input  logic        flush_e,       // Flush the execute stage
    input  logic [1:0]  forward_a_e,   // Forward into ALU source A from memory or writeback stage 
    input  logic [1:0]  forward_b_e,   // Forward into ALU source B from memory or writeback stage
    input  logic        pc_src_e,      // Selects PC source, from branch or PC+4
    input  logic [2:0]  alu_control_e, // Selects ALU arithmetic operation
    input  logic        alu_src_a_e,   // Selects ALU source A
    input  logic        alu_src_b_e,   // Selects ALU source B
    output logic        zero_e,        // Zero flag from ALU operation
    // Memory stage signals
    input  logic        mem_write_m,   // Data memory write enable in memory stage
    input  logic [31:0] read_data_m,   // Result of data memory address to read
    output logic [31:0] write_data_m,  // Data memory address to write
    output logic [31:0] alu_result_m,  // Data memory address to read
    // Writeback stage signals
    input  logic        reg_write_w,   // Register file write enable in writeback stage
    input  logic [1:0]  result_src_w,  // Selects the final output source in writeback stage
    // Hazard Unit only signals
    output logic [4:0]  rs1_d,         // Source 1 register of instruction in decode stage
    output logic [4:0]  rs2_d,         // Source 2 register of instruction in decode stage
    output logic [4:0]  rs1_e,         // Source 1 register of instruction in execute stage
    output logic [4:0]  rs2_e,         // Source 2 register of instruction in execute stage
    output logic [4:0]  rd_e,          // Destination register in execute stage
    output logic [4:0]  rd_m,          // Destination register in memory stage
    output logic [4:0]  rd_w           // Destination register in writeback stage
);
    // Fetch stage signals
    logic [31:0] pc_next_f;
    logic [31:0] pc_plus4_f;
    // Decode stage signals 
    logic [31:0] instr_d;
    logic [31:0] pc_d; 
    logic [31:0] pc_plus4_d;
    logic [31:0] rd1_d;
    logic [31:0] rd2_d;
    logic [31:0] imm_ext_d;
    logic [4:0]  rd_d;
    // Execute stage signals 
    logic [31:0] rd1_e;
    logic [31:0] rd2_e;
    logic [31:0] pc_e; 
    logic [31:0] imm_ext_e; 
    logic [31:0] src_a_e; 
    logic [31:0] src_b_e; 
    logic [31:0] src_a_forward_e;
    logic [31:0] alu_result_e;
    logic [31:0] write_data_e;
    logic [31:0] pc_plus4_e;
    logic [31:0] pc_target_e;
    // Memory stage signals 
    logic [31:0] pc_plus4_m;
    // Writeback stage signals 
    logic [31:0] alu_result_w;
    logic [31:0] read_data_w;
    logic [31:0] pc_plus4_w;
    logic [31:0] result_w;

    // Fetch stage pipeline register and logic
    mux2    #(32) pc_mux(pc_plus4_f, pc_target_e, pc_src_e, pc_next_f); 
    flopenr #(32) pc_reg(clk_i, rst_ni, ~stall_f, pc_next_f, pc_f); 
    adder         pc_add(pc_f, 32'h4, pc_plus4_f);

    // Decode stage pipeline register and logic 
    flopenrc #(96) regD(clk_i, rst_ni, flush_d, ~stall_d,
                        {instr_f, pc_f, pc_plus4_f},
                        {instr_d, pc_d, pc_plus4_d});

    assign op_d = instr_d[6:0];
    assign funct3_d = instr_d[14:12]; 
    assign funct7b5_d = instr_d[30];
    assign rs1_d = instr_d[19:15]; 
    assign rs2_d = instr_d[24:20]; 
    assign rd_d = instr_d[11:7];

    regfile rf(clk_i, reg_write_w, rs1_d, rs2_d, rd_w, result_w, rd1_d, rd2_d); 
    extend  sign_ext(instr_d[31:7], imm_src_d, imm_ext_d);

    // Execute stage pipeline register and logic 
    floprc #(175) regE(clk_i, rst_ni, flush_e,
                       {rd1_d, rd2_d, pc_d, rs1_d, rs2_d, rd_d, imm_ext_d, pc_plus4_d},
                       {rd1_e, rd2_e, pc_e, rs1_e, rs2_e, rd_e, imm_ext_e, pc_plus4_e});

    mux3 #(32) forward_ae_mux(rd1_e, result_w, alu_result_m, forward_a_e, src_a_forward_e); 
    mux2 #(32) srca_mux(src_a_forward_e, 32'b0, alu_src_a_e, src_a_e);
    mux3 #(32) forward_be_mux(rd2_e, result_w, alu_result_m, forward_b_e, write_data_e); 
    mux2 #(32) srcb_mux(write_data_e, imm_ext_e, alu_src_b_e, src_b_e);
    alu        alu(src_a_e, src_b_e, alu_control_e, alu_result_e, zero_e); 
    adder      branch_adder(imm_ext_e, pc_e, pc_target_e);

    // Memory stage pipeline register 
    flopr #(101) regM(clk_i, rst_ni,
                      {alu_result_e, write_data_e, rd_e, pc_plus4_e},
                      {alu_result_m, write_data_m, rd_m, pc_plus4_m});

    // Writeback stage pipeline register and logic 
    flopr #(101) regW(clk_i, rst_ni,
                      {alu_result_m, read_data_m, rd_m, pc_plus4_m},
                      {alu_result_w, read_data_w, rd_w, pc_plus4_w});

    mux3 #(32) result_mux(alu_result_w, read_data_w, pc_plus4_w, result_src_w, result_w);
endmodule

// Hazard Unit dealing with data and control hazards, solving via forward, stall, and flush
module hazard(
    input  logic [4:0] rs1_d,           // Source register 1 in decode stage, extracted from instruction
    input  logic [4:0] rs2_d,           // Source register 2 in decode stage, extracted from instruction 
    input  logic [4:0] rs1_e,           // Source register 1 in execute stage
    input  logic [4:0] rs2_e,           // Source register 2 in execute stage
    input  logic [4:0] rd_e,            // Dest register in execute stage
    input  logic [4:0] rd_m,            // Dest register in memory stage
    input  logic [4:0] rd_w,            // Dest register in writeback stage
    input  logic       pc_src_e,        // PC src select, from branch or PC+4
    input  logic       result_src_b0_e, // Bit 0 of resultSrcE, indicates if its a Load operation
    input  logic       reg_write_m,     // Register write enable in memory stage
    input  logic       reg_write_w,     // Register write enable in writeback stage
    output logic [1:0] forward_a_e,     // Select initial ALU srcA 
    output logic [1:0] forward_b_e,     // Select initial ALU srcB
    output logic       stall_f,         // Stall fetch stage pipeline register
    output logic       stall_d,         // Stall decode stage pipeline register
    output logic       flush_d,         // Flush decode stage pipeline register
    output logic       flush_e          // Flush execute stage pipeline register
); 
    logic lw_stall_d;

    // Forward to solve data hazards when possible
    // Hazard: if RS1 or RS2 (execute) == RD (memory or writeback) and regwrite (memory or writeback)
    //         forward ALUResult (memory) or Result (writeback) into ALU source A and B
    always_comb begin
        forward_a_e = 2'b00; 
        forward_b_e = 2'b00; 
        if (rs1_e != 5'b0)
            // If both memory and writeback stage contains dst that matches either src reg, then mem is taken as it's most recent
            if      ((rs1_e == rd_m) & reg_write_m) forward_a_e = 2'b10; 
            else if ((rs1_e == rd_w) & reg_write_w) forward_a_e = 2'b01;
        if (rs2_e != 5'b0)
            if      ((rs2_e == rd_m) & reg_write_m) forward_b_e = 2'b10; 
            else if ((rs2_e == rd_w) & reg_write_w) forward_b_e = 2'b01;
    end

    // Stall when a load hazard occurs
    // Hazard: load instruction does not finish reading data until end of (memory), but subsequent instructions may immediately depend on it
    //         stall the following dependent instructions in (decode), by stalling the previous pipeline registers
    //         also need to flushE the following pipeline register to prevent bogus data from propagating forward
    assign lw_stall_d = result_src_b0_e & ((rs1_d == rd_e) | (rs2_d == rd_e)); 
    assign stall_d = lw_stall_d;
    assign stall_f = lw_stall_d;

    // Flush when a branch is taken or a load introduces a bubble
    // Hazard: branch e.g. BEQ causes a hazard, pipeline does not know which instruction to fetch next as branch decision has not been made yet
    //         if branch taken indicated by pc_src_e, flush the (decode and execute) pipeline registers as we no longer need to execute them
    assign flush_d = pc_src_e;
    assign flush_e = lw_stall_d | pc_src_e; 
endmodule

// Three ported register file
module regfile(
    input  logic        clk_i,
    input  logic        we3,
    input  logic [4:0]  a1, 
    input  logic [4:0]  a2, 
    input  logic [4:0]  a3, 
    input  logic [31:0] wd3,
    output logic [31:0] rd1,
    output logic [31:0] rd2
);
    logic [31:0] rf[31:0];

    // Write third port on falling edge of clock (start, writeback)
    always_ff @(negedge clk_i) 
        if (we3) rf[a3] <= wd3;

    // Read two ports combinationally on rising edge of clock (end, decode)
    // Read register 0 hardwired to 0
    assign rd1 = (a1 != 0) ? rf[a1] : 0; 
    assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

// Sign extend unit
module extend(
    input  logic [31:7] instr,   // 
    input  logic [2:0]  imm_src, // Selects the type of instruction passed into instr
    output logic [31:0] imm_ext  // Resulting sign extended value
);
    // Sign extend the immediate based on ImmSrc type instruction
    // e.g. I-type has 12-bit signed immediate
    //      B-type has 13-bit signed immediate
    always_comb 
        case(imm_src)
            3'b000:  imm_ext = {{20{instr[31]}}, instr[31:20]};                                // I-type
            3'b001:  imm_ext = {{20{instr[31]}}, instr[31:25], instr[11:7]};                   // S-type (stores)
            3'b010:  imm_ext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};   // B-type (branches)
            3'b011:  imm_ext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; // J-type (jal)
            3'b100:  imm_ext = {instr[31:12], 12'b0};                                          // U-type (lui, auipc)
            default: imm_ext = 32'bx;                                                          // Undefined
        endcase 
endmodule

// Instruction memory
module imem(
    input  logic [31:0] a,
    output logic [31:0] rd
); 
    // 64 elements of 32-bit size
    logic [31:0] RAM[63:0];

    // Load test file of RISC-V assembly instructions into memory
    initial
        $readmemh("riscvtest.mem", RAM);

    // Word aligned memory
    assign rd = RAM[a[31:2]];
endmodule

// Data memory
module dmem(
    input  logic        clk_i,
    input  logic        we,
    input  logic [31:0] a,
    input  logic [31:0] wd,
    output logic [31:0] rd
);
    logic [31:0] RAM[63:0];

    // Word aligned memory
    assign rd = RAM[a[31:2]];

    always_ff @(posedge clk_i)
        if (we) RAM[a[31:2]] <= wd; 
endmodule

module adder(
    input  [31:0] a,
    input  [31:0] b, 
    output [31:0] y
);
    assign y = a + b; 
endmodule

module flopr #(
    parameter WIDTH = 8
)(
    input  logic             clk_i, 
    input  logic             rst_ni,
    input  logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q
);
    always_ff @(posedge clk_i, posedge rst_ni) 
        if (rst_ni) q <= 0;
        else       q <= d; 
endmodule

module flopenr #(
    parameter WIDTH = 8
)(
    input  logic             clk_i, 
    input  logic             rst_ni, 
    input  logic             en,
    input  logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q
);
    always_ff @(posedge clk_i, posedge rst_ni) 
        if (rst_ni)  q <= 0;
        else if (en) q <= d; 
endmodule

module flopenrc #(
    parameter WIDTH = 8
)(
    input  logic             clk_i,
    input  logic             rst_ni,
    input  logic             clear, 
    input  logic             en, 
    input  logic [WIDTH-1:0] d,
    output logic [WIDTH-1:0] q
);
    always_ff @(posedge clk_i, posedge rst_ni) 
        if (rst_ni)    q <= 0;
        else if (en)
            if (clear) q <= 0; 
            else       q <= d;
endmodule

module floprc #(
    parameter WIDTH = 8
)(
    input  logic             clk_i, 
    input  logic             rst_ni, 
    input  logic             clear,
    input  logic [WIDTH-1:0] d, 
    output logic [WIDTH-1:0] q
);
    always_ff @(posedge clk_i, posedge rst_ni) 
        if (rst_ni)    q <= 0;
        else
            if (clear) q <= 0; 
            else       q <= d;
endmodule

module mux2 #(
    parameter WIDTH = 8
)(
    input  logic [WIDTH-1:0] d0,
    input  logic [WIDTH-1:0] d1,
    input  logic             s, 
    output logic [WIDTH-1:0] y
);
    assign y = s ? d1 : d0; 
endmodule

module mux3 #(
    parameter WIDTH = 8
)(
    input  logic [WIDTH-1:0] d0,
    input  logic [WIDTH-1:0] d1,
    input  logic [WIDTH-1:0] d2,
    input  logic [1:0]       s,
    output logic [WIDTH-1:0] y
);
    assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule
