// Your code
module CHIP(clk,
            rst_n,
            // For mem_D
            mem_wen_D,
            mem_addr_D,
            mem_wdata_D,
            mem_rdata_D,
            // For mem_I
            mem_addr_I,
            mem_rdata_I);

    input         clk, rst_n ;
    // For mem_D
    output        mem_wen_D  ;
    output [31:0] mem_addr_D ;
    output [31:0] mem_wdata_D;
    input  [31:0] mem_rdata_D;    
    // For mem_I
    output [31:0] mem_addr_I ; 
    input  [31:0] mem_rdata_I;   
    
    //---------------------------------------//
    // Do not modify this part!!!            //
    // Exception: You may change wire to reg //
    reg    [31:0] PC          ;              //
    wire   [31:0] PC_nxt      ;              //
    wire          regWrite    ;              //
    wire   [ 4:0] rs1, rs2, rd;              //
    wire   [31:0] rs1_data    ;              //
    wire   [31:0] rs2_data    ;              //
    wire   [31:0] rd_data     ;              //
    //---------------------------------------//

    assign rs1 = mem_rdata_I[19:15];
    assign rs2 = mem_rdata_I[24:20];
    assign rd = mem_rdata_I[11:7];
    assign mem_addr_I = PC;

    // Todo: other wire/reg

    wire   [0:0]  Branch;
    wire   [2:0]  ALUOp;
    wire   [0:0]  MemRead; 
    wire   [0:0]  MemWrite; 
    wire   [0:0]  MemtoReg;
    wire   [0:0]  ALUSrc;
    wire   [0:0]  auipc;
    wire   [0:0]  Jump;
    wire   [0:0]  Jump_control;
    wire   [31:0] ALU_result;
    wire   [3:0]  control_ALU;
    wire   [31:0] alu_input_mux_1;
    wire   [31:0] alu_input_mux_2;
    wire   [0:0]  Zero;
    wire   [0:0]  Bge;
    wire   [0:0] Beq_Bge;
    wire   [31:0] PC_next_0, PC_next_1, PC_final;
    wire   [0:0]  PC_control;
    wire   [31:0] imm_gen;
    wire   [31:0] imm_gen_shift;
    wire   [31:0] rd_temp;
    wire   [31:0] rd_temp_2;
    wire   [31:0] jump_src;
    wire   [31:0] jump_dest;
    wire   [0:0]  mulDiv_valid;
    wire   [0:0]  mulDiv_mode;
    wire   [0:0]  mulDiv_ready;
    wire   [31:0] mulDiv_result;
    integer next = 4;
    
    assign mem_wen_D = MemWrite;
    assign mem_wdata_D = rs2_data;
    assign mem_addr_D = ALU_result;
    assign Beq_Bge = Zero || Bge;

    //---------------------------------------//
    // Do not modify this part!!!            //
    reg_file reg0(                           //
        .clk(clk),                           //
        .rst_n(rst_n),                       //
        .wen(regWrite),                      //
        .a1(rs1),                            //
        .a2(rs2),                            //
        .aw(rd),                             //
        .d(rd_data),                         //
        .q1(rs1_data),                       //
        .q2(rs2_data));                      //
    //---------------------------------------//

    // Todo: any combinational/sequential circuit
    
    Control Control(
        .opcode(mem_rdata_I[6:0]),
        .Branch(Branch),
        .MemRead(MemRead),
        .MemtoReg(MemtoReg),
        .ALUOp(ALUOp),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .RegWrite(regWrite),
        .auipc(auipc),
        .Jump(Jump),
        .Jump_control(Jump_control)
    );

    Imm_Gen imm_gen_unit(
        .instruction(mem_rdata_I),
        .imm_gen(imm_gen)
    );

    Adder_4 adder_imm_4(
        .PC(PC),
        .imm(next),
        .PC_next_0(PC_next_0)
    );

    Adder_32 adder_32_bits(
        .in_a(PC),
        .in_b(imm_gen_shift),
        .out(PC_next_1)
    );

    Mux_32 mux_rs_PC(
        .in_a(rs1_data),
        .in_b(PC),
        .control(Jump_control),
        .out(jump_src)
    );

    Adder_32 adder_jump(
        .in_a(jump_src),
        .in_b(imm_gen),
        .out(jump_dest)
    );

    Mux_32 mux_PC(
        .in_a(PC_next_0),
        .in_b(PC_next_1),
        .control(PC_control),
        .out(PC_final)
    );

    Mux_32 mux_PC_jump(
        .in_a(PC_final),
        .in_b(jump_dest),
        .control(Jump),
        .out(PC_nxt)
    );

    Mux_32 mux_memory(
        .in_a(ALU_result),
        .in_b(mem_rdata_D),
        .control(MemtoReg),
        .out(rd_temp)
    );

    Mux_32 mux_jump(
        .in_a(rd_temp),
        .in_b(PC_next_0),
        .control(Jump),
        .out(rd_temp_2)
    );

    Mux_32 mux_mul(
        .in_a(rd_temp_2),
        .in_b(mulDiv_result),
        .control(mulDiv_valid),
        .out(rd_data)
    );

    Mux_32 mux_ALU_1(
        .in_a(rs1_data),
        .in_b(PC),
        .control(auipc),
        .out(alu_input_mux_1)
    );

    Mux_32 mux_ALU_2(
        .in_a(rs2_data),
        .in_b(imm_gen),
        .control(ALUSrc),
        .out(alu_input_mux_2)
    );

    Shift_left_1 shift_unit(
        .imm_gen(imm_gen),
        .imm_gen_shift(imm_gen_shift)
    );

    AND_32 branch_or_not(
        .in_a(Branch),
        .in_b(Beq_Bge),
        .PC_control(PC_control)
    );

    ALU simple_alu(
        .ALUCtl(control_ALU),
        .in_a(alu_input_mux_1),
        .in_b(alu_input_mux_2),
        .ALUResult(ALU_result),
        .Zero(Zero),
        .Bge(Bge)
    );

    ALU_control ALU_control_unit(
        .ALUOp(ALUOp),
        .inst(mem_rdata_I),
        .ALUCtl(control_ALU),
        .mulDiv_valid(mulDiv_valid),
        .mulDiv_mode(mulDiv_mode)
    );

    mulDiv MUL_unit(
        .clk(clk),
        .rst_n(rst_n),
        .valid(mulDiv_valid),
        .ready(mulDiv_ready),
        .mode(mulDiv_mode),
        .in_A(alu_input_mux_1),
        .in_B(alu_input_mux_2),
        .out(mulDiv_result)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) 
        begin
            PC <= 32'h00010000; // where text in memory starts
        end
        else 
        begin
            if(mulDiv_valid==1 && mulDiv_ready==0)
            begin
                PC <= PC;
            end
            else 
            begin
                PC <= PC_nxt;    
            end
        end
    end
    
endmodule

module Control(opcode,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,auipc,Jump,Jump_control);

    input  [6:0] opcode;
    output [0:0] Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,auipc,Jump,Jump_control;
    output [2:0] ALUOp;

    reg [0:0] Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite,auipc,Jump,Jump_control;
    reg [2:0] ALUOp;

    always @(opcode) begin
        if(opcode==7'b0010111)  // auipc
        begin
            ALUOp = 3'b111;
            MemtoReg = 1'b0;
            MemWrite = 1'b0;
            MemRead = 1'b0;
            Branch = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            auipc = 1'b1;
            Jump = 1'b0;
            Jump_control = 1'b0;
        end

        else if(opcode==7'b1100111) // jalr
        begin
            ALUOp = 3'b001;
            MemtoReg = 1'b0;
            MemWrite = 1'b0;
            MemRead = 1'b0;
            Branch = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            auipc = 1'b0;
            Jump = 1'b1;
            Jump_control = 1'b0;   // use rs
        end

        else if(opcode==7'b1101111) // jal
        begin
            ALUOp = 3'b000;
            MemtoReg = 1'b0;
            MemWrite = 1'b0;
            MemRead = 1'b0;
            Branch = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            auipc = 1'b0;
            Jump = 1'b1;
            Jump_control = 1'b1;
        end

        else if(opcode==7'b1100011) // beq
        begin
            ALUOp = 3'b101;
            MemtoReg = 1'bx;
            MemWrite = 1'b0;
            MemRead = 1'b0;
            Branch = 1'b1;
            ALUSrc = 1'b0;
            RegWrite = 1'b0;
            auipc = 1'b0;
            Jump = 1'b0;
            Jump_control = 1'b0;
        end

        else if(opcode==7'b0000011) // lw
        begin
            ALUOp = 3'b010;
            MemtoReg = 1'b1;
            MemWrite = 1'b0;
            MemRead = 1'b1;
            Branch = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1; 
            auipc = 1'b0;
            Jump = 1'b0;
            Jump_control = 1'b0;
        end

        else if(opcode==7'b0100011) //sw
        begin
            ALUOp = 3'b011;
            MemtoReg = 1'bx;
            MemWrite = 1'b1;
            MemRead = 1'b0;
            Branch = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b0;
            auipc = 1'b0;
            Jump = 1'b0;
            Jump_control = 1'b0;
        end

        else if(opcode==7'b0010011) // addi slti srai alli
        begin
            ALUOp = 3'b100;
            MemtoReg = 1'b0;
            MemWrite = 1'b0;
            MemRead = 1'b0;
            Branch = 1'b0;
            ALUSrc = 1'b1;
            RegWrite = 1'b1;
            auipc = 1'b0;
            Jump = 1'b0;
            Jump_control = 1'b0;
        end

        else if(opcode==7'b0110011) // R-type
        begin
            ALUOp = 3'b110;
            MemtoReg = 1'b0;
            MemWrite = 1'b0;
            MemRead = 1'b0;
            Branch = 1'b0;
            ALUSrc = 1'b0;
            RegWrite = 1'b1;
            auipc = 1'b0;
            Jump = 1'b0;
            Jump_control = 1'b0;
        end

        else 
        begin
            Branch = 1'b0;
            MemWrite = 1'bx;
            MemtoReg = 1'bx;
            MemRead = 1'bx;
            ALUOp = 3'bx;
            RegWrite = 1'bx;
            ALUSrc = 1'bx;    
            auipc = 1'bx;
            Jump = 1'b0;
            Jump_control = 1'bx;
        end
    end

endmodule

module Imm_Gen(instruction,imm_gen);

    input  [31:0] instruction;
    output [31:0] imm_gen;
    reg    [31:0] imm_gen;

    always @(instruction) begin
       if(instruction[6:0]==7'b0010111)  // auipc
       begin  
           imm_gen = {instruction[31:12],12'b0};
       end

       else if(instruction[6:0]==7'b1100011)  // beq
       begin
            imm_gen[4:1] = instruction[11:8];
            imm_gen[10:5] = instruction[30:25];
            imm_gen[11] = instruction[7];
            imm_gen[12] = instruction[31];
            imm_gen = {{20{imm_gen[12]}},imm_gen[12:1]};
       end

       else if((instruction[6:0]==7'b0010011) ||  (instruction[6:0]==7'b1100111) || (instruction[6:0]==7'b0000011))
       begin                                  // addi slti jalr lw
            imm_gen[11:0] = instruction[31:20];
            imm_gen = {{20{imm_gen[11]}},imm_gen[11:0]};
       end
       
       else if(instruction[6:0]==7'b0100011)  // sw
       begin
            imm_gen[4:0] = instruction[11:7];
            imm_gen[11:5] = instruction[31:25];
            imm_gen = {{20{imm_gen[11]}},imm_gen[11:0]};
       end

       else if(instruction[6:0]==7'b1101111)  // jal
       begin
            imm_gen[10:1] = instruction[30:21];
            imm_gen[11] = instruction[20];
            imm_gen[19:12] = instruction[19:12];
            imm_gen[20] = instruction[31];
            imm_gen = {{11{imm_gen[20]}},imm_gen[20:1],1'b0};
       end

       else   // R-type
       begin
            imm_gen = {32{1'b0}};
       end
    end

endmodule

module Adder_4(PC,imm,PC_next_0);

    input  [31:0] PC,imm;
    output [31:0] PC_next_0;
    assign PC_next_0 = PC + imm;

endmodule

module Adder_32(in_a,in_b,out);

    input signed [31:0] in_a,in_b;
    output [31:0] out;
    assign out = in_a + in_b;

endmodule

module Shift_left_1(imm_gen,imm_gen_shift);

    input  [31:0] imm_gen;
    output [31:0] imm_gen_shift;
    assign imm_gen_shift[31:1] = imm_gen[30:0];
    assign imm_gen_shift[0] = 1'b0;

endmodule

module Mux_32(in_a,in_b,control,out);

    input  [31:0] in_a,in_b;
    input  [0:0]  control;
    output [31:0] out;
    assign out = (control==1'b0) ? in_a : in_b;

endmodule

module AND_32(in_a,in_b,PC_control);

    input in_a,in_b;
    output PC_control;
    assign PC_control = in_a && in_b;

endmodule

module ALU(ALUCtl, in_a, in_b, ALUResult, Zero, Bge);

    input  [ 3:0] ALUCtl;
    input  [31:0] in_a, in_b;
    output [31:0] ALUResult;
    output [ 0:0] Zero,Bge;

    reg    [31:0] alu_out; 
    assign ALUResult = alu_out;
    assign Zero = ( (alu_out == 0) ? 1'b1 : 1'b0 ) && ( ALUCtl==4 );  // beq
    assign Bge = ( ($signed(in_a) >= $signed(in_b) ) ? 1'b1 : 1'b0 ) && ( ALUCtl==5 );  // bge

    always @(*) begin
        case (ALUCtl)
            0 : alu_out = in_a & in_b;
            1 : alu_out = in_a | in_b;
            2 : alu_out = in_a ^ in_b;
            3 : alu_out = in_a + in_b;
            4 : alu_out = in_a - in_b;
            5 : alu_out = (in_a < in_b) ? 32'h1 : 32'h0;
            6 : alu_out = in_a << in_b;
            7 : alu_out = in_a >> in_b;
            default: alu_out = in_a;
        endcase
    end

endmodule

module ALU_control (ALUOp, inst, ALUCtl, mulDiv_valid, mulDiv_mode);

    input  [ 2:0] ALUOp;
    input  [31:0] inst;
    output [ 3:0] ALUCtl;
    output mulDiv_valid, mulDiv_mode;

    // I-type instruction
    parameter I_func_mask = 32'b00000000000000000111000001111111;
    parameter inst_ADDI   = 32'b00000000000000000000000000010011;
    parameter inst_STLI   = 32'b00000000000000000010000000010011;
    parameter inst_SLLI   = 32'b00000000000000000001000000010011;
    parameter inst_SRLI   = 32'b00000000000000000101000000010011;

    // R-type instruction
    parameter R_func_mask = 32'b11111110000000000111000001111111;
    parameter inst_ADD    = 32'b00000000000000000000000000110011;
    parameter inst_SUB    = 32'b01000000000000000000000000110011;
    parameter inst_STL    = 32'b00000000000000000010000000110011;
    parameter inst_XOR    = 32'b00000000000000000100000000110011;
    parameter inst_OR     = 32'b00000000000000000110000000110011;
    parameter inst_AND    = 32'b00000000000000000111000000110011;
    parameter inst_MUL    = 32'b00000010000000000000000000110011;
    parameter inst_DIV    = 32'b00000010000000000100000000110011;

    reg   [ 3:0] ctrl;
    reg   [ 0:0] valid;
    reg   [ 0:0] mode; 

    assign ALUCtl = ctrl;
    assign mulDiv_valid = valid;
    assign mulDiv_mode = mode;

    always @(*) begin

        // initialize
        valid = 0;
        mode = 0;

        case(ALUOp)
            3'b000 : ctrl = 3; // JAL
            3'b001 : ctrl = 3; // JALR
            3'b010 : ctrl = 3; // LW
            3'b011 : ctrl = 3; // SW
            3'b100 : begin // I type
                case(inst & I_func_mask)
                    inst_ADDI: ctrl = 3;  // ADDI
                    inst_STLI: ctrl = 5;  // SLTI
                    inst_SLLI: ctrl = 6;  // SLLI
                    inst_SRLI: ctrl = 7;  // SRLI
                    default  : ctrl = 15; // should not happen
                endcase
            end
            3'b101 : begin
                case(inst[14:12])
                    3'b000: ctrl = 4; // BEQ
                    3'b101: ctrl = 5; // BGE
                    default: ctrl = 15; // should not happen
                endcase
            end
            3'b110 : begin // R-type
                case(inst & R_func_mask)
                    inst_AND: ctrl = 0; // AND
                    inst_OR : ctrl = 1; // OR
                    inst_XOR: ctrl = 2; // XOR
                    inst_ADD: ctrl = 3; // ADD
                    inst_SUB: ctrl = 4; // SUB
                    inst_STL: ctrl = 5; // STL
                    inst_MUL: begin
                        ctrl = 15;
                        valid = 1;
                        mode  = 0;
                    end
                    inst_DIV: begin
                        ctrl = 15;
                        valid = 1;
                        mode  = 1;
                    end
                    default: ctrl = 15; // should not happen
                endcase
            end
            3'b111 : ctrl = 3;  // AUIPC
            default: ctrl = 15; // should not happen
        endcase 
    end
    
endmodule

module reg_file(clk, rst_n, wen, a1, a2, aw, d, q1, q2);

    // constants
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth

    input clk, rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] d;
    input [addr_width-1:0] a1, a2, aw;

    output [BITS-1:0] q1, q2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign q1 = mem[a1];
    assign q2 = mem[a2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (aw == i)) ? d : mem[i];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end
    end

endmodule

module mulDiv(clk, rst_n, valid, ready, mode, in_A, in_B, out);
    
    input         clk, rst_n;
    input         valid;
    input  [ 0:0]  mode; // mode: 0: mulu, 1: divu
    output        ready;
    input  [31:0] in_A, in_B;
    output [31:0] out;
    
    // Definition of states
    parameter IDLE = 2'b0;
    parameter MUL  = 2'd1;
    parameter DIV  = 2'd2;
    parameter OUT  = 2'd3;

    reg  [ 2:0] state, state_nxt;
    reg  [ 4:0] counter, counter_nxt;
    reg  [63:0] shreg, shreg_nxt;
    reg  [31:0] alu_in, alu_in_nxt;
    reg  [32:0] alu_out;

    // Wire assignments
    // Output control: Relate output port “out” to shift register when ready
    // Output is default wire-typed, so we can use it directly.
    assign ready = (state == OUT) ? 1 : 0;
    assign out = (state == OUT) ? shreg[31:0] : 32'b0;

    // Combinational always block
    // Next-state logic of state machine
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid == 1'b1) begin
                    if (mode == 0) begin
                        state_nxt = MUL;
                    end else if (mode == 1) begin
                        state_nxt = DIV;
                    end else begin
                        state_nxt = IDLE;
                    end
                end else begin
                    state_nxt = IDLE;
                end
            end
            MUL : begin
                if (counter == 31) begin
                    state_nxt = OUT;
                end else begin
                    state_nxt = MUL;
                end
            end
            DIV : begin
                if (counter == 31) begin
                    state_nxt = OUT;
                end else begin
                    state_nxt = DIV;
                end
            end
            OUT : state_nxt = IDLE;
            default : state_nxt = IDLE;
        endcase
    end

    // Counter
    // Counter counts from 0 to 31 when the state is MUL or DIV. Otherwise, keep it zero
    always @(*) begin
        if (state == MUL || state == DIV) begin
            counter_nxt = counter + 1;
        end else begin
            counter_nxt = 0;
        end
    end

    // ALU input
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) alu_in_nxt = in_B;
                else       alu_in_nxt = 0;
            end
            OUT : alu_in_nxt = 0;
            default: alu_in_nxt = alu_in;
        endcase
    end

    // ALU output
    // MUL: addition with the high 32 bits of shreg
    // DIV: subtraction to the high 32 bits of shreg
    always @(*) begin
        case(state)
            MUL: begin
                if (shreg[0]) alu_out = {1'b0, (shreg >> 32)} + {1'b0, alu_in};
                else alu_out = {1'b0, (shreg >> 32)};
            end
            DIV: begin
                if ((shreg >> 32) > alu_in) begin
                    alu_out = {1'b0, (shreg >> 32)} - {1'b0, alu_in};
                end else begin
                    alu_out = {1'b0, (shreg >> 32)};
                end
            end
            default: alu_out = 32'b0;
        endcase
    end
    
    // Shift register
    // Load data to the low 32 bits when the state is IDLE and valid = 1
    // Multiplication: update data and right shift; Division: update data and left shift
    always @(*) begin
        case(state)
            IDLE: begin
                if (valid) begin
                    if (mode == 1) shreg_nxt = {32'b0, in_A} << 1;
                    else shreg_nxt = {32'b0, in_A};
                end else begin
                    shreg_nxt = 64'b0;
                end
            end
            MUL: begin
                shreg_nxt = {alu_out[31:0], shreg[31:0]} >> 1;
                shreg_nxt[63] = alu_out[32];
            end
            DIV: begin
                if (counter == 31) begin
                    shreg_nxt = {alu_out[31:0], shreg[31:0]} << 1;
                    shreg_nxt = {shreg_nxt[63:32] >> 1, shreg_nxt[31:0]};
                end else begin
                    shreg_nxt = {alu_out[31:0], shreg[31:0]} << 1;
                end
                if ((shreg >> 32) > alu_in) shreg_nxt[0] = 1;
                else shreg_nxt[0] = 0;

            end
            default: shreg_nxt = shreg;
        endcase
    end

    // Sequential always block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            counter <= 0;
        end
        else begin
            state <= state_nxt;
            alu_in <= alu_in_nxt;
            counter <= counter_nxt;
            shreg <= shreg_nxt;
        end
    end

endmodule