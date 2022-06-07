`timescale 1ns/10ps
`define CYCLE 10.0
`define NUM_PAT 10

module testbench;
    reg         clk, rst_n, valid;
    reg  [ 2:0] ctrl;
    reg  [31:0] in_A, in_B;
    wire [31:0] out;
    integer     err;

    parameter FUNC_ADD = 3'd0;
    parameter FUNC_SUB = 3'd1;
    parameter FUNC_SLL = 3'd2;
    parameter FUNC_XOR = 3'd3;
    parameter FUNC_SRL = 3'd4;
    parameter FUNC_SRA = 3'd5;
    parameter FUNC_OR  = 3'd6;
    parameter FUNC_AND = 3'd7;

    MY_DESIGN u_design(
        .clk(clk),
        .rst_n(rst_n),
        .valid(valid),
        .ctrl(ctrl),
        .in_A(in_A),
        .in_B(in_B),
        .out(out)
    );
    // clk
    initial begin
        clk = 1;
        forever #(`CYCLE*0.5) clk = !clk;
    end
    // rst_n
    initial begin
        rst_n = 1;
        #(`CYCLE*0.5) rst_n = 0;
        #(`CYCLE*1.0) rst_n = 1;
    end
    // Run tasks
    initial begin
        valid = 0;
        ctrl  = 0;
        in_A  = 0;
        in_B  = 0;
        err   = 0;
        #(`CYCLE*2.5)
        $display("***********************");
        $display("* Begin Simulation... *");
        $display("***********************");
        test_func(FUNC_ADD, `NUM_PAT);
        test_func(FUNC_AND, `NUM_PAT);
        test_func(FUNC_OR , `NUM_PAT);
        test_func(FUNC_SLL, `NUM_PAT);
        test_func(FUNC_SRA, `NUM_PAT);
        test_func(FUNC_SRL, `NUM_PAT);
        test_func(FUNC_SUB, `NUM_PAT);
        test_func(FUNC_XOR, `NUM_PAT);
        $display("***********************");
        $display("* Fisnish Simulation  *");
        $display("***********************");
        $display("* Simulation Result:  *");
        if (err == 0) begin
            $display("*   You pass!         *");
        end
        else begin
            $display("*   You fail!         *");
        end
        $display("***********************");
        $finish;
    end
    // nWave
    initial begin
        $fsdbDumpfile("my_design.fsdb");
        $fsdbDumpvars;
    end

    task test_func;
        input [ 2:0] func_name;
        input [31:0] num_pat;
        reg   [8*3:1] func_str;
        reg   [31:0] ans;
        integer pat;
        begin
            case(func_name)
                FUNC_ADD: func_str = "ADD";
                FUNC_AND: func_str = "AND";
                FUNC_OR : func_str = "OR ";
                FUNC_SLL: func_str = "SLL";
                FUNC_SRA: func_str = "SRA";
                FUNC_SRL: func_str = "SRL";
                FUNC_SUB: func_str = "SUB";
                FUNC_XOR: func_str = "XOR";
            endcase
            $display("  Test function: %s", func_str);
            valid = 1;
            ctrl = func_name;
            #(`CYCLE);
            valid = 0;
            ctrl = 0;
            #(`CYCLE*0.5);
            repeat(num_pat) begin
                pat = $random;
                in_A = pat;
                pat = $random;
                in_B = {27'b0, pat[4:0]};
                case(func_name)
                    FUNC_ADD: ans = in_A + in_B;
                    FUNC_AND: ans = in_A & in_B;
                    FUNC_OR : ans = in_A | in_B;
                    FUNC_SLL: ans = in_A << in_B;
                    FUNC_SRA: ans = $signed(in_A) >>> in_B;
                    FUNC_SRL: ans = in_A >> in_B;
                    FUNC_SUB: ans = in_A - in_B;
                    FUNC_XOR: ans = in_A ^ in_B;
                endcase
                #(`CYCLE*0.9);
                if (ans !== out) begin
                    err = err + 1;
                    $display("    Error! in_A: %8H. in_B: %8H. Expected result: %8H. Your result: %8H",in_A,in_B,ans,out);
                end
                #(`CYCLE*0.1);
            end
            #(`CYCLE*0.5);
        end
    endtask
endmodule                                                                                   
                                                                                                