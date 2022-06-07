module MY_DESIGN(
    clk,
    rst_n,
    valid,
    ctrl,
    in_A,
    in_B,
    out
);
    // Port list
    input         clk, rst_n, valid;
    input  [ 2:0] ctrl;
    input  [31:0] in_A, in_B;
    output [31:0] out;
    // Wire and reg
    wire   [ 2:0] FSM_ctrl, ALU_ctrl;
    // Instantiate sub-modules
    FSM FSM_INST(
        .clk(clk),
        .rst_n(rst_n),
        .valid(valid),
        .FSM_ctrl(FSM_ctrl),
        .ALU_ctrl(ALU_ctrl)
    );
    ALU ALU_INST(
        .ctrl(ALU_ctrl),
        .in_A(in_A),
        .in_B(in_B),
        .out(out)
    );
    // Continuous assignment
    assign FSM_ctrl = ctrl;
endmodule

module FSM(
    clk,
    rst_n,
    valid,
    FSM_ctrl,
    ALU_ctrl
);
    // Port list
    input        clk, rst_n, valid;
    input  [2:0] FSM_ctrl;
    output [2:0] ALU_ctrl;
    // Wire and reg
    reg    [2:0] state, state_nxt;
    // Output logic
    assign ALU_ctrl = state;
    // Next-state logic
    always @(*) begin
        if (valid) begin
            state_nxt = FSM_ctrl;
        end
        else begin
            state_nxt = state;
        end
    end
    // Sequential block
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= 0;
        end
        else begin
            state <= state_nxt;
        end
    end
endmodule

module ALU(
    ctrl,
    in_A,
    in_B,
    out
);
    // Port list
    input  [ 2:0] ctrl;
    input  [31:0] in_A, in_B;
    output [31:0] out;
    // Wire and reg
    reg    [31:0] out;
    // Combinational block
    always @(*) begin
        case(ctrl)
            3'd0: out = in_A + in_B;
            3'd1: out = in_A - in_B;
            3'd2: out = in_A << in_B;
            3'd3: out = in_A ^ in_B;
            3'd4: out = in_A >> in_B;
            3'd5: out = $signed(in_A) >>> in_B;
            3'd6: out = in_A | in_B;
            3'd7: out = in_A & in_B;
            // Full-case, neglect default case
        endcase
    end
endmodule
