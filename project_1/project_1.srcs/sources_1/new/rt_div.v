`timescale 1ns/1ps

// Restoring divider: quo = ({tn,16'b0}) / a, low 32 bits (matches s32((tn<<16)//a)).
// Both inputs are positive when used. Takes 49 cycles from start to done.
module rt_div (
    input         clk,
    input         start,         // 1-cycle pulse; inputs are latched here
    input  [31:0] tn,
    input  [31:0] a,
    output reg    done,          // level: goes high when q is valid, cleared by next start
    output [31:0] q
);
    reg [47:0] num;
    reg [31:0] den;
    reg [32:0] rem;
    reg [47:0] quo;
    reg [5:0]  cnt;
    reg        run;

    wire [32:0] rem_sh = {rem[31:0], num[47]};
    wire        ge     = (rem_sh >= {1'b0, den});

    assign q = quo[31:0];

    always @(posedge clk) begin
        if (start) begin
            num  <= {tn, 16'b0};
            den  <= a;
            rem  <= 33'd0;
            quo  <= 48'd0;
            cnt  <= 6'd47;
            run  <= 1'b1;
            done <= 1'b0;
        end else if (run) begin
            num <= num << 1;
            rem <= ge ? (rem_sh - {1'b0, den}) : rem_sh;
            quo <= {quo[46:0], ge};
            if (cnt == 0) begin
                run  <= 1'b0;
                done <= 1'b1;
            end else
                cnt <= cnt - 1'b1;
        end
    end

    initial begin run = 0; done = 0; end
endmodule
