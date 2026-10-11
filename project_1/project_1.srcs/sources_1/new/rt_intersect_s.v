`timescale 1ns/1ps

// Shared-multiplier Moller-Trumbore. 12 multipliers used in 2 passes.
// start (1-cycle pulse) -> done (1-cycle pulse) 7 clocks later.
// Do not issue a new start until done. hit/a_n/tn_n hold until the next test.
// Bit-exact with rt_intersect.v / rt_intersect_p.v.
module rt_intersect_s (
    input         clk,
    input         start,
    input  [95:0] o, d, v0, v1, v2,
    output        done,
    output reg    hit,
    output reg [31:0] a_n,
    output reg [31:0] tn_n
);
    localparam signed [33:0] DET_EPS = 34'sd16;

    // sr[i] high = "the test is in step i"
    reg [6:0] sr = 7'b0;
    always @(posedge clk) sr <= {sr[5:0], start};
    assign done = sr[6];
    wire sel = sr[2];                 // 0 = cross-product pass, 1 = dot-product pass

    // ---- Latch edges, s = o - v0, and d when the test starts ----
    reg [95:0] E1, E2, S, D;
    always @(posedge clk) if (start) begin
        E1 <= {v1[95:64]-v0[95:64], v1[63:32]-v0[63:32], v1[31:0]-v0[31:0]};
        E2 <= {v2[95:64]-v0[95:64], v2[63:32]-v0[63:32], v2[31:0]-v0[31:0]};
        S  <= {o[95:64] -v0[95:64], o[63:32] -v0[63:32], o[31:0] -v0[31:0]};
        D  <= d;
    end
    wire signed [31:0] e1x=E1[31:0], e1y=E1[63:32], e1z=E1[95:64];
    wire signed [31:0] e2x=E2[31:0], e2y=E2[63:32], e2z=E2[95:64];
    wire signed [31:0] sx =S[31:0],  sy =S[63:32],  sz =S[95:64];
    wire signed [31:0] dx =D[31:0],  dy =D[63:32],  dz =D[95:64];

    reg signed [31:0] hx, hy, hz, qx, qy, qz;

    // ---- Operand muxes: (cross pass) : (dot pass) ----
    wire signed [31:0] a0  = sel ? e1x : dy;   wire signed [31:0] b0  = sel ? hx : e2z;
    wire signed [31:0] a1  = sel ? e1y : dz;   wire signed [31:0] b1  = sel ? hy : e2y;
    wire signed [31:0] a2  = sel ? e1z : dz;   wire signed [31:0] b2  = sel ? hz : e2x;
    wire signed [31:0] a3  = sel ? sx  : dx;   wire signed [31:0] b3  = sel ? hx : e2z;
    wire signed [31:0] a4  = sel ? sy  : dx;   wire signed [31:0] b4  = sel ? hy : e2y;
    wire signed [31:0] a5  = sel ? sz  : dy;   wire signed [31:0] b5  = sel ? hz : e2x;
    wire signed [31:0] a6  = sel ? dx  : sy;   wire signed [31:0] b6  = sel ? qx : e1z;
    wire signed [31:0] a7  = sel ? dy  : sz;   wire signed [31:0] b7  = sel ? qy : e1y;
    wire signed [31:0] a8  = sel ? dz  : sz;   wire signed [31:0] b8  = sel ? qz : e1x;
    wire signed [31:0] a9  = sel ? e2x : sx;   wire signed [31:0] b9  = sel ? qx : e1z;
    wire signed [31:0] a10 = sel ? e2y : sx;   wire signed [31:0] b10 = sel ? qy : e1y;
    wire signed [31:0] a11 = sel ? e2z : sy;   wire signed [31:0] b11 = sel ? qz : e1x;

    // ---- 12 shared multipliers, full 64-bit products registered ----
    reg signed [63:0] P [0:11];
    always @(posedge clk) begin
        P[0]  <= a0  * b0;   P[1]  <= a1  * b1;   P[2]  <= a2  * b2;
        P[3]  <= a3  * b3;   P[4]  <= a4  * b4;   P[5]  <= a5  * b5;
        P[6]  <= a6  * b6;   P[7]  <= a7  * b7;   P[8]  <= a8  * b8;
        P[9]  <= a9  * b9;   P[10] <= a10 * b10;  P[11] <= a11 * b11;
    end

    // ---- Step 1: truncate (>>>16) and finish the cross products ----
    always @(posedge clk) if (sr[1]) begin
        hx <= (P[0]  >>> 16) - (P[1]  >>> 16);
        hy <= (P[2]  >>> 16) - (P[3]  >>> 16);
        hz <= (P[4]  >>> 16) - (P[5]  >>> 16);
        qx <= (P[6]  >>> 16) - (P[7]  >>> 16);
        qy <= (P[8]  >>> 16) - (P[9]  >>> 16);
        qz <= (P[10] >>> 16) - (P[11] >>> 16);
    end

    // ---- Step 3: truncate each product, then sum the dot products ----
    reg signed [31:0] a_raw, u_raw, v_raw, t_raw;
    always @(posedge clk) if (sr[3]) begin
        a_raw <= (P[0]  >>> 16) + (P[1]  >>> 16) + (P[2]  >>> 16);
        u_raw <= (P[3]  >>> 16) + (P[4]  >>> 16) + (P[5]  >>> 16);
        v_raw <= (P[6]  >>> 16) + (P[7]  >>> 16) + (P[8]  >>> 16);
        t_raw <= (P[9]  >>> 16) + (P[10] >>> 16) + (P[11] >>> 16);
    end

    // ---- Step 4: normalize sign, form U+V ----
    wire signed [33:0] a_x = {{2{a_raw[31]}}, a_raw};
    wire signed [33:0] u_x = {{2{u_raw[31]}}, u_raw};
    wire signed [33:0] v_x = {{2{v_raw[31]}}, v_raw};
    wire signed [33:0] t_x = {{2{t_raw[31]}}, t_raw};
    wire neg = a_raw[31];

    reg signed [33:0] A_r, U_r, V_r, T_r, UV_r;
    always @(posedge clk) if (sr[4]) begin
        A_r  <= neg ? -a_x : a_x;
        U_r  <= neg ? -u_x : u_x;
        V_r  <= neg ? -v_x : v_x;
        T_r  <= neg ? -t_x : t_x;
        UV_r <= (neg ? -u_x : u_x) + (neg ? -v_x : v_x);
    end

    // ---- Step 5: compare, outputs hold until the next test ----
    always @(posedge clk) if (sr[5]) begin
        hit  <= (A_r >= DET_EPS) && (U_r >= 0) && (U_r <= A_r) &&
                (V_r >= 0) && (UV_r <= A_r) && (T_r > 0);
        a_n  <= A_r[31:0];
        tn_n <= T_r[31:0];
    end
endmodule