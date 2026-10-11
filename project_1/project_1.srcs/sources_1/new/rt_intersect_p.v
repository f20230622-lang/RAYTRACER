`timescale 1ns/1ps

// Pipelined Moller-Trumbore. Latency 7 clocks, throughput 1 test/clock.
// Bit-exact with rt_intersect.v. Multiplier outputs are registered as full
// 64-bit products; the >>>16 truncation happens in the following stage.
module rt_intersect_p (
    input         clk,
    input         valid_in,
    input  [95:0] o, d, v0, v1, v2,
    output        valid_out,
    output reg    hit,
    output reg [31:0] a_n,
    output reg [31:0] tn_n
);
    localparam signed [33:0] DET_EPS = 34'sd16;

    reg [6:0] vp = 7'b0;
    always @(posedge clk) vp <= {vp[5:0], valid_in};
    assign valid_out = vp[6];

    // ---- L1: edges, s = o - v0 ----
    reg [95:0] E1, E2, S, D;
    always @(posedge clk) begin
        E1 <= {v1[95:64]-v0[95:64], v1[63:32]-v0[63:32], v1[31:0]-v0[31:0]};
        E2 <= {v2[95:64]-v0[95:64], v2[63:32]-v0[63:32], v2[31:0]-v0[31:0]};
        S  <= {o[95:64] -v0[95:64], o[63:32] -v0[63:32], o[31:0] -v0[31:0]};
        D  <= d;
    end
    wire signed [31:0] e1x=E1[31:0], e1y=E1[63:32], e1z=E1[95:64];
    wire signed [31:0] e2x=E2[31:0], e2y=E2[63:32], e2z=E2[95:64];
    wire signed [31:0] sx =S[31:0],  sy =S[63:32],  sz =S[95:64];
    wire signed [31:0] dx =D[31:0],  dy =D[63:32],  dz =D[95:64];

    // ---- L2: full 64-bit products for the two cross products ----
    reg signed [63:0] m2 [0:11];
    reg [95:0] E1_2, E2_2, S_2, D_2;
    always @(posedge clk) begin
        // h = cross(d, e2)
        m2[0] <= dy * e2z;  m2[1] <= dz * e2y;
        m2[2] <= dz * e2x;  m2[3] <= dx * e2z;
        m2[4] <= dx * e2y;  m2[5] <= dy * e2x;
        // q = cross(s, e1)
        m2[6]  <= sy * e1z; m2[7]  <= sz * e1y;
        m2[8]  <= sz * e1x; m2[9]  <= sx * e1z;
        m2[10] <= sx * e1y; m2[11] <= sy * e1x;
        E1_2 <= E1; E2_2 <= E2; S_2 <= S; D_2 <= D;
    end

    // ---- L3: truncate (>>>16) and finish the cross products ----
    reg signed [31:0] hx, hy, hz, qx, qy, qz;
    reg [95:0] E1_3, E2_3, S_3, D_3;
    always @(posedge clk) begin
        hx <= (m2[0] >>> 16) - (m2[1] >>> 16);
        hy <= (m2[2] >>> 16) - (m2[3] >>> 16);
        hz <= (m2[4] >>> 16) - (m2[5] >>> 16);
        qx <= (m2[6] >>> 16) - (m2[7] >>> 16);
        qy <= (m2[8] >>> 16) - (m2[9] >>> 16);
        qz <= (m2[10] >>> 16) - (m2[11] >>> 16);
        E1_3 <= E1_2; E2_3 <= E2_2; S_3 <= S_2; D_3 <= D_2;
    end

    wire signed [31:0] e1x3=E1_3[31:0], e1y3=E1_3[63:32], e1z3=E1_3[95:64];
    wire signed [31:0] e2x3=E2_3[31:0], e2y3=E2_3[63:32], e2z3=E2_3[95:64];
    wire signed [31:0] sx3 =S_3[31:0],  sy3 =S_3[63:32],  sz3 =S_3[95:64];
    wire signed [31:0] dx3 =D_3[31:0],  dy3 =D_3[63:32],  dz3 =D_3[95:64];

    // ---- L4: full 64-bit products for the four dot products ----
    reg signed [63:0] m4 [0:11];
    always @(posedge clk) begin
        m4[0]  <= e1x3 * hx; m4[1]  <= e1y3 * hy; m4[2]  <= e1z3 * hz;  // a
        m4[3]  <= sx3  * hx; m4[4]  <= sy3  * hy; m4[5]  <= sz3  * hz;  // u
        m4[6]  <= dx3  * qx; m4[7]  <= dy3  * qy; m4[8]  <= dz3  * qz;  // v
        m4[9]  <= e2x3 * qx; m4[10] <= e2y3 * qy; m4[11] <= e2z3 * qz;  // t
    end

    // ---- L5: truncate each product, then sum ----
    reg signed [31:0] a_raw, u_raw, v_raw, t_raw;
    always @(posedge clk) begin
        a_raw <= (m4[0] >>> 16) + (m4[1]  >>> 16) + (m4[2]  >>> 16);
        u_raw <= (m4[3] >>> 16) + (m4[4]  >>> 16) + (m4[5]  >>> 16);
        v_raw <= (m4[6] >>> 16) + (m4[7]  >>> 16) + (m4[8]  >>> 16);
        t_raw <= (m4[9] >>> 16) + (m4[10] >>> 16) + (m4[11] >>> 16);
    end

    // ---- L6: normalize sign, form U+V ----
    wire signed [33:0] a_x = {{2{a_raw[31]}}, a_raw};
    wire signed [33:0] u_x = {{2{u_raw[31]}}, u_raw};
    wire signed [33:0] v_x = {{2{v_raw[31]}}, v_raw};
    wire signed [33:0] t_x = {{2{t_raw[31]}}, t_raw};

    wire neg = a_raw[31];
    reg signed [33:0] A_r, U_r, V_r, T_r, UV_r;
    always @(posedge clk) begin
        A_r  <= neg ? -a_x : a_x;
        U_r  <= neg ? -u_x : u_x;
        V_r  <= neg ? -v_x : v_x;
        T_r  <= neg ? -t_x : t_x;
        UV_r <= (neg ? -u_x : u_x) + (neg ? -v_x : v_x);
    end

    // ---- L7: compare ----
    always @(posedge clk) begin
        hit  <= (A_r >= DET_EPS) && (U_r >= 0) && (U_r <= A_r) &&
                (V_r >= 0) && (UV_r <= A_r) && (T_r > 0);
        a_n  <= A_r[31:0];
        tn_n <= T_r[31:0];
    end
endmodule