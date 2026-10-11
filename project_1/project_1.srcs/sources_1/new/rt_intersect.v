`timescale 1ns/1ps

// Combinational Moller-Trumbore hit test, bit-exact with golden.py.
// Vectors are packed {z, y, x}, each component signed Q16.16.
module rt_intersect (
    input  [95:0] o,        // ray origin
    input  [95:0] d,        // ray direction
    input  [95:0] v0, v1, v2,
    output        hit,
    output [31:0] a_n,      // determinant (valid when hit)
    output [31:0] tn_n      // t * determinant (valid when hit)
);
    localparam F = 16;
    localparam signed [33:0] DET_EPS = 34'sd16;

    // Q16.16 multiply: 64-bit product, arithmetic shift, keep low 32 bits
    function signed [31:0] mul;
        input signed [31:0] x, y;
        reg signed [63:0] p;
        begin
            p   = x * y;
            mul = p >>> F;
        end
    endfunction

    function [95:0] sub3;
        input [95:0] a, b;
        begin
            sub3 = {a[95:64] - b[95:64],
                    a[63:32] - b[63:32],
                    a[31:0]  - b[31:0]};
        end
    endfunction

    function [95:0] cross3;
        input [95:0] a, b;
        reg signed [31:0] ax, ay, az, bx, by, bz, rx, ry, rz;
        begin
            ax = a[31:0]; ay = a[63:32]; az = a[95:64];
            bx = b[31:0]; by = b[63:32]; bz = b[95:64];
            rx = mul(ay, bz) - mul(az, by);
            ry = mul(az, bx) - mul(ax, bz);
            rz = mul(ax, by) - mul(ay, bx);
            cross3 = {rz, ry, rx};
        end
    endfunction

    function signed [31:0] dot3;
        input [95:0] a, b;
        reg signed [31:0] ax, ay, az, bx, by, bz;
        begin
            ax = a[31:0]; ay = a[63:32]; az = a[95:64];
            bx = b[31:0]; by = b[63:32]; bz = b[95:64];
            dot3 = mul(ax, bx) + mul(ay, by) + mul(az, bz);
        end
    endfunction

    // ---- Moller-Trumbore ----
    wire [95:0] e1 = sub3(v1, v0);
    wire [95:0] e2 = sub3(v2, v0);
    wire [95:0] h  = cross3(d, e2);
    wire [95:0] s  = sub3(o, v0);
    wire [95:0] q  = cross3(s, e1);

    wire signed [31:0] a_raw = dot3(e1, h);   // determinant
    wire signed [31:0] u_raw = dot3(s,  h);   // u * a
    wire signed [31:0] v_raw = dot3(d,  q);   // v * a
    wire signed [31:0] t_raw = dot3(e2, q);   // t * a

    // Widen to 34 bits so negation and u+v never wrap
    // (Python integers don't wrap here, so this keeps us bit-exact)
    wire signed [33:0] a_x = {{2{a_raw[31]}}, a_raw};
    wire signed [33:0] u_x = {{2{u_raw[31]}}, u_raw};
    wire signed [33:0] v_x = {{2{v_raw[31]}}, v_raw};
    wire signed [33:0] t_x = {{2{t_raw[31]}}, t_raw};

    // Normalize so the determinant is positive (double-sided triangles)
    wire neg = a_raw[31];
    wire signed [33:0] A  = neg ? -a_x : a_x;
    wire signed [33:0] U  = neg ? -u_x : u_x;
    wire signed [33:0] V  = neg ? -v_x : v_x;
    wire signed [33:0] T  = neg ? -t_x : t_x;
    wire signed [33:0] UV = U + V;

    assign hit = (A >= DET_EPS) &&
                 (U >= 0) && (U <= A) &&
                 (V >= 0) && (UV <= A) &&
                 (T > 0);

    assign a_n  = A[31:0];
    assign tn_n = T[31:0];
endmodule