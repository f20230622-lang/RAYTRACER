`timescale 1ns/1ps

// Ray tracer: shared 3-lane multiplier, shared hit-test unit.
// Movable camera (C), checker floor (D), 4x anti-aliasing (E),
// two lights with shadows + Phong specular (F).
// Mirrors trace_pixel()/pixel_aa() in golden_rt.py bit for bit.
module rt_trace_s #(
    parameter NUM_VERTS = 12,
    parameter NUM_TRIS  = 14,
    parameter IMG_W     = 64,
    parameter IMG_H     = 64,
    parameter signed [31:0] FOV_HW = 32768,     // unused, kept so the testbench connects
    parameter VERT_FILE = "C:/Users/khnay/Desktop/Ray/verts.mem",
    parameter TRI_FILE  = "C:/Users/khnay/Desktop/Ray/tris.mem",
    parameter NORM_FILE = "C:/Users/khnay/Desktop/Ray/normals.mem",
    parameter MAT_FILE  = "C:/Users/khnay/Desktop/Ray/mat.mem"
)(
    input             clk,
    input             rst,
    output reg        pix_valid,
    output reg [23:0] pix_rgb,
    output reg        done
);
    `include "C:/Users/khnay/Desktop/Ray/rt_params.vh"

    localparam signed [31:0] ONE = 32'sd65536;

    // ---- Scene memories ----
    reg [31:0] verts [0:NUM_VERTS*3-1];
    reg [31:0] tris  [0:NUM_TRIS*3-1];
    reg [31:0] nrm   [0:NUM_TRIS*3-1];
    reg [31:0] mat   [0:NUM_TRIS*16-1];  // r,g,b,refl,flag,r2,g2,b2,spec,(7 pad)
    initial begin
        $readmemh(VERT_FILE, verts);
        $readmemh(TRI_FILE,  tris);
        $readmemh(NORM_FILE, nrm);
        $readmemh(MAT_FILE,  mat);
    end

    function [7:0] to8;
        input signed [31:0] x;
        begin
            if (x[31])              to8 = 8'd0;
            else if (x[30:8] > 255) to8 = 8'd255;
            else                    to8 = x[15:8];
        end
    endfunction

    function [95:0] add3;
        input [95:0] a, b;
        begin
            add3 = {a[95:64] + b[95:64], a[63:32] + b[63:32], a[31:0] + b[31:0]};
        end
    endfunction

    // ---- States ----
    localparam [4:0]
        S_INIT=0,  S_CSTART=1, S_CWAIT=2, S_DSTART=3, S_DIV=4,  S_CADV=5,
        S_PT=6,    S_DN=7,     S_SO=8,    S_SSTART=9, S_SWAIT=10,
        S_DL0=11,  S_DL1=12,   S_LIGHT=13,
        S_RD=14,   S_RV=15,    S_SP0=16,  S_SP1=17,   S_SQ=18,  S_SPM=19,
        S_LOC=20,  S_WL=21,    S_ACC=22,  S_WN=23,
        S_SKY=24,  S_SKC=25,   S_SKA=26,  S_EMIT=27,  S_FIN=28;

    reg [4:0] state;
    reg [1:0] ph;                       // sub-step of a 3-clock multiply operation
    reg [1:0] sq;                       // squaring pass 0..2
    reg [1:0] smp;                      // AA sample 0..3
    integer   px, py, ti, depth, best_id;

    reg [95:0] ro, rd, n_r, so_r, p_r, R_r;
    reg signed [31:0] w, acc_r, acc_g, acc_b, best_t;
    reg        have_best, bl0, bl1, is_sh, sh_l, reflect_r;
    reg signed [31:0] d0_r, d1_r, light_r, loc_r, loc_g, loc_b, wl_r, dn2_r, k_r;
    reg signed [31:0] s0_r, s1_r, sp_r;
    reg signed [31:0] sk_r, sk_g, sk_b;
    reg signed [31:0] sum_r, sum_g, sum_b, t_ar, t_ag, t_ab;
    reg signed [32:0] t_kw;
    reg [95:0] d_row, d_cur;

    // ---- Shared 3-lane multiplier: load ma/mb, product next clock, use the clock after ----
    reg signed [31:0] ma0, ma1, ma2, mb0, mb1, mb2;
    reg signed [63:0] mp0, mp1, mp2;
    always @(posedge clk) begin
        mp0 <= ma0 * mb0;
        mp1 <= ma1 * mb1;
        mp2 <= ma2 * mb2;
    end
    wire signed [31:0] r0 = mp0[47:16];      // == mul(a,b): (a*b) >>> 16, low 32 bits
    wire signed [31:0] r1 = mp1[47:16];
    wire signed [31:0] r2 = mp2[47:16];
    wire signed [31:0] rsum = r0 + r1 + r2;  // dot-product sum

    // ---- Vector components ----
    wire signed [31:0] rdx = rd[31:0],  rdy = rd[63:32],  rdz = rd[95:64];
    wire signed [31:0] nx  = n_r[31:0], ny  = n_r[63:32], nz  = n_r[95:64];
    wire signed [31:0] Rx  = R_r[31:0], Ry  = R_r[63:32], Rz  = R_r[95:64];

    // ---- Constant vectors ----
    wire [95:0] L0V     = {RT_LIGHT_Z, RT_LIGHT_Y, RT_LIGHT_X};
    wire [95:0] L1V     = {RT_L1_Z,    RT_L1_Y,    RT_L1_X};
    wire [95:0] CAM_POS = {RT_CAM_Z,   RT_CAM_Y,   RT_CAM_X};
    wire [95:0] D00_V   = {RT_D00_Z,   RT_D00_Y,   RT_D00_X};
    wire [95:0] DX_V    = {RT_DX_Z,    RT_DX_Y,    RT_DX_X};
    wire [95:0] DY_V    = {RT_DY_Z,    RT_DY_Y,    RT_DY_X};
    wire [95:0] SO0 = {RT_SO0_Z, RT_SO0_Y, RT_SO0_X};
    wire [95:0] SO1 = {RT_SO1_Z, RT_SO1_Y, RT_SO1_X};
    wire [95:0] SO2 = {RT_SO2_Z, RT_SO2_Y, RT_SO2_X};
    wire [95:0] SO3 = {RT_SO3_Z, RT_SO3_Y, RT_SO3_X};
    wire [95:0] so_sel = (smp == 2'd0) ? SO0 : (smp == 2'd1) ? SO1 :
                         (smp == 2'd2) ? SO2 : SO3;

    // ---- Scene reads (shift-add indexing, no multipliers) ----
    wire [15:0] ti3 = (ti << 1) + ti;
    wire [15:0] i0  = tris[ti3];
    wire [15:0] i1  = tris[ti3 + 1];
    wire [15:0] i2  = tris[ti3 + 2];
    wire [15:0] a0i = (i0 << 1) + i0;
    wire [15:0] a1i = (i1 << 1) + i1;
    wire [15:0] a2i = (i2 << 1) + i2;
    wire [95:0] v0 = {verts[a0i+2], verts[a0i+1], verts[a0i]};
    wire [95:0] v1 = {verts[a1i+2], verts[a1i+1], verts[a1i]};
    wire [95:0] v2 = {verts[a2i+2], verts[a2i+1], verts[a2i]};

    wire [15:0] bi3  = (best_id << 1) + best_id;
    wire [15:0] bi16 = best_id << 4;
    wire [95:0] nrm_v = {nrm[bi3+2], nrm[bi3+1], nrm[bi3]};
    wire signed [31:0] mat_r    = mat[bi16];
    wire signed [31:0] mat_g    = mat[bi16+1];
    wire signed [31:0] mat_b    = mat[bi16+2];
    wire signed [31:0] mat_refl = mat[bi16+3];
    wire signed [31:0] mat_flag = mat[bi16+4];
    wire signed [31:0] mat_r2   = mat[bi16+5];
    wire signed [31:0] mat_g2   = mat[bi16+6];
    wire signed [31:0] mat_b2   = mat[bi16+7];
    wire signed [31:0] mat_spec = mat[bi16+8];
    // checker: bit 16 of hit-point x and z (p_r = {z, y, x})
    wire use2 = (mat_flag != 32'sd0) && (p_r[16] ^ p_r[80]);

    // total diffuse, clamped to 1.0
    wire signed [32:0] dsum_w = d0_r + d1_r;
    wire signed [31:0] dsum   = (dsum_w > 33'sd65536) ? ONE : dsum_w[31:0];

    // ---- Hit-test unit (closest-hit and both shadow scans) ----
    wire [95:0] q_o = is_sh ? so_r : ro;
    wire [95:0] q_d = is_sh ? (sh_l ? L1V : L0V) : rd;
    wire        i_start = (state == S_CSTART) || (state == S_SSTART);
    wire        i_done, i_hit;
    wire [31:0] a_n, tn_n;
    rt_intersect_s isect (.clk(clk), .start(i_start), .o(q_o), .d(q_d),
                          .v0(v0), .v1(v1), .v2(v2),
                          .done(i_done), .hit(i_hit), .a_n(a_n), .tn_n(tn_n));

    // ---- Divider ----
    wire        div_start = (state == S_DSTART);
    wire        div_done;
    wire [31:0] div_q;
    rt_div divider (.clk(clk), .start(div_start), .tn(tn_n), .a(a_n),
                    .done(div_done), .q(div_q));

    // ---- Control ----
    always @(posedge clk) begin
        pix_valid <= 1'b0;
        if (rst) begin
            state <= S_INIT; ph <= 2'd0; sq <= 2'd0; smp <= 2'd0;
            px <= 0; py <= 0; ti <= 0; depth <= 0; best_id <= 0; best_t <= 0;
            d_row <= D00_V; d_cur <= D00_V;
            done <= 1'b0; is_sh <= 1'b0; sh_l <= 1'b0; have_best <= 1'b0;
            bl0 <= 1'b0; bl1 <= 1'b0; reflect_r <= 1'b0;
            acc_r <= 0; acc_g <= 0; acc_b <= 0; w <= ONE;
            sum_r <= 0; sum_g <= 0; sum_b <= 0;
        end else begin
            case (state)
            // ---------- setup (one AA sample) ----------
            S_INIT: begin
                ro <= CAM_POS;  rd <= add3(d_cur, so_sel);
                acc_r <= 0; acc_g <= 0; acc_b <= 0;
                w <= ONE; depth <= 0; ti <= 0;
                have_best <= 1'b0; is_sh <= 1'b0; sh_l <= 1'b0; ph <= 2'd0;
                state <= S_CSTART;
            end

            // ---------- closest-hit scan ----------
            S_CSTART: state <= S_CWAIT;
            S_CWAIT:  if (i_done) state <= i_hit ? S_DSTART : S_CADV;
            S_DSTART: state <= S_DIV;
            S_DIV: if (div_done) begin
                if (!have_best || $signed(div_q) < best_t) begin
                    best_id   <= ti;
                    best_t    <= div_q;
                    have_best <= 1'b1;
                end
                state <= S_CADV;
            end
            S_CADV: begin
                if (ti == NUM_TRIS - 1) state <= have_best ? S_PT : S_SKY;
                else begin ti <= ti + 1; state <= S_CSTART; end
            end

            // ---------- hit point: p = o + d*t ----------
            S_PT: begin
                if (ph == 0) begin
                    ma0 <= rdx; ma1 <= rdy; ma2 <= rdz;
                    mb0 <= best_t; mb1 <= best_t; mb2 <= best_t;
                end
                if (ph == 2) begin
                    p_r <= {ro[95:64] + r2, ro[63:32] + r1, ro[31:0] + r0};
                    ph <= 0; state <= S_DN;
                end else ph <= ph + 1;
            end

            // ---------- facing normal: flip if dot(d,n) >= 0 ----------
            S_DN: begin
                if (ph == 0) begin
                    ma0 <= nrm_v[31:0]; ma1 <= nrm_v[63:32]; ma2 <= nrm_v[95:64];
                    mb0 <= rdx;         mb1 <= rdy;          mb2 <= rdz;
                end
                if (ph == 2) begin
                    n_r <= rsum[31] ? nrm_v
                                    : {-nrm_v[95:64], -nrm_v[63:32], -nrm_v[31:0]};
                    ph <= 0; state <= S_SO;
                end else ph <= ph + 1;
            end

            // ---------- shadow-ray origin: so = p + n*bias ----------
            S_SO: begin
                if (ph == 0) begin
                    ma0 <= nx; ma1 <= ny; ma2 <= nz;
                    mb0 <= RT_BIAS; mb1 <= RT_BIAS; mb2 <= RT_BIAS;
                end
                if (ph == 2) begin
                    so_r <= {p_r[95:64] + r2, p_r[63:32] + r1, p_r[31:0] + r0};
                    ti <= 0; is_sh <= 1'b1; sh_l <= 1'b0;
                    ph <= 0; state <= S_SSTART;
                end else ph <= ph + 1;
            end

            // ---------- shadow scans: light 0, then light 1 (any hit blocks) ----------
            S_SSTART: state <= S_SWAIT;
            S_SWAIT: if (i_done) begin
                if (i_hit || ti == NUM_TRIS - 1) begin
                    if (!sh_l) begin
                        bl0 <= i_hit; sh_l <= 1'b1; ti <= 0; state <= S_SSTART;
                    end else begin
                        bl1 <= i_hit; state <= S_DL0;
                    end
                end else begin
                    ti <= ti + 1; state <= S_SSTART;
                end
            end

            // ---------- diffuse light 0 = max(0, dot(n, L0)) unless blocked ----------
            S_DL0: begin
                if (ph == 0) begin
                    ma0 <= nx; ma1 <= ny; ma2 <= nz;
                    mb0 <= RT_LIGHT_X; mb1 <= RT_LIGHT_Y; mb2 <= RT_LIGHT_Z;
                end
                if (ph == 2) begin
                    d0_r <= bl0 ? 32'sd0 : ((rsum > 32'sd0) ? rsum : 32'sd0);
                    is_sh <= 1'b0; sh_l <= 1'b0;
                    ph <= 0; state <= S_DL1;
                end else ph <= ph + 1;
            end

            // ---------- diffuse light 1 ----------
            S_DL1: begin
                if (ph == 0) begin
                    ma0 <= nx; ma1 <= ny; ma2 <= nz;
                    mb0 <= RT_L1_X; mb1 <= RT_L1_Y; mb2 <= RT_L1_Z;
                end
                if (ph == 2) begin
                    d1_r <= bl1 ? 32'sd0 : ((rsum > 32'sd0) ? rsum : 32'sd0);
                    ph <= 0; state <= S_LIGHT;
                end else ph <= ph + 1;
            end

            // ---------- light = ambient + (1-ambient)*min(1, d0+d1) ----------
            S_LIGHT: begin
                if (ph == 0) begin
                    ma0 <= ONE - RT_AMBIENT; mb0 <= dsum;
                end
                if (ph == 2) begin
                    light_r <= RT_AMBIENT + r0;
                    ph <= 0; state <= S_RD;
                end else ph <= ph + 1;
            end

            // ---------- R = d - 2*dot(d,n)*n : dn2 ----------
            S_RD: begin
                if (ph == 0) begin
                    ma0 <= rdx; ma1 <= rdy; ma2 <= rdz;
                    mb0 <= nx;  mb1 <= ny;  mb2 <= nz;
                end
                if (ph == 2) begin
                    dn2_r <= {rsum[30:0], 1'b0};
                    ph <= 0; state <= S_RV;
                end else ph <= ph + 1;
            end

            // ---------- R = d - dn2*n (kept in R_r, used for specular and reflection) ----------
            S_RV: begin
                if (ph == 0) begin
                    ma0 <= dn2_r; ma1 <= dn2_r; ma2 <= dn2_r;
                    mb0 <= nx;    mb1 <= ny;    mb2 <= nz;
                end
                if (ph == 2) begin
                    R_r <= {rdz - r2, rdy - r1, rdx - r0};
                    ph <= 0; state <= S_SP0;
                end else ph <= ph + 1;
            end

            // ---------- specular base for light 0: max(0, dot(R, L0)) unless blocked ----------
            S_SP0: begin
                if (ph == 0) begin
                    ma0 <= Rx; ma1 <= Ry; ma2 <= Rz;
                    mb0 <= RT_LIGHT_X; mb1 <= RT_LIGHT_Y; mb2 <= RT_LIGHT_Z;
                end
                if (ph == 2) begin
                    s0_r <= bl0 ? 32'sd0 : ((rsum > 32'sd0) ? rsum : 32'sd0);
                    ph <= 0; state <= S_SP1;
                end else ph <= ph + 1;
            end

            // ---------- specular base for light 1 ----------
            S_SP1: begin
                if (ph == 0) begin
                    ma0 <= Rx; ma1 <= Ry; ma2 <= Rz;
                    mb0 <= RT_L1_X; mb1 <= RT_L1_Y; mb2 <= RT_L1_Z;
                end
                if (ph == 2) begin
                    s1_r <= bl1 ? 32'sd0 : ((rsum > 32'sd0) ? rsum : 32'sd0);
                    sq <= 2'd0;
                    ph <= 0; state <= S_SQ;
                end else ph <= ph + 1;
            end

            // ---------- square both bases three times (^8) ----------
            S_SQ: begin
                if (ph == 0) begin
                    ma0 <= s0_r; mb0 <= s0_r;
                    ma1 <= s1_r; mb1 <= s1_r;
                end
                if (ph == 2) begin
                    s0_r <= r0; s1_r <= r1;
                    ph <= 0;
                    if (sq == 2'd2) begin sq <= 2'd0; state <= S_SPM; end
                    else sq <= sq + 1'b1;
                end else ph <= ph + 1;
            end

            // ---------- sp = spec_strength * (s0 + s1) ----------
            S_SPM: begin
                if (ph == 0) begin
                    ma0 <= mat_spec; mb0 <= s0_r + s1_r;
                end
                if (ph == 2) begin
                    sp_r <= r0;
                    ph <= 0; state <= S_LOC;
                end else ph <= ph + 1;
            end

            // ---------- local color = material * light + specular ----------
            S_LOC: begin
                if (ph == 0) begin
                    ma0 <= use2 ? mat_r2 : mat_r;
                    ma1 <= use2 ? mat_g2 : mat_g;
                    ma2 <= use2 ? mat_b2 : mat_b;
                    mb0 <= light_r; mb1 <= light_r; mb2 <= light_r;
                end
                if (ph == 2) begin
                    loc_r <= r0 + sp_r; loc_g <= r1 + sp_r; loc_b <= r2 + sp_r;
                    reflect_r <= (mat_refl > 32'sd0) && (depth < RT_MAX_BOUNCES);
                    ph <= 0; state <= S_WL;
                end else ph <= ph + 1;
            end

            // ---------- wl = reflect ? w*(1-refl) : w ----------
            S_WL: begin
                if (ph == 0) begin
                    ma0 <= w; mb0 <= ONE - mat_refl;
                end
                if (ph == 2) begin
                    wl_r <= reflect_r ? r0 : w;
                    ph <= 0; state <= S_ACC;
                end else ph <= ph + 1;
            end

            // ---------- acc += wl * local ----------
            S_ACC: begin
                if (ph == 0) begin
                    ma0 <= wl_r; ma1 <= wl_r; ma2 <= wl_r;
                    mb0 <= loc_r; mb1 <= loc_g; mb2 <= loc_b;
                end
                if (ph == 2) begin
                    acc_r <= acc_r + r0;
                    acc_g <= acc_g + r1;
                    acc_b <= acc_b + r2;
                    ph <= 0; state <= reflect_r ? S_WN : S_EMIT;
                end else ph <= ph + 1;
            end

            // ---------- w = w*refl, d = R, o = so, next bounce ----------
            S_WN: begin
                if (ph == 0) begin
                    ma0 <= w; mb0 <= mat_refl;
                end
                if (ph == 2) begin
                    w  <= r0;
                    rd <= R_r;
                    ro <= so_r;
                    depth <= depth + 1; ti <= 0; have_best <= 1'b0;
                    ph <= 0; state <= S_CSTART;
                end else ph <= ph + 1;
            end

            // ---------- miss: sky gradient ----------
            S_SKY: begin
                t_kw = $signed({rd[63], rd[63:32]}) + 33'sd65536;
                t_kw = t_kw >>> 1;
                if (t_kw < 0)          k_r <= 32'sd0;
                else if (t_kw > 65536) k_r <= ONE;
                else                   k_r <= t_kw[31:0];
                ph <= 0; state <= S_SKC;
            end
            S_SKC: begin
                if (ph == 0) begin
                    ma0 <= RT_SKY_HI_R - RT_SKY_LO_R;
                    ma1 <= RT_SKY_HI_G - RT_SKY_LO_G;
                    ma2 <= RT_SKY_HI_B - RT_SKY_LO_B;
                    mb0 <= k_r; mb1 <= k_r; mb2 <= k_r;
                end
                if (ph == 2) begin
                    sk_r <= RT_SKY_LO_R + r0;
                    sk_g <= RT_SKY_LO_G + r1;
                    sk_b <= RT_SKY_LO_B + r2;
                    ph <= 0; state <= S_SKA;
                end else ph <= ph + 1;
            end
            S_SKA: begin
                if (ph == 0) begin
                    ma0 <= w; ma1 <= w; ma2 <= w;
                    mb0 <= sk_r; mb1 <= sk_g; mb2 <= sk_b;
                end
                if (ph == 2) begin
                    acc_r <= acc_r + r0;
                    acc_g <= acc_g + r1;
                    acc_b <= acc_b + r2;
                    ph <= 0; state <= S_EMIT;
                end else ph <= ph + 1;
            end

            // ---------- accumulate AA samples, output pixel, advance ----------
            S_EMIT: begin
                t_ar = (smp == 2'd0) ? acc_r : (sum_r + acc_r);
                t_ag = (smp == 2'd0) ? acc_g : (sum_g + acc_g);
                t_ab = (smp == 2'd0) ? acc_b : (sum_b + acc_b);
                if (smp != 2'd3) begin
                    sum_r <= t_ar; sum_g <= t_ag; sum_b <= t_ab;
                    smp <= smp + 1'b1;
                    state <= S_INIT;
                end else begin
                    pix_valid <= 1'b1;
                    pix_rgb   <= {to8(t_ar >>> 2), to8(t_ag >>> 2), to8(t_ab >>> 2)};
                    smp <= 2'd0;
                    if (px == IMG_W - 1) begin
                        px <= 0;
                        if (py == IMG_H - 1) state <= S_FIN;
                        else begin
                            py <= py + 1;
                            d_row <= add3(d_row, DY_V);
                            d_cur <= add3(d_row, DY_V);
                            state <= S_INIT;
                        end
                    end else begin
                        px <= px + 1;
                        d_cur <= add3(d_cur, DX_V);
                        state <= S_INIT;
                    end
                end
            end

            S_FIN:   done <= 1'b1;
            default: state <= S_FIN;
            endcase
        end
    end
endmodule