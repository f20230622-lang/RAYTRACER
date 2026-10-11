`timescale 1ns/1ps

// Ray tracer FSM: primary + shadow + reflection rays, RGB output.
// Mirrors trace_pixel() in golden_rt.py bit for bit.
module rt_trace #(
    parameter NUM_VERTS = 12,
    parameter NUM_TRIS  = 14,
    parameter IMG_W     = 64,
    parameter IMG_H     = 64,
    parameter signed [31:0] FOV_HW = 32768,
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
    localparam signed [31:0] STEP = (2 * FOV_HW) / IMG_W;
    localparam signed [31:0] HH   = (STEP * IMG_H) / 2;

    // ---- Memories ----
    reg [31:0] verts [0:NUM_VERTS*3-1];
    reg [31:0] tris  [0:NUM_TRIS*3-1];
    reg [31:0] nrm   [0:NUM_TRIS*3-1];
    reg [31:0] mat   [0:NUM_TRIS*4-1];
    initial begin
        $readmemh(VERT_FILE, verts);
        $readmemh(TRI_FILE,  tris);
        $readmemh(NORM_FILE, nrm);
        $readmemh(MAT_FILE,  mat);
    end

    // ---- Fixed-point helpers (same math as golden.py) ----
    function signed [31:0] mul;
        input signed [31:0] x, y;
        reg signed [63:0] p;
        begin p = x * y; mul = p >>> 16; end
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

    // base + vec * s   (per component)
    function [95:0] fma3;
        input [95:0] base, vec;
        input signed [31:0] s;
        reg signed [31:0] vx, vy, vz, bx, by, bz;
        begin
            vx = vec[31:0];  vy = vec[63:32];  vz = vec[95:64];
            bx = base[31:0]; by = base[63:32]; bz = base[95:64];
            fma3 = {bz + mul(vz, s), by + mul(vy, s), bx + mul(vx, s)};
        end
    endfunction

    function [95:0] neg3;
        input [95:0] a;
        reg signed [31:0] x, y, z;
        begin
            x = a[31:0]; y = a[63:32]; z = a[95:64];
            neg3 = {-z, -y, -x};
        end
    endfunction

    function [7:0] to8;
        input signed [31:0] x;
        begin
            if (x[31])              to8 = 8'd0;
            else if (x[30:8] > 255) to8 = 8'd255;
            else                    to8 = x[15:8];
        end
    endfunction

    // ---- State ----
    localparam S_INIT  = 4'd0,  S_TEST  = 4'd1,  S_DIV   = 4'd2,  S_ADV   = 4'd3,
               S_CDONE = 4'd4,  S_SKY   = 4'd5,  S_POINT = 4'd6,  S_STEST = 4'd7,
               S_SHADE = 4'd8,  S_EMIT  = 4'd9,  S_FIN   = 4'd10;

    reg [3:0] state;
    integer   px, py, ti, depth;

    reg [95:0] ro, rd;              // current ray
    reg [95:0] n_r, so_r;           // facing normal, offset hit point
    reg signed [31:0] w;            // reflection weight
    reg signed [31:0] acc_r, acc_g, acc_b;
    reg        have_best, blocked, is_sh;
    integer    best_id;
    reg signed [31:0] best_t;

    // ---- Primary ray for the current pixel ----
    wire signed [31:0] ray_u = -FOV_HW + (STEP / 2) + px * STEP;
    wire signed [31:0] ray_v =  HH     - (STEP / 2) - py * STEP;
    wire [95:0] ray_d = {ONE, ray_v, ray_u};

    wire [95:0] L_vec = {RT_LIGHT_Z, RT_LIGHT_Y, RT_LIGHT_X};

    // ---- Intersection unit shared by closest-hit and shadow queries ----
    wire [31:0] i0 = tris[ti*3 + 0];
    wire [31:0] i1 = tris[ti*3 + 1];
    wire [31:0] i2 = tris[ti*3 + 2];
    wire [95:0] v0 = {verts[i0*3+2], verts[i0*3+1], verts[i0*3+0]};
    wire [95:0] v1 = {verts[i1*3+2], verts[i1*3+1], verts[i1*3+0]};
    wire [95:0] v2 = {verts[i2*3+2], verts[i2*3+1], verts[i2*3+0]};

    wire [95:0] q_o = is_sh ? so_r  : ro;
    wire [95:0] q_d = is_sh ? L_vec : rd;

    wire        hit;
    wire [31:0] a_n, tn_n;
    rt_intersect isect (.o(q_o), .d(q_d), .v0(v0), .v1(v1), .v2(v2),
                        .hit(hit), .a_n(a_n), .tn_n(tn_n));

    wire        div_start = (state == S_TEST) && hit;
    wire        div_done;
    wire [31:0] div_q;
    rt_div divider (.clk(clk), .start(div_start), .tn(tn_n), .a(a_n),
                    .done(div_done), .q(div_q));

    // ---- Temporaries (blocking, used within one state) ----
    reg [95:0] t_p, t_n, t_so, t_nr, t_rd;
    reg signed [31:0] t_dn, t_dl, t_diff, t_light, t_wl, t_dn2, t_k;
    reg signed [31:0] t_mr, t_mg, t_mb, t_refl, t_lr, t_lg, t_lb;
    reg signed [32:0] t_kw;
    reg signed [31:0] t_sr, t_sg, t_sb;
    reg               t_reflect;

    always @(posedge clk) begin
        pix_valid <= 1'b0;
        if (rst) begin
            state <= S_INIT;
            px <= 0; py <= 0; ti <= 0; depth <= 0;
            done <= 1'b0; is_sh <= 1'b0; have_best <= 1'b0;
            blocked <= 1'b0; best_id <= 0; best_t <= 0;
            acc_r <= 0; acc_g <= 0; acc_b <= 0; w <= ONE;
        end else begin
            case (state)
                S_INIT: begin
                    ro <= 96'd0;  rd <= ray_d;
                    acc_r <= 0; acc_g <= 0; acc_b <= 0;
                    w <= ONE; depth <= 0; ti <= 0;
                    have_best <= 1'b0; is_sh <= 1'b0;
                    state <= S_TEST;
                end

                // ---- closest-hit scan ----
                S_TEST: state <= hit ? S_DIV : S_ADV;

                S_DIV: if (div_done) begin
                    if (!have_best || $signed(div_q) < best_t) begin
                        best_id   <= ti;
                        best_t    <= div_q;
                        have_best <= 1'b1;
                    end
                    state <= S_ADV;
                end

                S_ADV: begin
                    if (ti == NUM_TRIS - 1) state <= S_CDONE;
                    else begin ti <= ti + 1; state <= S_TEST; end
                end

                S_CDONE: state <= have_best ? S_POINT : S_SKY;

                // ---- miss: sky gradient ----
                S_SKY: begin
                    t_rd = rd;
                    t_kw = $signed({t_rd[63], t_rd[63:32]}) + 33'sd65536;
                    t_kw = t_kw >>> 1;
                    if (t_kw < 0)           t_k = 32'sd0;
                    else if (t_kw > 65536)  t_k = ONE;
                    else                    t_k = t_kw[31:0];
                    t_sr = RT_SKY_LO_R + mul(RT_SKY_HI_R - RT_SKY_LO_R, t_k);
                    t_sg = RT_SKY_LO_G + mul(RT_SKY_HI_G - RT_SKY_LO_G, t_k);
                    t_sb = RT_SKY_LO_B + mul(RT_SKY_HI_B - RT_SKY_LO_B, t_k);
                    acc_r <= acc_r + mul(w, t_sr);
                    acc_g <= acc_g + mul(w, t_sg);
                    acc_b <= acc_b + mul(w, t_sb);
                    state <= S_EMIT;
                end

                // ---- hit: point, facing normal, shadow-ray origin ----
                S_POINT: begin
                    t_p  = fma3(ro, rd, best_t);
                    t_nr = {nrm[best_id*3+2], nrm[best_id*3+1], nrm[best_id*3+0]};
                    t_dn = dot3(rd, t_nr);
                    t_n  = (t_dn < 0) ? t_nr : neg3(t_nr);
                    t_so = fma3(t_p, t_n, RT_BIAS);
                    n_r   <= t_n;
                    so_r  <= t_so;
                    is_sh <= 1'b1;
                    ti    <= 0;
                    state <= S_STEST;
                end

                // ---- shadow scan: any hit blocks the light ----
                S_STEST: begin
                    if (hit) begin
                        blocked <= 1'b1;  state <= S_SHADE;
                    end else if (ti == NUM_TRIS - 1) begin
                        blocked <= 1'b0;  state <= S_SHADE;
                    end else
                        ti <= ti + 1;
                end

                // ---- shade, accumulate, maybe spawn reflection ray ----
                S_SHADE: begin
                    is_sh   <= 1'b0;
                    t_dl    = dot3(n_r, L_vec);
                    t_diff  = blocked ? 32'sd0 : ((t_dl > 0) ? t_dl : 32'sd0);
                    t_light = RT_AMBIENT + mul(ONE - RT_AMBIENT, t_diff);
                    t_mr    = mat[best_id*4 + 0];
                    t_mg    = mat[best_id*4 + 1];
                    t_mb    = mat[best_id*4 + 2];
                    t_refl  = mat[best_id*4 + 3];
                    t_lr = mul(t_mr, t_light);
                    t_lg = mul(t_mg, t_light);
                    t_lb = mul(t_mb, t_light);
                    t_reflect = (t_refl > 0) && (depth < RT_MAX_BOUNCES);
                    t_wl = t_reflect ? mul(w, ONE - t_refl) : w;
                    acc_r <= acc_r + mul(t_wl, t_lr);
                    acc_g <= acc_g + mul(t_wl, t_lg);
                    acc_b <= acc_b + mul(t_wl, t_lb);

                    if (!t_reflect) state <= S_EMIT;
                    else begin
                        t_dn2 = dot3(rd, n_r) <<< 1;          // s32(2*dot(d,n))
                        t_rd  = {rd[95:64] - mul(t_dn2, n_r[95:64]),
                                 rd[63:32] - mul(t_dn2, n_r[63:32]),
                                 rd[31:0]  - mul(t_dn2, n_r[31:0])};
                        rd    <= t_rd;
                        ro    <= so_r;
                        w     <= mul(w, t_refl);
                        depth <= depth + 1;
                        ti    <= 0;
                        have_best <= 1'b0;
                        state <= S_TEST;
                    end
                end

                // ---- output pixel, advance ----
                S_EMIT: begin
                    pix_valid <= 1'b1;
                    pix_rgb   <= {to8(acc_r), to8(acc_g), to8(acc_b)};
                    if (px == IMG_W - 1) begin
                        px <= 0;
                        if (py == IMG_H - 1) state <= S_FIN;
                        else begin py <= py + 1; state <= S_INIT; end
                    end else begin
                        px <= px + 1; state <= S_INIT;
                    end
                end

                S_FIN:   done <= 1'b1;
                default: state <= S_FIN;
            endcase
        end
    end
endmodule