`timescale 1ns/1ps

// For each pixel (row-major, row 0 = top): test every triangle, divide on hit,
// keep the smallest t (strict '<', so the lower triangle index wins ties),
// then emit {id, t}. Mirrors render_fixed() in golden.py.
module rt_render #(
    parameter NUM_VERTS = 12,
    parameter NUM_TRIS  = 14,
    parameter IMG_W     = 128,
    parameter IMG_H     = 128,
    parameter signed [31:0] FOV_HW = 32768,       // Q16.16 half-width at z=1
    parameter VERT_FILE = "verts.mem",
    parameter TRI_FILE  = "tris.mem"
)(
    input             clk,
    input             rst,
    output reg        pix_valid,                  // 1-cycle pulse per finished pixel
    output reg [31:0] pix_id,
    output reg [31:0] pix_t,
    output reg        done                        // stays high after the last pixel
);
    localparam [31:0] MISS = 32'hFFFFFFFF;

    // ---- Scene memories (loaded at time 0) ----
    reg [31:0] verts [0:NUM_VERTS*3-1];
    reg [31:0] tris  [0:NUM_TRIS*3-1];
    initial begin
        $readmemh(VERT_FILE, verts);
        $readmemh(TRI_FILE,  tris);
    end

    // ---- Camera constants (same integer math as make_rays_fixed) ----
    localparam signed [31:0] STEP = (2 * FOV_HW) / IMG_W;
    localparam signed [31:0] HH   = (STEP * IMG_H) / 2;

    // ---- State ----
    localparam S_TEST = 3'd0, S_DIV = 3'd1, S_ADV = 3'd2,
               S_EMIT = 3'd3, S_FIN = 3'd4;

    reg [2:0]   state;
    integer     px, py, ti;
    reg [31:0]  best_id;
    reg signed [31:0] best_t;

    // ---- Ray for current pixel (origin = 0, direction z = 1.0) ----
    wire signed [31:0] ray_u = -FOV_HW + (STEP / 2) + px * STEP;
    wire signed [31:0] ray_v =  HH     - (STEP / 2) - py * STEP;
    wire [95:0] ray_d = {32'h00010000, ray_v, ray_u};

    // ---- Current triangle ----
    wire [31:0] i0 = tris[ti*3 + 0];
    wire [31:0] i1 = tris[ti*3 + 1];
    wire [31:0] i2 = tris[ti*3 + 2];
    wire [95:0] v0 = {verts[i0*3+2], verts[i0*3+1], verts[i0*3+0]};
    wire [95:0] v1 = {verts[i1*3+2], verts[i1*3+1], verts[i1*3+0]};
    wire [95:0] v2 = {verts[i2*3+2], verts[i2*3+1], verts[i2*3+0]};

    // ---- Hit test + divider ----
    wire        hit;
    wire [31:0] a_n, tn_n;
    rt_intersect isect (.o(96'd0), .d(ray_d), .v0(v0), .v1(v1), .v2(v2),
                        .hit(hit), .a_n(a_n), .tn_n(tn_n));

    wire        div_start = (state == S_TEST) && hit;
    wire        div_done;
    wire [31:0] div_q;
    rt_div divider (.clk(clk), .start(div_start), .tn(tn_n), .a(a_n),
                    .done(div_done), .q(div_q));

    // ---- Control ----
    always @(posedge clk) begin
        pix_valid <= 1'b0;
        if (rst) begin
            state   <= S_TEST;
            px <= 0; py <= 0; ti <= 0;
            best_id <= MISS;
            best_t  <= 32'sd0;
            done    <= 1'b0;
        end else begin
            case (state)
                S_TEST: state <= hit ? S_DIV : S_ADV;

                S_DIV: if (div_done) begin
                    if (best_id == MISS || $signed(div_q) < best_t) begin
                        best_id <= ti;
                        best_t  <= div_q;
                    end
                    state <= S_ADV;
                end

                S_ADV: begin
                    if (ti == NUM_TRIS - 1) state <= S_EMIT;
                    else begin ti <= ti + 1; state <= S_TEST; end
                end

                S_EMIT: begin
                    pix_valid <= 1'b1;
                    pix_id    <= best_id;
                    pix_t     <= best_t;
                    best_id   <= MISS;
                    best_t    <= 32'sd0;
                    ti        <= 0;
                    if (px == IMG_W - 1) begin
                        px <= 0;
                        if (py == IMG_H - 1) state <= S_FIN;
                        else begin py <= py + 1; state <= S_TEST; end
                    end else begin
                        px <= px + 1;
                        state <= S_TEST;
                    end
                end

                S_FIN: done <= 1'b1;
                default: state <= S_FIN;
            endcase
        end
    end
endmodule