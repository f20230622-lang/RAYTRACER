`timescale 1ns/1ps

module top_uart (
    input  clk,          // 100 MHz
    input  btnC,         // reset / re-render
    output RsTx,         // to the USB-UART bridge
    output led0          // on when the frame is finished
);
    `include "C:/Users/khnay/Desktop/Ray/params.vh"

    localparam NPIX = IMG_W * IMG_H;
    localparam AW   = $clog2(NPIX);

    // ---- 100 MHz -> 50 MHz ----
    wire fb_o, c0, c0_b, locked;
    MMCME2_BASE #(
        .CLKIN1_PERIOD(10.0), .DIVCLK_DIVIDE(1), .CLKFBOUT_MULT_F(8.0),
        .CLKOUT0_DIVIDE_F(16.0)
    ) mmcm (
        .CLKIN1(clk), .CLKFBIN(fb_o), .CLKFBOUT(fb_o),
        .CLKOUT0(c0), .LOCKED(locked), .PWRDWN(1'b0), .RST(1'b0)
    );
    BUFG bg (.I(c0), .O(c0_b));

    reg [3:0] rs = 4'hF;
    always @(posedge c0_b) rs <= {rs[2:0], ~locked | btnC};
    wire rst = rs[3];

    // ---- Ray tracer ----
    wire        pix_valid, done;
    wire [23:0] pix_rgb;
    rt_trace_s #(.NUM_VERTS(NUM_VERTS), .NUM_TRIS(NUM_TRIS),
                 .IMG_W(IMG_W), .IMG_H(IMG_H), .FOV_HW(FOV_HW)) rt (
        .clk(c0_b), .rst(rst),
        .pix_valid(pix_valid), .pix_rgb(pix_rgb), .done(done));
    assign led0 = done;

    // ---- Framebuffer (BRAM) ----
    (* ram_style = "block" *) reg [23:0] fb [0:NPIX-1];
    reg [AW-1:0] wa;
    reg [AW-1:0] pi;
    reg [23:0]   rdata;
    always @(posedge c0_b) begin
        if (rst) wa <= 0;
        else if (pix_valid) begin
            fb[wa] <= pix_rgb;
            wa <= wa + 1'b1;
        end
        rdata <= fb[pi];
    end

    // ---- UART ----
    reg        tx_start = 1'b0;
    reg  [7:0] tx_data  = 8'd0;
    wire       busy;
    uart_tx #(.CLKS_PER_BIT(434)) utx (
        .clk(c0_b), .start(tx_start), .data(tx_data), .tx(RsTx), .busy(busy));

    // ---- Sender: header, then every pixel as R,G,B; repeat forever ----
    localparam S_WAIT = 3'd0, S_HDR = 3'd1, S_RD = 3'd2,
               S_RD2  = 3'd3, S_SEND = 3'd4, S_GAP = 3'd5;
    reg [2:0]  st = S_WAIT;
    reg [1:0]  hi = 0, bi = 0;
    reg [23:0] px = 0;
    reg [23:0] gap = 0;
    wire can_send = !busy && !tx_start;

    always @(posedge c0_b) begin
        tx_start <= 1'b0;
        if (rst) begin
            st <= S_WAIT; hi <= 0; bi <= 0; pi <= 0; gap <= 0;
        end else case (st)
            S_WAIT: if (done) begin hi <= 0; st <= S_HDR; end

            S_HDR: if (can_send) begin
                tx_data  <= (hi == 2'd0) ? 8'hA5 :
                            (hi == 2'd1) ? 8'h5A :
                            (hi == 2'd2) ? IMG_W[7:0] : IMG_H[7:0];
                tx_start <= 1'b1;
                if (hi == 2'd3) begin pi <= 0; st <= S_RD; end
                else hi <= hi + 1'b1;
            end

            S_RD:  st <= S_RD2;                       // BRAM read latency
            S_RD2: begin px <= rdata; bi <= 0; st <= S_SEND; end

            S_SEND: if (can_send) begin
                tx_data  <= (bi == 2'd0) ? px[23:16] :
                            (bi == 2'd1) ? px[15:8]  : px[7:0];
                tx_start <= 1'b1;
                if (bi == 2'd2) begin
                    if (pi == NPIX - 1) begin gap <= 0; st <= S_GAP; end
                    else begin pi <= pi + 1'b1; st <= S_RD; end
                end else bi <= bi + 1'b1;
            end

            S_GAP: begin                              // about 0.33 s, then resend
                gap <= gap + 1'b1;
                if (gap == 24'hFFFFFF) begin hi <= 0; st <= S_HDR; end
            end
            default: st <= S_WAIT;
        endcase
    end
endmodule