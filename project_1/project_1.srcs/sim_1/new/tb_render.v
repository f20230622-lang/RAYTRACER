`timescale 1ns/1ps

module tb_render;
    `include "C:/Users/khnay/Desktop/Ray/params.vh"

    reg clk = 0, rst = 1;
    always #5 clk = ~clk;

    wire        pix_valid, done;
    wire [31:0] pix_id, pix_t;

    rt_render #(
        .NUM_VERTS(NUM_VERTS), .NUM_TRIS(NUM_TRIS),
        .IMG_W(IMG_W), .IMG_H(IMG_H), .FOV_HW(FOV_HW),
        .VERT_FILE("C:/Users/khnay/Desktop/Ray/verts.mem"),
        .TRI_FILE ("C:/Users/khnay/Desktop/Ray/tris.mem")
    ) dut (.clk(clk), .rst(rst), .pix_valid(pix_valid),
           .pix_id(pix_id), .pix_t(pix_t), .done(done));

    integer fd, n, pct, last_pct, k;

    task print_bar;
        input integer percent;
        begin
            $write("Rendering   [");
            for (k = 0; k < 20; k = k + 1)
                if (k < percent / 5) $write("#"); else $write("-");
            $display("] %0d%%", percent);
        end
    endtask

    initial begin
        fd = $fopen("C:/Users/khnay/Desktop/Ray/hw_out.txt", "w");
        n = 0; last_pct = -1;
        repeat (4) @(posedge clk);
        rst <= 1'b0;
    end

    always @(posedge clk) begin
        if (pix_valid) begin
            $fdisplay(fd, "%08x %08x", pix_id, pix_t);
            n = n + 1;
            pct = (n * 100) / (IMG_W * IMG_H);
            if (pct / 5 != last_pct / 5) begin
                print_bar(pct);
                last_pct = pct;
            end
        end
        if (done) begin
            $fclose(fd);
            $display("Done. Wrote %0d pixels at t = %0t", n, $time);
            $finish;
        end
    end
endmodule