`timescale 1ns/1ps

module tb_intersect_s;
    localparam MAXW = 200000;
    reg [31:0] vec [0:MAXW-1];

    reg clk = 0;
    always #5 clk = ~clk;

    reg         start = 0;
    reg  [95:0] o, d, v0, v1, v2;
    wire        done, hit;
    wire [31:0] a_n, tn_n;

    rt_intersect_s dut (.clk(clk), .start(start), .o(o), .d(d),
                        .v0(v0), .v1(v1), .v2(v2),
                        .done(done), .hit(hit), .a_n(a_n), .tn_n(tn_n));

    integer k, base, nvec, errors, hits, last_pct, pct;
    reg [63:0] num, den;
    reg [31:0] t_calc;

    initial begin
        $readmemh("C:/Users/khnay/Desktop/Ray/vectors.mem", vec);
        nvec = vec[0];
        if (^nvec === 1'bx || nvec == 0) begin
            $display("ERROR: vectors.mem did not load."); $finish;
        end
        $display("Testing %0d vectors (shared multipliers)", nvec);
        errors = 0; hits = 0; last_pct = -1;

        for (k = 0; k < nvec; k = k + 1) begin
            base = 1 + k * 19;
            @(negedge clk);
            o  = {vec[base+2],  vec[base+1],  vec[base+0]};
            d  = {vec[base+5],  vec[base+4],  vec[base+3]};
            v0 = {vec[base+8],  vec[base+7],  vec[base+6]};
            v1 = {vec[base+11], vec[base+10], vec[base+9]};
            v2 = {vec[base+14], vec[base+13], vec[base+12]};
            start = 1;
            @(negedge clk);
            start = 0;
            while (done !== 1'b1) @(negedge clk);

            if (hit !== vec[base+15][0]) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH #%0d: hit=%b expected=%b", k, hit, vec[base+15][0]);
            end else if (hit) begin
                hits = hits + 1;
                num = {32'b0, tn_n} << 16;
                den = {32'b0, a_n};
                t_calc = num / den;
                if (a_n !== vec[base+16] || tn_n !== vec[base+17] || t_calc !== vec[base+18]) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("MISMATCH #%0d: a=%h/%h tn=%h/%h t=%h/%h", k,
                                 a_n, vec[base+16], tn_n, vec[base+17], t_calc, vec[base+18]);
                end
            end

            pct = ((k + 1) * 100) / nvec;
            if (pct / 20 != last_pct / 20) begin
                $display("Progress %0d%%", pct);
                last_pct = pct;
            end
        end

        $display("-----------------------------------------");
        $display("Vectors: %0d   Hits checked: %0d   Errors: %0d", nvec, hits, errors);
        if (errors == 0) $display("PASS"); else $display("FAIL");
        $finish;
    end
endmodule