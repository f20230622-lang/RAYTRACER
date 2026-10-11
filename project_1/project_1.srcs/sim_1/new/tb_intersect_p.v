`timescale 1ns/1ps

module tb_intersect_p;
    localparam MAXW = 200000;
    reg [31:0] vec [0:MAXW-1];

    reg clk = 0;
    always #5 clk = ~clk;

    reg         valid_in = 0;
    reg  [95:0] o, d, v0, v1, v2;
    wire        valid_out, hit;
    wire [31:0] a_n, tn_n;

    rt_intersect_p dut (.clk(clk), .valid_in(valid_in), .o(o), .d(d),
                        .v0(v0), .v1(v1), .v2(v2),
                        .valid_out(valid_out), .hit(hit), .a_n(a_n), .tn_n(tn_n));

    integer k, nvec, base, errors, hits, kout, bo;
    reg [63:0] num, den;
    reg [31:0] t_calc;

    // ---- driver: one vector per clock ----
    initial begin
        $readmemh("C:/Users/khnay/Desktop/Ray/vectors.mem", vec);
        nvec = vec[0];
        if (^nvec === 1'bx || nvec == 0) begin
            $display("ERROR: vectors.mem did not load."); $finish;
        end
        $display("Testing %0d vectors (pipelined)", nvec);
        errors = 0; hits = 0; kout = 0;
        for (k = 0; k < nvec; k = k + 1) begin
            @(posedge clk); #1;
            base = 1 + k * 19;
            o  = {vec[base+2],  vec[base+1],  vec[base+0]};
            d  = {vec[base+5],  vec[base+4],  vec[base+3]};
            v0 = {vec[base+8],  vec[base+7],  vec[base+6]};
            v1 = {vec[base+11], vec[base+10], vec[base+9]};
            v2 = {vec[base+14], vec[base+13], vec[base+12]};
            valid_in = 1;
        end
        @(posedge clk); #1; valid_in = 0;
        repeat (20) @(posedge clk);
        $display("-----------------------------------------");
        $display("Vectors: %0d   Outputs seen: %0d   Hits checked: %0d   Errors: %0d",
                 nvec, kout, hits, errors);
        if (errors == 0 && kout == nvec) $display("PASS"); else $display("FAIL");
        $finish;
    end

    // ---- checker: outputs arrive in order, 6 clocks later ----
    always @(posedge clk) if (valid_out) begin
        bo = 1 + kout * 19;
        if (hit !== vec[bo+15][0]) begin
            errors = errors + 1;
            if (errors <= 10)
                $display("MISMATCH #%0d: hit=%b expected=%b", kout, hit, vec[bo+15][0]);
        end else if (hit) begin
            hits = hits + 1;
            num = {32'b0, tn_n} << 16;
            den = {32'b0, a_n};
            t_calc = num / den;
            if (a_n !== vec[bo+16] || tn_n !== vec[bo+17] || t_calc !== vec[bo+18]) begin
                errors = errors + 1;
                if (errors <= 10)
                    $display("MISMATCH #%0d: a=%h/%h tn=%h/%h t=%h/%h", kout,
                             a_n, vec[bo+16], tn_n, vec[bo+17], t_calc, vec[bo+18]);
            end
        end
        kout = kout + 1;
    end
endmodule