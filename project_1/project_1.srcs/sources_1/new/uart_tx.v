`timescale 1ns/1ps

// 8N1 UART transmitter. Pulse start for 1 clock with data; wait for busy to drop.
module uart_tx #(parameter CLKS_PER_BIT = 434)(   // 50 MHz / 115200
    input        clk,
    input        start,
    input  [7:0] data,
    output reg   tx = 1'b1,
    output       busy
);
    reg [9:0]  sh;
    reg [3:0]  bitn = 0;
    reg [15:0] cnt  = 0;
    reg        active = 1'b0;
    assign busy = active;

    always @(posedge clk) begin
        if (!active) begin
            tx <= 1'b1;
            if (start) begin
                sh <= {1'b1, data, 1'b0};   // stop, data[7:0], start
                active <= 1'b1;
                cnt <= 0; bitn <= 0;
                tx <= 1'b0;                 // start bit
            end
        end else begin
            if (cnt == CLKS_PER_BIT - 1) begin
                cnt <= 0;
                if (bitn == 4'd9) begin
                    active <= 1'b0;         // stop bit lasted a full period
                    tx <= 1'b1;
                end else begin
                    bitn <= bitn + 1'b1;
                    tx <= sh[bitn + 1];
                end
            end else
                cnt <= cnt + 1'b1;
        end
    end
endmodule
