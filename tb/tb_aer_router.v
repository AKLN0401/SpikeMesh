`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 02:01:58 PM
// Design Name: 
// Module Name: tb_aer_router
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


// Self-checking testbench. DUT is the router at (X=0, Y=0).
module tb_aer_router;
    reg         clk = 0;
    reg         rst_n = 0;
    reg  [79:0] in_data = 0;
    reg  [4:0]  in_valid = 0;
    wire [4:0]  in_ready;
    wire [79:0] out_data;
    wire [4:0]  out_valid;
    reg  [4:0]  out_ready = 5'b11111;
 
    aer_router #(.X(0), .Y(0), .AW(2)) dut (
        .clk(clk), .rst_n(rst_n),
        .in_data(in_data), .in_valid(in_valid), .in_ready(in_ready),
        .out_data(out_data), .out_valid(out_valid), .out_ready(out_ready)
    );
 
    always #5 clk = ~clk;
 
    // port: 0=L 1=N 2=E 3=S 4=W
    function [15:0] mk;
        input [1:0] t; input dx; input dy; input [7:0] ax; input [3:0] d;
        mk = {t, dx, dy, ax, d};
    endfunction
 
    integer errors = 0;
    integer cnt [0:4];
    reg [15:0] last [0:4];
    integer p, n, accepted, base;
 
    // output monitor
    always @(posedge clk) begin
        for (p = 0; p < 5; p = p + 1)
            if (out_valid[p] && out_ready[p]) begin
                cnt[p]  = cnt[p] + 1;
                last[p] = out_data[p*16 +: 16];
            end
    end
 
    task send;
        input integer port;
        input [15:0] pkt;
        begin
            @(negedge clk);
            in_data[port*16 +: 16] = pkt;
            in_valid[port] = 1'b1;
            @(negedge clk);
            in_valid[port] = 1'b0;
        end
    endtask
 
    task expect_out;
        input integer port;
        input integer exp_cnt;
        input [15:0]  exp_last;
        input [127:0] name;
        begin
            if (cnt[port] !== exp_cnt || last[port] !== exp_last) begin
                $display("FAIL %0s: port %0d cnt=%0d (exp %0d) last=%h (exp %h)",
                         name, port, cnt[port], exp_cnt, last[port], exp_last);
                errors = errors + 1;
            end else
                $display("PASS %0s", name);
        end
    endtask
 
    initial begin
        for (p = 0; p < 5; p = p + 1) begin cnt[p] = 0; last[p] = 16'h0; end
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);
 
        // T1: Local -> (1,0) goes East
        send(0, mk(2'd0, 1'b1, 1'b0, 8'd5, 4'd1));
        repeat (4) @(negedge clk);
        expect_out(2, 1, mk(2'd0, 1'b1, 1'b0, 8'd5, 4'd1), "local->east");
 
        // T2: Local -> (0,1) goes North
        send(0, mk(2'd0, 1'b0, 1'b1, 8'd9, 4'd0));
        repeat (4) @(negedge clk);
        expect_out(1, 1, mk(2'd0, 1'b0, 1'b1, 8'd9, 4'd0), "local->north");
 
        // T3: from East input, dest (0,0) -> Local
        send(2, mk(2'd0, 1'b0, 1'b0, 8'd33, 4'd2));
        repeat (4) @(negedge clk);
        expect_out(0, 1, mk(2'd0, 1'b0, 1'b0, 8'd33, 4'd2), "east->local");
 
        // T4: X before Y: dest (1,1) goes East, not North
        send(3, mk(2'd0, 1'b1, 1'b1, 8'd7, 4'd0));
        repeat (4) @(negedge clk);
        expect_out(2, 2, mk(2'd0, 1'b1, 1'b1, 8'd7, 4'd0), "xy-order");
 
        // T5: contention. Two inputs, same output (East), same cycle.
        base = cnt[2];
        @(negedge clk);
        in_data[1*16 +: 16] = mk(2'd0, 1'b1, 1'b0, 8'd100, 4'd0);
        in_data[3*16 +: 16] = mk(2'd0, 1'b1, 1'b0, 8'd101, 4'd0);
        in_valid[1] = 1'b1; in_valid[3] = 1'b1;
        @(negedge clk);
        in_valid[1] = 1'b0; in_valid[3] = 1'b0;
        repeat (6) @(negedge clk);
        if (cnt[2] !== base + 2) begin
            $display("FAIL contention: delivered %0d, expected 2", cnt[2] - base);
            errors = errors + 1;
        end else
            $display("PASS contention (both delivered)");
 
        // T6: backpressure. East blocked, fill, then release; nothing lost, in order.
        base = cnt[2];
        out_ready[2] = 1'b0;
        accepted = 0;
        for (n = 0; n < 10; n = n + 1) begin
            @(negedge clk);
            in_data[15:0] = mk(2'd0, 1'b1, 1'b0, n[7:0], 4'd0);
            in_valid[0] = 1'b1;
            @(posedge clk);
            if (in_ready[0]) accepted = accepted + 1;
        end
        @(negedge clk);
        in_valid[0] = 1'b0;
        if (in_ready[0] !== 1'b0) begin
            $display("FAIL backpressure: input not stalled");
            errors = errors + 1;
        end
        out_ready[2] = 1'b1;
        repeat (accepted + 6) @(negedge clk);
        if (cnt[2] !== base + accepted || last[2][11:4] !== accepted - 1) begin
            $display("FAIL backpressure: delivered %0d of %0d, last axon %0d",
                     cnt[2] - base, accepted, last[2][11:4]);
            errors = errors + 1;
        end else
            $display("PASS backpressure (%0d accepted, all delivered in order)", accepted);
 
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end
endmodule