`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 06:59:38 PM
// Design Name: 
// Module Name: tb_syn_ram
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

// Self-checking testbench for syn_sram (default parameters).
// Entry = {target[7:0], weight[7:0]}.
module tb_syn_sram;
    reg         clk = 0;
    reg         rst_n = 0;
    reg         cfg_we = 0;
    reg  [9:0]  cfg_addr = 0;
    reg  [15:0] cfg_data = 0;
    reg         in_valid = 0;
    wire        in_ready;
    reg  [9:0]  in_addr = 0;
    reg  [3:0]  in_delay = 0;
    reg         in_last = 0;
    wire        out_valid;
    reg         out_ready = 1;
    wire [7:0]  out_target;
    wire [7:0]  out_weight;
    wire [3:0]  out_delay;
    wire        out_last;

    syn_sram dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(cfg_we), .cfg_addr(cfg_addr), .cfg_data(cfg_data),
        .in_valid(in_valid), .in_ready(in_ready), .in_addr(in_addr),
        .in_delay(in_delay), .in_last(in_last),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_target(out_target), .out_weight(out_weight),
        .out_delay(out_delay), .out_last(out_last)
    );

    always #5 clk = ~clk;

    // collector
    integer    got_cnt = 0;
    reg [7:0]  g_tgt [0:63];
    reg [7:0]  g_wgt [0:63];
    reg [3:0]  g_dly [0:63];
    reg        g_lst [0:63];
    always @(posedge clk) begin
        if (out_valid && out_ready) begin
            g_tgt[got_cnt] = out_target;
            g_wgt[got_cnt] = out_weight;
            g_dly[got_cnt] = out_delay;
            g_lst[got_cnt] = out_last;
            got_cnt = got_cnt + 1;
        end
    end

    integer errors = 0;
    integer start, guard;
    reg [7:0] hold_t, hold_w;

    task cfg_write;
        input [9:0] a; input [7:0] t; input [7:0] w;
        begin
            @(negedge clk);
            cfg_addr = a; cfg_data = {t, w}; cfg_we = 1'b1;
            @(negedge clk);
            cfg_we = 1'b0;
        end
    endtask

    // one request, held until accepted
    task issue;
        input [9:0] a; input [3:0] d; input l;
        begin
            @(negedge clk);
            in_addr = a; in_delay = d; in_last = l; in_valid = 1'b1;
            guard = 0;
            while (in_ready !== 1'b1 && guard < 100) begin
                @(negedge clk); guard = guard + 1;
            end
            @(negedge clk);
            in_valid = 1'b0;
        end
    endtask

    task expect_entry;
        input integer idx;
        input [7:0] t; input [7:0] w; input [3:0] d; input l;
        input [255:0] name;
        begin
            if (g_tgt[idx] !== t || g_wgt[idx] !== w || g_dly[idx] !== d || g_lst[idx] !== l) begin
                $display("FAIL %0s: got tgt=%0d w=%h dly=%0d last=%b (exp %0d %h %0d %b)",
                         name, g_tgt[idx], g_wgt[idx], g_dly[idx], g_lst[idx], t, w, d, l);
                errors = errors + 1;
            end else
                $display("PASS %0s", name);
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        if (out_valid !== 1'b0 || in_ready !== 1'b1) begin
            $display("FAIL reset state"); errors = errors + 1;
        end else $display("PASS reset state");

        cfg_write(10'd10, 8'd1, 8'h05);
        cfg_write(10'd11, 8'd2, 8'hFD);   // -3
        cfg_write(10'd12, 8'd3, 8'h7F);

        // T1: single read, 1-cycle latency, sideband carried
        start = got_cnt;
        issue(10'd11, 4'd9, 1'b1);
        repeat (3) @(negedge clk);
        if (got_cnt !== start + 1) begin
            $display("FAIL single read: delivered %0d", got_cnt - start); errors = errors + 1;
        end else
            expect_entry(start, 8'd2, 8'hFD, 4'd9, 1'b1, "single read");

        // T2: back-to-back reads, one per cycle, no bubbles
        start = got_cnt;
        @(negedge clk);
        in_valid = 1'b1; in_addr = 10'd10; in_delay = 4'd1; in_last = 1'b0;
        @(negedge clk);
        in_addr = 10'd11; in_last = 1'b0;
        @(negedge clk);
        in_addr = 10'd12; in_last = 1'b1;
        @(negedge clk);
        in_valid = 1'b0;
        repeat (3) @(negedge clk);
        if (got_cnt !== start + 3) begin
            $display("FAIL back-to-back: delivered %0d", got_cnt - start); errors = errors + 1;
        end else begin
            expect_entry(start,   8'd1, 8'h05, 4'd1, 1'b0, "b2b entry 0");
            expect_entry(start+1, 8'd2, 8'hFD, 4'd1, 1'b0, "b2b entry 1");
            expect_entry(start+2, 8'd3, 8'h7F, 4'd1, 1'b1, "b2b entry 2");
        end

        // T3: stalled consumer holds data and back-pressures the sender
        out_ready = 1'b0;
        start = got_cnt;
        issue(10'd10, 4'd5, 1'b0);              // A
        @(negedge clk);
        hold_t = out_target; hold_w = out_weight;
        @(negedge clk);
        in_addr = 10'd12; in_delay = 4'd6; in_last = 1'b1; in_valid = 1'b1;   // B
        repeat (3) @(negedge clk);
        if (out_valid !== 1'b1 || in_ready !== 1'b0 || got_cnt !== start ||
            out_target !== hold_t || out_weight !== hold_w) begin
            $display("FAIL stall hold: out_valid=%b in_ready=%b delivered=%0d tgt=%0d",
                     out_valid, in_ready, got_cnt - start, out_target);
            errors = errors + 1;
        end else $display("PASS stall hold");
        out_ready = 1'b1;                        // A drains while B is accepted
        @(negedge clk);
        in_valid = 1'b0;
        repeat (3) @(negedge clk);
        if (got_cnt !== start + 2) begin
            $display("FAIL stall release: delivered %0d", got_cnt - start); errors = errors + 1;
        end else begin
            expect_entry(start,   8'd1, 8'h05, 4'd5, 1'b0, "stall release A");
            expect_entry(start+1, 8'd3, 8'h7F, 4'd6, 1'b1, "stall release B");
        end

        // T4: overwrite an entry, read returns new data
        cfg_write(10'd11, 8'd200, 8'h42);
        start = got_cnt;
        issue(10'd11, 4'd0, 1'b1);
        repeat (3) @(negedge clk);
        if (got_cnt !== start + 1) begin
            $display("FAIL overwrite: delivered %0d", got_cnt - start); errors = errors + 1;
        end else
            expect_entry(start, 8'd200, 8'h42, 4'd0, 1'b1, "overwrite");

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #100000;
        $display("TIMEOUT");
        $finish;
    end
endmodule