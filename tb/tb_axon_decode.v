`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 04:51:09 PM
// Design Name: 
// Module Name: tb_axon_decode
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


`timescale 1ns/1ps
// Self-checking directed testbench for axon_decode (default parameters).
// LUT entry format: {base[9:0], count[5:0]}.
module tb_axon_decode;
    reg         clk = 0;
    reg         rst_n = 0;
    reg         cfg_we = 0;
    reg  [7:0]  cfg_addr = 0;
    reg  [15:0] cfg_data = 0;
    reg         in_valid = 0;
    wire        in_ready;
    reg  [7:0]  in_axon = 0;
    reg  [3:0]  in_delay = 0;
    wire        out_valid;
    reg         out_ready = 1;
    wire [9:0]  out_syn_addr;
    wire [3:0]  out_delay;
    wire        out_last;

    axon_decode dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(cfg_we), .cfg_addr(cfg_addr), .cfg_data(cfg_data),
        .in_valid(in_valid), .in_ready(in_ready),
        .in_axon(in_axon), .in_delay(in_delay),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_syn_addr(out_syn_addr), .out_delay(out_delay), .out_last(out_last)
    );

    always #5 clk = ~clk;

    // ---------------- output collector ----------------
    integer    got_cnt = 0;
    reg [9:0]  got_addr [0:127];
    reg        got_last [0:127];
    reg [3:0]  got_dly  [0:127];

    always @(posedge clk) begin
        if (out_valid && out_ready) begin
            got_addr[got_cnt] = out_syn_addr;
            got_last[got_cnt] = out_last;
            got_dly [got_cnt] = out_delay;
            got_cnt = got_cnt + 1;
        end
    end

    // ---------------- helpers ----------------
    integer errors = 0;
    integer i, start, guard;

    task cfg_write;
        input [7:0] axon;
        input [9:0] base;
        input [5:0] count;
        begin
            @(negedge clk);
            cfg_addr = axon;
            cfg_data = {base, count};
            cfg_we   = 1'b1;
            @(negedge clk);
            cfg_we   = 1'b0;
        end
    endtask

    // Present a spike; hold it until accepted, then drop valid.
    task send;
        input [7:0] axon;
        input [3:0] dly;
        begin
            @(negedge clk);
            in_axon  = axon;
            in_delay = dly;
            in_valid = 1'b1;
            guard = 0;
            while (in_ready !== 1'b1 && guard < 1000) begin
                @(negedge clk);
                guard = guard + 1;
            end
            @(negedge clk);          // accepted at the posedge in between
            in_valid = 1'b0;
        end
    endtask

    // Wait until the DUT is idle (ready for a spike and not emitting).
    task wait_idle;
        begin
            guard = 0;
            @(negedge clk);
            while (!(in_ready === 1'b1 && out_valid === 1'b0) && guard < 1000) begin
                @(negedge clk);
                guard = guard + 1;
            end
            @(negedge clk);
        end
    endtask

    // Check entries [from, from+cnt) are base, base+1, ... with last only on the final one.
    task check_burst;
        input integer from;
        input [9:0]   base;
        input integer cnt;
        input [3:0]   dly;
        input [255:0] name;
        integer j;
        reg bad;
        begin
            bad = 0;
            if (got_cnt !== from + cnt) bad = 1;
            else
                for (j = 0; j < cnt; j = j + 1)
                    if (got_addr[from+j] !== base + j[9:0] ||
                        got_last[from+j] !== (j == cnt-1) ||
                        got_dly [from+j] !== dly)
                        bad = 1;
            if (bad) begin
                $display("FAIL %0s: got_cnt=%0d (exp %0d)", name, got_cnt, from + cnt);
                errors = errors + 1;
            end else
                $display("PASS %0s", name);
        end
    endtask

    // ---------------- tests ----------------
    initial begin
        repeat (3) @(negedge clk);
        if (in_ready !== 1'b0 && in_ready !== 1'b1) $display("note: in_ready unknown in reset");
        rst_n = 1;
        repeat (2) @(negedge clk);

        if (in_ready !== 1'b1 || out_valid !== 1'b0) begin
            $display("FAIL reset state: in_ready=%b out_valid=%b", in_ready, out_valid);
            errors = errors + 1;
        end else
            $display("PASS reset state");

        // LUT: axon 5 -> base 100 x3, axon 9 -> base 200 x1, axon 7 -> none,
        //      axon 255 -> base 500 x63
        cfg_write(8'd5,   10'd100, 6'd3);
        cfg_write(8'd9,   10'd200, 6'd1);
        cfg_write(8'd7,   10'd300, 6'd0);
        cfg_write(8'd255, 10'd500, 6'd63);

        // T1: three-target burst
        start = got_cnt;
        send(8'd5, 4'd3);
        wait_idle;
        check_burst(start, 10'd100, 3, 4'd3, "burst of 3");

        // T2: single target, last asserted on the only entry
        start = got_cnt;
        send(8'd9, 4'd7);
        wait_idle;
        check_burst(start, 10'd200, 1, 4'd7, "single target");

        // T3: count = 0, spike is dropped and DUT returns to idle
        start = got_cnt;
        send(8'd7, 4'd1);
        wait_idle;
        check_burst(start, 10'd0, 0, 4'd0, "zero fan-out dropped");

        // T4: backpressure. Output stalled: address must hold, input must stall.
        out_ready = 1'b0;
        start = got_cnt;
        send(8'd5, 4'd2);
        repeat (6) @(negedge clk);
        if (out_valid !== 1'b1 || out_syn_addr !== 10'd100 || in_ready !== 1'b0 ||
            got_cnt !== start) begin
            $display("FAIL backpressure hold: out_valid=%b addr=%0d in_ready=%b delivered=%0d",
                     out_valid, out_syn_addr, in_ready, got_cnt - start);
            errors = errors + 1;
        end else
            $display("PASS backpressure hold");
        out_ready = 1'b1;
        wait_idle;
        check_burst(start, 10'd100, 3, 4'd2, "backpressure release");

        // T5: back-to-back spikes, second waits for in_ready
        start = got_cnt;
        send(8'd5, 4'd1);
        send(8'd9, 4'd2);
        wait_idle;
        if (got_cnt !== start + 4 ||
            got_addr[start]   !== 10'd100 || got_dly[start]   !== 4'd1 ||
            got_addr[start+1] !== 10'd101 || got_dly[start+1] !== 4'd1 ||
            got_addr[start+2] !== 10'd102 || got_dly[start+2] !== 4'd1 ||
            got_last[start+2] !== 1'b1 ||
            got_addr[start+3] !== 10'd200 || got_dly[start+3] !== 4'd2 ||
            got_last[start+3] !== 1'b1) begin
            $display("FAIL back-to-back: delivered %0d (exp 4)", got_cnt - start);
            errors = errors + 1;
        end else
            $display("PASS back-to-back");

        // T6: maximum fan-out (63)
        start = got_cnt;
        send(8'd255, 4'd15);
        wait_idle;
        check_burst(start, 10'd500, 63, 4'd15, "max fan-out 63");

        // T7: reconfigure an existing axon
        cfg_write(8'd5, 10'd300, 6'd2);
        start = got_cnt;
        send(8'd5, 4'd4);
        wait_idle;
        check_burst(start, 10'd300, 2, 4'd4, "LUT reconfigure");

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

    // watchdog
    initial begin
        #200000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
