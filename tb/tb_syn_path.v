`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 07:40:17 PM
// Design Name: 
// Module Name: tb_syn_path
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

// Integration testbench: axon_decode -> syn_sram (default parameters).
module tb_syn_path;
    reg         clk = 0;
    reg         rst_n = 0;

    // axon_decode host/spike side
    reg         a_cfg_we = 0;
    reg  [7:0]  a_cfg_addr = 0;
    reg  [15:0] a_cfg_data = 0;
    reg         in_valid = 0;
    wire        in_ready;
    reg  [7:0]  in_axon = 0;
    reg  [3:0]  in_delay = 0;

    // link
    wire        l_valid, l_ready, l_last;
    wire [9:0]  l_addr;
    wire [3:0]  l_delay;

    // syn_sram host/result side
    reg         s_cfg_we = 0;
    reg  [9:0]  s_cfg_addr = 0;
    reg  [15:0] s_cfg_data = 0;
    wire        out_valid;
    reg         out_ready = 1;
    wire [7:0]  out_target, out_weight;
    wire [3:0]  out_delay;
    wire        out_last;

    axon_decode u_ax (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(a_cfg_we), .cfg_addr(a_cfg_addr), .cfg_data(a_cfg_data),
        .in_valid(in_valid), .in_ready(in_ready),
        .in_axon(in_axon), .in_delay(in_delay),
        .out_valid(l_valid), .out_ready(l_ready),
        .out_syn_addr(l_addr), .out_delay(l_delay), .out_last(l_last)
    );

    syn_sram u_sy (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(s_cfg_we), .cfg_addr(s_cfg_addr), .cfg_data(s_cfg_data),
        .in_valid(l_valid), .in_ready(l_ready), .in_addr(l_addr),
        .in_delay(l_delay), .in_last(l_last),
        .out_valid(out_valid), .out_ready(out_ready),
        .out_target(out_target), .out_weight(out_weight),
        .out_delay(out_delay), .out_last(out_last)
    );

    always #5 clk = ~clk;

    // optional ready toggling
    reg toggle_en = 0;
    always @(negedge clk) if (toggle_en) out_ready = ~out_ready;

    integer    got_cnt = 0;
    reg [7:0]  g_tgt [0:127];
    reg [7:0]  g_wgt [0:127];
    reg [3:0]  g_dly [0:127];
    reg        g_lst [0:127];
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

    task a_cfg;
        input [7:0] ax; input [9:0] base; input [5:0] cnt;
        begin
            @(negedge clk);
            a_cfg_addr = ax; a_cfg_data = {base, cnt}; a_cfg_we = 1'b1;
            @(negedge clk);
            a_cfg_we = 1'b0;
        end
    endtask

    task s_cfg;
        input [9:0] a; input [7:0] t; input [7:0] w;
        begin
            @(negedge clk);
            s_cfg_addr = a; s_cfg_data = {t, w}; s_cfg_we = 1'b1;
            @(negedge clk);
            s_cfg_we = 1'b0;
        end
    endtask

    task send;
        input [7:0] ax; input [3:0] d;
        begin
            @(negedge clk);
            in_axon = ax; in_delay = d; in_valid = 1'b1;
            guard = 0;
            while (in_ready !== 1'b1 && guard < 2000) begin
                @(negedge clk); guard = guard + 1;
            end
            @(negedge clk);
            in_valid = 1'b0;
        end
    endtask

    task drain;
        begin
            guard = 0;
            @(negedge clk);
            while (!(in_ready === 1'b1 && l_valid === 1'b0 && out_valid === 1'b0) && guard < 2000) begin
                @(negedge clk); guard = guard + 1;
            end
            repeat (2) @(negedge clk);
        end
    endtask

    task chk;
        input integer idx;
        input [7:0] t; input [7:0] w; input [3:0] d; input l;
        input [255:0] name;
        begin
            if (g_tgt[idx] !== t || g_wgt[idx] !== w || g_dly[idx] !== d || g_lst[idx] !== l) begin
                $display("FAIL %0s: idx %0d got %0d %h %0d %b (exp %0d %h %0d %b)",
                         name, idx, g_tgt[idx], g_wgt[idx], g_dly[idx], g_lst[idx], t, w, d, l);
                errors = errors + 1;
            end else
                $display("PASS %0s", name);
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        // axon 5 -> 100..102, axon 9 -> 200, axon 7 -> none
        a_cfg(8'd5, 10'd100, 6'd3);
        a_cfg(8'd9, 10'd200, 6'd1);
        a_cfg(8'd7, 10'd300, 6'd0);
        s_cfg(10'd100, 8'd10, 8'h05);
        s_cfg(10'd101, 8'd11, 8'hFD);
        s_cfg(10'd102, 8'd12, 8'h14);
        s_cfg(10'd200, 8'd40, 8'h7F);

        // I1: 3-target spike
        start = got_cnt;
        send(8'd5, 4'd2);
        drain;
        if (got_cnt !== start + 3) begin
            $display("FAIL spike axon5: delivered %0d", got_cnt - start); errors = errors + 1;
        end else begin
            chk(start,   8'd10, 8'h05, 4'd2, 1'b0, "axon5 entry 0");
            chk(start+1, 8'd11, 8'hFD, 4'd2, 1'b0, "axon5 entry 1");
            chk(start+2, 8'd12, 8'h14, 4'd2, 1'b1, "axon5 entry 2 (last)");
        end

        // I2: single target
        start = got_cnt;
        send(8'd9, 4'd3);
        drain;
        if (got_cnt !== start + 1) begin
            $display("FAIL spike axon9: delivered %0d", got_cnt - start); errors = errors + 1;
        end else
            chk(start, 8'd40, 8'h7F, 4'd3, 1'b1, "axon9 single");

        // I3: zero fan-out
        start = got_cnt;
        send(8'd7, 4'd1);
        drain;
        if (got_cnt !== start) begin
            $display("FAIL zero fan-out: delivered %0d", got_cnt - start); errors = errors + 1;
        end else $display("PASS zero fan-out");

        // I4: back-to-back spikes
        start = got_cnt;
        send(8'd5, 4'd1);
        send(8'd9, 4'd2);
        drain;
        if (got_cnt !== start + 4) begin
            $display("FAIL back-to-back spikes: delivered %0d", got_cnt - start); errors = errors + 1;
        end else begin
            chk(start,   8'd10, 8'h05, 4'd1, 1'b0, "b2b spike0 e0");
            chk(start+2, 8'd12, 8'h14, 4'd1, 1'b1, "b2b spike0 e2");
            chk(start+3, 8'd40, 8'h7F, 4'd2, 1'b1, "b2b spike1 e0");
        end

        // I5: consumer toggles ready every cycle; nothing lost or duplicated
        toggle_en = 1'b1;
        start = got_cnt;
        send(8'd5, 4'd4);
        send(8'd9, 4'd5);
        send(8'd5, 4'd6);
        drain;
        toggle_en = 1'b0;
        out_ready = 1'b1;
        repeat (4) @(negedge clk);
        if (got_cnt !== start + 7) begin
            $display("FAIL toggled ready: delivered %0d (exp 7)", got_cnt - start); errors = errors + 1;
        end else begin
            chk(start,   8'd10, 8'h05, 4'd4, 1'b0, "toggle s0 e0");
            chk(start+1, 8'd11, 8'hFD, 4'd4, 1'b0, "toggle s0 e1");
            chk(start+2, 8'd12, 8'h14, 4'd4, 1'b1, "toggle s0 e2");
            chk(start+3, 8'd40, 8'h7F, 4'd5, 1'b1, "toggle s1 e0");
            chk(start+4, 8'd10, 8'h05, 4'd6, 1'b0, "toggle s2 e0");
            chk(start+5, 8'd11, 8'hFD, 4'd6, 1'b0, "toggle s2 e1");
            chk(start+6, 8'd12, 8'h14, 4'd6, 1'b1, "toggle s2 e2");
        end

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #400000;
        $display("TIMEOUT");
        $finish;
    end
endmodule