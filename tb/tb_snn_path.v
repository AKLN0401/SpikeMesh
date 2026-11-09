`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 08:17:23 PM
// Design Name: 
// Module Name: tb_snn_path
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

// Integration testbench: axon_decode -> syn_sram -> neuron_unit (16 neurons).
// All neurons: thr = 80, leak shift = 2, refractory period 2 (module default).
module tb_snn_path;
    localparam NA = 4;

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

    // axon_decode -> syn_sram
    wire        l_valid, l_ready, l_last;
    wire [9:0]  l_addr;
    wire [3:0]  l_delay;

    // syn_sram host side
    reg         s_cfg_we = 0;
    reg  [9:0]  s_cfg_addr = 0;
    reg  [NA+8-1:0] s_cfg_data = 0;

    // syn_sram -> neuron_unit
    wire        e_valid, e_ready;
    wire [NA-1:0] e_target;
    wire [7:0]  e_weight;
    wire [3:0]  e_delay;
    wire        e_last;

    // neuron_unit host / outputs
    reg         n_cfg_we = 0;
    reg  [NA-1:0] n_cfg_addr = 0;
    reg  [38:0] n_cfg_data = 0;
    reg         tick = 0;
    wire        spk_valid;
    reg         spk_ready = 1;
    wire [NA-1:0] spk_id;
    wire        busy, tick_missed;

    axon_decode u_ax (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(a_cfg_we), .cfg_addr(a_cfg_addr), .cfg_data(a_cfg_data),
        .in_valid(in_valid), .in_ready(in_ready),
        .in_axon(in_axon), .in_delay(in_delay),
        .out_valid(l_valid), .out_ready(l_ready),
        .out_syn_addr(l_addr), .out_delay(l_delay), .out_last(l_last)
    );

    syn_sram #(.NEU_AW(NA)) u_sy (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(s_cfg_we), .cfg_addr(s_cfg_addr), .cfg_data(s_cfg_data),
        .in_valid(l_valid), .in_ready(l_ready), .in_addr(l_addr),
        .in_delay(l_delay), .in_last(l_last),
        .out_valid(e_valid), .out_ready(e_ready),
        .out_target(e_target), .out_weight(e_weight),
        .out_delay(e_delay), .out_last(e_last)
    );

    neuron_unit #(.NEU_AW(NA)) u_nu (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(n_cfg_we), .cfg_addr(n_cfg_addr), .cfg_data(n_cfg_data),
        .tick(tick),
        .ev_valid(e_valid), .ev_ready(e_ready),
        .ev_target(e_target), .ev_weight(e_weight),
        .spk_valid(spk_valid), .spk_ready(spk_ready), .spk_id(spk_id),
        .busy(busy), .tick_missed(tick_missed)
    );

    always #5 clk = ~clk;

    // spike collector
    integer got_cnt = 0;
    integer got_id [0:63];
    always @(posedge clk)
        if (spk_valid && spk_ready) begin
            got_id[got_cnt] = spk_id;
            got_cnt = got_cnt + 1;
        end

    integer errors = 0;
    integer guard, n, start;

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
        input [9:0] a; input [NA-1:0] t; input [7:0] w;
        begin
            @(negedge clk);
            s_cfg_addr = a; s_cfg_data = {t, w}; s_cfg_we = 1'b1;
            @(negedge clk);
            s_cfg_we = 1'b0;
        end
    endtask

    task n_cfg;
        input [NA-1:0] nn;
        begin
            @(negedge clk);
            n_cfg_addr = nn;
            n_cfg_data = {3'd0, 4'd2, 16'd80, 16'd0};   // {refr, leak, thr, v}
            n_cfg_we   = 1'b1;
            @(negedge clk);
            n_cfg_we   = 1'b0;
        end
    endtask

    task send_spike;
        input [7:0] ax;
        begin
            @(negedge clk);
            in_axon = ax; in_delay = 4'd0; in_valid = 1'b1;
            guard = 0;
            while (in_ready !== 1'b1 && guard < 2000) begin
                @(negedge clk); guard = guard + 1;
            end
            @(negedge clk);
            in_valid = 1'b0;
        end
    endtask

    task wait_idle;
        begin
            guard = 0;
            @(negedge clk);
            while (!(in_ready === 1'b1 && l_valid === 1'b0 && e_valid === 1'b0 &&
                     busy === 1'b0 && spk_valid === 1'b0) && guard < 20000) begin
                @(negedge clk); guard = guard + 1;
            end
            repeat (2) @(negedge clk);
        end
    endtask

    task do_tick;
        begin
            @(negedge clk);
            tick = 1'b1;
            @(negedge clk);
            tick = 1'b0;
            wait_idle;
        end
    endtask

    // check one neuron's membrane and refractory counter (hierarchical read)
    task chk_state;
        input integer nn;
        input integer exp_v;
        input integer exp_r;
        input [255:0] name;
        reg [38:0] ent;
        integer dv, dr;
        begin
            ent = u_nu.mem[nn];
            dv  = $signed(ent[15:0]);
            dr  = ent[38:36];
            if (dv !== exp_v || dr !== exp_r) begin
                $display("FAIL %0s: neuron %0d v=%0d refr=%0d (exp v=%0d refr=%0d)",
                         name, nn, dv, dr, exp_v, exp_r);
                errors = errors + 1;
            end else
                $display("PASS %0s", name);
        end
    endtask

    task chk_spikes;
        input integer from;
        input integer cnt;
        input integer id0;
        input integer id1;
        input [255:0] name;
        begin
            if (got_cnt !== from + cnt ||
                (cnt >= 1 && got_id[from]   !== id0) ||
                (cnt >= 2 && got_id[from+1] !== id1)) begin
                $display("FAIL %0s: spikes=%0d (exp %0d)", name, got_cnt - from, cnt);
                errors = errors + 1;
            end else
                $display("PASS %0s", name);
        end
    endtask

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        for (n = 0; n < 16; n = n + 1) n_cfg(n);

        // axon 5 -> synapses 100..102 = (n1,+60) (n2,+60) (n1,+60)
        // axon 9 -> synapse 200       = (n2,+40)
        a_cfg(8'd5, 10'd100, 6'd3);
        a_cfg(8'd9, 10'd200, 6'd1);
        s_cfg(10'd100, 4'd1, 8'd60);
        s_cfg(10'd101, 4'd2, 8'd60);
        s_cfg(10'd102, 4'd1, 8'd60);
        s_cfg(10'd200, 4'd2, 8'd40);

        // S1: n1 integrates 120 -> leaks to 90 >= 80: fires. n2: 60 -> 45.
        start = got_cnt;
        send_spike(8'd5);
        wait_idle;
        chk_state(1, 120, 0, "S1 n1 integrated before tick");
        chk_state(2, 60, 0,  "S1 n2 integrated before tick");
        do_tick;
        chk_spikes(start, 1, 1, 0, "S1 spike from n1 only");
        chk_state(1, 0, 2,  "S1 n1 reset and refractory");
        chk_state(2, 45, 0, "S1 n2 leaked");

        // S2: n1 refractory (input discarded). n2: 45+60=105 -> 79 < 80: no spike.
        start = got_cnt;
        send_spike(8'd5);
        wait_idle;                                  // drain the spike pipeline before the tick
        do_tick;
        chk_spikes(start, 0, 0, 0, "S2 no spike");
        chk_state(1, 0, 1,  "S2 n1 still refractory");
        chk_state(2, 79, 0, "S2 n2 below threshold");

        // S3: n2: 79+40=119 -> 90 >= 80: fires. n1 refractory ends.
        start = got_cnt;
        send_spike(8'd9);
        wait_idle;                                  // drain the spike pipeline before the tick
        do_tick;
        chk_spikes(start, 1, 2, 0, "S3 spike from n2");
        chk_state(2, 0, 2, "S3 n2 reset and refractory");
        chk_state(1, 0, 0, "S3 n1 refractory ended");

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
