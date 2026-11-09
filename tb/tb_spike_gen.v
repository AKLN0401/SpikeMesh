`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 11:26:20 PM
// Design Name: 
// Module Name: tb_spike_gen
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
// Self-checking testbench for spike_gen (NEU_AW = 4, 16 neurons).
// Entry = {en, dest_x, dest_y, axon_id[7:0], delay[3:0]}.
module tb_spike_gen;
    localparam NA = 4;
    reg          clk = 0;
    reg          rst_n = 0;
    reg          cfg_we = 0;
    reg  [NA-1:0] cfg_addr = 0;
    reg  [14:0]  cfg_data = 0;
    reg          spk_valid = 0;
    wire         spk_ready;
    reg  [NA-1:0] spk_id = 0;
    wire         pkt_valid;
    reg          pkt_ready = 1;
    wire [15:0]  pkt_data;

    spike_gen #(.NEU_AW(NA), .PKT_TYPE(2'b01)) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(cfg_we), .cfg_addr(cfg_addr), .cfg_data(cfg_data),
        .spk_valid(spk_valid), .spk_ready(spk_ready), .spk_id(spk_id),
        .pkt_valid(pkt_valid), .pkt_ready(pkt_ready), .pkt_data(pkt_data)
    );

    always #5 clk = ~clk;

    // reference model of the table
    reg [14:0] m_tab [0:15];
    function [15:0] m_pkt;
        input [14:0] e;
        begin m_pkt = {2'b01, e[13:0]}; end
    endfunction

    // packet collector
    integer got_cnt = 0;
    reg [15:0] got [0:1023];
    always @(posedge clk)
        if (pkt_valid && pkt_ready) begin
            got[got_cnt] = pkt_data;
            got_cnt = got_cnt + 1;
        end

    // expected-packet queue (model), filled when a spike is accepted
    integer exp_cnt = 0;
    reg [15:0] expq [0:1023];
    integer acc_cnt = 0;
    always @(posedge clk)
        if (spk_valid && spk_ready) begin
            acc_cnt = acc_cnt + 1;
            if (m_tab[spk_id][14]) begin
                expq[exp_cnt] = m_pkt(m_tab[spk_id]);
                exp_cnt = exp_cnt + 1;
            end
        end

    integer errors = 0;
    integer start, guard, i, k;
    reg [31:0] lcg;
    reg        rdy_en = 0;
    reg [31:0] lcg2 = 32'h5678;

    function [31:0] nxt;
        input [31:0] x;
        begin nxt = x * 32'd1664525 + 32'd1013904223; end
    endfunction

    // independent random pkt_ready driver
    always @(negedge clk) if (rdy_en) begin
        lcg2 = nxt(lcg2);
        pkt_ready = lcg2[17];
    end

    task cfg;
        input [NA-1:0] n; input en; input dx; input dy; input [7:0] ax; input [3:0] d;
        begin
            @(negedge clk);
            cfg_addr = n; cfg_data = {en, dx, dy, ax, d}; cfg_we = 1'b1;
            m_tab[n] = {en, dx, dy, ax, d};
            @(negedge clk);
            cfg_we = 1'b0;
        end
    endtask

    // present one spike, held until accepted
    task spike;
        input [NA-1:0] n;
        begin
            @(negedge clk);
            spk_id = n; spk_valid = 1'b1;
            guard = 0;
            while (spk_ready !== 1'b1 && guard < 5000) begin
                @(negedge clk); guard = guard + 1;
            end
            if (guard >= 5000) begin
                $display("FAIL timeout waiting for spk_ready"); errors = errors + 1;
            end
            @(negedge clk);
            spk_valid = 1'b0;
        end
    endtask

    task settle;
        begin
            guard = 0;
            @(negedge clk);
            while (pkt_valid === 1'b1 && guard < 5000) begin
                @(negedge clk); guard = guard + 1;
            end
            repeat (3) @(negedge clk);
        end
    endtask

    task chk_pkt;
        input integer idx;
        input [15:0] exp;
        input [255:0] name;
        begin
            if (got[idx] !== exp) begin
                $display("FAIL %0s: got %h exp %h", name, got[idx], exp);
                errors = errors + 1;
            end else $display("PASS %0s", name);
        end
    endtask

    initial begin
        for (i = 0; i < 16; i = i + 1) m_tab[i] = 15'd0;
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        if (pkt_valid !== 1'b0 || spk_ready !== 1'b1) begin
            $display("FAIL reset state"); errors = errors + 1;
        end else $display("PASS reset state");

        // program all 16 entries: even = routed, odd = unrouted
        for (i = 0; i < 16; i = i + 1)
            if (i % 2 == 0)
                cfg(i, 1'b1, i[0], i[1], 8'h10 + i, i[3:0]);
            else
                cfg(i, 1'b0, 1'b1, 1'b1, 8'hFF, 4'hF);
        cfg(4'd3, 1'b1, 1'b1, 1'b0, 8'h2A, 4'd5);       // neuron 3 routed to (1,0)

        // P1: single spike, packet fields and type
        start = got_cnt;
        spike(4'd3);
        settle;
        if (got_cnt !== start + 1) begin
            $display("FAIL single spike: %0d packets", got_cnt - start); errors = errors + 1;
        end else begin
            chk_pkt(start, {2'b01, 1'b1, 1'b0, 8'h2A, 4'd5}, "single spike packet");
        end

        // P2: unrouted neuron produces no packet and is consumed
        start = got_cnt;
        spike(4'd5);
        settle;
        if (got_cnt !== start) begin
            $display("FAIL unrouted: %0d packets", got_cnt - start); errors = errors + 1;
        end else $display("PASS unrouted neuron dropped");

        // P3: back-to-back spikes, one per cycle, no bubbles
        start = got_cnt;
        @(negedge clk);
        spk_valid = 1'b1;
        for (k = 0; k < 8; k = k + 1) begin
            spk_id = k[3:0] * 2;                       // neurons 0,2,...,14 (all routed)
            @(negedge clk);
        end
        spk_valid = 1'b0;
        settle;
        if (got_cnt !== start + 8) begin
            $display("FAIL back-to-back: %0d packets (exp 8)", got_cnt - start); errors = errors + 1;
        end else begin
            for (k = 0; k < 8; k = k + 1)
                if (got[start+k] !== m_pkt(m_tab[k*2])) begin
                    $display("FAIL back-to-back packet %0d", k); errors = errors + 1;
                end
            $display("PASS back-to-back (8 packets in order)");
        end

        // P4: stalled consumer holds data and back-pressures the neuron unit
        pkt_ready = 1'b0;
        start = got_cnt;
        spike(4'd2);                                   // A accepted into the stage
        @(negedge clk);
        begin : hold_blk
            reg [15:0] hold;
            hold = pkt_data;
            spk_id = 4'd4; spk_valid = 1'b1;           // B waits
            repeat (3) @(negedge clk);
            if (pkt_valid !== 1'b1 || spk_ready !== 1'b0 || pkt_data !== hold || got_cnt !== start) begin
                $display("FAIL stall hold: pkt_valid=%b spk_ready=%b", pkt_valid, spk_ready);
                errors = errors + 1;
            end else $display("PASS stall hold");
        end
        pkt_ready = 1'b1;
        @(negedge clk);
        spk_valid = 1'b0;
        settle;
        if (got_cnt !== start + 2 || got[start] !== m_pkt(m_tab[2]) || got[start+1] !== m_pkt(m_tab[4])) begin
            $display("FAIL stall release: %0d packets", got_cnt - start); errors = errors + 1;
        end else $display("PASS stall release (A then B)");

        // P5: unrouted spike behind a stalled packet: ready only after the packet drains
        // (entry for neuron 5 has en = 0, but the stage is full of a routed packet)
        pkt_ready = 1'b0;
        spike(4'd6);
        @(negedge clk);
        spk_id = 4'd5; spk_valid = 1'b1;
        repeat (2) @(negedge clk);
        if (spk_ready !== 1'b0) begin
            $display("FAIL stage full must stall next spike"); errors = errors + 1;
        end else $display("PASS stage full stalls next spike");
        pkt_ready = 1'b1;
        @(negedge clk);
        spk_valid = 1'b0;
        settle;

        // P6: overwrite an entry
        cfg(4'd3, 1'b1, 1'b0, 1'b1, 8'h77, 4'd9);
        start = got_cnt;
        spike(4'd3);
        settle;
        if (got_cnt !== start + 1)  begin
            $display("FAIL overwrite: %0d packets", got_cnt - start); errors = errors + 1;
        end else chk_pkt(start, {2'b01, 1'b0, 1'b1, 8'h77, 4'd9}, "overwrite entry");

        // P7: random spikes, random pkt_ready; compare with the model queue
        lcg = 32'hC0FFEE;
        exp_cnt = 0; acc_cnt = 0; got_cnt = 0;
        rdy_en = 1'b1;
        for (i = 0; i < 300; i = i + 1) begin
            lcg = nxt(lcg);
            @(negedge clk);
            spk_valid = lcg[20];
            spk_id    = lcg[27:24];
            // hold the request until accepted
            guard = 0;
            while (spk_valid && spk_ready !== 1'b1 && guard < 5000) begin
                @(negedge clk); guard = guard + 1;
            end
        end
        @(negedge clk);
        spk_valid = 1'b0;
        guard = 0;
        while (guard < 200 && pkt_valid !== 1'b0) begin @(negedge clk); guard = guard + 1; end
        repeat (40) @(negedge clk);
        rdy_en = 1'b0; pkt_ready = 1'b1;
        repeat (4) @(negedge clk);
        if (got_cnt !== exp_cnt) begin
            $display("FAIL random: %0d packets, model %0d", got_cnt, exp_cnt); errors = errors + 1;
        end else begin : cmp
            integer bad;
            bad = 0;
            for (k = 0; k < exp_cnt; k = k + 1)
                if (got[k] !== expq[k]) bad = bad + 1;
            if (bad != 0) begin
                $display("FAIL random: %0d packets differ from model", bad); errors = errors + 1;
            end else $display("PASS random spikes vs model");
        end
        $display("note: random test accepted %0d spikes, %0d routed", acc_cnt, exp_cnt);

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
