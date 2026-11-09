`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 11:19:03 PM
// Design Name: 
// Module Name: tb_mesh
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
// System testbench: 2x2 mesh of neurocores (16 neurons per core) and a
// local-loop deadlock test. The testbench acts as the host: it programs the
// cores, then pulses a global tick only when the whole mesh is idle.
// Core index i = 2*y + x. Neuron entry {refr, leak, thr, v}: thr 80, leak 2, refr period 2.

// Single core + router Local port only: every packet to (0,0) loops back.
module loop_dut #(parameter SPK_FIFO = 1) (
    input clk, input rst_n,
    input cfg_we, input [1:0] cfg_bank, input [9:0] cfg_addr, input [38:0] cfg_data,
    input tick, output idle
);
    wire        c2r_v, c2r_r, r2c_v, r2c_r;
    wire [15:0] c2r_d, r2c_d;
    wire [79:0] rin  = {64'd0, c2r_d};
    wire [4:0]  rinv = {4'd0, c2r_v};
    wire [4:0]  rinr;
    wire [79:0] rout;
    wire [4:0]  routv;
    assign c2r_r = rinr[0];
    assign r2c_v = routv[0];
    assign r2c_d = rout[15:0];

    neurocore #(.NEU_AW(4), .SPK_FIFO(SPK_FIFO)) u_core (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(cfg_we), .cfg_bank(cfg_bank), .cfg_addr(cfg_addr), .cfg_data(cfg_data),
        .tick(tick), .idle(idle), .tick_missed(),
        .pkt_in_valid(r2c_v), .pkt_in_ready(r2c_r), .pkt_in_data(r2c_d),
        .pkt_out_valid(c2r_v), .pkt_out_ready(c2r_r), .pkt_out_data(c2r_d)
    );
    aer_router #(.X(0), .Y(0), .AW(2)) u_r (
        .clk(clk), .rst_n(rst_n),
        .in_data(rin), .in_valid(rinv), .in_ready(rinr),
        .out_data(rout), .out_valid(routv), .out_ready({4'b1111, r2c_r})
    );
endmodule

module tb_mesh;
    reg clk = 0;
    reg rst_n = 0;
    always #5 clk = ~clk;

    // ---------------- mesh ----------------
    reg  [3:0]   cfg_we   = 0;
    reg  [7:0]   cfg_bank = 0;
    reg  [39:0]  cfg_addr = 0;
    reg  [155:0] cfg_data = 0;
    reg          tick = 0;
    wire [3:0]   core_idle, tick_missed;
    wire         net_active;

    aer_mesh2x2 #(.NEU_AW(4), .SPK_FIFO(1), .RAW(2)) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(cfg_we), .cfg_bank(cfg_bank), .cfg_addr(cfg_addr), .cfg_data(cfg_data),
        .tick(tick), .core_idle(core_idle), .tick_missed(tick_missed), .net_active(net_active)
    );

    // ---------------- loop duts: A with spike FIFO, B without ----------------
    reg         l_we = 0;
    reg  [1:0]  l_bank = 0;
    reg  [9:0]  l_addr = 0;
    reg  [38:0] l_data = 0;
    reg         l_tick = 0;
    wire        la_idle, lb_idle;
    loop_dut #(.SPK_FIFO(1)) loopA (.clk(clk), .rst_n(rst_n), .cfg_we(l_we), .cfg_bank(l_bank),
        .cfg_addr(l_addr), .cfg_data(l_data), .tick(l_tick), .idle(la_idle));
    loop_dut #(.SPK_FIFO(0)) loopB (.clk(clk), .rst_n(rst_n), .cfg_we(l_we), .cfg_bank(l_bank),
        .cfg_addr(l_addr), .cfg_data(l_data), .tick(l_tick), .idle(lb_idle));

    // ---------------- packet monitor (router Local output -> core) ----------------
    integer rcv_cnt [0:3];
    reg [15:0] rcv_pkt [0:63];
    integer mi;
    initial for (mi = 0; mi < 4; mi = mi + 1) rcv_cnt[mi] = 0;
    always @(posedge clk) begin
        for (mi = 0; mi < 4; mi = mi + 1)
            if (dut.r2c_valid[mi] && dut.r2c_ready[mi]) begin
                rcv_pkt[mi*16 + rcv_cnt[mi]] = dut.r2c_data[mi*16 +: 16];
                rcv_cnt[mi] = rcv_cnt[mi] + 1;
            end
    end

    integer errors = 0;
    integer guard, i, k, same;

    // ---------------- host tasks ----------------
    task cfg;
        input integer core; input [1:0] bank; input [9:0] addr; input [38:0] data;
        begin
            @(negedge clk);
            cfg_we = 4'd1 << core;
            cfg_bank[core*2 +: 2]  = bank;
            cfg_addr[core*10 +: 10] = addr;
            cfg_data[core*39 +: 39] = data;
            @(negedge clk);
            cfg_we = 4'd0;
        end
    endtask

    task set_neu;     // thr 80, leak 2
        input integer core; input integer n; input [15:0] v;
        begin cfg(core, 2'd2, n, {3'd0, 4'd2, 16'd80, v}); end
    endtask
    task set_route;
        input integer core; input integer n; input en; input dx; input dy; input [7:0] ax; input [3:0] d;
        begin cfg(core, 2'd3, n, {24'd0, en, dx, dy, ax, d}); end
    endtask
    task set_axon;
        input integer core; input integer ax; input [9:0] base; input [5:0] cnt;
        begin cfg(core, 2'd0, ax, {23'd0, base, cnt}); end
    endtask
    task set_syn;
        input integer core; input integer a; input [3:0] t; input [7:0] w;
        begin cfg(core, 2'd1, a, {27'd0, t, w}); end
    endtask

    task mesh_idle;     // idle for 12 consecutive cycles
        integer stable;
        begin
            guard = 0; stable = 0;
            while (stable < 12 && guard < 5000) begin
                @(negedge clk);
                if (&core_idle && !net_active) stable = stable + 1; else stable = 0;
                guard = guard + 1;
            end
            if (guard >= 5000) begin
                $display("FAIL mesh did not become idle (deadlock?) core_idle=%b", core_idle);
                errors = errors + 1;
            end
        end
    endtask

    task mesh_tick;
        begin
            mesh_idle;
            @(negedge clk); tick = 1'b1;
            @(negedge clk); tick = 1'b0;
            mesh_idle;
        end
    endtask

    // all router FIFOs empty (independent of the idle logic)
    function fifos_empty;
        input dummy;
        begin
            fifos_empty = &dut.u_r0.f_empty & &dut.u_r1.f_empty & &dut.u_r2.f_empty & &dut.u_r3.f_empty;
        end
    endfunction

    function [38:0] get_ent;
        input integer core; input integer n;
        begin
            case (core)
                0: get_ent = dut.g_core[0].u_core.u_nu.mem[n];
                1: get_ent = dut.g_core[1].u_core.u_nu.mem[n];
                2: get_ent = dut.g_core[2].u_core.u_nu.mem[n];
                default: get_ent = dut.g_core[3].u_core.u_nu.mem[n];
            endcase
        end
    endfunction

    task chk_n;
        input integer core; input integer n; input integer exp_v; input integer exp_r;
        input [255:0] name;
        reg [38:0] e; integer dv, dr;
        begin
            e = get_ent(core, n);
            dv = $signed(e[15:0]); dr = e[38:36];
            if (dv !== exp_v || dr !== exp_r) begin
                $display("FAIL %0s: core%0d n%0d v=%0d refr=%0d (exp v=%0d refr=%0d)",
                         name, core, n, dv, dr, exp_v, exp_r);
                errors = errors + 1;
            end else $display("PASS %0s", name);
        end
    endtask

    task chk_rcv;
        input integer core; input integer idx; input [15:0] exp; input integer total;
        input [255:0] name;
        begin
            if (rcv_cnt[core] !== total || rcv_pkt[core*16 + idx] !== exp) begin
                $display("FAIL %0s: core%0d received %0d (exp %0d), pkt %h (exp %h)",
                         name, core, rcv_cnt[core], total, rcv_pkt[core*16 + idx], exp);
                errors = errors + 1;
            end else $display("PASS %0s", name);
        end
    endtask

    task chk_quiet;
        input [255:0] name;
        begin
            if (!fifos_empty(1'b0) || net_active || tick_missed !== 4'd0) begin
                $display("FAIL %0s: fifos_empty=%b net_active=%b tick_missed=%b",
                         name, fifos_empty(1'b0), net_active, tick_missed);
                errors = errors + 1;
            end else $display("PASS %0s", name);
        end
    endtask

    // ---------------- loop-dut tasks ----------------
    task lcfg;
        input [1:0] bank; input [9:0] addr; input [38:0] data;
        begin
            @(negedge clk);
            l_we = 1'b1; l_bank = bank; l_addr = addr; l_data = data;
            @(negedge clk);
            l_we = 1'b0;
        end
    endtask

    function [38:0] lent;
        input integer which; input integer n;
        begin
            if (which == 0) lent = loopA.u_core.u_nu.mem[n];
            else            lent = loopB.u_core.u_nu.mem[n];
        end
    endfunction

    reg a_done, b_done;
    // M4 model state
    integer m_v [0:63], m_thr [0:63], m_lk [0:63], m_rf [0:63], sp [0:63];
    integer r_en [0:63], r_dst [0:63];
    integer s_cnt [0:255], s_tgt [0:767], s_w [0:767];
    reg [31:0] lcg;
    integer g, c, n, j, d, t, bad, vl, n_sp, n_pk = 0, tot_spk, tot_pkt;
    reg [38:0] e;
    function [31:0] nxt;
        input [31:0] x;
        begin nxt = x * 32'd1664525 + 32'd1013904223; end
    endfunction
    integer a_cyc, b_cyc;

    initial begin
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        // ---- initialise all neurons (v = 0, no route) of all cores ----
        for (i = 0; i < 4; i = i + 1)
            for (k = 0; k < 16; k = k + 1) begin
                set_neu(i, k, 16'd0);
                set_route(i, k, 1'b0, 1'b0, 1'b0, 8'd0, 4'd0);
            end

        // ================= M1: five-hop chain across the mesh =================
        // core0.n1 -> core3.n2 -> core0.n5 -> core0.n6 (local) -> core1.n0 -> core2.n3
        set_neu(0, 1, 16'd200);
        set_route(0, 1, 1'b1, 1'b1, 1'b1, 8'd7, 4'd3);   // to core3 axon 7
        set_axon(3, 7, 10'd20, 6'd1);  set_syn(3, 20, 4'd2, 8'd120);
        set_route(3, 2, 1'b1, 1'b0, 1'b0, 8'd9, 4'd1);   // to core0 axon 9
        set_axon(0, 9, 10'd30, 6'd1);  set_syn(0, 30, 4'd5, 8'd120);
        set_route(0, 5, 1'b1, 1'b0, 1'b0, 8'd3, 4'd2);   // to core0 axon 3 (self)
        set_axon(0, 3, 10'd40, 6'd1);  set_syn(0, 40, 4'd6, 8'd120);
        set_route(0, 6, 1'b1, 1'b1, 1'b0, 8'd4, 4'd3);   // to core1 axon 4
        set_axon(1, 4, 10'd50, 6'd1);  set_syn(1, 50, 4'd0, 8'd120);
        set_route(1, 0, 1'b1, 1'b0, 1'b1, 8'd1, 4'd0);   // to core2 axon 1
        set_axon(2, 1, 10'd60, 6'd1);  set_syn(2, 60, 4'd3, 8'd40);

        mesh_idle;
        if (core_idle !== 4'b1111) begin $display("FAIL cores idle after config"); errors = errors + 1; end
        else $display("PASS cores idle after config");

        mesh_tick;  // T1
        chk_n(0, 1, 0, 2,   "M1 T1 core0.n1 fired and reset");
        chk_rcv(3, 0, {2'b00, 1'b1, 1'b1, 8'd7, 4'd3}, 1, "M1 T1 packet reached core3");
        chk_n(3, 2, 120, 0, "M1 T1 core3.n2 integrated 120");
        chk_quiet("M1 T1 network quiet");

        mesh_tick;  // T2
        chk_n(3, 2, 0, 2,   "M1 T2 core3.n2 fired");
        chk_rcv(0, 0, {2'b00, 1'b0, 1'b0, 8'd9, 4'd1}, 1, "M1 T2 packet reached core0");
        chk_n(0, 5, 120, 0, "M1 T2 core0.n5 integrated 120");

        mesh_tick;  // T3
        chk_n(0, 5, 0, 2,   "M1 T3 core0.n5 fired");
        chk_rcv(0, 1, {2'b00, 1'b0, 1'b0, 8'd3, 4'd2}, 2, "M1 T3 local packet to core0");
        chk_n(0, 6, 120, 0, "M1 T3 core0.n6 integrated 120");

        mesh_tick;  // T4
        chk_n(0, 6, 0, 2,   "M1 T4 core0.n6 fired");
        chk_rcv(1, 0, {2'b00, 1'b1, 1'b0, 8'd4, 4'd3}, 1, "M1 T4 packet reached core1");
        chk_n(1, 0, 120, 0, "M1 T4 core1.n0 integrated 120");

        mesh_tick;  // T5
        chk_n(1, 0, 0, 2,   "M1 T5 core1.n0 fired");
        chk_rcv(2, 0, {2'b00, 1'b0, 1'b1, 8'd1, 4'd0}, 1, "M1 T5 packet reached core2");
        chk_n(2, 3, 40, 0,  "M1 T5 core2.n3 integrated 40");

        mesh_tick;  // T6
        chk_n(2, 3, 30, 0,  "M1 T6 core2.n3 leaked to 30");
        if (rcv_cnt[0] !== 2 || rcv_cnt[1] !== 1 || rcv_cnt[2] !== 1 || rcv_cnt[3] !== 1) begin
            $display("FAIL M1 total packets %0d %0d %0d %0d", rcv_cnt[0], rcv_cnt[1], rcv_cnt[2], rcv_cnt[3]);
            errors = errors + 1;
        end else $display("PASS M1 total packet counts (2,1,1,1)");
        chk_quiet("M1 end network quiet");

        // ================= M2: all four cores fire to the next core at once =================
        for (i = 0; i < 4; i = i + 1) begin
            set_neu(i, 8, 16'd200);
            set_route(i, 8, 1'b1, ((i+1)%4) & 1, (((i+1)%4) >> 1) & 1, 8'd11, 4'd0);
            set_axon(i, 11, 10'd70, 6'd1);
            set_syn(i, 70, 4'd9, 8'd120);
        end
        for (i = 0; i < 4; i = i + 1) rcv_cnt[i] = 0;
        mesh_tick;
        same = 1;
        for (i = 0; i < 4; i = i + 1) begin
            chk_n(i, 8, 0, 2,   "M2 n8 fired and reset");
            chk_n(i, 9, 120, 0, "M2 n9 integrated 120");
            if (rcv_cnt[i] !== 1) same = 0;
        end
        if (!same) begin $display("FAIL M2 each core must receive exactly one packet"); errors = errors + 1; end
        else $display("PASS M2 one packet per core");
        mesh_tick;
        for (i = 0; i < 4; i = i + 1) chk_n(i, 9, 0, 2, "M2 n9 fired, no packet");
        same = 1;
        for (i = 0; i < 4; i = i + 1) if (rcv_cnt[i] !== 1) same = 0;
        if (!same) begin $display("FAIL M2 unrouted spike produced a packet"); errors = errors + 1; end
        else $display("PASS M2 unrouted spikes produced no packets");
        chk_quiet("M2 end network quiet");

        // ================= M3: local-loop stress, 16 neurons all routed to self =================
        // Neuron k: v = 200, route to (0,0) axon k; axon k -> synapse k -> (neuron k, +50).
        for (k = 0; k < 16; k = k + 1) begin
            lcfg(2'd2, k, {3'd0, 4'd2, 16'd80, 16'd200});
            lcfg(2'd3, k, {24'd0, 1'b1, 1'b0, 1'b0, k[7:0], 4'd0});
            lcfg(2'd0, k, {23'd0, k[9:0], 6'd1});
            lcfg(2'd1, k, {27'd0, k[3:0], 8'd50});
        end
        repeat (4) @(negedge clk);
        @(negedge clk); l_tick = 1'b1;
        @(negedge clk); l_tick = 1'b0;
        a_done = 0; b_done = 0; a_cyc = 0; b_cyc = 0;
        for (guard = 0; guard < 6000 && !(a_done && b_done); guard = guard + 1) begin
            @(negedge clk);
            if (!a_done) begin a_cyc = a_cyc + 1; if (la_idle) begin repeat (12) @(negedge clk); if (la_idle) a_done = 1; end end
            if (!b_done) begin b_cyc = b_cyc + 1; if (lb_idle) begin repeat (12) @(negedge clk); if (lb_idle) b_done = 1; end end
        end
        if (!a_done) begin
            $display("FAIL M3 with spike FIFO: core did not return to idle"); errors = errors + 1;
        end else begin
            same = 1;
            for (k = 0; k < 16; k = k + 1) begin
                if ($signed(lent(0,k) & 39'hFFFF) !== 50 || lent(0,k) >> 36 !== 2) same = 0;
            end
            if (!same) begin $display("FAIL M3 with spike FIFO: wrong final state"); errors = errors + 1; end
            else $display("PASS M3 with spike FIFO: 16 local spikes delivered, every neuron v=50 refr=2 (idle after ~%0d cycles)", a_cyc);
        end
        if (!b_done) begin
            $display("note M3 without spike FIFO: DEADLOCK (not idle after %0d cycles)", guard);
            $display("note   neuron_unit state=%0d (3 = sweep waiting on a spike), ev_ready=%b, spk_valid=%b, sweep index=%0d",
                     loopB.u_core.u_nu.st, loopB.u_core.u_nu.ev_ready, loopB.u_core.u_nu.spk_v, loopB.u_core.u_nu.idx);
        end
        else         $display("note M3 without spike FIFO: completed in ~%0d cycles (no deadlock for this load)", b_cyc);


        // ================= M4: random mesh traffic against a behavioural model =================
        // 64 neurons (4 cores x 16). Neuron g = 16*core + n has axon id g on every core.
        // Per (core, axon): 0..3 synapses {target, weight}. Weights are small so no
        // membrane saturates (the order of events then cannot change the result).
        lcg = 32'hBEEF01;
        for (g = 0; g < 64; g = g + 1) begin
            lcg = nxt(lcg); m_v[g]   = ($unsigned(lcg[23:16]) % 220) - 40;
            lcg = nxt(lcg); m_thr[g] = 30 + ($unsigned(lcg[23:16]) % 41);
            lcg = nxt(lcg); m_lk[g]  = 1 + ($unsigned(lcg[23:16]) % 4);
            m_rf[g] = 0;
            lcg = nxt(lcg); r_en[g]  = (lcg[23:16] % 10) < 8;
            lcg = nxt(lcg); r_dst[g] = lcg[23:16] % 4;
        end
        for (c = 0; c < 4; c = c + 1)
            for (g = 0; g < 64; g = g + 1) begin
                lcg = nxt(lcg); s_cnt[c*64+g] = lcg[23:16] % 4;
                for (j = 0; j < 3; j = j + 1) begin
                    lcg = nxt(lcg); s_tgt[(c*64+g)*3+j] = lcg[23:16] % 16;
                    lcg = nxt(lcg); s_w[(c*64+g)*3+j]   = ($unsigned(lcg[23:16]) % 111) - 20;
                end
            end
        for (c = 0; c < 4; c = c + 1) begin
            for (n = 0; n < 16; n = n + 1) begin
                g = c*16 + n;
                cfg(c, 2'd2, n, {3'd0, m_lk[g][3:0], m_thr[g][15:0], m_v[g][15:0]});
                cfg(c, 2'd3, n, {24'd0, r_en[g], r_dst[g][0], r_dst[g][1], g[7:0], 4'd0});
            end
            for (g = 0; g < 64; g = g + 1) begin
                set_axon(c, g, g*3, s_cnt[c*64+g]);
                for (j = 0; j < 3; j = j + 1)
                    set_syn(c, g*3+j, s_tgt[(c*64+g)*3+j], s_w[(c*64+g)*3+j]);
            end
        end
        for (i = 0; i < 4; i = i + 1) rcv_cnt[i] = 0;
        tot_spk = 0; tot_pkt = 0;
        for (t = 0; t < 24; t = t + 1) begin
            // host injects: overwrite V of three random neurons (full entry write, idle mesh)
            for (j = 0; j < 3; j = j + 1) begin
                lcg = nxt(lcg); g = lcg[23:16] % 64;
                lcg = nxt(lcg); m_v[g] = 90 + (lcg[23:16] % 100);
                cfg(g/16, 2'd2, g%16, {m_rf[g][2:0], m_lk[g][3:0], m_thr[g][15:0], m_v[g][15:0]});
            end
            // model: sweep, then deliver the events of this tick's spikes
            n_sp = 0;
            for (g = 0; g < 64; g = g + 1) begin
                if (m_rf[g] != 0) begin m_rf[g] = m_rf[g] - 1; m_v[g] = 0; sp[g] = 0; end
                else begin
                    vl = m_v[g] - (m_v[g] >>> m_lk[g]);
                    if (vl >= m_thr[g]) begin sp[g] = 1; m_v[g] = 0; m_rf[g] = 2; end
                    else begin sp[g] = 0; m_v[g] = vl; end
                end
            end
            for (g = 0; g < 64; g = g + 1)
                if (sp[g]) begin
                    n_sp = n_sp + 1;
                    if (r_en[g]) begin
                        n_pk = n_pk + 1;
                        for (j = 0; j < s_cnt[r_dst[g]*64+g]; j = j + 1) begin
                            d = r_dst[g]*16 + s_tgt[(r_dst[g]*64+g)*3+j];
                            m_v[d] = m_v[d] + s_w[(r_dst[g]*64+g)*3+j];
                        end
                    end
                end
            tot_spk = tot_spk + n_sp;
            mesh_tick;
            // compare all 64 neurons
            bad = 0;
            for (g = 0; g < 64; g = g + 1) begin
                e = get_ent(g/16, g%16);
                if ($signed(e[15:0]) !== m_v[g] || e[38:36] !== m_rf[g]) begin
                    if (bad < 4) $display("  mismatch tick %0d core%0d n%0d: dut v=%0d refr=%0d model v=%0d refr=%0d",
                         t, g/16, g%16, $signed(e[15:0]), e[38:36], m_v[g], m_rf[g]);
                    bad = bad + 1;
                end
            end
            if (bad != 0) begin
                $display("FAIL M4 tick %0d: %0d neurons differ from model", t, bad); errors = errors + 1;
            end
        end
        tot_pkt = rcv_cnt[0] + rcv_cnt[1] + rcv_cnt[2] + rcv_cnt[3];
        if (tot_pkt !== n_pk) begin
            $display("FAIL M4 packets delivered %0d, model routed %0d", tot_pkt, n_pk); errors = errors + 1;
        end else $display("PASS M4 packets delivered = routed spikes (%0d)", tot_pkt);
        if (errors == 0) $display("PASS M4 random mesh traffic vs model (24 ticks, %0d spikes)", tot_spk);
        chk_quiet("M4 end network quiet");

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #5000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule
