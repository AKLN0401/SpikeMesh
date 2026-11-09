`timescale 1ns/1ps
// Engineer: Khoi Nguyen
// Self-checking testbench for neuron_unit (NEU_AW = 4, 16 neurons; other parameters default).
// A behavioral model written with integer arithmetic runs alongside the DUT.
module tb_neuron_unit;
    localparam NA = 4;
    localparam NN = 1 << NA;
    localparam REFR_P = 2;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         cfg_we = 0;
    reg  [NA-1:0]  cfg_addr = 0;
    reg  [38:0] cfg_data = 0;
    reg         tick = 0;
    reg         ev_valid = 0;
    wire        ev_ready;
    reg  [NA-1:0]  ev_target = 0;
    reg  [7:0]  ev_weight = 0;
    wire        spk_valid;
    reg         spk_ready = 1;
    wire [NA-1:0]  spk_id;
    wire        busy;
    wire        tick_missed;

    neuron_unit #(.NEU_AW(NA), .REFR_P(REFR_P)) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(cfg_we), .cfg_addr(cfg_addr), .cfg_data(cfg_data),
        .tick(tick),
        .ev_valid(ev_valid), .ev_ready(ev_ready),
        .ev_target(ev_target), .ev_weight(ev_weight),
        .spk_valid(spk_valid), .spk_ready(spk_ready), .spk_id(spk_id),
        .busy(busy), .tick_missed(tick_missed)
    );

    always #5 clk = ~clk;

    // ---------------- spike collector ----------------
    integer got_cnt = 0;
    integer got_id [0:2047];
    always @(posedge clk)
        if (spk_valid && spk_ready) begin
            got_id[got_cnt] = spk_id;
            got_cnt = got_cnt + 1;
        end

    // ---------------- behavioral model ----------------
    integer m_v    [0:NN-1];
    integer m_thr  [0:NN-1];
    integer m_leak [0:NN-1];
    integer m_refr [0:NN-1];
    integer exp_cnt = 0;
    integer exp_id [0:2047];

    task model_event;
        input integer t;
        input integer w;
        integer s;
        begin
            s = m_v[t] + w;
            if (s > 32767)  s = 32767;
            if (s < -32768) s = -32768;
            m_v[t] = s;
        end
    endtask

    task model_tick;
        integer n, vl;
        begin
            ticks_modeled = ticks_modeled + 1;
            for (n = 0; n < NN; n = n + 1) begin
                if (m_refr[n] != 0) begin
                    m_refr[n] = m_refr[n] - 1;
                    m_v[n] = 0;
                end else begin
                    vl = m_v[n] - (m_v[n] >>> m_leak[n]);
                    if (vl >= m_thr[n]) begin
                        exp_id[exp_cnt] = n;
                        exp_cnt = exp_cnt + 1;
                        m_v[n] = 0;
                        m_refr[n] = REFR_P;
                    end else
                        m_v[n] = vl;
                end
            end
        end
    endtask

    // ---------------- drivers ----------------
    integer errors = 0;
    integer guard;
    integer ticks_modeled = 0;     // ticks the model has applied
    integer sweeps_started = 0;    // sweeps the DUT has actually started
    always @(posedge clk) if (dut.start_sweep) sweeps_started = sweeps_started + 1;

    // write DUT state entry and model together
    task set_neuron;
        input integer n;
        input integer v;
        input integer thr;
        input integer leak;
        input integer refr;
        reg [15:0] v16, t16;
        begin
            v16 = v; t16 = thr;
            @(negedge clk);
            cfg_addr = n;
            cfg_data = {refr[2:0], leak[3:0], t16, v16};
            cfg_we   = 1'b1;
            @(negedge clk);
            cfg_we   = 1'b0;
            m_v[n] = v; m_thr[n] = thr; m_leak[n] = leak; m_refr[n] = refr;
        end
    endtask

    task send_ev;
        input integer t;
        input integer w;
        reg [7:0] w8;
        begin
            w8 = w;
            @(negedge clk);
            ev_target = t; ev_weight = w8; ev_valid = 1'b1;
            guard = 0;
            while (ev_ready !== 1'b1 && guard < 5000) begin
                @(negedge clk); guard = guard + 1;
            end
            if (guard >= 5000) begin
                $display("FAIL timeout: event never accepted (target %0d)", t);
                errors = errors + 1;
            end
            @(negedge clk);
            ev_valid = 1'b0;
            model_event(t, w);
        end
    endtask

    // one-cycle tick; does not wait for the sweep
    task do_tick;
        begin
            // the DUT holds one pending tick; wait for the slot so none is dropped
            guard = 0;
            while (dut.tick_pend === 1'b1 && guard < 20000) begin
                @(negedge clk); guard = guard + 1;
            end
            if (guard >= 20000) begin
                $display("FAIL timeout: no free tick slot");
                errors = errors + 1;
            end
            @(negedge clk);
            tick = 1'b1;
            @(negedge clk);
            tick = 1'b0;
            model_tick;
        end
    endtask

    task wait_idle;
        begin
            guard = 0;
            @(negedge clk);
            while (!(busy === 1'b0 && spk_valid === 1'b0) && guard < 20000) begin
                @(negedge clk); guard = guard + 1;
            end
            if (guard >= 20000) begin
                $display("FAIL timeout: DUT never went idle");
                errors = errors + 1;
            end
            repeat (2) @(negedge clk);
        end
    endtask

    // compare spikes and every neuron's state against the model
    task check_all;
        input [255:0] name;
        integer n, bad;
        reg [38:0] ent;
        integer dv, dr;
        begin
            bad = 0;
            if (got_cnt !== exp_cnt) bad = bad + 1;
            else
                for (n = 0; n < exp_cnt; n = n + 1)
                    if (got_id[n] !== exp_id[n]) bad = bad + 1;
            for (n = 0; n < NN; n = n + 1) begin
                ent = dut.mem[n];
                dv  = $signed(ent[15:0]);
                dr  = ent[38:36];
                if (dv !== m_v[n] || dr !== m_refr[n]) begin
                    bad = bad + 1;
                    $display("  neuron %0d: dut v=%0d refr=%0d, model v=%0d refr=%0d",
                             n, dv, dr, m_v[n], m_refr[n]);
                end
            end
            if (bad != 0) begin
                $display("FAIL %0s (spikes dut=%0d model=%0d)", name, got_cnt, exp_cnt);
                errors = errors + 1;
            end else
                $display("PASS %0s", name);
        end
    endtask

    // random spk_ready driver: its own process and LCG, so the main thread can never
    // deadlock waiting on a ready that only the main thread would change
    reg        rdy_en = 1'b0;
    reg [31:0] lcg2   = 32'hBEEF;
    always @(negedge clk)
        if (rdy_en) begin
            lcg2      = lcg2 * 32'd1664525 + 32'd1013904223;
            spk_ready = ((lcg2[30:16] % 3) != 0);
        end

    // simulator-independent pseudo-random generator (LCG)
    reg [31:0] lcg = 32'h1234;
    function [31:0] rnd;
        input dummy;
        begin
            lcg = lcg * 32'd1664525 + 32'd1013904223;
            rnd = {17'b0, lcg[30:16]};
        end
    endfunction

    integer i, k, t, n_ev, tk;

    initial begin
        for (i = 0; i < NN; i = i + 1) begin
            m_v[i] = 0; m_thr[i] = 100; m_leak[i] = 2; m_refr[i] = 0;
        end
        repeat (3) @(negedge clk);
        rst_n = 1;
        repeat (2) @(negedge clk);

        // initialise every neuron: v=0, thr=100, leak=2, refr=0
        for (i = 0; i < NN; i = i + 1) set_neuron(i, 0, 100, 2, 0);
        check_all("init state");

        // D1: integrate only, no tick
        send_ev(3, 30);
        send_ev(3, -10);
        send_ev(7, 55);
        wait_idle;
        check_all("integrate (events only)");

        // D2: leak, no fire
        do_tick;
        wait_idle;
        check_all("leak on tick");

        // D3: saturation up and down
        set_neuron(4, 32760, 30000, 15, 0);
        set_neuron(5, -32760, 100, 15, 0);
        send_ev(4, 100);
        send_ev(5, -100);
        wait_idle;
        check_all("saturation");

        // D4: fire, reset, refractory (neuron 1: thr 100, leak 2)
        set_neuron(1, 0, 100, 2, 0);
        send_ev(1, 120);
        do_tick;  wait_idle;                       // 120-30=90 <100: no fire
        check_all("below threshold after leak");
        send_ev(1, 50);
        do_tick;  wait_idle;                       // 90+50=140 -> 105 >=100: fire
        check_all("fire");
        send_ev(1, 127);
        do_tick;  wait_idle;                       // refractory: discarded, refr 2->1
        check_all("refractory 1");
        send_ev(1, 127);
        do_tick;  wait_idle;                       // refr 1->0
        check_all("refractory 2");
        send_ev(1, 127);
        do_tick;  wait_idle;                       // fires again
        check_all("fire again");

        // D4b: threshold boundary. 120 leaks (shift 2) to exactly 90.
        set_neuron(12, 0, 90, 2, 0);              // thr == leaked value: must fire
        set_neuron(13, 0, 91, 2, 0);              // thr == leaked value + 1: must not fire
        send_ev(12, 120);
        send_ev(13, 120);
        do_tick;  wait_idle;
        check_all("threshold boundary (>=)");
        if (exp_id[exp_cnt-1] !== 12) begin
            $display("FAIL boundary: model did not fire neuron 12"); errors = errors + 1;
        end

        // D5: spike back-pressure: three neurons fire in one tick
        set_neuron(8,  0, 100, 2, 0);
        set_neuron(10, 0, 100, 2, 0);
        set_neuron(14, 0, 100, 2, 0);
        send_ev(8, 127); send_ev(8, 100);
        send_ev(10, 127); send_ev(10, 100);
        send_ev(14, 127); send_ev(14, 100);
        wait_idle;
        spk_ready = 1'b0;
        i = got_cnt;
        do_tick;
        repeat (60) @(negedge clk);
        if (spk_valid !== 1'b1 || busy !== 1'b1 || got_cnt !== i) begin
            $display("FAIL spike stall: spk_valid=%b busy=%b delivered=%0d",
                     spk_valid, busy, got_cnt - i);
            errors = errors + 1;
        end else
            $display("PASS spike stall (sweep held, nothing lost)");
        spk_ready = 1'b1;
        wait_idle;
        check_all("spikes after release (ordered)");

        // D6: events are held off while a sweep is pending
        do_tick;
        send_ev(2, 40);                            // must wait for the sweep
        wait_idle;
        check_all("event after tick ordering");

        // D7: tick pending and dropped tick
        tk = 0;
        @(negedge clk);
        tick = 1'b1;
        repeat (3) @(negedge clk);
        tick = 1'b0;
        model_tick; model_tick;                    // 2 sweeps happen, 3rd tick dropped
        wait_idle;
        if (tick_missed !== 1'b1) begin
            $display("FAIL tick_missed not set"); errors = errors + 1;
        end else
            $display("PASS tick_missed set");
        check_all("2 sweeps after 3 rapid ticks");

        // R: random events and ticks against the model
        for (i = 0; i < NN; i = i + 1)
            set_neuron(i, 0, 40 + (rnd(0) % 120), 1 + (rnd(0) % 4), 0);
        k = exp_cnt;
        rdy_en = 1'b1;
        for (t = 0; t < 100; t = t + 1) begin
            n_ev = rnd(0) % 7;
            for (i = 0; i < n_ev; i = i + 1)
                send_ev(rnd(0) % NN, (rnd(0) % 140) - 20);
            do_tick;
            if ((rnd(0) % 4) == 0) wait_idle;
        end
        rdy_en = 1'b0;
        spk_ready = 1'b1;
        wait_idle;
        if (sweeps_started !== ticks_modeled) begin
            $display("FAIL tick/sweep count: model ticks=%0d, DUT sweeps=%0d (a tick was dropped)",
                     ticks_modeled, sweeps_started);
            errors = errors + 1;
        end else
            $display("PASS tick/sweep count (%0d)", sweeps_started);
        check_all("random events and ticks vs model");
        if (exp_cnt - k < 20) begin
            $display("FAIL random test too quiet: only %0d spikes", exp_cnt - k);
            errors = errors + 1;
        end else
            $display("note: random test produced %0d spikes", exp_cnt - k);

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #20000000;
        $display("TIMEOUT");
        $finish;
    end
endmodule