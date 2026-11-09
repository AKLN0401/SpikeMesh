`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 07:43:08 PM
// Design Name: 
// Module Name: neuron_unit
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


// Neuron unit: time-multiplexed LIF datapath + state memory. Verilog-2001.
//
// Model (discrete time, one timestep = one `tick`):
//   events (any time):  V <- sat(V + weight)            (integrate, no threshold test)
//   on tick, for every neuron n = 0..N-1 in order:
//       if refr != 0 :  refr <- refr - 1;  V <- 0       (refractory: input discarded)
//       else         :  V_l = V - (V >>> leak)           (arithmetic shift, leak = decay shift)
//                       if V_l >= thr : spike n;  V <- 0;  refr <- REFR_P
//                       else          : V <- V_l
//
// State memory entry (one per neuron), {refr, leak, thr, v}:
//   v    [V_W]   membrane potential, signed, saturating
//   thr  [V_W]   firing threshold, signed
//   leak [LK_W]  decay shift: V decays by V>>>leak per tick (0 = full reset each tick)
//   refr [RC_W]  refractory counter (timesteps left); REFR_P loaded on a spike
// The memory has no reset: the host must write EVERY neuron's entry before the
// first tick (the tick sweep visits all 2**NEU_AW neurons).
//
// Timing: one event takes 2 cycles; one tick sweep takes 2 cycles per neuron.
// Events are not accepted while a sweep is pending or running. A tick that
// arrives while another is already pending is dropped and sets tick_missed.
// Host config writes (cfg_we) must not overlap activity: wait for busy == 0.
//
// Spike output is valid/ready; the sweep stalls on a firing neuron until the
// output register is free, so spikes are never lost and come out in neuron order.
module neuron_unit #(
    parameter NEU_AW = 8,    // 256 neurons
    parameter W_W    = 8,    // weight width (two's complement)
    parameter V_W    = 16,   // membrane / threshold width (two's complement)
    parameter LK_W   = 4,    // leak shift width
    parameter RC_W   = 3,    // refractory counter width
    parameter REFR_P = 2     // refractory period in timesteps (< 2**RC_W)
) (
    input                          clk,
    input                          rst_n,

    // host: write one full state entry
    input                          cfg_we,
    input  [NEU_AW-1:0]            cfg_addr,
    input  [RC_W+LK_W+2*V_W-1:0]   cfg_data,   // {refr, leak, thr, v}

    // timestep tick (one-cycle pulse)
    input                          tick,        //One-cycle pulse that starts a timestep (the leak and fire sweep)

    // synaptic events from syn_sram
    input                          ev_valid,
    output                         ev_ready,
    input  [NEU_AW-1:0]            ev_target,       //A synaptic event from the synapse SRAM: which neuron and how much
    input  [W_W-1:0]               ev_weight,

    // output spikes
    output                         spk_valid,       //Output spike (the ID of the neuron that fired), with handshake
    input                          spk_ready,
    output [NEU_AW-1:0]            spk_id,

    output                         busy,            //A sweep is pending or running, or an event is in progress
    output reg                     tick_missed      //A tick was dropped because one was already pending (sticky)
);
    localparam ENT_W = RC_W + LK_W + 2*V_W;         //Entry width: 3 + 4 + 16 + 16 = 39 bits

    localparam [1:0] S_IDLE = 2'd0, S_EV_WR = 2'd1, S_SW_RD = 2'd2, S_SW_WR = 2'd3;     //The four states (0 to 3): idle, write back an event, read for the sweep, compute and write for the sweep
    localparam [NEU_AW-1:0] LAST_N = {NEU_AW{1'b1}};                                    //All ones, the last neuron ID, so the sweep knows when to stop
    localparam [V_W-1:0]    V_MAX  = {1'b0, {(V_W-1){1'b1}}};                           //Saturation limits: 0x7FFF and 0x8000
    localparam [V_W-1:0]    V_MIN  = {1'b1, {(V_W-1){1'b0}}};

    reg [ENT_W-1:0]  mem [0:(1<<NEU_AW)-1];                     //State memory, one entry per neuron
    reg [ENT_W-1:0]  rd_q;                                      //Registered memory read output
    reg [1:0]        st;                                        //Current state
    reg [NEU_AW-1:0] idx, tgt_q;                                //Neuron being visited by the sweep
    reg [W_W-1:0]    w_q;                                       //Event target and weight, held while the event is written back
    reg              tick_pend;                                 //A tick has been requested but its sweep has not started
    reg              spk_v;                                     //Output spike register: valid flag and neuron ID
    reg [NEU_AW-1:0] spk_q;

    // ---------------- control ----------------
    wire start_sweep = (st == S_IDLE) && tick_pend;             //Begin a sweep when idle and a tick is waiting
    assign ev_ready  = (st == S_IDLE) && !tick_pend;            //Accept events only when idle and no sweep is waiting (the tick wins)
    wire accept      = ev_valid && ev_ready;                    //An event is taken this cycle

    // ---------------- unpack the entry read last cycle ----------------
    wire [V_W-1:0]  q_v    = rd_q[V_W-1:0];                    
    wire [V_W-1:0]  q_thr  = rd_q[2*V_W-1:V_W];                 
    wire [LK_W-1:0] q_leak = rd_q[2*V_W+LK_W-1:2*V_W];          
    wire [RC_W-1:0] q_refr = rd_q[ENT_W-1:2*V_W+LK_W];          

    // ---------------- event: saturating add ----------------
    wire [V_W:0]    ev_sum = {q_v[V_W-1], q_v} + {{(V_W+1-W_W){w_q[W_W-1]}}, w_q};              //Signed views of V and the threshold, so shifts and compares are signed
    wire            ev_ovf = ev_sum[V_W] ^ ev_sum[V_W-1];                                       //Overflow if the top two bits differ
    wire [V_W-1:0]  ev_v   = ev_ovf ? (ev_sum[V_W] ? V_MIN : V_MAX) : ev_sum[V_W-1:0];          //On overflow clamp to the limit (negative overflow to V_MIN, positive to V_MAX); otherwise take the sum
    wire [ENT_W-1:0] ev_ent = {q_refr, q_leak, q_thr, ev_v};                                    //Rebuilds the entry with only V changed

    // ---------------- tick: leak, refractory, fire ----------------
    wire signed [V_W-1:0] q_v_s   = q_v;                    //Signed views of V and the threshold, so shifts and compares are signed
    wire signed [V_W-1:0] q_thr_s = q_thr;                  //Arithmetic right shift: V divided by 2^leak, rounding down
    wire signed [V_W-1:0] v_shift = q_v_s >>> q_leak;       //The leaked value, V minus its decay
    wire signed [V_W-1:0] v_leak  = q_v_s - v_shift;        

    wire refr_act = (q_refr != {RC_W{1'b0}});               //Neuron is in its refractory period
    wire fire     = !refr_act && (v_leak >= q_thr_s);       //Fires only if not refractory and the leaked value reaches the threshold

    wire [V_W-1:0]  sw_v    = (refr_act || fire) ? {V_W{1'b0}} : v_leak;                //V is reset to 0 on refractory or fire, else the leaked value
    wire [RC_W-1:0] sw_refr = refr_act ? (q_refr - 1'b1) : (fire ? REFR_P : q_refr);    //Count down while refractory, load REFR_P on a spike, else keep (0)
    wire [ENT_W-1:0] sw_ent = {sw_refr, q_leak, q_thr, sw_v};                           //Rebuilt entry with the new V and refractory counter

    // spike output register
    wire can_emit = !spk_v || spk_ready;                            //The output register is empty or being drained this cycle
    wire stall    = (st == S_SW_WR) && fire && !can_emit;           //A neuron fires but the output is still full, so the sweep must wait
    wire emit     = (st == S_SW_WR) && fire && can_emit;            //The spike goes into the output register this cycle

    // ---------------- memory ----------------
    wire              rd_en   = accept || (st == S_SW_RD);          //Read when an event is accepted, or in the sweep read state
    wire [NEU_AW-1:0] rd_addr = (st == S_SW_RD) ? idx : ev_target;  //Sweep reads the next neuron; events read their target

    wire              wr_ev   = (st == S_EV_WR);                    //Write back in S_EV_WR; write back in S_SW_WR unless stalled
    wire              wr_sw   = (st == S_SW_WR) && !stall;          //Any of the three sources writes
    wire              m_we    = cfg_we || wr_ev || wr_sw;           //Address and data: host first, then event, then sweep
    wire [NEU_AW-1:0] m_wa    = cfg_we ? cfg_addr : (wr_ev ? tgt_q : idx);      
    wire [ENT_W-1:0]  m_wd    = cfg_we ? cfg_data : (wr_ev ? ev_ent : sw_ent);

    always @(posedge clk) begin                     
        if (m_we)  mem[m_wa] <= m_wd;               
        if (rd_en) rd_q      <= mem[rd_addr];
    end

    // ---------------- state machine ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_IDLE;
            idx         <= {NEU_AW{1'b0}};
            tgt_q       <= {NEU_AW{1'b0}};
            w_q         <= {W_W{1'b0}};
            tick_pend   <= 1'b0;
            tick_missed <= 1'b0;
            spk_v       <= 1'b0;
            spk_q       <= {NEU_AW{1'b0}};
        end else begin
            if (tick) begin
                tick_pend <= 1'b1;
                if (tick_pend && !start_sweep) tick_missed <= 1'b1;
            end else if (start_sweep) begin
                tick_pend <= 1'b0;
            end

            if (emit) begin
                spk_v <= 1'b1;
                spk_q <= idx;
            end else if (spk_ready) begin
                spk_v <= 1'b0;
            end

            case (st)
                S_IDLE: begin
                    if (start_sweep) begin
                        idx <= {NEU_AW{1'b0}};
                        st  <= S_SW_RD;
                    end else if (accept) begin
                        tgt_q <= ev_target;
                        w_q   <= ev_weight;
                        st    <= S_EV_WR;
                    end
                end
                S_EV_WR: st <= S_IDLE;
                S_SW_RD: st <= S_SW_WR;
                S_SW_WR: begin
                    if (!stall) begin
                        if (idx == LAST_N) st <= S_IDLE;
                        else begin
                            idx <= idx + 1'b1;
                            st  <= S_SW_RD;
                        end
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    assign spk_valid = spk_v;
    assign spk_id    = spk_q;
    assign busy      = (st != S_IDLE) || tick_pend;
endmodule
