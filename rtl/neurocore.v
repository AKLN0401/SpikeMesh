`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 11:12:27 PM
// Design Name: 
// Module Name: neurocore
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


// Neurocore: one node of the mesh. Verilog-2001.
//
//   pkt_in --> axon_decode --> syn_sram --> neuron_unit --> [spike FIFO] --> spike_gen --> pkt_out
//
// pkt_in / pkt_out use the router's 16-bit packet and connect to the router
// Local output / Local input (port 0).
//
// Host configuration: one write port, bank selected by cfg_bank
//   0: axon LUT        {base[SYN_AW-1:0], count[5:0]}                 (cfg_addr = axon id)
//   1: synapse SRAM    {target[NEU_AW-1:0], weight[7:0]}              (cfg_addr = synapse address)
//   2: neuron state    {refr[2:0], leak[3:0], thr[15:0], v[15:0]}     (cfg_addr = neuron id)
//   3: spike table     {en, dest_x, dest_y, axon_id[7:0], delay[3:0]} (cfg_addr = neuron id)
// Write only while idle is high. All memories have no reset: the host must
// initialise every neuron (banks 2 and 3) and every synapse/axon it uses.
//
// SPK_FIFO = 1 puts a FIFO of 2**NEU_AW spikes between neuron_unit and
// spike_gen. A sweep emits at most one spike per neuron, so the FIFO can
// never fill during one sweep and the sweep never waits for the network.
// SPK_FIFO = 0 connects them directly (the sweep stalls while the network
// is back-pressured).
//
// idle = the core holds no spike, event or packet. The host must pulse
// `tick` only when every core is idle and the network is empty.
module neurocore #(
    parameter NEU_AW   = 8,
    parameter AXON_AW  = 8,
    parameter SYN_AW   = 10,
    parameter SPK_FIFO = 1
) (
    input               clk,
    input               rst_n,

    // host
    input               cfg_we,
    input  [1:0]        cfg_bank,
    input  [9:0]        cfg_addr,
    input  [38:0]       cfg_data,
    input               tick,
    output              idle,
    output              tick_missed,

    // from router Local output
    input               pkt_in_valid,
    output              pkt_in_ready,
    input  [15:0]       pkt_in_data,

    // to router Local input
    output              pkt_out_valid,
    input               pkt_out_ready,
    output [15:0]       pkt_out_data
);
    // ---------------- config decode ----------------
    wire ax_we = cfg_we && (cfg_bank == 2'd0);
    wire sy_we = cfg_we && (cfg_bank == 2'd1);
    wire nu_we = cfg_we && (cfg_bank == 2'd2);
    wire sg_we = cfg_we && (cfg_bank == 2'd3);

    // ---------------- axon decode ----------------
    wire               l_valid, l_ready, l_last;
    wire [SYN_AW-1:0]  l_addr;
    wire [3:0]         l_delay;
    wire               ax_in_ready;

    axon_decode #(.AXON_AW(AXON_AW), .SYN_AW(SYN_AW)) u_ax (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(ax_we), .cfg_addr(cfg_addr[AXON_AW-1:0]), .cfg_data(cfg_data[SYN_AW+6-1:0]),
        .in_valid(pkt_in_valid), .in_ready(ax_in_ready),
        .in_axon(pkt_in_data[11:4]), .in_delay(pkt_in_data[3:0]),
        .out_valid(l_valid), .out_ready(l_ready),
        .out_syn_addr(l_addr), .out_delay(l_delay), .out_last(l_last)
    );
    assign pkt_in_ready = ax_in_ready;

    // ---------------- synapse SRAM ----------------
    wire               e_valid, e_ready, e_last;
    wire [NEU_AW-1:0]  e_target;
    wire [7:0]         e_weight;
    wire [3:0]         e_delay;

    syn_sram #(.SYN_AW(SYN_AW), .NEU_AW(NEU_AW)) u_sy (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(sy_we), .cfg_addr(cfg_addr[SYN_AW-1:0]), .cfg_data(cfg_data[NEU_AW+8-1:0]),
        .in_valid(l_valid), .in_ready(l_ready), .in_addr(l_addr),
        .in_delay(l_delay), .in_last(l_last),
        .out_valid(e_valid), .out_ready(e_ready),
        .out_target(e_target), .out_weight(e_weight),
        .out_delay(e_delay), .out_last(e_last)
    );

    // ---------------- neuron unit ----------------
    wire               nu_spk_valid, nu_spk_ready, nu_busy;
    wire [NEU_AW-1:0]  nu_spk_id;

    neuron_unit #(.NEU_AW(NEU_AW)) u_nu (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(nu_we), .cfg_addr(cfg_addr[NEU_AW-1:0]), .cfg_data(cfg_data),
        .tick(tick),
        .ev_valid(e_valid), .ev_ready(e_ready),
        .ev_target(e_target), .ev_weight(e_weight),
        .spk_valid(nu_spk_valid), .spk_ready(nu_spk_ready), .spk_id(nu_spk_id),
        .busy(nu_busy), .tick_missed(tick_missed)
    );

    // ---------------- optional spike FIFO ----------------
    wire               sg_valid, sg_ready, f_empty;
    wire [NEU_AW-1:0]  sg_id;

    generate
        if (SPK_FIFO != 0) begin : g_fifo
            wire f_full;
            aer_fifo #(.W(NEU_AW), .AW(NEU_AW)) u_sf (
                .clk(clk), .rst_n(rst_n),
                .wr_en(nu_spk_valid & ~f_full), .wr_data(nu_spk_id), .full(f_full),
                .rd_en(sg_valid & sg_ready),    .rd_data(sg_id),     .empty(f_empty)
            );
            assign nu_spk_ready = ~f_full;
            assign sg_valid     = ~f_empty;
        end else begin : g_direct
            assign nu_spk_ready = sg_ready;
            assign sg_valid     = nu_spk_valid;
            assign sg_id        = nu_spk_id;
            assign f_empty      = 1'b1;
        end
    endgenerate

    // ---------------- spike generator ----------------
    spike_gen #(.NEU_AW(NEU_AW)) u_sg (
        .clk(clk), .rst_n(rst_n),
        .cfg_we(sg_we), .cfg_addr(cfg_addr[NEU_AW-1:0]), .cfg_data(cfg_data[14:0]),
        .spk_valid(sg_valid), .spk_ready(sg_ready), .spk_id(sg_id),
        .pkt_valid(pkt_out_valid), .pkt_ready(pkt_out_ready), .pkt_data(pkt_out_data)
    );

    assign idle = ax_in_ready & ~l_valid & ~e_valid & ~nu_busy & ~nu_spk_valid
                & f_empty & ~pkt_out_valid;
endmodule
