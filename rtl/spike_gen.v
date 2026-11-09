`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 08:29:34 PM
// Design Name: 
// Module Name: spike_gen
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
// Spike generator (AER out): neuron spike -> router packet. Verilog-2001.
//
// Each neuron has one routing entry {en, dest_x, dest_y, axon_id, delay}
// written by the host. A spike from the neuron unit reads its entry and is
// turned into the 16-bit router packet
//     {type[1:0], dest_x, dest_y, axon_id[7:0], delay[3:0]}
// and offered on a valid/ready link (connect to router local input, port 0).
// A neuron whose entry has en = 0 is not routed: its spike is consumed and
// no packet is produced.
//
// One result per cycle when the consumer keeps up. The table has no reset:
// the host must write every entry that can spike (a neuron whose entry was
// never written has an undefined en bit).
module spike_gen #(
    parameter NEU_AW   = 8,         // neuron ID width (match neuron_unit)
    parameter PKT_TYPE = 2'b00      // value placed in packet type [15:14]
) (
    input                 clk,
    input                 rst_n,

    // host: write one routing entry {en, dest_x, dest_y, axon_id, delay}
    input                 cfg_we,
    input  [NEU_AW-1:0]   cfg_addr,
    input  [14:0]         cfg_data,

    // spikes from neuron_unit
    input                 spk_valid,
    output                spk_ready,
    input  [NEU_AW-1:0]   spk_id,

    // AER packet to the router
    output                pkt_valid,
    input                 pkt_ready,
    output [15:0]         pkt_data
);
    reg [14:0] mem [0:(1<<NEU_AW)-1];
    reg [14:0] rd_q;
    reg        v;                       // output stage holds a spike

    wire en_q = rd_q[14];

    assign pkt_valid = v & en_q;
    // stage can take a new spike when empty, unrouted, or being consumed
    assign spk_ready = ~v | ~en_q | pkt_ready;
    wire   fire      = spk_valid & spk_ready;

    always @(posedge clk) begin
        if (cfg_we) mem[cfg_addr] <= cfg_data;
        if (fire)   rd_q          <= mem[spk_id];
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      v <= 1'b0;
        else if (fire)   v <= 1'b1;
        else if (spk_ready) v <= 1'b0;
    end

    // rd_q[13]=dest_x [12]=dest_y [11:4]=axon_id [3:0]=delay
    assign pkt_data = {PKT_TYPE[1:0], rd_q[13:0]};
endmodule
