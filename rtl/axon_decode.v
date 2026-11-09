`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 03:41:30 PM
// Design Name: 
// Module Name: axon_decode
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


module axon_decode #(
    parameter AXON_AW =8,       //Axon ID width, so the LUT has 2⁸ = 256 entries
    parameter SYN_AW =10,       //Synapse SRAM address width (1024 entries)
    parameter CNT_W =6,         //Width of the fan-out count (max 63 targets per axon)
    parameter DLY_W =4)         //	Spike delay field width, passed through unchanged
    (
    input clk,
    input rst_n,
    input cfg_we,               //Host write enable for one LUT entry
    input [AXON_AW-1:0] cfg_addr,       //Which axon's LUT entry to write
    input [SYN_AW+CNT_W-1:0] cfg_data,  //The entry {base, count}: first synapse address and number of synapses
    input                     in_valid, //Upstream says "a spike is on the input bus"
    output                    in_ready, //indicator for thhis block can take a spike (high only in IDLE)
    input  [AXON_AW-1:0]      in_axon,  //Axon ID of the spike, used as the LUT index
    input  [DLY_W-1:0]        in_delay, //Delay field of the spike packet, forwarded with each request
    output                    out_valid, //A synapse request is on the output bus (high only in EMIT)
    input                     out_ready, //Downstream (synapse SRAM stage) can take the request
    output [SYN_AW-1:0]       out_syn_addr, //Current synapse SRAM address to read
    output [DLY_W-1:0]        out_delay, //The held spike delay
    output                    out_last //This is the final address of the burst

    );
  localparam LW = SYN_AW + CNT_W;       //computes the entry width, so the LUT word is 16 bits by default.
    localparam [1:0] S_IDLE = 2'd0, S_LOOK = 2'd1, S_EMIT = 2'd2;       // S_IDLE/S_LOOK/S_EMIT	= State encodings (0, 1, 2)

    reg [1:0] st;  reg [LW-1:0] lut [0:(1<<AXON_AW)-1];  reg [LW-1:0] lut_q;    //declares the table as a memory array. Synthesis can infer it as flops or an SRAM macro.
    reg [SYN_AW-1:0] addr;  reg [CNT_W-1:0] remain;  reg [DLY_W-1:0] dly;

    wire accept = (st == S_IDLE) && in_valid;       //the spike is accepted only when idle and upstream is offering one.
    wire fire   = (st == S_EMIT) && out_ready;      //the output transfer occurs only while emitting and downstream is ready.

    assign in_ready     = (st == S_IDLE);           //accepts new spikes only when idle, which gives one spike in flight at a time
    assign out_valid    = (st == S_EMIT);           //a request is offered only in EMIT.
    assign out_syn_addr = addr;                     //direct register outputs.
    assign out_delay    = dly;
    assign out_last     = (remain == {{(CNT_W-1){1'b0}}, 1'b1});    //a combinational flag, high when exactly one entry is left

    always @(posedge clk) begin           // LUT: sync write, registered read
        if (cfg_we) lut[cfg_addr] <= cfg_data;          //the synchronous write the host uses to program connectivity.
        if (accept) lut_q <= lut[in_axon];              //the registered read, captured at the same edge the spike is accepted. The registered read is what lets you swap the array for an SRAM macro
    end

    always @(posedge clk or negedge rst_n) begin //FSM block (async reset)
        if (!rst_n) begin
            st <= S_IDLE; addr <= 0; remain <= 0; dly <= 0;
        end else case (st)
            S_IDLE: if (accept) begin dly <= in_delay; st <= S_LOOK; end        //latches the delay and moves on. The LUT read is happening at this same edge.
            S_LOOK: begin
                addr   <= lut_q[LW-1:CNT_W];    //takes the top bits, base.
                remain <= lut_q[CNT_W-1:0];     //takes the low bits, count.  
                st     <= (lut_q[CNT_W-1:0] == 0) ? S_IDLE : S_EMIT;    //drops the spike if the axon has no fan-out.
            end
            S_EMIT: if (fire) begin
                addr <= addr + 1'b1; remain <= remain - 1'b1;       //steps to the next synapse.
                if (out_last) st <= S_IDLE;                         //; finishes the burst on the last entry. out_last is evaluated before the edge, so the last request is the one being consumed now.
            end
            default: st <= S_IDLE;              //recovers from the unused state value 3.
        endcase
    end
endmodule
