`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 05:41:12 PM
// Design Name: 
// Module Name: syn_sram
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

// Synapse SRAM (weight memory) with a valid/ready read stage. Verilog-2001.
//
// Each entry holds {target_neuron, weight}. The address comes from axon_decode
// (out_syn_addr); the spike delay and the "last entry of this spike" flag ride
// along as sideband so the neuron unit can tell when a spike's fan-out ends.
//
// Read: 1-cycle latency. Output stage is one register with valid/ready:
//   a request is accepted when the output register is empty or being consumed,
//   so a stalled consumer holds the data stable and back-pressures the sender.
// Write: synchronous host port (config phase). A read and a write to the same
//   address in the same cycle returns the OLD data.
// Weight is two's complement. The array has no reset: the host must program
// every entry that will be read.
module syn_sram #(
    parameter SYN_AW = 10,   // 1024 entries
    parameter NEU_AW = 8,    // target neuron ID width
    parameter W_W    = 8,    // weight width
    parameter DLY_W  = 4
) (
    input                        clk,
    input                        rst_n,

    // host write port: {target, weight}
    input                        cfg_we,
    input  [SYN_AW-1:0]          cfg_addr,
    input  [NEU_AW+W_W-1:0]      cfg_data,

    // read request from axon_decode
    input                        in_valid,
    output                       in_ready,
    input  [SYN_AW-1:0]          in_addr,
    input  [DLY_W-1:0]           in_delay,
    input                        in_last,

    // read result to the neuron unit
    output                       out_valid,
    input                        out_ready,
    output [NEU_AW-1:0]          out_target,
    output [W_W-1:0]             out_weight,
    output [DLY_W-1:0]           out_delay,
    output                       out_last
);
    localparam EW = NEU_AW + W_W;       //Entry width.

    reg [EW-1:0]    mem [0:(1<<SYN_AW)-1];      //The weight array, 1024 entries of 16 bits. Synthesis can make this flops or an SRAM macro.
    reg [EW-1:0]    rd_q;                       //Holds the entry that was read; this is the registered SRAM output.
    reg             v;                          //The output stage is full (a result is waiting). v for "valid"
    reg [DLY_W-1:0] dly_q;                      //Sideband stored next to rd_q so it stays aligned with the data.
    reg             last_q;                     
    
    assign in_ready = ~v | out_ready;           //Accept a new request if the stage is empty, or if the current result is being consumed this cycle.
    wire   fire     = in_valid & in_ready;      //A request is accepted at this clock edge.

    // memory: sync write, registered read with read enable
    always @(posedge clk) begin
        if (cfg_we) mem[cfg_addr] <= cfg_data;  //Synchronous host write.
        if (fire)   rd_q          <= mem[in_addr];          //Read only on an accepted request. The read enable lets this map to a macro, and the data stays stable while the consumer stalls. A write to the same address in the same cycle returns the old data.
    end

    // sideband and valid
    always @(posedge clk or negedge rst_n) begin            
        if (!rst_n) begin                       //Async reset of the control state. The array and rd_q are not reset, so the host must program every entry it reads.
            v      <= 1'b0;
            dly_q  <= {DLY_W{1'b0}};
            last_q <= 1'b0;
        end else begin
            if (fire) begin                     //New result: mark valid and capture the sideband.
                v      <= 1'b1;
                dly_q  <= in_delay;
                last_q <= in_last;
            end else if (out_ready) begin       //The result was consumed and no new request arrived, so the stage empties. If out_ready is low, nothing changes and the data holds.
                v      <= 1'b0;
            end
        end
    end

    assign out_valid  = v;                      //Valid is the register.
    assign out_target = rd_q[EW-1:W_W];         //Upper 8 bits are the target neuron.
    assign out_weight = rd_q[W_W-1:0];          //Lower 8 bits are the weight.
    assign out_delay  = dly_q;                  //egistered sideband.
    assign out_last   = last_q;
endmodule
