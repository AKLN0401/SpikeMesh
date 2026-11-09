`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 11:17:01 AM
// Design Name: Neuromophic chip
// Module Name: aer_fifo
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
//wptr: write pointer
//rptr:read pointer

module aer_fifo #( 
    parameter W = 16, //Width of one entry in bits. (2 type + 1 dest_x + 1 dest_y + 8 axon_id + 4 delay)
    parameter AW =2   //Address width. Depth is 2^AW, so 4 entries by default.
    )( 
    input clk,      //Clock. All reads and writes happen on the rising edge.
    input rst_n,    //Active-low asynchronous reset. It clears both pointers, so the FIFO becomes empty.
    input wr_en,    //Write request. The wr_data value is stored on the next clock edge if the FIFO isn’t full.
    input [W-1:0] wr_data, //The packet to store.
    input rd_en,    //Read request
    output full,    //High when all 2^AW entries are used. The router turns this into in_ready = ~full, which is the backpressure signal to the upstream sender.
    output [W-1:0]rd_data,  //The oldest entry. It is always visible without needing rd_en first (first-word-fall-through), so the router can read the header and decide the route before popping.
    output empty    //High when there is nothing to read. The router uses it to avoid requesting an output with stale data.
    );
    
    reg [W-1:0] mem [0:(1<<AW)-1];  //mem has 2^AW slots (4 by default), each W bits wide.
    reg [AW:0]  wptr, rptr;         //wptr is where the next packet is written. rptr is where the oldest packet is read.
 
    assign empty   = (wptr == rptr); //Empty: both pointers are identical, so the reader has caught up with the writer. 
    assign full    = (wptr[AW] != rptr[AW]) && (wptr[AW-1:0] == rptr[AW-1:0]); //Full: both point at the same slot, but the top bits differ, so the writer has gone around once more than the reader and every slot holds unread data.
    assign rd_data = mem[rptr[AW-1:0]]; //The oldest entry is always on rd_data, with no clock delay (first-word-fall-through). The router can look at the header and pick the route before popping.
 
    always @(posedge clk or negedge rst_n) begin        //Reset sets both pointers to 0, which means empty. The memory contents are not cleared, because they don't need to be.
        if (!rst_n) begin
            wptr <= {(AW+1){1'b0}};
            rptr <= {(AW+1){1'b0}};
            
        end else begin
            if (wr_en && !full) begin
                mem[wptr[AW-1:0]] <= wr_data;           //Write: store the packet in the slot wptr points at, then advance wptr.
                wptr <= wptr + 1'b1;
            end
            if (rd_en && !empty)                        //Read: rd_en only advances rptr. The old data stays in memory until it is overwritten
                rptr <= rptr + 1'b1;
        end
    end
endmodule
    
