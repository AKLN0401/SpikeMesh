`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 11:17:01 AM
// Design Name: 
// Module Name: aer_router
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


module aer_router #(
    parameter X =0,     //this router's x (0/1 for 2x2)
    parameter Y =0,     //this router's y (0/1 for 2x2)
    parameter AW =2    //input FIFO depth = 2^AW 
    )(
    
    input              clk,
    input              rst_n,
 
    input      [79:0]  in_data,         //Incoming packets: 5 ports × 16 bits = 80 bits. Bits [15:0] are the Local input, [31:16] North, [47:32] East, [63:48] South, [79:64] West.
    input      [4:0]   in_valid,        //	One bit per input port. High means "a packet is on in_data for this port now." It goes to the FIFO's wr_en.
    output     [4:0]   in_ready,        //One bit per input port. High means the FIFO can accept a packet. It equals ~full. Together with in_valid it forms the handshake: a packet transfers only when both are high on a clock edge.
 
    output     [79:0]  out_data,        //Outgoing packets, same flattened layout as in_data, one 16-bit slice per output port.
    output     [4:0]   out_valid,       //One bit per output port. High means that port's out_data slice holds a valid packet.
    input      [4:0]   out_ready        //One bit per output port, driven by the downstream neighbor. High means it can accept the packet. If it is low, the router holds its output register until it goes high (backpressure).

    );
    
      localparam W = 16;        //Packet width in bits (2 type + 1 dest_x + 1 dest_y + 8 axon_id + 4 delay). It is a localparam, so it can't be overridden from outside
 
    // one-hot output request for a packet header
    function [4:0] route_xy;
        input dx;
        input dy;
        begin
            if      (dx > X) route_xy = 5'b00100; // East
            else if (dx < X) route_xy = 5'b10000; // West
            else if (dy > Y) route_xy = 5'b00010; // North
            else if (dy < Y) route_xy = 5'b01000; // South
            else             route_xy = 5'b00001; // Local
        end
    endfunction
 
    // ---------------- input FIFOs + route compute ----------------
    wire [79:0] f_dout;
    wire [4:0]  f_empty, f_full, pop;
    wire [24:0] req_vec;            // req_vec[i*5+o] = input i requests output o
 
    genvar gi;
    generate
        for (gi = 0; gi < 5; gi = gi + 1) begin : g_in
            aer_fifo #(.W(W), .AW(AW)) u_fifo (
                .clk    (clk),
                .rst_n  (rst_n),
                .wr_en  (in_valid[gi]),
                .wr_data(in_data[gi*W +: W]),
                .full   (f_full[gi]),
                .rd_en  (pop[gi]),
                .rd_data(f_dout[gi*W +: W]),
                .empty  (f_empty[gi])
            );
            assign in_ready[gi] = ~f_full[gi];
            assign req_vec[gi*5 +: 5] = f_empty[gi] ? 5'b00000
                                      : route_xy(f_dout[gi*W + 13], f_dout[gi*W + 12]);
        end
    endgenerate
 
    // ---------------- output arbiters ----------------
    reg  [4:0]  out_valid_r;
    reg  [79:0] out_data_r;
    reg  [14:0] rr;                 // 3-bit round-robin pointer per output
    wire [4:0]  can_accept = ~out_valid_r | out_ready;
 
    integer o, k, idx;
    reg         found;
    reg  [24:0] gnt_vec;            // gnt_vec[o*5+i] = output o grants input i
    reg  [79:0] nxt_data;
    reg  [14:0] rr_nxt;
 
    always @* begin
        gnt_vec  = 25'b0;
        nxt_data = 80'b0;
        rr_nxt   = rr;
        for (o = 0; o < 5; o = o + 1) begin
            found = 1'b0;
            for (k = 0; k < 5; k = k + 1) begin
                idx = (rr[o*3 +: 3] + k) % 5;
                if (!found && can_accept[o] && req_vec[idx*5 + o]) begin
                    found                = 1'b1;
                    gnt_vec[o*5 + idx]   = 1'b1;
                    nxt_data[o*W +: W]   = f_dout[idx*W +: W];
                    rr_nxt[o*3 +: 3]     = (idx + 1) % 5;
                end
            end
        end
    end
 
    // an input is popped when any output granted it
    genvar gp;
    generate
        for (gp = 0; gp < 5; gp = gp + 1) begin : g_pop
            assign pop[gp] = gnt_vec[0*5+gp] | gnt_vec[1*5+gp] | gnt_vec[2*5+gp]
                           | gnt_vec[3*5+gp] | gnt_vec[4*5+gp];
        end
    endgenerate
 
    // ---------------- output registers ----------------
    integer q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_valid_r <= 5'b0;
            out_data_r  <= 80'b0;
            rr          <= 15'b0;
        end else begin
            rr <= rr_nxt;
            for (q = 0; q < 5; q = q + 1) begin
                if (can_accept[q]) begin
                    out_valid_r[q] <= |gnt_vec[q*5 +: 5];
                    if (|gnt_vec[q*5 +: 5])
                        out_data_r[q*W +: W] <= nxt_data[q*W +: W];
                end
            end
        end
    end
 
    assign out_valid = out_valid_r;
    assign out_data  = out_data_r;
endmodule

