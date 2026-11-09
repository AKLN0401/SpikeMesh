`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Khoi Nguyen
// 
// Create Date: 11/08/2026 11:17:55 PM
// Design Name: 
// Module Name: aer_mesh2x2
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
// 2x2 mesh: four neurocores, four AER routers. Verilog-2001.
// Core (x,y) is attached to the Local port of router (x,y).
// x grows East, y grows North. Router ports: 0=Local 1=North 2=East 3=South 4=West.
// Host interface: one config write port per core (bank/addr/data as in neurocore)
// and one global tick. Unused edge ports are tied off.
//
// core_idle[i] : core i holds no spike/event/packet (index = 2*y + x)
// net_active   : any router input or output is carrying a packet this cycle
module aer_mesh2x2 #(
    parameter NEU_AW   = 8,
    parameter SPK_FIFO = 1,
    parameter RAW      = 2          // router input FIFO depth = 2**RAW
) (
    input               clk,
    input               rst_n,

    input  [3:0]        cfg_we,
    input  [7:0]        cfg_bank,        // 2 bits per core
    input  [39:0]       cfg_addr,        // 10 bits per core
    input  [155:0]      cfg_data,        // 39 bits per core
    input               tick,
    output [3:0]        core_idle,
    output [3:0]        tick_missed,
    output              net_active
);
    // router <-> core
    wire [3:0]  c2r_valid, c2r_ready, r2c_valid, r2c_ready;
    wire [63:0] c2r_data, r2c_data;

    // router buses
    wire [79:0] in_d  [0:3];
    wire [4:0]  in_v  [0:3];
    wire [4:0]  in_r  [0:3];
    wire [79:0] out_d [0:3];
    wire [4:0]  out_v [0:3];
    wire [4:0]  out_r [0:3];

    // index: 0=(0,0) 1=(1,0) 2=(0,1) 3=(1,1)
    genvar i;
    generate
        for (i = 0; i < 4; i = i + 1) begin : g_core
            neurocore #(.NEU_AW(NEU_AW), .SPK_FIFO(SPK_FIFO)) u_core (
                .clk(clk), .rst_n(rst_n),
                .cfg_we(cfg_we[i]), .cfg_bank(cfg_bank[i*2 +: 2]),
                .cfg_addr(cfg_addr[i*10 +: 10]), .cfg_data(cfg_data[i*39 +: 39]),
                .tick(tick), .idle(core_idle[i]), .tick_missed(tick_missed[i]),
                .pkt_in_valid(r2c_valid[i]), .pkt_in_ready(r2c_ready[i]), .pkt_in_data(r2c_data[i*16 +: 16]),
                .pkt_out_valid(c2r_valid[i]), .pkt_out_ready(c2r_ready[i]), .pkt_out_data(c2r_data[i*16 +: 16])
            );
        end
    endgenerate

    aer_router #(.X(0), .Y(0), .AW(RAW)) u_r0 (.clk(clk), .rst_n(rst_n),
        .in_data(in_d[0]), .in_valid(in_v[0]), .in_ready(in_r[0]),
        .out_data(out_d[0]), .out_valid(out_v[0]), .out_ready(out_r[0]));
    aer_router #(.X(1), .Y(0), .AW(RAW)) u_r1 (.clk(clk), .rst_n(rst_n),
        .in_data(in_d[1]), .in_valid(in_v[1]), .in_ready(in_r[1]),
        .out_data(out_d[1]), .out_valid(out_v[1]), .out_ready(out_r[1]));
    aer_router #(.X(0), .Y(1), .AW(RAW)) u_r2 (.clk(clk), .rst_n(rst_n),
        .in_data(in_d[2]), .in_valid(in_v[2]), .in_ready(in_r[2]),
        .out_data(out_d[2]), .out_valid(out_v[2]), .out_ready(out_r[2]));
    aer_router #(.X(1), .Y(1), .AW(RAW)) u_r3 (.clk(clk), .rst_n(rst_n),
        .in_data(in_d[3]), .in_valid(in_v[3]), .in_ready(in_r[3]),
        .out_data(out_d[3]), .out_valid(out_v[3]), .out_ready(out_r[3]));

    // ---- local ports (port 0) ----
    genvar j;
    generate
        for (j = 0; j < 4; j = j + 1) begin : g_loc
            assign in_d[j][15:0]  = c2r_data[j*16 +: 16];
            assign in_v[j][0]     = c2r_valid[j];
            assign c2r_ready[j]   = in_r[j][0];
            assign r2c_data[j*16 +: 16] = out_d[j][15:0];
            assign r2c_valid[j]   = out_v[j][0];
            assign out_r[j][0]    = r2c_ready[j];
        end
    endgenerate

    // ---- links. A.out[East=2] -> B.in[West=4], A.out[West=4] -> B.in[East=2]
    //            A.out[North=1] -> B.in[South=3], A.out[South=3] -> B.in[North=1]
    // r0(0,0) <-> r1(1,0)  (East/West)
    assign in_d[1][4*16 +: 16] = out_d[0][2*16 +: 16];
    assign in_v[1][4]          = out_v[0][2];
    assign out_r[0][2]         = in_r[1][4];
    assign in_d[0][2*16 +: 16] = out_d[1][4*16 +: 16];
    assign in_v[0][2]          = out_v[1][4];
    assign out_r[1][4]         = in_r[0][2];
    // r2(0,1) <-> r3(1,1)
    assign in_d[3][4*16 +: 16] = out_d[2][2*16 +: 16];
    assign in_v[3][4]          = out_v[2][2];
    assign out_r[2][2]         = in_r[3][4];
    assign in_d[2][2*16 +: 16] = out_d[3][4*16 +: 16];
    assign in_v[2][2]          = out_v[3][4];
    assign out_r[3][4]         = in_r[2][2];
    // r0(0,0) <-> r2(0,1)  (North/South)
    assign in_d[2][3*16 +: 16] = out_d[0][1*16 +: 16];
    assign in_v[2][3]          = out_v[0][1];
    assign out_r[0][1]         = in_r[2][3];
    assign in_d[0][1*16 +: 16] = out_d[2][3*16 +: 16];
    assign in_v[0][1]          = out_v[2][3];
    assign out_r[2][3]         = in_r[0][1];
    // r1(1,0) <-> r3(1,1)
    assign in_d[3][3*16 +: 16] = out_d[1][1*16 +: 16];
    assign in_v[3][3]          = out_v[1][1];
    assign out_r[1][1]         = in_r[3][3];
    assign in_d[1][1*16 +: 16] = out_d[3][3*16 +: 16];
    assign in_v[1][1]          = out_v[3][3];
    assign out_r[3][3]         = in_r[1][1];

    // ---- tie-offs for edge ports (no neighbour) ----
    // r0: North->r2, East->r1 used; South(3), West(4) are edges
    assign in_d[0][3*16 +: 16] = 16'd0;  assign in_v[0][3] = 1'b0;  assign out_r[0][3] = 1'b1;
    assign in_d[0][4*16 +: 16] = 16'd0;  assign in_v[0][4] = 1'b0;  assign out_r[0][4] = 1'b1;
    // r1: North->r3, West->r0 used; East(2), South(3) are edges
    assign in_d[1][2*16 +: 16] = 16'd0;  assign in_v[1][2] = 1'b0;  assign out_r[1][2] = 1'b1;
    assign in_d[1][3*16 +: 16] = 16'd0;  assign in_v[1][3] = 1'b0;  assign out_r[1][3] = 1'b1;
    // r2: South->r0, East->r3 used; North(1), West(4) are edges
    assign in_d[2][1*16 +: 16] = 16'd0;  assign in_v[2][1] = 1'b0;  assign out_r[2][1] = 1'b1;
    assign in_d[2][4*16 +: 16] = 16'd0;  assign in_v[2][4] = 1'b0;  assign out_r[2][4] = 1'b1;
    // r3: South->r1, West->r2 used; North(1), East(2) are edges
    assign in_d[3][1*16 +: 16] = 16'd0;  assign in_v[3][1] = 1'b0;  assign out_r[3][1] = 1'b1;
    assign in_d[3][2*16 +: 16] = 16'd0;  assign in_v[3][2] = 1'b0;  assign out_r[3][2] = 1'b1;

    assign net_active = (|in_v[0]) | (|in_v[1]) | (|in_v[2]) | (|in_v[3])
                      | (|out_v[0]) | (|out_v[1]) | (|out_v[2]) | (|out_v[3]);
endmodule