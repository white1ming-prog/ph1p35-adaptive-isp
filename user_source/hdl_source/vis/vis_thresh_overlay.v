`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_thresh_overlay
//
// Pass-through tap that draws an 8-pixel-wide red vertical bar at
// x = threshold * 5, so a 0..255 threshold maps onto a 0..1275 column of the
// 1280-wide picture. The bar's position is the on-screen readout of the
// adaptive threshold: brighter scene -> higher threshold -> bar moves right.
//
// Purely combinational on the pixel path, so it adds no pipeline latency and
// cannot disturb the handshake.
//----------------------------------------------------------------------------
module vis_thresh_overlay #(
    parameter IMG_WIDTH = 1280,
    parameter BAR_W     = 8,
    parameter BX_BITS   = 9          // (IMG_WIDTH/4) beat counter width
)(
    input               I_clk,
    input               I_rst_n,

    input               I_tuser,
    input               I_tlast,
    input               I_tvalid,
    input      [95:0]   I_tdata,
    input               I_tready,

    input      [7:0]    I_thresh,

    output reg [95:0]   O_tdata,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg          O_tvalid,
    output              O_tready
);

    assign O_tready = I_tready;

    // ----- beat counter, reset at the start of every line -------------------
    reg [BX_BITS-1:0] bx;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n)
            bx <= {BX_BITS{1'b0}};
        else if (I_tvalid) begin
            if (I_tuser || I_tlast)
                bx <= {BX_BITS{1'b0}};
            else
                bx <= bx + 1'b1;
        end
    end

    // ----- bar window -------------------------------------------------------
    // threshold * 5 == (threshold << 2) + threshold, no multiplier needed.
    wire [11:0] bar_lo = {4'b0, (I_thresh << 2)} + {4'b0, I_thresh};
    wire [11:0] bar_hi = bar_lo + BAR_W - 1;

    wire [11:0] x_base = {3'b000, bx, 2'b00};      // beat * 4
    wire [11:0] x0 = x_base + 12'd0;
    wire [11:0] x1 = x_base + 12'd1;
    wire [11:0] x2 = x_base + 12'd2;
    wire [11:0] x3 = x_base + 12'd3;

    wire hit0 = (x0 >= bar_lo) && (x0 <= bar_hi);
    wire hit1 = (x1 >= bar_lo) && (x1 <= bar_hi);
    wire hit2 = (x2 >= bar_lo) && (x2 <= bar_hi);
    wire hit3 = (x3 >= bar_lo) && (x3 <= bar_hi);

    localparam [23:0] BAR_COLOR = 24'hff0000;

    always @(*) begin
        O_tdata  = I_tdata;
        O_tlast  = I_tlast;
        O_tuser  = I_tuser;
        O_tvalid = I_tvalid;

        if (hit0) O_tdata[95:72] = BAR_COLOR;
        if (hit1) O_tdata[71:48] = BAR_COLOR;
        if (hit2) O_tdata[47:24] = BAR_COLOR;
        if (hit3) O_tdata[23:0]  = BAR_COLOR;
    end

endmodule
