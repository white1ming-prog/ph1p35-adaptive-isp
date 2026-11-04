`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_binarize
//
// Adaptive black/white output. Each pixel is reduced to the 1/4 + 1/2 + 1/4
// luma and compared against the per-frame Otsu threshold, so the split point
// follows the scene instead of being a compile-time constant.
//
// This is the basic requirement's "image processing algorithm" (binarization
// is one of the listed options) and it is structurally different from the
// reference design's Sobel edge detector.
//
// Purely combinational on the pixel path, so it adds no pipeline latency.
//----------------------------------------------------------------------------
module vis_binarize (
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

    function [7:0] lum;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        begin
            lum = {2'b00, r[7:2]} + {1'b0, g[7:1]} + {2'b00, b[7:2]};
        end
    endfunction

    wire [7:0] y0 = lum(I_tdata[95:88], I_tdata[87:80], I_tdata[79:72]);
    wire [7:0] y1 = lum(I_tdata[71:64], I_tdata[63:56], I_tdata[55:48]);
    wire [7:0] y2 = lum(I_tdata[47:40], I_tdata[39:32], I_tdata[31:24]);
    wire [7:0] y3 = lum(I_tdata[23:16], I_tdata[15:8],  I_tdata[7:0]);

    localparam [23:0] WHITE = 24'hffffff;
    localparam [23:0] BLACK = 24'h000000;

    always @(*) begin
        O_tdata  = I_tdata;
        O_tlast  = I_tlast;
        O_tuser  = I_tuser;
        O_tvalid = I_tvalid;

        O_tdata[95:72] = (y0 >= I_thresh) ? WHITE : BLACK;
        O_tdata[71:48] = (y1 >= I_thresh) ? WHITE : BLACK;
        O_tdata[47:24] = (y2 >= I_thresh) ? WHITE : BLACK;
        O_tdata[23:0]  = (y3 >= I_thresh) ? WHITE : BLACK;
    end

endmodule
