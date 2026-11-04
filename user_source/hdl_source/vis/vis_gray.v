`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_gray
//
// Luma-only output for the grayscale mode. Every pixel is reduced to
// Y = 1/4 R + 1/2 G + 1/4 B and replicated back onto R, G and B so the rest of
// the pipeline keeps seeing the same 96-bit RGB format.
//
// Purely combinational on the pixel path.
//----------------------------------------------------------------------------
module vis_gray (
    input               I_clk,
    input               I_rst_n,

    input               I_tuser,
    input               I_tlast,
    input               I_tvalid,
    input      [95:0]   I_tdata,
    input               I_tready,

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

    always @(*) begin
        O_tdata  = {y0,y0,y0, y1,y1,y1, y2,y2,y2, y3,y3,y3};
        O_tlast  = I_tlast;
        O_tuser  = I_tuser;
        O_tvalid = I_tvalid;
    end

endmodule
