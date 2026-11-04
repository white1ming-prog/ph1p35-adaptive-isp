`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_auto_gain
//
// Digital automatic brightness for the "adaptive image" mode. The per-frame
// luma sum from vis_frame_stats selects a gain level once per frame. The gain
// is applied to all three channels of every pixel.
//
// Only shifts and adds are used, so there is no multiplier. The four levels
// are 1x, 1.5x, 3x and 32x; the last level also lifts non-zero dark pixels.
//
// Result is clipped to 0..255, so a bright scene can never wrap.
//
// Note this is a DIGITAL gain applied after capture - it cannot recover detail
// that the sensor already clipped. It changes the picture the way a real
// exposure adjustment looks, but the sensor's own exposure is untouched.
//----------------------------------------------------------------------------

module vis_auto_gain #(
parameter SUM_SHIFT = 20,
    parameter [7:0] LO = 8'd132,     // kept for compatibility
    parameter [7:0] HI = 8'd162
)(
input I_clk,
input I_rst_n,

input I_tuser,
input I_tlast,
input I_tvalid,
input [95:0] I_tdata,
input I_tready,

input [31:0] I_luma_sum,
input I_frame_done,

output reg [95:0] O_tdata,
output reg O_tlast,
output reg O_tuser,
output reg O_tvalid,
output O_tready,
output reg [1:0] O_level
);

assign O_tready = I_tready;

wire [7:0] mean8 = I_luma_sum >> SUM_SHIFT;

always @(posedge I_clk or negedge I_rst_n) begin
if(!I_rst_n)
O_level <= 2'd0;
else if(I_frame_done) begin
if(mean8 >= 8'd145)
O_level <= 2'd0;
else if(mean8 >= 8'd90)
O_level <= 2'd1;
else if(mean8 >= 8'd20)
O_level <= 2'd2;
else
O_level <= 2'd3;
end
end

function [7:0] gain8;
input [7:0] c;
input [1:0] l;
reg [11:0] t;
begin
case(l)
2'd0: t = {4'b0,c};
2'd1: t = {4'b0,c} + ({4'b0,c} >> 1);
2'd2: t = {3'b0,c,2'b0} - {4'b0,c};
default: t = {1'b0,c,5'b0}
          + ((c == 8'd0) ? 12'd0 : 12'd16);
endcase
gain8 = (t > 12'd255) ? 8'd255 : t[7:0];
end
endfunction

function [23:0] gain_pixel;
input [23:0] p;
input [1:0] l;
begin
gain_pixel = { gain8(p[23:16], l),
gain8(p[15:8], l),
gain8(p[7:0], l) };
end
endfunction

// ========= 新增：组合中间信号 + 寄存器输出 =========
wire [95:0] tdata_w;
wire tlast_w, tuser_w, tvalid_w;

assign tdata_w = { gain_pixel(I_tdata[95:72], O_level),
gain_pixel(I_tdata[71:48], O_level),
gain_pixel(I_tdata[47:24], O_level),
gain_pixel(I_tdata[23:0], O_level) };
assign tlast_w = I_tlast;
assign tuser_w = I_tuser;
assign tvalid_w = I_tvalid;

always @(posedge I_clk or negedge I_rst_n) begin
if(!I_rst_n) begin
O_tdata <= 96'd0;
O_tlast <= 1'b0;
O_tuser <= 1'b0;
O_tvalid <= 1'b0;
end else begin
O_tdata <= tdata_w;
O_tlast <= tlast_w;
O_tuser <= tuser_w;
O_tvalid <= tvalid_w;
end
end

endmodule
