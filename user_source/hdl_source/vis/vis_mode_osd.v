`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_mode_osd
//
// Draws the current mode number as a large digit in the top right corner of
// the frame, so the on-screen readout matches the switch that is up.
//
//   mode 0 (no switch) -> "1"   original picture
//   mode 1 (SW1)       -> "2"   Scharr edge
//   mode 2 (SW2)       -> "3"   binarize
//
// This overlays the ISP stream directly instead of touching the vendor
// hdmi_mixer, so the original OSD (logo and counter) is left untouched.
// Purely combinational on the pixel path.
//----------------------------------------------------------------------------
module vis_mode_osd (
    input               I_clk,
    input               I_rst_n,

    input               I_tuser,
    input               I_tlast,
    input               I_tvalid,
    input      [95:0]   I_tdata,
    input               I_tready,

    input      [1:0]    I_mode,

    output reg [95:0]   O_tdata,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg          O_tvalid,
    output              O_tready
);

    assign O_tready = I_tready;

    // 5x7 font scaled by 8 -> a 40 x 56 cell.
    localparam [11:0] X0    = 12'd1160;
    localparam [11:0] Y0    = 12'd40;
    localparam [11:0] CELLW = 12'd40;
    localparam [11:0] CELLH = 12'd56;

    localparam [23:0] COL_BG = 24'h003060;   // dark blue backing
    localparam [23:0] COL_FG = 24'hffffff;   // white digit

    reg [9:0] y;
    reg [8:0] bx;

    wire sof = I_tvalid & I_tuser;
    wire eol = I_tvalid & I_tlast;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            y  <= 10'd0;
            bx <= 9'd0;
        end else if(I_tvalid) begin
            if(sof) begin
                // bx is already 0 here; clearing it again would stall the
                // beat counter and shift the whole line.
                y <= 10'd0;
            end else if(eol) begin
                y  <= y + 1'b1;
                bx <= 9'd0;
            end else begin
                bx <= bx + 1'b1;
            end
        end
    end

    // digit = mode + 1, so the screen shows 1 / 2 / 3
    wire [1:0] digit = I_mode + 2'd1;

    // 5x7 bitmap of one row, MSB is the left column.
    function [4:0] glyph_row;
        input [1:0] d;
        input [2:0] row;
        begin
            case(d)
                2'd1: case(row)
                    3'd0: glyph_row = 5'b00100;
                    3'd1: glyph_row = 5'b01100;
                    3'd2: glyph_row = 5'b00100;
                    3'd3: glyph_row = 5'b00100;
                    3'd4: glyph_row = 5'b00100;
                    3'd5: glyph_row = 5'b00100;
                    default: glyph_row = 5'b01110;
                endcase
                2'd2: case(row)
                    3'd0: glyph_row = 5'b01110;
                    3'd1: glyph_row = 5'b10001;
                    3'd2: glyph_row = 5'b00001;
                    3'd3: glyph_row = 5'b00010;
                    3'd4: glyph_row = 5'b00100;
                    3'd5: glyph_row = 5'b01000;
                    default: glyph_row = 5'b11111;
                endcase
                default: case(row)
                    3'd0: glyph_row = 5'b11111;
                    3'd1: glyph_row = 5'b00010;
                    3'd2: glyph_row = 5'b00100;
                    3'd3: glyph_row = 5'b00010;
                    3'd4: glyph_row = 5'b00001;
                    3'd5: glyph_row = 5'b10001;
                    default: glyph_row = 5'b01110;
                endcase
            endcase
        end
    endfunction

    wire [11:0] x_base = {3'b000, bx, 2'b00};   // beat * 4

    wire in_y = (y >= Y0[9:0]) && (y < (Y0 + CELLH));
    wire [2:0] row = (y - Y0[9:0]) >> 3;

    wire [11:0] x0 = x_base + 12'd0;
    wire [11:0] x1 = x_base + 12'd1;
    wire [11:0] x2 = x_base + 12'd2;
    wire [11:0] x3 = x_base + 12'd3;

    wire in_x0 = (x0 >= X0) && (x0 < (X0 + CELLW));
    wire in_x1 = (x1 >= X0) && (x1 < (X0 + CELLW));
    wire in_x2 = (x2 >= X0) && (x2 < (X0 + CELLW));
    wire in_x3 = (x3 >= X0) && (x3 < (X0 + CELLW));

    wire [2:0] col0 = (x0 - X0) >> 3;
    wire [2:0] col1 = (x1 - X0) >> 3;
    wire [2:0] col2 = (x2 - X0) >> 3;
    wire [2:0] col3 = (x3 - X0) >> 3;

    wire [4:0] bits = glyph_row(digit, row);

    wire hit0 = in_y && in_x0;
    wire hit1 = in_y && in_x1;
    wire hit2 = in_y && in_x2;
    wire hit3 = in_y && in_x3;

    wire on0 = hit0 && bits[3'd4 - col0];
    wire on1 = hit1 && bits[3'd4 - col1];
    wire on2 = hit2 && bits[3'd4 - col2];
    wire on3 = hit3 && bits[3'd4 - col3];

    always @(*) begin
        O_tdata  = I_tdata;
        O_tlast  = I_tlast;
        O_tuser  = I_tuser;
        O_tvalid = I_tvalid;

        if(hit0) O_tdata[95:72] = on0 ? COL_FG : COL_BG;
        if(hit1) O_tdata[71:48] = on1 ? COL_FG : COL_BG;
        if(hit2) O_tdata[47:24] = on2 ? COL_FG : COL_BG;
        if(hit3) O_tdata[23:0]  = on3 ? COL_FG : COL_BG;
    end

endmodule
