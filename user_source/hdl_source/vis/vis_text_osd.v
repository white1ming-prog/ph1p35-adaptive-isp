`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_text_osd
//
// Draws the current mode as a line of Chinese text, using the 16x16 glyphs in
// vis_glyph_rom scaled 2x (32x32 per character, 256x32 for the whole line).
//
//   mode 0 -> 模式一：原始图像
//   mode 1 -> 模式二：边缘检测
//   mode 2 -> 模式三：二值化
//
// Drawn on the ISP stream so the vendor hdmi_mixer (logo and counter) is left
// untouched. Purely combinational on the pixel path.
//----------------------------------------------------------------------------
module vis_text_osd (
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

    localparam [11:0] X0     = 12'd380;   // clear of the logo (x<160) and of
    localparam [11:0] Y0     = 12'd20;    // the vendor OSD line (y~120)
    localparam [11:0] LINE_W = 12'd256;   // 8 glyphs * 32 px
    localparam [11:0] LINE_H = 12'd32;

    localparam [23:0] COL_BG = 24'h003060;
    localparam [23:0] COL_FG = 24'hffffff;

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
                // This beat is bx = 0; the next one must be 1, so seed the
                // counter here instead of leaving it to a second 0.
                y  <= 10'd0;
                bx <= 9'd1;
            end else if(eol) begin
                y  <= y + 1'b1;
                bx <= 9'd0;
            end else begin
                bx <= bx + 1'b1;
            end
        end
    end

    // Glyph index per character position. 5'd31 means "blank".
    function [4:0] str_char;
        input [1:0] m;
        input [2:0] pos;
        begin
            case(m)
                2'd1: case(pos)
                    3'd0: str_char = 5'd0;     // 模
                    3'd1: str_char = 5'd1;     // 式
                    3'd2: str_char = 5'd8;     // 二
                    3'd3: str_char = 5'd3;     // ：
                    3'd4: str_char = 5'd9;     // 边
                    3'd5: str_char = 5'd10;    // 缘
                    3'd6: str_char = 5'd11;    // 检
                    default: str_char = 5'd12; // 测
                endcase
                2'd2: case(pos)
                    3'd0: str_char = 5'd0;
                    3'd1: str_char = 5'd1;
                    3'd2: str_char = 5'd13;    // 三
                    3'd3: str_char = 5'd3;
                    3'd4: str_char = 5'd14;    // 值
                    3'd5: str_char = 5'd15;    // 化
                    default: str_char = 5'd31; // blank
                endcase
                default: case(pos)
                    3'd0: str_char = 5'd0;
                    3'd1: str_char = 5'd1;
                    3'd2: str_char = 5'd2;     // 一
                    3'd3: str_char = 5'd3;
                    3'd4: str_char = 5'd4;     // 原
                    3'd5: str_char = 5'd5;     // 始
                    3'd6: str_char = 5'd6;     // 图
                    default: str_char = 5'd7;  // 像
                endcase
            endcase
        end
    endfunction

    wire [11:0] x_base = {3'b000, bx, 2'b00};
    wire        in_y   = (y >= Y0[9:0]) && (y < (Y0 + LINE_H));
    wire [3:0]  row    = (y - Y0[9:0]) >> 1;

    wire [11:0] xj [0:3];
    wire [11:0] dxj[0:3];
    wire        inj[0:3];
    wire [2:0]  posj[0:3];
    wire [3:0]  colj[0:3];
    wire [4:0]  cij[0:3];
    wire [15:0] bitsj[0:3];

    assign xj[0] = x_base;
    assign xj[1] = x_base + 12'd1;
    assign xj[2] = x_base + 12'd2;
    assign xj[3] = x_base + 12'd3;

    genvar j;
    generate
        for (j = 0; j < 4; j = j + 1) begin : g_px
            assign dxj[j]  = xj[j] - X0;
            assign inj[j]  = in_y && (xj[j] >= X0) && (xj[j] < (X0 + LINE_W));
            assign posj[j] = dxj[j] >> 5;          // /32 -> character position
            assign colj[j] = (dxj[j] >> 1) & 4'hF; // /2 -> glyph column
            assign cij[j]  = str_char(I_mode, posj[j]);

            vis_glyph_rom u_rom (
                .I_char_idx (cij[j][3:0]),
                .I_row      (row),
                .O_bits     (bitsj[j])
            );
        end
    endgenerate

    wire on0 = inj[0] && (cij[0] < 5'd16) && bitsj[0][4'd15 - colj[0]];
    wire on1 = inj[1] && (cij[1] < 5'd16) && bitsj[1][4'd15 - colj[1]];
    wire on2 = inj[2] && (cij[2] < 5'd16) && bitsj[2][4'd15 - colj[2]];
    wire on3 = inj[3] && (cij[3] < 5'd16) && bitsj[3][4'd15 - colj[3]];

    always @(*) begin
        O_tdata  = I_tdata;
        O_tlast  = I_tlast;
        O_tuser  = I_tuser;
        O_tvalid = I_tvalid;

        if(inj[0]) O_tdata[95:72] = on0 ? COL_FG : COL_BG;
        if(inj[1]) O_tdata[71:48] = on1 ? COL_FG : COL_BG;
        if(inj[2]) O_tdata[47:24] = on2 ? COL_FG : COL_BG;
        if(inj[3]) O_tdata[23:0]  = on3 ? COL_FG : COL_BG;
    end

endmodule
