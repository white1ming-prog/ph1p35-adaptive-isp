`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_text_osd_disp
//
// Display-domain banner: the current mode name plus the measured
// capture-to-display latency, drawn on top of the picture.
//
//   mode 0 -> 模式一：原始图像
//   mode 1 -> 模式二：边缘检测
//   mode 2 -> 模式三：灰度化
//   mode 3 -> 模式四：运动检测
//   mode 4 -> 模式五：自适应二值化
//
// It lives between hdmi_mixer and hdmi_tx, so the character position is locked
// to the uivtc timing generator instead of the MIPI receive clock. Drawing the
// same text inside the ISP stream jittered by a pixel or two per frame because
// that frame start comes from the MIPI receiver.
//
// 16x16 glyphs scaled 2x -> 32x32 per character. One row of 16 characters is
// 512 px wide, which leaves room for the longest banner (10 glyphs) plus the
// latency readout in the last four slots.
//----------------------------------------------------------------------------
module vis_text_osd_disp (
    input               I_clk,        // HDMI pixel clock
    input               I_rst_n,

    input               I_vsync,
    input               I_hsync,
    input               I_de,
    input      [23:0]   I_data,

    input      [2:0]    I_mode,       // 0..4 = mode 1..5
    input      [7:0]    I_lat_ms,     // capture-to-display latency in ms

    output reg          O_vsync,
    output reg          O_hsync,
    output reg          O_de,
    output reg [23:0]   O_data
);

    localparam [11:0] X0     = 12'd380;
    localparam [11:0] Y0     = 12'd20;
    localparam [11:0] LINE_W = 12'd512;   // 16 glyphs * 32 px
    localparam [11:0] LINE_H = 12'd32;

    localparam [23:0] COL_BG = 24'h003060;
    localparam [23:0] COL_FG = 24'hffffff;

    reg [11:0] x, y;
    reg        de_d, vs_d;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x <= 12'd0; y <= 12'd0; de_d <= 1'b0; vs_d <= 1'b0;
        end else begin
            de_d <= I_de;
            vs_d <= I_vsync;

            // Any vsync transition marks the frame boundary.
            if(I_vsync != vs_d)
                y <= 12'd0;
            else if(!I_de && de_d)
                y <= y + 1'b1;

            if(I_de)
                x <= x + 1'b1;
            else
                x <= 12'd0;
        end
    end

    // Glyph index per character slot. 6'd63 is a blank.
    function [5:0] str_char;
        input [2:0] m;
        input [3:0] pos;
        begin
            case(m)
                // 模式二：边缘检测
                3'd1: case(pos)
                    4'd0: str_char = 6'd0;     // 模
                    4'd1: str_char = 6'd1;     // 式
                    4'd2: str_char = 6'd8;     // 二
                    4'd3: str_char = 6'd3;     // ：
                    4'd4: str_char = 6'd9;     // 边
                    4'd5: str_char = 6'd10;    // 缘
                    4'd6: str_char = 6'd11;    // 检
                    4'd7: str_char = 6'd12;    // 测
                    default: str_char = 6'd63; // blank
                endcase
                // 模式三：灰度化
                3'd2: case(pos)
                    4'd0: str_char = 6'd0;     // 模
                    4'd1: str_char = 6'd1;     // 式
                    4'd2: str_char = 6'd13;    // 三
                    4'd3: str_char = 6'd3;     // ：
                    4'd4: str_char = 6'd40;    // 灰
                    4'd5: str_char = 6'd41;    // 度
                    4'd6: str_char = 6'd15;    // 化
                    default: str_char = 6'd63;
                endcase
                // 模式四：运动检测
                3'd3: case(pos)
                    4'd0: str_char = 6'd0;     // 模
                    4'd1: str_char = 6'd1;     // 式
                    4'd2: str_char = 6'd16;    // 四
                    4'd3: str_char = 6'd3;     // ：
                    4'd4: str_char = 6'd17;    // 运
                    4'd5: str_char = 6'd18;    // 动
                    4'd6: str_char = 6'd11;    // 检
                    4'd7: str_char = 6'd12;    // 测
                    default: str_char = 6'd63; // blank
                endcase
                // 模式五：自适应图像
                3'd4: case(pos)
                    4'd0: str_char = 6'd0;     // 模
                    4'd1: str_char = 6'd1;     // 式
                    4'd2: str_char = 6'd19;    // 五
                    4'd3: str_char = 6'd3;     // ：
                    4'd4: str_char = 6'd24;    // 自
                    4'd5: str_char = 6'd25;    // 适
                    4'd6: str_char = 6'd26;    // 应
                    4'd7: str_char = 6'd6;     // 图
                    4'd8: str_char = 6'd7;     // 像
                    default: str_char = 6'd63; // blank
                endcase
                // 模式一：原始图像
                default: case(pos)
                    4'd0: str_char = 6'd0;     // 模
                    4'd1: str_char = 6'd1;     // 式
                    4'd2: str_char = 6'd2;     // 一
                    4'd3: str_char = 6'd3;     // ：
                    4'd4: str_char = 6'd4;     // 原
                    4'd5: str_char = 6'd5;     // 始
                    4'd6: str_char = 6'd6;     // 图
                    4'd7: str_char = 6'd7;     // 像
                    default: str_char = 6'd63; // blank
                endcase
            endcase
        end
    endfunction

    wire        in_box = (x >= X0) && (x < (X0 + LINE_W)) &&
                         (y >= Y0) && (y < (Y0 + LINE_H));
    wire [3:0]  pos    = (x - X0) >> 5;          // /32 -> character slot
    wire [3:0]  col    = ((x - X0) >> 1) & 4'hF; // /2  -> glyph column
    wire [3:0]  row    = (y - Y0) >> 1;

    // Latency readout in the last four slots: two digits, then "ms".
    // Digit glyphs start at index 27, 'm' is 38 and 's' is 39.
    reg [5:0] d_tens, d_units;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            d_tens  <= 6'd27;
            d_units <= 6'd27;
        end else if(I_vsync != vs_d) begin
            d_tens  <= 6'd27 + (I_lat_ms / 8'd10);
            d_units <= 6'd27 + (I_lat_ms % 8'd10);
        end
    end

    wire [5:0] mode_char = str_char(I_mode, pos);
    // Slots 0..7 hold the mode name, 8..11 are blank, 12..15 hold "XXms".
    // Getting this boundary wrong makes the mode table's default branch repeat
    // its last glyph across the gap - which is exactly what happened when this
    // comparison read (pos < 4'd12).
    wire [5:0] cidx = (pos < 4'd12) ? mode_char :
                      (pos == 4'd12) ? d_tens    :
                      (pos == 4'd13) ? d_units   :
                      (pos == 4'd14) ? 6'd38 :   // 'm'
                      (pos == 4'd15) ? 6'd39 :   // 's'
                                       6'd63;    // blank

    wire [15:0] gbits;

    vis_glyph_rom u_rom (
        .I_char_idx (cidx),
        .I_row      (row),
        .O_bits     (gbits)
    );

    wire on = in_box && (cidx < 6'd42) && gbits[4'd15 - col];

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_vsync <= 1'b0;
            O_hsync <= 1'b0;
            O_de    <= 1'b0;
            O_data  <= 24'd0;
        end else begin
            O_vsync <= I_vsync;
            O_hsync <= I_hsync;
            O_de    <= I_de;
            O_data  <= in_box ? (on ? COL_FG : COL_BG) : I_data;
        end
    end

endmodule
