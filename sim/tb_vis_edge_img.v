`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// tb_vis_edge_img
//
// 给 vis_edge_96 灌一整帧合成图(带噪点、带弱边缘), 把输入灰度和输出边图
// 各存成一张 PGM, 用来直接看图检查:
//   * 竖线/横线/斜线/圆环 是不是都在, 有没有断
//   * 线宽是不是 1 像素(清晰), 不是 2 像素糊在一起
//   * 平坦区加了噪点之后有没有冒出孤立小白点
//   * 只有 14 级跳变的弱边缘有没有被保住(双阈值滞后的作用)
//----------------------------------------------------------------------------
module tb_vis_edge_img;

    localparam IMG_WIDTH  = 640;
    localparam IMG_HEIGHT = 360;
    localparam GRP_W      = IMG_WIDTH / 4;
    localparam TH         = 15'd300;

    reg        clk = 1'b0;
    reg        rst_n = 1'b0;
    reg [95:0] i_tdata = 96'd0;
    reg        i_tvalid = 1'b0;
    reg        i_tlast = 1'b0;
    reg        i_tuser = 1'b0;

    wire [95:0] o_tdata;
    wire        o_tvalid, o_tlast, o_tuser, o_tready;

    always #5 clk = ~clk;

    vis_edge_96 #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT)
    ) dut (
        .I_clk    (clk),
        .I_rst_n  (rst_n),
        .I_tlast  (i_tlast),
        .I_tuser  (i_tuser),
        .I_tdata  (i_tdata),
        .I_tvalid (i_tvalid),
        .I_tready (1'b1),
        .I_edge_th(TH),
        .O_tlast  (o_tlast),
        .O_tuser  (o_tuser),
        .O_tdata  (o_tdata),
        .O_tvalid (o_tvalid),
        .O_tready (o_tready)
    );

    // ---- 噪声: 16bit LFSR, 取低 3 位当 -3..+4 的抖动 ----
    reg [15:0] lfsr = 16'hACE1;
    always @(posedge clk)
        lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

    // ---- 合成图 ----
    function [7:0] pat;
        input [10:0] x;
        input [10:0] y;
        reg   [7:0] v;
        reg  [10:0] dx, dy;
        reg  [21:0] d2;
        begin
            v = 8'd40;                                        // 背景
            if ((x % 40) < 3)            v = 8'd200;          // 竖条
            if ((y % 30) < 2)            v = 8'd180;          // 横条
            if ((x > y*2 - 2) && (x < y*2 + 2)) v = 8'd150;   // 斜线
            dx = (x > 160) ? (x - 160) : (160 - x);
            dy = (y > 90)  ? (y -  90) : ( 90 - y);
            d2 = dx*dx + dy*dy;
            if ((d2 > 58*58) && (d2 < 62*62)) v = 8'd160;     // 圆环
            // 弱边缘: 只有 14 级跳变, mag = 16*14 = 224 < 强边门限 300,
            // 只能靠滞后判定接上来。
            if ((x > 200) && (x < 280) && (y > 20) && (y < 60)) v = 8'd54;
            pat = v;
        end
    endfunction

    integer fi, fo;
    integer i, g, y, p, f;
    reg [10:0] px, py;
    reg [8:0]  pv;
    reg [7:0]  pv8;

    initial begin
        fi = $fopen("in_gray.pgm", "wb");
        fo = $fopen("out_edge.pgm", "wb");
        $fwrite(fi, "P5\n%0d %0d\n255\n", IMG_WIDTH, IMG_HEIGHT);
        $fwrite(fo, "P5\n%0d %0d\n255\n", IMG_WIDTH, IMG_HEIGHT);

        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        // 两帧: 第一帧喂进去(行缓存要两行才有完整窗口), 第二帧才存图
        for (f = 0; f < 2; f = f + 1) begin
            for (i = 0; i < IMG_HEIGHT * GRP_W; i = i + 1) begin
                y  = i / GRP_W;
                g  = i % GRP_W;
                i_tvalid = 1'b1;
                i_tuser  = (i == 0);
                i_tlast  = (g == GRP_W - 1);
                for (p = 0; p < 4; p = p + 1) begin
                    px = g * 4 + p;
                    py = y;
                    pv = pat(px, py) + lfsr[2:0] - 3;   // 加 -3..+4 的噪点
                    if (pv < 0)   pv = 0;
                    if (pv > 255) pv = 255;
                    pv8 = pv[7:0];
                    i_tdata[24*(3-p) +: 24] = {pv8, pv8, pv8};
                    if (f == 1) $fwrite(fi, "%c", pv8);
                end
                @(negedge clk);
            end
            i_tvalid = 1'b0; i_tlast = 1'b0; i_tuser = 1'b0; i_tdata = 96'd0;
            repeat (8) @(negedge clk);
        end

        i_tvalid = 1'b0; i_tlast = 1'b0; i_tuser = 1'b0;
        repeat (20) @(negedge clk);
        $fclose(fi);
        $fclose(fo);
        $display("TB done: frames=%0d", f);
        $finish;
    end

    // ---- 输出采集: 第二帧开始逐个像素写盘 ----
    integer frame = 0;
    integer cnt   = 0;
    reg [7:0] ob;

    always @(posedge clk) begin
        if (o_tvalid) begin
            if (o_tuser) begin
                frame = frame + 1;
                cnt   = 0;
            end
            if (frame == 2 && cnt < IMG_WIDTH * IMG_HEIGHT) begin
                ob = o_tdata[95:88];  $fwrite(fo, "%c", ob); cnt = cnt + 1;
                ob = o_tdata[71:64];  $fwrite(fo, "%c", ob); cnt = cnt + 1;
                ob = o_tdata[47:40];  $fwrite(fo, "%c", ob); cnt = cnt + 1;
                ob = o_tdata[23:16];  $fwrite(fo, "%c", ob); cnt = cnt + 1;
            end
        end
    end

endmodule
