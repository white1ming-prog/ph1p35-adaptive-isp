`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_edge_96 -- Scharr 边缘检测 (全分辨率 + NMS + 双阈值滞后)
//
// 96bit 输入 = 4 x RGB888, 也就是每拍 4 个像素。本模块对这 4 个像素各算一个
// 梯度, 一行 1280 个像素就得到 1280 个梯度采样: 水平方向不再有采样间隔,
// 竖线和斜线不会再一段一段地丢, 线宽也只有 1 个像素(旧版每 2 个像素共用一个
// 结果再复制, 线条是 2 像素宽且发虚)。
//
// 算子: Scharr 3 10 3 / 0 0 0 / -3 -10 -3(和参考设计的 Sobel 1 2 1 不同),
// 幅值取 L1 范数 |gx| + |gy|, 上限 4080 + 4080 = 8160, 14bit 放得下, 不会像
// 旧版 gx*gx + gy*gy 那样在 15bit 里溢出截断(那正是屏幕上的"噪点"来源)。
//
// 先做因果 3x3 非极大值抑制: 当前幅值必须大于上排三个邻居，并大于
// 同排左右邻居。相等响应优先保留左/上的一个像素，避免粗边和响应平台。
// 随后做双阈值滞后 (Canny 思路):
//   mag >= I_edge_th        -> 强边, 直接保留
//   mag >= I_edge_th/2      -> 弱边, 只有当它旁边(左/右/上一行)已经有边时才保留
//   孤立的一个弱响应 = 噪点, 被丢掉; 真实的弱边缘只要和强边相连就接上。
// NMS 把多像素响应压成单像素线，双阈值再连接真实弱边。
//
// 阈值 I_edge_th 的物理含义: Scharr 权重和为 16, 所以一个 h 级灰阶的阶跃
// 产生 mag = 16*h。I_edge_th = 260 相当于约 16 级灰阶的跳变(强边),
// 弱边门限自动取一半, 约 8 级。
//
// 四级流水: 灰度+行缓存读出 -> 加权和 -> 梯度+幅值 -> NMS+滞后判定+输出。
// 拆成四级而不是三级, 是因为 1280x720@60 的像素时钟只有 9.66ns, 加权和与
// 梯度放在同一级里时序收不住。tuser/tlast/tvalid 同步延 4 拍, 位置计数也
// 同步延拍, 保证屏蔽条件和数据对齐。
//
// 输出比输入晚一行(窗口中心是上一行), 画面整体下移一行, 帧结构不变。
//----------------------------------------------------------------------------
module vis_edge_96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720
)(
    input               I_clk,
    input               I_rst_n,
    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    input               I_tready,
    input      [14:0]   I_edge_th,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [95:0]   O_tdata,
    output reg          O_tvalid,
    output              O_tready
);

    // 本模块不产生反压: 每个时钟都能吃进一拍, 下游忙不忙由上游负责。
    assign O_tready = I_tready;

    localparam GRP_W = IMG_WIDTH >> 2;          // 每行的拍数 = 1280/4 = 320
    localparam [10:0] GRP_LAST = GRP_W - 1;     // 最后一组的组号
    localparam [10:0] Y_LAST   = IMG_HEIGHT - 1;

//--------------------------------------------------------------------------
// 输入拆分与灰度: Y = R/4 + G/2 + B/4, 只有移位和加法
//--------------------------------------------------------------------------
    wire [23:0] pix0 = I_tdata[95:72];
    wire [23:0] pix1 = I_tdata[71:48];
    wire [23:0] pix2 = I_tdata[47:24];
    wire [23:0] pix3 = I_tdata[23:0];

    wire [7:0] gray0 = {2'b0,pix0[23:18]} + {1'b0,pix0[15:9]} + {2'b0,pix0[7:2]};
    wire [7:0] gray1 = {2'b0,pix1[23:18]} + {1'b0,pix1[15:9]} + {2'b0,pix1[7:2]};
    wire [7:0] gray2 = {2'b0,pix2[23:18]} + {1'b0,pix2[15:9]} + {2'b0,pix2[7:2]};
    wire [7:0] gray3 = {2'b0,pix3[23:18]} + {1'b0,pix3[15:9]} + {2'b0,pix3[7:2]};

//--------------------------------------------------------------------------
// 行缓存: 每组 4 个灰度, 共 32bit
//   line1_mem = 上一行 (y-1), line2_mem = 上两行 (y-2), 当前行 (y) 直接算。
//   窗口 = [y-2, y-1, y], 中心落在 y-1。
//--------------------------------------------------------------------------
    (* ram_style = "block" *) reg [31:0] line1_mem [0:GRP_W-1];
    (* ram_style = "block" *) reg [31:0] line2_mem [0:GRP_W-1];

    // 上一行的 4 个幅值 + 4 个最终边标志。幅值给 NMS，标志给滞后连接。
    (* ram_style = "block" *) reg [63:0] metric_mem [0:GRP_W-1];

    reg [10:0] x_grp;
    reg [10:0] y_pos;

    wire [31:0] top_word = line2_mem[x_grp];   // y-2
    wire [31:0] mid_word = line1_mem[x_grp];   // y-1
    // 读地址用延过一拍的组号 (x1, 见第一级): 三级移位之后, flag_d1/d2/d3
    // 正好落在右一组 / 本组 / 左一组, 也就是右上 / 正上 / 左上三个邻居。
    wire [3:0]  flag_q;
    wire [59:0] mag_q;

//--------------------------------------------------------------------------
// 第一级: 灰度、行缓存读出、流控各打一拍, 三者严格对齐
//   同时记住上一组最后那一列(4x-1), 窗口最左列的 3 个像素需要它。
//--------------------------------------------------------------------------
    reg [31:0] top_r, mid_r, cur_r;
    reg  [7:0] tprev_r, mprev_r, cprev_r;
    reg        v1, u1, l1;
    reg [10:0] x1, y1;

    assign {flag_q, mag_q} = metric_mem[x1];

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            top_r <= 32'd0; mid_r <= 32'd0; cur_r <= 32'd0;
            tprev_r <= 8'd0; mprev_r <= 8'd0; cprev_r <= 8'd0;
            v1 <= 1'b0; u1 <= 1'b0; l1 <= 1'b0;
            x1 <= 11'd0; y1 <= 11'd0;
        end else begin
            top_r <= top_word;
            mid_r <= mid_word;
            cur_r <= {gray3, gray2, gray1, gray0};
            // 本组的 pixel3 就是下一组的 "左一列" (列 4x+3 = 4(x+1)-1)
            tprev_r <= top_r[31:24];
            mprev_r <= mid_r[31:24];
            cprev_r <= cur_r[31:24];
            v1 <= I_tvalid; u1 <= I_tuser; l1 <= I_tlast;
            x1 <= x_grp; y1 <= y_pos;
        end
    end

//--------------------------------------------------------------------------
// 第二级 A: 加权和
//   处理第 x 组时, 读地址已经是 x+1, 所以行缓存读出的 [7:0] 正好是列 4x+4,
//   也就是本组第 3 号像素右边的那一列; 当前行那一列则是本拍输入的 gray0。
//   于是每行都能拿到 6 列: [4x-1, 4x, 4x+1, 4x+2, 4x+3, 4x+4], 足够算出
//   4 个中心(4x .. 4x+3)的窗口。
//
//   HT_j = 3*T(j-1) + 10*T(j) + 3*T(j+1)   (上两行, 横向加权)
//   HC_j = 3*C(j-1) + 10*C(j) + 3*C(j+1)   (当前行, 横向加权)
//   V_k  = 3*T(k) + 10*M(k) + 3*C(k)       (跨三行, 竖向加权)
//   3x = (x<<1)+x, 10x = (x<<3)+(x<<1), 依然没有乘法器。
//--------------------------------------------------------------------------
    wire [7:0] t_nxt = top_word[7:0];
    wire [7:0] m_nxt = mid_word[7:0];
    wire [7:0] c_nxt = gray0;

    function [11:0] w3;
        input [7:0] a;
        begin
            w3 = ({4'h0, a} << 1) + {4'h0, a};
        end
    endfunction

    function [11:0] w10;
        input [7:0] a;
        begin
            w10 = ({4'h0, a} << 3) + ({4'h0, a} << 1);
        end
    endfunction

    // 3*a + 10*b + 3*c, 最大 4080, 12bit 放得下
    function [11:0] wsum;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        begin
            wsum = w3(a) + w10(b) + w3(c);
        end
    endfunction

    // 上两行 (T) 与当前行 (C) 的 4 个横向加权和
    wire [11:0] ht0 = wsum(tprev_r,      top_r[7:0],  top_r[15:8]);
    wire [11:0] ht1 = wsum(top_r[7:0],   top_r[15:8], top_r[23:16]);
    wire [11:0] ht2 = wsum(top_r[15:8],  top_r[23:16],top_r[31:24]);
    wire [11:0] ht3 = wsum(top_r[23:16], top_r[31:24],t_nxt);

    wire [11:0] hc0 = wsum(cprev_r,      cur_r[7:0],  cur_r[15:8]);
    wire [11:0] hc1 = wsum(cur_r[7:0],   cur_r[15:8], cur_r[23:16]);
    wire [11:0] hc2 = wsum(cur_r[15:8],  cur_r[23:16],cur_r[31:24]);
    wire [11:0] hc3 = wsum(cur_r[23:16], cur_r[31:24],c_nxt);

    // 跨三行的竖向加权和, 6 列各一个
    wire [11:0] v_m1 = wsum(tprev_r,      mprev_r,      cprev_r);
    wire [11:0] v_c0 = wsum(top_r[7:0],   mid_r[7:0],   cur_r[7:0]);
    wire [11:0] v_c1 = wsum(top_r[15:8],  mid_r[15:8],  cur_r[15:8]);
    wire [11:0] v_c2 = wsum(top_r[23:16], mid_r[23:16], cur_r[23:16]);
    wire [11:0] v_c3 = wsum(top_r[31:24], mid_r[31:24], cur_r[31:24]);
    wire [11:0] v_p4 = wsum(t_nxt,        m_nxt,        c_nxt);

    reg [11:0] ht0_r, ht1_r, ht2_r, ht3_r;
    reg [11:0] hc0_r, hc1_r, hc2_r, hc3_r;
    reg [11:0] vm1_r, v0_r, v1_r, v2_r, v3_r, vp4_r;
    reg        v2a, u2a, l2a;
    reg [10:0] x2, y2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            ht0_r <= 12'd0; ht1_r <= 12'd0; ht2_r <= 12'd0; ht3_r <= 12'd0;
            hc0_r <= 12'd0; hc1_r <= 12'd0; hc2_r <= 12'd0; hc3_r <= 12'd0;
            vm1_r <= 12'd0; v0_r <= 12'd0; v1_r <= 12'd0;
            v2_r <= 12'd0;  v3_r <= 12'd0; vp4_r <= 12'd0;
            v2a <= 1'b0; u2a <= 1'b0; l2a <= 1'b0;
            x2 <= 11'd0; y2 <= 11'd0;
        end else begin
            ht0_r <= ht0; ht1_r <= ht1; ht2_r <= ht2; ht3_r <= ht3;
            hc0_r <= hc0; hc1_r <= hc1; hc2_r <= hc2; hc3_r <= hc3;
            vm1_r <= v_m1; v0_r <= v_c0; v1_r <= v_c1;
            v2_r <= v_c2;  v3_r <= v_c3; vp4_r <= v_p4;
            v2a <= v1; u2a <= u1; l2a <= l1;
            x2 <= x1; y2 <= y1;
        end
    end

//--------------------------------------------------------------------------
// 第二级 B: 梯度与幅值
//   gx_j = HC_j - HT_j          (检测横线)
//   gy_j = V(j+1) - V(j-1)      (检测竖线)
//   mag_j = |gx_j| + |gy_j|
//--------------------------------------------------------------------------
    wire signed [12:0] gx0 = $signed({1'b0, hc0_r}) - $signed({1'b0, ht0_r});
    wire signed [12:0] gx1 = $signed({1'b0, hc1_r}) - $signed({1'b0, ht1_r});
    wire signed [12:0] gx2 = $signed({1'b0, hc2_r}) - $signed({1'b0, ht2_r});
    wire signed [12:0] gx3 = $signed({1'b0, hc3_r}) - $signed({1'b0, ht3_r});

    wire signed [12:0] gy0 = $signed({1'b0, v1_r})  - $signed({1'b0, vm1_r});
    wire signed [12:0] gy1 = $signed({1'b0, v2_r})  - $signed({1'b0, v0_r});
    wire signed [12:0] gy2 = $signed({1'b0, v3_r})  - $signed({1'b0, v1_r});
    wire signed [12:0] gy3 = $signed({1'b0, vp4_r}) - $signed({1'b0, v2_r});

    function [12:0] abs13;
        input signed [12:0] x;
        begin
            abs13 = x[12] ? (-x) : x;
        end
    endfunction

    // 上限 8160, 14bit 放得下; 留成 15bit 是为了和 I_edge_th 同位宽比较
    wire [14:0] mag0 = {2'b00, abs13(gx0)} + {2'b00, abs13(gy0)};
    wire [14:0] mag1 = {2'b00, abs13(gx1)} + {2'b00, abs13(gy1)};
    wire [14:0] mag2 = {2'b00, abs13(gx2)} + {2'b00, abs13(gy2)};
    wire [14:0] mag3 = {2'b00, abs13(gx3)} + {2'b00, abs13(gy3)};

    // 量化梯度方向供 NMS 使用:
    //   0 = 竖边/水平梯度, 比较左右
    //   1 = 横边/垂直梯度, 比较上方
    //   2 = 对角 \, 比较左上
    //   3 = 对角 /, 比较右上
    function [1:0] nms_dir;
        input signed [12:0] gx;
        input signed [12:0] gy;
        reg [12:0] ax, ay;
        begin
            ax = abs13(gx);
            ay = abs13(gy);
            if((ay >> 1) > ax)
                nms_dir = 2'd0;
            else if((ax >> 1) > ay)
                nms_dir = 2'd1;
            else if(gx[12] ^ gy[12])
                nms_dir = 2'd3;
            else
                nms_dir = 2'd2;
        end
    endfunction

    wire [1:0] dir0 = nms_dir(gx0, gy0);
    wire [1:0] dir1 = nms_dir(gx1, gy1);
    wire [1:0] dir2 = nms_dir(gx2, gy2);
    wire [1:0] dir3 = nms_dir(gx3, gy3);

    reg [14:0] mag0_r, mag1_r, mag2_r, mag3_r;
    reg [1:0]  dir0_r, dir1_r, dir2_r, dir3_r;
    reg        v3, u3, l3;
    reg [10:0] x3, y3;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            mag0_r <= 15'd0; mag1_r <= 15'd0; mag2_r <= 15'd0; mag3_r <= 15'd0;
            dir0_r <= 2'd0; dir1_r <= 2'd0; dir2_r <= 2'd0; dir3_r <= 2'd0;
            v3 <= 1'b0; u3 <= 1'b0; l3 <= 1'b0;
            x3 <= 11'd0; y3 <= 11'd0;
        end else begin
            mag0_r <= mag0; mag1_r <= mag1; mag2_r <= mag2; mag3_r <= mag3;
            dir0_r <= dir0; dir1_r <= dir1; dir2_r <= dir2; dir3_r <= dir3;
            v3 <= v2a; u3 <= u2a; l3 <= l2a;
            x3 <= x2; y3 <= y2;
        end
    end

//--------------------------------------------------------------------------
// 第三级: NMS + 双阈值滞后判定 + 输出 + 行缓存/状态缓存写入
//
//   NMS: 根据 gx/gy 主方向选择左右、上下或斜向邻居，只和梯度
//        方向上的邻居比较，避免把沿边缘方向的连续响应误删。
//   强边: NMS 后 mag >= I_edge_th
//   弱边: NMS 后 mag >= I_edge_th/2, 且旁边已经有边才保留
//
//   邻居取法:
//     左右: 本拍 4 像素 + 上一组 pixel3 + 下一组 pixel0
//     上: metric_mem 的三级移位，覆盖左上 / 正上 / 右上
//   行号 < 3 时上一行的标志还没写进去(BRAM 不能复位), 所以只用左右邻居。
//--------------------------------------------------------------------------
    wire [14:0] th_hi = I_edge_th;
    wire [14:0] th_lo = {1'b0, I_edge_th[14:1]};   // 弱边门限 = 强边的一半

    // 上一行幅值和边标志的三级移位。分组间隔正好覆盖每个中心像素的
    // 左上 / 正上 / 右上三个邻居。
    reg [3:0] flag_d1, flag_d2, flag_d3;
    reg [59:0] mag_d1, mag_d2, mag_d3;
    reg       pflag3_r;             // 上一组 pixel3 的最终结果
    reg [14:0] pmag3_r;             // 上一组 pixel3 的梯度幅值

    // 窗口有效性: 需要完整的三行, 且最左列/最右列有真实邻居
    wire row_ok = (y3 >= 11'd2) && !u3;
    wire ok0 = row_ok && (x3 != 11'd0);
    wire ok1 = row_ok;
    wire ok2 = row_ok;
    wire ok3 = row_ok && (x3 != GRP_LAST);
    wire up_en = (y3 >= 11'd3);

    // 右边一列(列 4x+4): 本拍正在算下一组，mag0 就是它。
    wire st_nxt0 = (mag0 >= th_hi);

    // 水平方向 NMS：左邻使用 >=，右邻使用 >，相同响应平台保留最左像素。
    wire peak_h0 = (mag0_r >= pmag3_r) && (mag0_r > mag1_r);
    wire peak_h1 = (mag1_r >= mag0_r) && (mag1_r > mag2_r);
    wire peak_h2 = (mag2_r >= mag1_r) && (mag2_r > mag3_r);
    wire peak_h3 = (mag3_r >= mag2_r) && (mag3_r > mag0);

    // 纵向和对角方向只和上排对应邻居比较。严格大于使相等平台保留
    // 更靠上的那一行，又不会像“同时要求大于上排三个邻居”那样切断
    // 一条连续竖线。
    wire peak_u0 = !up_en || (mag0_r > mag_d2[14:0]);
    wire peak_u1 = !up_en || (mag1_r > mag_d2[29:15]);
    wire peak_u2 = !up_en || (mag2_r > mag_d2[44:30]);
    wire peak_u3 = !up_en || (mag3_r > mag_d2[59:45]);

    wire peak_ul0 = !up_en || (mag0_r > mag_d3[59:45]);
    wire peak_ul1 = !up_en || (mag1_r > mag_d2[14:0]);
    wire peak_ul2 = !up_en || (mag2_r > mag_d2[29:15]);
    wire peak_ul3 = !up_en || (mag3_r > mag_d2[44:30]);

    wire peak_ur0 = !up_en || (mag0_r > mag_d2[29:15]);
    wire peak_ur1 = !up_en || (mag1_r > mag_d2[44:30]);
    wire peak_ur2 = !up_en || (mag2_r > mag_d2[59:45]);
    wire peak_ur3 = !up_en || (mag3_r > mag_d1[14:0]);

    wire peak0 = (dir0_r == 2'd0) ? peak_h0 :
                 (dir0_r == 2'd1) ? peak_u0 :
                 (dir0_r == 2'd2) ? peak_ul0 : peak_ur0;
    wire peak1 = (dir1_r == 2'd0) ? peak_h1 :
                 (dir1_r == 2'd1) ? peak_u1 :
                 (dir1_r == 2'd2) ? peak_ul1 : peak_ur1;
    wire peak2 = (dir2_r == 2'd0) ? peak_h2 :
                 (dir2_r == 2'd1) ? peak_u2 :
                 (dir2_r == 2'd2) ? peak_ul2 : peak_ur2;
    wire peak3 = (dir3_r == 2'd0) ? peak_h3 :
                 (dir3_r == 2'd1) ? peak_u3 :
                 (dir3_r == 2'd2) ? peak_ul3 : peak_ur3;

    wire st0 = peak0 && (mag0_r >= th_hi);
    wire wk0 = peak0 && (mag0_r >= th_lo);
    wire st1 = peak1 && (mag1_r >= th_hi);
    wire wk1 = peak1 && (mag1_r >= th_lo);
    wire st2 = peak2 && (mag2_r >= th_hi);
    wire wk2 = peak2 && (mag2_r >= th_lo);
    wire st3 = peak3 && (mag3_r >= th_hi);
    wire wk3 = peak3 && (mag3_r >= th_lo);

    wire sup0 = (up_en & (flag_d3[3] | flag_d2[0] | flag_d2[1])) | pflag3_r | st1;
    wire sup1 = (up_en & (flag_d2[0] | flag_d2[1] | flag_d2[2])) | st2;
    wire sup2 = (up_en & (flag_d2[1] | flag_d2[2] | flag_d2[3])) | st3;
    wire sup3 = (up_en & (flag_d2[2] | flag_d2[3] | flag_d1[0])) | st_nxt0;

    wire e0 = ok0 & (st0 | (wk0 & sup0));
    wire e1 = ok1 & (st1 | (wk1 & (sup1 | e0)));
    wire e2 = ok2 & (st2 | (wk2 & (sup2 | e1)));
    wire e3 = ok3 & (st3 | (wk3 & (sup3 | e2)));

    wire [23:0] out0 = e0 ? 24'hffffff : 24'h000000;
    wire [23:0] out1 = e1 ? 24'hffffff : 24'h000000;
    wire [23:0] out2 = e2 ? 24'hffffff : 24'h000000;
    wire [23:0] out3 = e3 ? 24'hffffff : 24'h000000;

    reg v4, u4, l4;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tvalid <= 1'b0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            v4 <= 1'b0; u4 <= 1'b0; l4 <= 1'b0;
            flag_d1 <= 4'd0; flag_d2 <= 4'd0; flag_d3 <= 4'd0;
            mag_d1 <= 60'd0; mag_d2 <= 60'd0; mag_d3 <= 60'd0;
            pflag3_r <= 1'b0;
            pmag3_r <= 15'd0;
        end else begin
            flag_d1 <= flag_q;
            flag_d2 <= flag_d1;
            flag_d3 <= flag_d2;
            mag_d1 <= mag_q;
            mag_d2 <= mag_d1;
            mag_d3 <= mag_d2;
            pflag3_r <= e3;
            pmag3_r <= mag3_r;

            v4 <= v3; u4 <= u3; l4 <= l3;
            O_tvalid <= v3;
            O_tlast  <= l3;
            O_tuser  <= u3;
            if(v3)
                O_tdata <= {out0, out1, out2, out3};
            else
                O_tdata <= 96'd0;

            // 只在有效拍写入: 消隐期间 x3 是上一行的旧地址, 不写就不会
            // 把已经算好的幅值和标志覆盖掉。
            if(v3)
                metric_mem[x3] <= {e3, e2, e1, e0,
                                   mag3_r, mag2_r, mag1_r, mag0_r};
        end
    end

//--------------------------------------------------------------------------
// 行缓存与地址计数 (地址一律用当前拍的旧值, 读出来的是上一帧同一组)
//--------------------------------------------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x_grp <= 11'd0;
            y_pos <= 11'd0;
        end else if(I_tvalid) begin
            line2_mem[x_grp] <= mid_word;                       // y-1 -> y-2
            line1_mem[x_grp] <= {gray3, gray2, gray1, gray0};   // y   -> y-1

            if(I_tuser) begin
                x_grp <= 11'd1;
                y_pos <= 11'd0;
            end else if(x_grp == GRP_LAST) begin
                x_grp <= 11'd0;
                if(y_pos == Y_LAST)
                    y_pos <= 11'd0;
                else
                    y_pos <= y_pos + 1'b1;
            end else begin
                x_grp <= x_grp + 1'b1;
            end
        end
    end

endmodule
