`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// isp_top
//
// Original vendor ISP chain (demosaic -> AWB) with the self-adaptive
// pre-processing block inserted between AWB and the 96->128 bit packer.
// The vis_top block is a video pass-through; it only observes the stream and
// overlays a threshold marker, so the pixel path is unchanged.
//----------------------------------------------------------------------------
module isp_top (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    input [2:0] 	I_mode			,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready		,
    output 			O_motion_alarm	,
    output 			O_motion_any	,
    output [15:0] 	O_box_x0		,
    output [15:0] 	O_box_y0		,
    output [15:0] 	O_box_x1		,
    output [15:0] 	O_box_y1		,
    output 			O_low_light
);
 
wire         m_aixs_tvalid;
wire [127:0] m_aixs_tdata;
wire         m_aixs_tuser;
wire         m_aixs_tlast;
wire         m_aixs_tready;

wire [ 95:0] m_aixs_tdata_96;

wire         awb_O_tlast;
wire         awb_O_tuser;
wire [ 95:0] awb_O_tdata;
wire         awb_O_tvalid;
wire         awb_O_tready;

wire         awb_O_tuser_128;
wire         awb_O_tvalid_128;
wire [127:0] awb_O_tdata_128;

// vis_top outputs (passthrough or edge-detected stream).
wire         vis_O_tlast;
wire         vis_O_tuser;
wire [ 95:0] vis_O_tdata;
wire         vis_O_tvalid;
wire         vis_O_tready;

assign O_tdata  = awb_O_tdata_128;
assign O_tlast  = awb_O_tlast    ;
assign O_tuser  = awb_O_tuser_128;
assign O_tvalid = awb_O_tvalid_128;

// AWB now hands its stream to vis_top instead of straight to the packer.
assign awb_O_tready = vis_O_tready;


// ---------------------------------------------------------------------------
// Four-channel black level correction on the RAW10 stream, ahead of the
// demosaic. The vendor ships isp/BLC/BLC.v but never instantiates it, so
// without this stage the dark areas keep their fixed offset and the picture
// looks hazy. Each Bayer phase gets its own offset (R, Gr, Gb, B) and the
// result is clipped at zero so the subtraction can never wrap around.
// ---------------------------------------------------------------------------
localparam [9:0] BLC_R  = 10'd10;
localparam [9:0] BLC_G1 = 10'd12;
localparam [9:0] BLC_G2 = 10'd12;
localparam [9:0] BLC_B  = 10'd10;

localparam [9:0] NIGHT_BLC_R  = 10'd4;
localparam [9:0] NIGHT_BLC_G1 = 10'd6;
localparam [9:0] NIGHT_BLC_G2 = 10'd6;
localparam [9:0] NIGHT_BLC_B  = 10'd4;

wire        night_blc = (I_mode == 3'd4);
wire [9:0]  blc_r_off  = night_blc ? NIGHT_BLC_R  : BLC_R;
wire [9:0]  blc_g1_off = night_blc ? NIGHT_BLC_G1 : BLC_G1;
wire [9:0]  blc_g2_off = night_blc ? NIGHT_BLC_G2 : BLC_G2;
wire [9:0]  blc_b_off  = night_blc ? NIGHT_BLC_B  : BLC_B;

reg [39:0] blc_tdata;
reg [9:0]  blc_tdest;
reg        blc_tlast, blc_tuser, blc_tvalid;

always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        blc_tdata  <= 40'd0;
        blc_tdest  <= 10'd0;
        blc_tlast  <= 1'b0;
        blc_tuser  <= 1'b0;
        blc_tvalid <= 1'b0;
    end else begin
        blc_tlast  <= I_tlast;
        blc_tuser  <= I_tuser;
        blc_tvalid <= I_tvalid;
        blc_tdest  <= I_tdest;
        if(I_tvalid) begin
            blc_tdata[9:0]   <= (I_tdata[9:0]   > blc_r_off ) ? (I_tdata[9:0]   - blc_r_off ) : 10'd0;
            blc_tdata[19:10] <= (I_tdata[19:10] > blc_g1_off) ? (I_tdata[19:10] - blc_g1_off) : 10'd0;
            blc_tdata[29:20] <= (I_tdata[29:20] > blc_g2_off) ? (I_tdata[29:20] - blc_g2_off) : 10'd0;
            blc_tdata[39:30] <= (I_tdata[39:30] > blc_b_off ) ? (I_tdata[39:30] - blc_b_off ) : 10'd0;
        end
    end
end

demosaic #(
    .IMG_HEIGHT          (720 ),
    .IMG_WIDTH           (1280),
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,
    .I_rst_n 			(I_rst_n)	,
    .axi4s_video_tdata 	(blc_tdata)	,
    .axi4s_video_tdest 	(blc_tdest)	,
    .axi4s_video_tlast 	(blc_tlast)	,
    .axi4s_video_tvalid	(blc_tvalid)	,
    .axi4s_video_tuser 	(blc_tuser)	,
    .axi4s_video_tready	(I_tready)	,
    .O_tlast  			(m_aixs_tlast)	,
    .O_tuser  			(m_aixs_tuser)	,
    .O_tdata  			(m_aixs_tdata)	,
    .O_tvalid 			(m_aixs_tvalid)	,
    .O_tready   		(m_aixs_tready)
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

awb #(
    .IMG_HEIGHT(720 ),
    .IMG_WIDTH (1280)
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (m_aixs_tdata_96),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast ),
    .O_tuser (awb_O_tuser ),
    .O_tdata (awb_O_tdata ),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready)
);

// Per-frame statistics + adaptive Otsu threshold + visible threshold marker.
vis_top #(
    .IMG_WIDTH  (1280),
    .IMG_HEIGHT (720)
) u_vis_top (
    .I_clk        (axi4s_video_aclk),
    .I_rst_n      (I_rst_n),
    .I_tuser      (awb_O_tuser),
    .I_tlast      (awb_O_tlast),
    .I_tvalid     (awb_O_tvalid),
    .I_tdata      (awb_O_tdata),
    .I_tready     (vis_O_tready),
    // Switch-selected mode from design_top_wrapper:
    // 0 = passthrough, 1 = Scharr edge, 2 = binarize.
    .I_mode       (I_mode),
    .O_tdata      (vis_O_tdata),
    .O_tlast      (vis_O_tlast),
    .O_tuser      (vis_O_tuser),
    .O_tvalid     (vis_O_tvalid),
    .O_tready     (O_tready),
    .O_motion_alarm (O_motion_alarm),
    .O_motion_any   (O_motion_any),
    .O_box_x0       (O_box_x0),
    .O_box_y0       (O_box_y0),
    .O_box_x1       (O_box_x1),
    .O_box_y1       (O_box_y1),
    .O_low_light    (O_low_light)
);

data_96bit_to_128bit u_data_96bit_to_128bit(
    .I_clk              ( axi4s_video_aclk  ),
    .I_rst_n            ( I_rst_n           ),

    .I_96b_frame_start  ( vis_O_tuser       ),
    .I_96b_valid        ( vis_O_tvalid      ),
    .I_96b_data         ( vis_O_tdata       ),

    .O_128b_frame_start ( awb_O_tuser_128   ),
    .O_128b_valid       ( awb_O_tvalid_128  ),
    .O_128b_data        ( awb_O_tdata_128   )
);

endmodule

//----------------------------------------------------------------------------
// The four vis_* modules below are embedded in this file on purpose.
// TD regenerates each run's .prj from its in-memory file list, so sources that
// are registered only in the .al are ignored and the design compiles with a
// black-box vis_top. isp_top.v is already in the compile list, so keeping the
// modules here makes the build independent of TD's project bookkeeping.
// Regenerate this section with:  powershell -File sim/sync_embedded.ps1
//----------------------------------------------------------------------------

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_gauss3_96
//
// 3x3 Gaussian prefilter for the 96-bit edge-detection path. The input is
// four RGB888 pixels per beat. Luma is filtered once and copied to R/G/B so
// the downstream Scharr engine sees a clean monochrome image.
//
// Kernel:
//       1 2 1
//       2 4 2  / 16
//       1 2 1
//
// It is implemented as a vertical pass followed by a horizontal pass. The
// vertical values are registered before the last add tree, keeping the pixel
// clock path short enough for the 1280x720 stream.
//----------------------------------------------------------------------------
module vis_gauss3_96 #(
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
    output reg [95:0]   O_tdata,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg          O_tvalid,
    output              O_tready
);

    assign O_tready = I_tready;

    localparam GRP_W = IMG_WIDTH >> 2;
    localparam [10:0] GRP_LAST = GRP_W - 1;

    wire [23:0] pix0 = I_tdata[95:72];
    wire [23:0] pix1 = I_tdata[71:48];
    wire [23:0] pix2 = I_tdata[47:24];
    wire [23:0] pix3 = I_tdata[23:0];

    wire [7:0] gray0 = {2'b0,pix0[23:18]} + {1'b0,pix0[15:9]} + {2'b0,pix0[7:2]};
    wire [7:0] gray1 = {2'b0,pix1[23:18]} + {1'b0,pix1[15:9]} + {2'b0,pix1[7:2]};
    wire [7:0] gray2 = {2'b0,pix2[23:18]} + {1'b0,pix2[15:9]} + {2'b0,pix2[7:2]};
    wire [7:0] gray3 = {2'b0,pix3[23:18]} + {1'b0,pix3[15:9]} + {2'b0,pix3[7:2]};

    // line1 = previous row, line2 = row before that.
    (* ram_style = "block" *) reg [31:0] line1_mem [0:GRP_W-1];
    (* ram_style = "block" *) reg [31:0] line2_mem [0:GRP_W-1];

    reg [10:0] x_grp;
    reg [10:0] y_pos;

    wire [31:0] top_word = line2_mem[x_grp];
    wire [31:0] mid_word = line1_mem[x_grp];

    // Stage 1: capture the three raw luma rows for this four-pixel group.
    reg [31:0] top_r, mid_r, cur_r;
    reg [7:0]  tprev_r, mprev_r, cprev_r;
    reg        v1, u1, l1;
    reg [10:0] x1, y1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            top_r <= 32'd0;
            mid_r <= 32'd0;
            cur_r <= 32'd0;
            tprev_r <= 8'd0;
            mprev_r <= 8'd0;
            cprev_r <= 8'd0;
            v1 <= 1'b0;
            u1 <= 1'b0;
            l1 <= 1'b0;
            x1 <= 11'd0;
            y1 <= 11'd0;
        end else begin
            top_r <= top_word;
            mid_r <= mid_word;
            cur_r <= {gray3, gray2, gray1, gray0};
            tprev_r <= top_r[31:24];
            mprev_r <= mid_r[31:24];
            cprev_r <= cur_r[31:24];
            v1 <= I_tvalid;
            u1 <= I_tuser;
            l1 <= I_tlast;
            x1 <= x_grp;
            y1 <= y_pos;
        end
    end

    // The live words point one group ahead while stage 2 runs, so their first
    // pixel is the right-hand neighbour of the four pixels being filtered.
    wire [7:0] t_nxt = top_word[7:0];
    wire [7:0] m_nxt = mid_word[7:0];
    wire [7:0] c_nxt = gray0;

    // Vertical part of the separable Gaussian:
    //   A = top + 2*mid + current
    function [9:0] vsum;
        input [7:0] a;
        input [7:0] b;
        input [7:0] c;
        begin
            vsum = {2'b0, a} + {1'b0, b, 1'b0} + {2'b0, c};
        end
    endfunction

    wire [9:0] a_m1 = vsum(tprev_r,      mprev_r,      cprev_r);
    wire [9:0] a_c0 = vsum(top_r[7:0],   mid_r[7:0],   cur_r[7:0]);
    wire [9:0] a_c1 = vsum(top_r[15:8],  mid_r[15:8],  cur_r[15:8]);
    wire [9:0] a_c2 = vsum(top_r[23:16], mid_r[23:16], cur_r[23:16]);
    wire [9:0] a_c3 = vsum(top_r[31:24], mid_r[31:24], cur_r[31:24]);
    wire [9:0] a_p4 = vsum(t_nxt,        m_nxt,        c_nxt);

    reg [9:0] a_m1_r, a0_r, a1_r, a2_r, a3_r, a4_r;
    reg       v2, u2, l2;
    reg [10:0] x2, y2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            a_m1_r <= 10'd0;
            a0_r <= 10'd0;
            a1_r <= 10'd0;
            a2_r <= 10'd0;
            a3_r <= 10'd0;
            a4_r <= 10'd0;
            v2 <= 1'b0;
            u2 <= 1'b0;
            l2 <= 1'b0;
            x2 <= 11'd0;
            y2 <= 11'd0;
        end else begin
            a_m1_r <= a_m1;
            a0_r <= a_c0;
            a1_r <= a_c1;
            a2_r <= a_c2;
            a3_r <= a_c3;
            a4_r <= a_p4;
            v2 <= v1;
            u2 <= u1;
            l2 <= l1;
            x2 <= x1;
            y2 <= y1;
        end
    end

    // Horizontal part:
    //   out = (left + 2*centre + right + 8) / 16
    function [11:0] hsum;
        input [9:0] a;
        input [9:0] b;
        input [9:0] c;
        begin
            hsum = {2'b0, a} + {1'b0, b, 1'b0} + {2'b0, c} + 12'd8;
        end
    endfunction

    wire [7:0] blur0 = hsum(a_m1_r, a0_r, a1_r) >> 4;
    wire [7:0] blur1 = hsum(a0_r,   a1_r, a2_r) >> 4;
    wire [7:0] blur2 = hsum(a1_r,   a2_r, a3_r) >> 4;
    wire [7:0] blur3 = hsum(a2_r,   a3_r, a4_r) >> 4;

    wire row_ok = (y2 >= 11'd2) && !u2;
    wire ok0 = row_ok && (x2 != 11'd0);
    wire ok1 = row_ok;
    wire ok2 = row_ok;
    wire ok3 = row_ok && (x2 != GRP_LAST);

    wire [7:0] out0 = ok0 ? blur0 : 8'd0;
    wire [7:0] out1 = ok1 ? blur1 : 8'd0;
    wire [7:0] out2 = ok2 ? blur2 : 8'd0;
    wire [7:0] out3 = ok3 ? blur3 : 8'd0;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tdata <= 96'd0;
            O_tlast <= 1'b0;
            O_tuser <= 1'b0;
            O_tvalid <= 1'b0;
        end else begin
            O_tvalid <= v2;
            O_tlast  <= l2;
            O_tuser  <= u2;
            if(v2)
                O_tdata <= {{out0,out0,out0},
                            {out1,out1,out1},
                            {out2,out2,out2},
                            {out3,out3,out3}};
            else
                O_tdata <= 96'd0;
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            x_grp <= 11'd0;
            y_pos <= 11'd0;
        end else if(I_tvalid) begin
            line2_mem[x_grp] <= mid_word;
            line1_mem[x_grp] <= {gray3, gray2, gray1, gray0};

            if(I_tuser) begin
                x_grp <= 11'd1;
                y_pos <= 11'd0;
            end else if(x_grp == GRP_LAST) begin
                x_grp <= 11'd0;
                if(y_pos == IMG_HEIGHT - 1)
                    y_pos <= 11'd0;
                else
                    y_pos <= y_pos + 1'b1;
            end else begin
                x_grp <= x_grp + 1'b1;
            end
        end
    end

endmodule

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

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_brightness
//
// Brightness offset on the 96-bit RGB stream (4 pixels per beat). Each channel
// gets +BRIGHT_ADD - BRIGHT_SUB and is clipped to 0..255. No multiplier, so it
// costs almost nothing.
//
// Ported from the MiLianKe ui_Brightness_adj reference; the arithmetic and the
// clipping idiom (sign bit for negative, bit 8 for overflow) are kept as-is so
// the behaviour matches the vendor demo.
//----------------------------------------------------------------------------
module vis_brightness #(
    parameter [7:0] BRIGHT_ADD   = 8'd0,   // 0..255
    parameter [7:0] BRIGHT_SUB   = 8'd0    // 0..255
)(
    input               I_clk,
    input               I_rst_n,

    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    output              I_tready,

    output              O_tlast,
    output              O_tuser,
    output     [95:0]   O_tdata,
    output              O_tvalid,
    input               O_tready
);

    reg I_tlast_r, I_tuser_r, I_tvalid_r;

    wire [23:0] p0 = I_tdata[23:0];
    wire [23:0] p1 = I_tdata[47:24];
    wire [23:0] p2 = I_tdata[71:48];
    wire [23:0] p3 = I_tdata[95:72];

    wire [7:0] c [0:11];
    assign c[0]  = p0[23:16];
    assign c[1]  = p0[15:8];
    assign c[2]  = p0[7:0];
    assign c[3]  = p1[23:16];
    assign c[4]  = p1[15:8];
    assign c[5]  = p1[7:0];
    assign c[6]  = p2[23:16];
    assign c[7]  = p2[15:8];
    assign c[8]  = p2[7:0];
    assign c[9]  = p3[23:16];
    assign c[10] = p3[15:8];
    assign c[11] = p3[7:0];

    reg  [9:0] d [0:11];
    wire [7:0] o [0:11];

    genvar i;
    generate
        for (i = 0; i < 12; i = i + 1) begin : g_chan
            always @(posedge I_clk or negedge I_rst_n) begin
                if(!I_rst_n)
                    d[i] <= 10'd0;
                else if(I_tvalid)
                    d[i] <= {2'b00, c[i]} + {2'b00, BRIGHT_ADD} - {2'b00, BRIGHT_SUB};
            end

            assign o[i] = d[i][9] ? 8'd0 : (d[i][8] ? 8'd255 : d[i][7:0]);
        end
    endgenerate

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            {I_tlast_r, I_tuser_r, I_tvalid_r} <= 3'b0;
        else begin
            I_tlast_r  <= I_tlast;
            I_tuser_r  <= I_tuser;
            I_tvalid_r <= I_tvalid;
        end
    end

    assign O_tlast  = I_tlast_r;
    assign O_tuser  = I_tuser_r;
    assign O_tvalid = I_tvalid_r;
    assign I_tready = O_tready;

    assign O_tdata = { o[9], o[10], o[11],
                       o[6], o[7],  o[8],
                       o[3], o[4],  o[5],
                       o[0], o[1],  o[2] };

endmodule

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_contrast
//
// Contrast stretch around mid-grey:
//
//     out = 128 + (in - 128) * GAIN
//
// GAIN is expressed in quarter steps and implemented as shifts and adds, so no
// multiplier is inferred:
//     GAIN=4 -> 1.00x   GAIN=5 -> 1.25x   GAIN=6 -> 1.50x   GAIN=8 -> 2.00x
//
// The reference design (ui_Contrast_adj) does the same maths with an 8x8
// multiplier per channel; at 78% LUT and 30 free DSPs that does not fit, and
// quarter-step gain is plenty for a display tuning control.
//
// Two-stage pipeline; tuser/tlast/tvalid are delayed to match.
//----------------------------------------------------------------------------
module vis_contrast #(
    parameter [3:0] GAIN = 4'd5        // 4=1.0x, 5=1.25x, 6=1.5x, 8=2.0x
)(
    input               I_clk,
    input               I_rst_n,

    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    output              I_tready,

    output              O_tlast,
    output              O_tuser,
    output     [95:0]   O_tdata,
    output              O_tvalid,
    input               O_tready
);

    reg [1:0] I_tlast_r, I_tuser_r, I_tvalid_r;

    wire [23:0] p0 = I_tdata[23:0];
    wire [23:0] p1 = I_tdata[47:24];
    wire [23:0] p2 = I_tdata[71:48];
    wire [23:0] p3 = I_tdata[95:72];

    wire [7:0] c [0:11];
    assign c[0]  = p0[23:16];
    assign c[1]  = p0[15:8];
    assign c[2]  = p0[7:0];
    assign c[3]  = p1[23:16];
    assign c[4]  = p1[15:8];
    assign c[5]  = p1[7:0];
    assign c[6]  = p2[23:16];
    assign c[7]  = p2[15:8];
    assign c[8]  = p2[7:0];
    assign c[9]  = p3[23:16];
    assign c[10] = p3[15:8];
    assign c[11] = p3[7:0];

    reg  signed [9:0]  cent [0:11];    // (in - 128)
    reg  signed [11:0] scal [0:11];    // (in-128) * GAIN / 4
    wire signed [11:0] sum  [0:11];    // scaled + 128
    wire [7:0]         o    [0:11];

    genvar i;
    generate
        for (i = 0; i < 12; i = i + 1) begin : g_chan
            // stage 1: centre around mid-grey
            always @(posedge I_clk or negedge I_rst_n) begin
                if(!I_rst_n)
                    cent[i] <= 10'sd0;
                else if(I_tvalid)
                    cent[i] <= $signed({2'b00, c[i]}) - 10'sd128;
            end

            // stage 2: multiply by the quarter step, via shifts
            //   GAIN=4 -> d
            //   GAIN=5 -> d + d/4
            //   GAIN=6 -> d + d/2
            //   GAIN=8 -> d*2
            always @(posedge I_clk or negedge I_rst_n) begin
                if(!I_rst_n)
                    scal[i] <= 12'sd0;
                else if(I_tvalid_r[0]) begin
                    case (GAIN)
                        4'd4: scal[i] <= cent[i];
                        4'd5: scal[i] <= cent[i] + (cent[i] >>> 2);
                        4'd6: scal[i] <= cent[i] + (cent[i] >>> 1);
                        4'd7: scal[i] <= cent[i] + (cent[i] >>> 1) + (cent[i] >>> 2);
                        4'd8: scal[i] <= cent[i] <<< 1;
                        default: scal[i] <= cent[i];
                    endcase
                end
            end

            assign sum[i] = scal[i] + 12'sd128;

            // clip: below 0 or above 255
            assign o[i] = sum[i][11] ? 8'd0 :
                          (sum[i] > 12'sd255) ? 8'd255 : sum[i][7:0];
        end
    endgenerate

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            {I_tlast_r, I_tuser_r, I_tvalid_r} <= 6'b0;
        else begin
            I_tlast_r  <= {I_tlast_r[0],  I_tlast};
            I_tuser_r  <= {I_tuser_r[0],  I_tuser};
            I_tvalid_r <= {I_tvalid_r[0], I_tvalid};
        end
    end

    assign O_tlast  = I_tlast_r[1];
    assign O_tuser  = I_tuser_r[1];
    assign O_tvalid = I_tvalid_r[1];
    assign I_tready = O_tready;

    assign O_tdata = { o[9], o[10], o[11],
                       o[6], o[7],  o[8],
                       o[3], o[4],  o[5],
                       o[0], o[1],  o[2] };

endmodule

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_color_enh
//
// Colour enhancement for the "washed out / hazy" camera look. Each channel is
// pushed away from the pixel luma:
//
//     out = Y + 1.25 * (c - Y)
//
// 1.25 is d + (d >> 2), so there is no multiplier anywhere - only a subtract,
// a shift and two adds per channel. Results are clipped to 0..255, so strongly
// saturated pixels clamp instead of wrapping.
//
// Purely combinational on the pixel path, so it adds no latency.
//----------------------------------------------------------------------------
module vis_color_enh (
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

    // 1/4 R + 1/2 G + 1/4 B, maximum 253, no divider needed.
    function [7:0] lum;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        begin
            lum = {2'b00, r[7:2]} + {1'b0, g[7:1]} + {2'b00, b[7:2]};
        end
    endfunction

    function [7:0] push;
        input [7:0] c;
        input [7:0] y;
        reg signed [11:0] d;
        reg signed [11:0] r;
        begin
            d = $signed({4'b0000, c}) - $signed({4'b0000, y});
            // 1.25x saturation: d + (d >> 2). Deliberately gentle: any colour
            // unevenness the sensor/lens already has (radial lens shading,
            // demosaic false colour) gets multiplied by this factor too, so a
            // larger boost trades vividness for visible colour defects.
            r = $signed({4'b0000, y}) + d + (d >>> 2);
            if (r < 12'sd0)
                push = 8'd0;
            else if (r > 12'sd255)
                push = 8'd255;
            else
                push = r[7:0];
        end
    endfunction

    // Green trim. The sensor plus the vendor AWB leaves a slight green cast,
    // and the saturation boost amplifies it, so G is pulled down about 6%:
    // g - (g >> 4). Change to (g >> 3) for a stronger ~12% correction, or
    // remove the call below if the scene is genuinely green.
    function [7:0] g_trim;
        input [7:0] g;
        begin
            g_trim = g - (g >> 4);
        end
    endfunction

    function [23:0] enh_pixel;
        input [23:0] p;
        reg   [7:0]  y;
        begin
            y = lum(p[23:16], p[15:8], p[7:0]);
            // g_trim is disabled: the green cast is radial (lens shading), so a
            // global green reduction would push the corners magenta instead.
            // Re-enable ( g_trim(push(...)) ) only if the cast is global.
            enh_pixel = { push(p[23:16], y),
                          push(p[15:8],  y),
                          push(p[7:0],   y) };
        end
    endfunction

    // Gamma correction first, on all 12 channel values (4 px x 3 channels),
    // so the saturation boost below acts on display-referred data.
    wire [7:0] gr0, gg0, gb0, gr1, gg1, gb1, gr2, gg2, gb2, gr3, gg3, gb3;

    vis_gamma_rom u_g0r (.I_in(I_tdata[95:88]), .O_out(gr0));
    vis_gamma_rom u_g0g (.I_in(I_tdata[87:80]), .O_out(gg0));
    vis_gamma_rom u_g0b (.I_in(I_tdata[79:72]), .O_out(gb0));
    vis_gamma_rom u_g1r (.I_in(I_tdata[71:64]), .O_out(gr1));
    vis_gamma_rom u_g1g (.I_in(I_tdata[63:56]), .O_out(gg1));
    vis_gamma_rom u_g1b (.I_in(I_tdata[55:48]), .O_out(gb1));
    vis_gamma_rom u_g2r (.I_in(I_tdata[47:40]), .O_out(gr2));
    vis_gamma_rom u_g2g (.I_in(I_tdata[39:32]), .O_out(gg2));
    vis_gamma_rom u_g2b (.I_in(I_tdata[31:24]), .O_out(gb2));
    vis_gamma_rom u_g3r (.I_in(I_tdata[23:16]), .O_out(gr3));
    vis_gamma_rom u_g3g (.I_in(I_tdata[15:8]),  .O_out(gg3));
    vis_gamma_rom u_g3b (.I_in(I_tdata[7:0]),   .O_out(gb3));

    // ---- stage 1: register the gamma results ------------------------------
    // Gamma lookup -> saturation in a single cycle did not close timing
    // (SWNS fell to 0.009 ns), so the table outputs are registered here and the
    // saturation arithmetic gets a cycle of its own. The stream control is
    // delayed by the same one cycle so tuser/tlast stay aligned.
    reg [7:0] gr0_r, gg0_r, gb0_r, gr1_r, gg1_r, gb1_r;
    reg [7:0] gr2_r, gg2_r, gb2_r, gr3_r, gg3_r, gb3_r;
    reg       v1, u1, l1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            gr0_r <= 8'd0; gg0_r <= 8'd0; gb0_r <= 8'd0;
            gr1_r <= 8'd0; gg1_r <= 8'd0; gb1_r <= 8'd0;
            gr2_r <= 8'd0; gg2_r <= 8'd0; gb2_r <= 8'd0;
            gr3_r <= 8'd0; gg3_r <= 8'd0; gb3_r <= 8'd0;
            v1 <= 1'b0; u1 <= 1'b0; l1 <= 1'b0;
        end else begin
            gr0_r <= gr0; gg0_r <= gg0; gb0_r <= gb0;
            gr1_r <= gr1; gg1_r <= gg1; gb1_r <= gb1;
            gr2_r <= gr2; gg2_r <= gg2; gb2_r <= gb2;
            gr3_r <= gr3; gg3_r <= gg3; gb3_r <= gb3;
            v1 <= I_tvalid; u1 <= I_tuser; l1 <= I_tlast;
        end
    end

    // ---- stage 2: saturation, registered output ---------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
        end else begin
            O_tlast  <= l1;
            O_tuser  <= u1;
            O_tvalid <= v1;
            if(v1)
                O_tdata <= { enh_pixel({gr0_r,gg0_r,gb0_r}),
                             enh_pixel({gr1_r,gg1_r,gb1_r}),
                             enh_pixel({gr2_r,gg2_r,gb2_r}),
                             enh_pixel({gr3_r,gg3_r,gb3_r}) };
            else
                O_tdata <= 96'd0;
        end
    end

endmodule

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_gamma_rom -- 256x8 gamma lookup table, generated.
//   out = 255 * (in/255)^0.83
// Gamma < 1 lifts the midtones, which is what a sensor whose response does
// not match the monitor looks like it is missing: blacks stay black, but
// the hazy grey midtones come back up.
//----------------------------------------------------------------------------
module vis_gamma_rom (
    input      [7:0] I_in,
    output reg [7:0] O_out
);
    always @(*) begin
        case (I_in)
            8'd0: O_out = 8'd0;
            8'd1: O_out = 8'd3;
            8'd2: O_out = 8'd5;
            8'd3: O_out = 8'd6;
            8'd4: O_out = 8'd8;
            8'd5: O_out = 8'd10;
            8'd6: O_out = 8'd11;
            8'd7: O_out = 8'd13;
            8'd8: O_out = 8'd14;
            8'd9: O_out = 8'd16;
            8'd10: O_out = 8'd17;
            8'd11: O_out = 8'd19;
            8'd12: O_out = 8'd20;
            8'd13: O_out = 8'd22;
            8'd14: O_out = 8'd23;
            8'd15: O_out = 8'd24;
            8'd16: O_out = 8'd26;
            8'd17: O_out = 8'd27;
            8'd18: O_out = 8'd28;
            8'd19: O_out = 8'd30;
            8'd20: O_out = 8'd31;
            8'd21: O_out = 8'd32;
            8'd22: O_out = 8'd33;
            8'd23: O_out = 8'd35;
            8'd24: O_out = 8'd36;
            8'd25: O_out = 8'd37;
            8'd26: O_out = 8'd38;
            8'd27: O_out = 8'd40;
            8'd28: O_out = 8'd41;
            8'd29: O_out = 8'd42;
            8'd30: O_out = 8'd43;
            8'd31: O_out = 8'd44;
            8'd32: O_out = 8'd46;
            8'd33: O_out = 8'd47;
            8'd34: O_out = 8'd48;
            8'd35: O_out = 8'd49;
            8'd36: O_out = 8'd50;
            8'd37: O_out = 8'd51;
            8'd38: O_out = 8'd53;
            8'd39: O_out = 8'd54;
            8'd40: O_out = 8'd55;
            8'd41: O_out = 8'd56;
            8'd42: O_out = 8'd57;
            8'd43: O_out = 8'd58;
            8'd44: O_out = 8'd59;
            8'd45: O_out = 8'd60;
            8'd46: O_out = 8'd62;
            8'd47: O_out = 8'd63;
            8'd48: O_out = 8'd64;
            8'd49: O_out = 8'd65;
            8'd50: O_out = 8'd66;
            8'd51: O_out = 8'd67;
            8'd52: O_out = 8'd68;
            8'd53: O_out = 8'd69;
            8'd54: O_out = 8'd70;
            8'd55: O_out = 8'd71;
            8'd56: O_out = 8'd72;
            8'd57: O_out = 8'd74;
            8'd58: O_out = 8'd75;
            8'd59: O_out = 8'd76;
            8'd60: O_out = 8'd77;
            8'd61: O_out = 8'd78;
            8'd62: O_out = 8'd79;
            8'd63: O_out = 8'd80;
            8'd64: O_out = 8'd81;
            8'd65: O_out = 8'd82;
            8'd66: O_out = 8'd83;
            8'd67: O_out = 8'd84;
            8'd68: O_out = 8'd85;
            8'd69: O_out = 8'd86;
            8'd70: O_out = 8'd87;
            8'd71: O_out = 8'd88;
            8'd72: O_out = 8'd89;
            8'd73: O_out = 8'd90;
            8'd74: O_out = 8'd91;
            8'd75: O_out = 8'd92;
            8'd76: O_out = 8'd93;
            8'd77: O_out = 8'd94;
            8'd78: O_out = 8'd95;
            8'd79: O_out = 8'd96;
            8'd80: O_out = 8'd97;
            8'd81: O_out = 8'd98;
            8'd82: O_out = 8'd99;
            8'd83: O_out = 8'd100;
            8'd84: O_out = 8'd101;
            8'd85: O_out = 8'd102;
            8'd86: O_out = 8'd103;
            8'd87: O_out = 8'd104;
            8'd88: O_out = 8'd105;
            8'd89: O_out = 8'd106;
            8'd90: O_out = 8'd107;
            8'd91: O_out = 8'd108;
            8'd92: O_out = 8'd109;
            8'd93: O_out = 8'd110;
            8'd94: O_out = 8'd111;
            8'd95: O_out = 8'd112;
            8'd96: O_out = 8'd113;
            8'd97: O_out = 8'd114;
            8'd98: O_out = 8'd115;
            8'd99: O_out = 8'd116;
            8'd100: O_out = 8'd117;
            8'd101: O_out = 8'd118;
            8'd102: O_out = 8'd119;
            8'd103: O_out = 8'd120;
            8'd104: O_out = 8'd121;
            8'd105: O_out = 8'd122;
            8'd106: O_out = 8'd123;
            8'd107: O_out = 8'd124;
            8'd108: O_out = 8'd125;
            8'd109: O_out = 8'd126;
            8'd110: O_out = 8'd127;
            8'd111: O_out = 8'd128;
            8'd112: O_out = 8'd129;
            8'd113: O_out = 8'd130;
            8'd114: O_out = 8'd131;
            8'd115: O_out = 8'd132;
            8'd116: O_out = 8'd133;
            8'd117: O_out = 8'd134;
            8'd118: O_out = 8'd135;
            8'd119: O_out = 8'd135;
            8'd120: O_out = 8'd136;
            8'd121: O_out = 8'd137;
            8'd122: O_out = 8'd138;
            8'd123: O_out = 8'd139;
            8'd124: O_out = 8'd140;
            8'd125: O_out = 8'd141;
            8'd126: O_out = 8'd142;
            8'd127: O_out = 8'd143;
            8'd128: O_out = 8'd144;
            8'd129: O_out = 8'd145;
            8'd130: O_out = 8'd146;
            8'd131: O_out = 8'd147;
            8'd132: O_out = 8'd148;
            8'd133: O_out = 8'd149;
            8'd134: O_out = 8'd149;
            8'd135: O_out = 8'd150;
            8'd136: O_out = 8'd151;
            8'd137: O_out = 8'd152;
            8'd138: O_out = 8'd153;
            8'd139: O_out = 8'd154;
            8'd140: O_out = 8'd155;
            8'd141: O_out = 8'd156;
            8'd142: O_out = 8'd157;
            8'd143: O_out = 8'd158;
            8'd144: O_out = 8'd159;
            8'd145: O_out = 8'd160;
            8'd146: O_out = 8'd161;
            8'd147: O_out = 8'd161;
            8'd148: O_out = 8'd162;
            8'd149: O_out = 8'd163;
            8'd150: O_out = 8'd164;
            8'd151: O_out = 8'd165;
            8'd152: O_out = 8'd166;
            8'd153: O_out = 8'd167;
            8'd154: O_out = 8'd168;
            8'd155: O_out = 8'd169;
            8'd156: O_out = 8'd170;
            8'd157: O_out = 8'd170;
            8'd158: O_out = 8'd171;
            8'd159: O_out = 8'd172;
            8'd160: O_out = 8'd173;
            8'd161: O_out = 8'd174;
            8'd162: O_out = 8'd175;
            8'd163: O_out = 8'd176;
            8'd164: O_out = 8'd177;
            8'd165: O_out = 8'd178;
            8'd166: O_out = 8'd179;
            8'd167: O_out = 8'd179;
            8'd168: O_out = 8'd180;
            8'd169: O_out = 8'd181;
            8'd170: O_out = 8'd182;
            8'd171: O_out = 8'd183;
            8'd172: O_out = 8'd184;
            8'd173: O_out = 8'd185;
            8'd174: O_out = 8'd186;
            8'd175: O_out = 8'd187;
            8'd176: O_out = 8'd187;
            8'd177: O_out = 8'd188;
            8'd178: O_out = 8'd189;
            8'd179: O_out = 8'd190;
            8'd180: O_out = 8'd191;
            8'd181: O_out = 8'd192;
            8'd182: O_out = 8'd193;
            8'd183: O_out = 8'd194;
            8'd184: O_out = 8'd194;
            8'd185: O_out = 8'd195;
            8'd186: O_out = 8'd196;
            8'd187: O_out = 8'd197;
            8'd188: O_out = 8'd198;
            8'd189: O_out = 8'd199;
            8'd190: O_out = 8'd200;
            8'd191: O_out = 8'd201;
            8'd192: O_out = 8'd201;
            8'd193: O_out = 8'd202;
            8'd194: O_out = 8'd203;
            8'd195: O_out = 8'd204;
            8'd196: O_out = 8'd205;
            8'd197: O_out = 8'd206;
            8'd198: O_out = 8'd207;
            8'd199: O_out = 8'd208;
            8'd200: O_out = 8'd208;
            8'd201: O_out = 8'd209;
            8'd202: O_out = 8'd210;
            8'd203: O_out = 8'd211;
            8'd204: O_out = 8'd212;
            8'd205: O_out = 8'd213;
            8'd206: O_out = 8'd214;
            8'd207: O_out = 8'd214;
            8'd208: O_out = 8'd215;
            8'd209: O_out = 8'd216;
            8'd210: O_out = 8'd217;
            8'd211: O_out = 8'd218;
            8'd212: O_out = 8'd219;
            8'd213: O_out = 8'd220;
            8'd214: O_out = 8'd220;
            8'd215: O_out = 8'd221;
            8'd216: O_out = 8'd222;
            8'd217: O_out = 8'd223;
            8'd218: O_out = 8'd224;
            8'd219: O_out = 8'd225;
            8'd220: O_out = 8'd226;
            8'd221: O_out = 8'd226;
            8'd222: O_out = 8'd227;
            8'd223: O_out = 8'd228;
            8'd224: O_out = 8'd229;
            8'd225: O_out = 8'd230;
            8'd226: O_out = 8'd231;
            8'd227: O_out = 8'd232;
            8'd228: O_out = 8'd232;
            8'd229: O_out = 8'd233;
            8'd230: O_out = 8'd234;
            8'd231: O_out = 8'd235;
            8'd232: O_out = 8'd236;
            8'd233: O_out = 8'd237;
            8'd234: O_out = 8'd237;
            8'd235: O_out = 8'd238;
            8'd236: O_out = 8'd239;
            8'd237: O_out = 8'd240;
            8'd238: O_out = 8'd241;
            8'd239: O_out = 8'd242;
            8'd240: O_out = 8'd242;
            8'd241: O_out = 8'd243;
            8'd242: O_out = 8'd244;
            8'd243: O_out = 8'd245;
            8'd244: O_out = 8'd246;
            8'd245: O_out = 8'd247;
            8'd246: O_out = 8'd248;
            8'd247: O_out = 8'd248;
            8'd248: O_out = 8'd249;
            8'd249: O_out = 8'd250;
            8'd250: O_out = 8'd251;
            8'd251: O_out = 8'd252;
            8'd252: O_out = 8'd253;
            8'd253: O_out = 8'd253;
            8'd254: O_out = 8'd254;
            8'd255: O_out = 8'd255;
            default: O_out = 8'd0;
        endcase
    end
endmodule

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_sharpen
//
// Unsharp mask on the 96-bit ISP stream (4 x RGB888 per beat).
//
//   detail = Y - blur3x3(Y)          (the high frequency part)
//   out_c  = c + detail/2            (clipped)
//
// Only the LUMA is blurred, so the line buffers are 8 bits wide instead of 96,
// and the same detail is added back to R, G and B. Human vision is far more
// sensitive to luminance sharpness than to chroma sharpness, so the visual
// result is the same for a fraction of the BRAM.
//
// The blur kernel is the separable-free Gaussian [1 2 1; 2 4 2; 1 2 1], whose
// weights sum to 16. That makes the divide a >>4 shift - no divider and no
// multiplier anywhere. 2x and 4x terms are shifts as well.
//
// Two luma samples are produced per beat (each one the average of a pixel
// pair), which halves the line buffer depth and still gives 640 detail samples
// per line.
//----------------------------------------------------------------------------
module vis_sharpen #(
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

    // ---- stage 1: register luma, RGB and the stream control ----------------
    reg [7:0]  r0, r1, r2, r3;
    reg [95:0] rgb1;
    reg        v1, u1, l1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r0 <= 8'd0; r1 <= 8'd0; r2 <= 8'd0; r3 <= 8'd0;
            rgb1 <= 96'd0;
            v1 <= 1'b0; u1 <= 1'b0; l1 <= 1'b0;
        end else begin
            r0 <= y0; r1 <= y1; r2 <= y2; r3 <= y3;
            rgb1 <= I_tdata;
            v1 <= I_tvalid; u1 <= I_tuser; l1 <= I_tlast;
        end
    end

    // Two luma samples per beat, each the average of a pixel pair.
    wire [8:0] ps0 = {1'b0,r0} + {1'b0,r1};
    wire [8:0] ps1 = {1'b0,r2} + {1'b0,r3};
    wire [7:0] p0  = ps0[8:1];
    wire [7:0] p1  = ps1[8:1];

    localparam GRP = (IMG_WIDTH >> 2);

    (* ram_style = "block" *) reg [15:0] line1_mem [0:GRP-1];
    (* ram_style = "block" *) reg [15:0] line2_mem [0:GRP-1];

    reg [10:0] x_grp;
    reg [10:0] y_pos;

    wire [15:0] top_word = line2_mem[x_grp];
    wire [15:0] mid_word = line1_mem[x_grp];
    wire [15:0] bot_word = {p0,p1};

    wire [7:0] top0 = top_word[15:8];
    wire [7:0] top1 = top_word[7:0];
    wire [7:0] mid0 = mid_word[15:8];
    wire [7:0] mid1 = mid_word[7:0];

    reg [7:0] top_prev2, top_prev1;
    reg [7:0] mid_prev2, mid_prev1;
    reg [7:0] bot_prev2, bot_prev1;

    // ---- stage 2: 3x3 gaussian blur of the two samples --------------------
    // Window for sample 0:            Window for sample 1:
    //   tp2 tp1 top0                    tp1 top0 top1
    //   mp2 mp1 mid0                    mp1 mid0 mid1
    //   bp2 bp1 p0                      bp1 p0   p1
    // Weight 4 sits on the centre, weight 2 on the edge neighbours.
    // Balanced adder trees. Written as a left-associative chain this became
    // ten adders in series (SWNS -0.314 ns), so the terms are paired up here
    // instead: two levels to build each half, then two more to combine.
    wire [8:0]  c0a = {1'b0,top_prev2}  + {1'b0,top0};
    wire [8:0]  c0b = {1'b0,bot_prev2}  + {1'b0,p0};
    wire [9:0]  cor0 = {1'b0,c0a} + {1'b0,c0b};

    wire [8:0]  e0a = {1'b0,top_prev1}  + {1'b0,mid_prev2};
    wire [8:0]  e0b = {1'b0,mid0}       + {1'b0,bot_prev1};
    wire [9:0]  edg0 = {1'b0,e0a} + {1'b0,e0b};

    wire [11:0] bs0 = ({2'b0,cor0} + {1'b0,edg0,1'b0}) + {1'b0,mid_prev1,2'b0};

    wire [8:0]  c1a = {1'b0,top_prev1}  + {1'b0,top1};
    wire [8:0]  c1b = {1'b0,bot_prev1}  + {1'b0,p1};
    wire [9:0]  cor1 = {1'b0,c1a} + {1'b0,c1b};

    wire [8:0]  e1a = {1'b0,top0}       + {1'b0,mid_prev1};
    wire [8:0]  e1b = {1'b0,mid1}       + {1'b0,p0};
    wire [9:0]  edg1 = {1'b0,e1a} + {1'b0,e1b};

    wire [11:0] bs1 = ({2'b0,cor1} + {1'b0,edg1,1'b0}) + {1'b0,mid0,2'b0};

    wire [7:0] blur0 = bs0[11:4];
    wire [7:0] blur1 = bs1[11:4];

    // Window validity: the first two rows and the first column lack
    // neighbours. Declared here because the stage-2 register below samples it.
    wire window_valid = (y_pos >= 11'd2) && (x_grp != 11'd0) && !u1;

    // ---- stage 2: register the blur, the pixels and the control -----------
    // The blur tree and the sharpening arithmetic in one cycle did not fit:
    // at 76% LUT the router stretched the path so that 40% of the delay was
    // wire. Splitting the two halves lets each one be placed compactly.
    reg [7:0]  blur0_r, blur1_r, p0_r, p1_r;
    reg [95:0] rgb2;
    reg        wv2, v2, u2, l2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            blur0_r <= 8'd0; blur1_r <= 8'd0;
            p0_r    <= 8'd0; p1_r    <= 8'd0;
            rgb2    <= 96'd0;
            wv2 <= 1'b0; v2 <= 1'b0; u2 <= 1'b0; l2 <= 1'b0;
        end else begin
            blur0_r <= blur0; blur1_r <= blur1;
            p0_r    <= p0;    p1_r    <= p1;
            rgb2    <= rgb1;
            wv2 <= window_valid;
            v2  <= v1; u2 <= u1; l2 <= l1;
        end
    end

    // detail = current - blurred (signed), halved with an arithmetic shift
    wire signed [8:0] det0 = $signed({1'b0,p0_r}) - $signed({1'b0,blur0_r});
    wire signed [8:0] det1 = $signed({1'b0,p1_r}) - $signed({1'b0,blur1_r});
    wire signed [8:0] add0 = det0 >>> 1;
    wire signed [8:0] add1 = det1 >>> 1;

    function [7:0] sharp;
        input [7:0] c;
        input signed [8:0] h;
        reg signed [10:0] t;
        begin
            t = $signed({3'b000,c}) + h;
            if (t < 11'sd0)
                sharp = 8'd0;
            else if (t > 11'sd255)
                sharp = 8'd255;
            else
                sharp = t[7:0];
        end
    endfunction

    // ---- stage 3: registered output ---------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
            x_grp    <= 11'd0;
            y_pos    <= 11'd0;
            top_prev2 <= 8'd0; top_prev1 <= 8'd0;
            mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
            bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
        end else begin
            O_tlast  <= l2;
            O_tuser  <= u2;
            O_tvalid <= v2;

            if(v2) begin
                if(wv2)
                    O_tdata <= { sharp(rgb2[95:88], add0), sharp(rgb2[87:80], add0),
                                 sharp(rgb2[79:72], add0),
                                 sharp(rgb2[71:64], add0), sharp(rgb2[63:56], add0),
                                 sharp(rgb2[55:48], add0),
                                 sharp(rgb2[47:40], add1), sharp(rgb2[39:32], add1),
                                 sharp(rgb2[31:24], add1),
                                 sharp(rgb2[23:16], add1), sharp(rgb2[15:8],  add1),
                                 sharp(rgb2[7:0],   add1) };
                else
                    O_tdata <= rgb2;      // no full window yet: pass through
            end else
                O_tdata <= 96'd0;

            // line buffers and window position, on the stage-1 control
            if(v1) begin
                line2_mem[x_grp] <= mid_word;
                line1_mem[x_grp] <= bot_word;

                top_prev2 <= top0; top_prev1 <= top1;
                mid_prev2 <= mid0; mid_prev1 <= mid1;
                bot_prev2 <= p0;   bot_prev1 <= p1;

                if(u1) begin
                    x_grp <= 11'd1;
                    y_pos <= 11'd0;
                end else if(x_grp == (GRP - 1)) begin
                    x_grp <= 11'd0;
                    if(y_pos == (IMG_HEIGHT - 1))
                        y_pos <= 11'd0;
                    else
                        y_pos <= y_pos + 1'b1;
                    top_prev2 <= 8'd0; top_prev1 <= 8'd0;
                    mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
                    bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
                end else begin
                    x_grp <= x_grp + 1'b1;
                end
            end
        end
    end

endmodule

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_motion_grid
//
// Block-mean frame-difference motion detector, tapped on the 96-bit ISP
// stream (4 x RGB888 per beat, tuser = frame start, tlast = line end).
//
// A full 720p luma frame (7.4 Mbit) does not fit the 2160 Kbit of on-chip
// RAM, so this detector compares block means instead. The default is a 36x64
// grid of 20x20-pixel blocks, which costs 18 Kbit and keeps the common
// block-address network small enough to route at 103.5 MHz. The box filter
// also makes the detector robust against sensor noise.
//
// Reports, once per frame: a global bounding box in pixel coordinates, the
// number of moving blocks, and a motion flag. The first frame only fills the
// reference buffer and reports no motion.
//----------------------------------------------------------------------------
module vis_motion_grid #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter BLK        = 20,                 // block edge, multiple of 4
    parameter GX         = IMG_WIDTH  / BLK,   // blocks per row
    parameter GY         = IMG_HEIGHT / BLK,   // block rows
    parameter DIFF_TH    = 8'd12,              // block-mean difference
    // If more than this many blocks change at once the whole frame moved,
    // which means the sensor AE / AWB drifted rather than an object moving.
    // Half the grid still rejects a full-frame lighting change while leaving
    // room for a large moving object.
    parameter BLK_MAX    = (GX * GY) / 2,
    parameter SUM_BITS   = 16,                 // block accumulator
    parameter XB_BITS    = 7,
    parameter YB_BITS    = 6,
    parameter ADDR_BITS  = 12                  // >= ceil(log2(GX*GY))
)(
    input                   I_clk,
    input                   I_rst_n,

    input                   I_tuser,
    input                   I_tlast,
    input                   I_tvalid,
    input      [95:0]       I_tdata,

    output reg              O_frame_done,      // 1-cycle pulse at frame end
    output reg              O_any_motion,
    output reg [15:0]       O_box_x0,          // inclusive pixel coordinates
    output reg [15:0]       O_box_y0,
    output reg [15:0]       O_box_x1,
    output reg [15:0]       O_box_y1,
    output reg [15:0]       O_motion_blocks
);

    localparam Y_LAST        = IMG_HEIGHT - 1;
    localparam BEATS_PER_BLK = BLK / 4;

    // ----- pixel luma -------------------------------------------------------
    function [7:0] lum;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        begin
            lum = {2'b00, r[7:2]} + {1'b0, g[7:1]} + {2'b00, b[7:2]};
        end
    endfunction

    wire [7:0] l0 = lum(I_tdata[95:88], I_tdata[87:80], I_tdata[79:72]);
    wire [7:0] l1 = lum(I_tdata[71:64], I_tdata[63:56], I_tdata[55:48]);
    wire [7:0] l2 = lum(I_tdata[47:40], I_tdata[39:32], I_tdata[31:24]);
    wire [7:0] l3 = lum(I_tdata[23:16], I_tdata[15:8],  I_tdata[7:0]);
    wire [SUM_BITS-1:0] beat_sum = l0 + l1 + l2 + l3;

    // ----- block accumulators, one per column -------------------------------
    reg [SUM_BITS-1:0] blk_sum [0:GX-1];

    // ----- previous-frame block means ---------------------------------------
    reg [7:0] prev_mean [0:GX*GY-1];

    reg [ADDR_BITS-1:0] rd_addr;
    reg [7:0]           prev_rd;

    // ----- position ---------------------------------------------------------
    reg [9:0] y;
    reg [XB_BITS-1:0] xb;
    reg [YB_BITS-1:0] yb;
    reg [4:0] x_in_blk;
    reg [5:0] line_in_blk;

    wire sof = I_tvalid & I_tuser;
    wire eol = I_tvalid & I_tlast;
    wire frame_end        = eol & (y == Y_LAST[9:0]);
    wire last_line_of_row = (line_in_blk == BLK - 1);
    wire last_beat_of_blk = (x_in_blk == BEATS_PER_BLK - 1);
    wire finalize         = I_tvalid & last_line_of_row & last_beat_of_blk;

    wire [SUM_BITS-1:0] fin_sum = blk_sum[xb] + beat_sum;
    wire [ADDR_BITS-1:0] cur_addr = yb * GX + xb;

    // ----- two-stage finalize pipeline --------------------------------------
    // Stage 1 latches the finished block and issues the BRAM read; stage 2
    // compares against the previous-frame mean one cycle later, after the
    // registered read data is valid.
    reg                        s1_v;
    reg [7:0]                  s1_mean;
    reg [XB_BITS-1:0]          s1_xb;
    reg [YB_BITS-1:0]          s1_yb;
    reg [ADDR_BITS-1:0]        s1_addr;

    reg                        s2_v;
    reg [7:0]                  s2_mean;
    reg [XB_BITS-1:0]          s2_xb;
    reg [YB_BITS-1:0]          s2_yb;
    reg [ADDR_BITS-1:0]        s2_addr;

    wire [7:0] s2_diff = (s2_mean >= prev_rd) ? (s2_mean - prev_rd)
                                              : (prev_rd - s2_mean);
    wire s2_motion = s2_v & (s2_diff > DIFF_TH);

    // ----- frame-level accumulation ----------------------------------------
    reg        warm;
    reg [XB_BITS-1:0] min_xb, max_xb;
    reg [YB_BITS-1:0] min_yb, max_yb;
    reg        any_blk;
    reg [15:0] blk_cnt;

    // Delay the frame report until the last block of the frame has drained
    // through the two-stage finalize pipeline.
    reg [3:0]  fe_pipe;
    wire       fe_latch = fe_pipe[3];

    integer bi;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for (bi = 0; bi < GX; bi = bi + 1)
                blk_sum[bi] <= {SUM_BITS{1'b0}};
            y <= 10'd0;
            xb <= {XB_BITS{1'b0}};
            yb <= {YB_BITS{1'b0}};
            x_in_blk <= 5'd0;
            line_in_blk <= 6'd0;
            warm <= 1'b0;
            min_xb <= {XB_BITS{1'b1}};
            max_xb <= {XB_BITS{1'b0}};
            min_yb <= {YB_BITS{1'b1}};
            max_yb <= {YB_BITS{1'b0}};
            any_blk <= 1'b0;
            blk_cnt <= 16'd0;
            s1_v <= 1'b0; s2_v <= 1'b0;
            s1_mean <= 8'd0; s2_mean <= 8'd0;
            s1_xb <= {XB_BITS{1'b0}}; s2_xb <= {XB_BITS{1'b0}};
            s1_yb <= {YB_BITS{1'b0}}; s2_yb <= {YB_BITS{1'b0}};
            s1_addr <= {ADDR_BITS{1'b0}}; s2_addr <= {ADDR_BITS{1'b0}};
            rd_addr <= {ADDR_BITS{1'b0}};
            prev_rd <= 8'd0;
            fe_pipe <= 4'd0;
            O_frame_done <= 1'b0;
            O_any_motion <= 1'b0;
            O_box_x0 <= 16'd0;
            O_box_y0 <= 16'd0;
            O_box_x1 <= 16'd0;
            O_box_y1 <= 16'd0;
            O_motion_blocks <= 16'd0;
        end else begin
            O_frame_done <= 1'b0;
            fe_pipe <= {fe_pipe[2:0], frame_end};

            // Registered read of the previous-frame mean.
            prev_rd <= prev_mean[rd_addr];

            // ---- stage 2: compare, store, grow the box --------------------
            if (s2_motion && warm) begin
                blk_cnt <= blk_cnt + 1'b1;
                if (!any_blk) begin
                    min_xb <= s2_xb;  max_xb <= s2_xb;
                    min_yb <= s2_yb;  max_yb <= s2_yb;
                    any_blk <= 1'b1;
                end else begin
                    if (s2_xb < min_xb) min_xb <= s2_xb;
                    if (s2_xb > max_xb) max_xb <= s2_xb;
                    if (s2_yb < min_yb) min_yb <= s2_yb;
                    if (s2_yb > max_yb) max_yb <= s2_yb;
                end
            end
            if (s2_v)
                prev_mean[s2_addr] <= s2_mean;

            // ---- stage 1: shift -------------------------------------------------
            s2_v <= s1_v;
            s2_mean <= s1_mean;
            s2_xb <= s1_xb;
            s2_yb <= s1_yb;
            s2_addr <= s1_addr;

            // ---- accumulate the current beat into its block ---------------
            if (I_tvalid && !finalize)
                blk_sum[xb] <= blk_sum[xb] + beat_sum;

            // ---- stage 0: latch the finished block ------------------------
            if (finalize) begin
                // Include the beat being processed, then clear the accumulator
                // so the next block row starts from zero.
                s1_mean  <= fin_sum[SUM_BITS-1:8];
                s1_xb    <= xb;
                s1_yb    <= yb;
                s1_addr  <= cur_addr;
                rd_addr  <= cur_addr;
                s1_v     <= 1'b1;
                blk_sum[xb] <= {SUM_BITS{1'b0}};
            end else
                s1_v <= 1'b0;

            // ---- frame position -------------------------------------------
            if (I_tvalid) begin
                if (sof) begin
                    y  <= 10'd0;
                    xb <= {XB_BITS{1'b0}};
                    yb <= {YB_BITS{1'b0}};
                    x_in_blk <= 5'd0;
                    line_in_blk <= 6'd0;
                    min_xb <= {XB_BITS{1'b1}};
                    max_xb <= {XB_BITS{1'b0}};
                    min_yb <= {YB_BITS{1'b1}};
                    max_yb <= {YB_BITS{1'b0}};
                    any_blk <= 1'b0;
                    blk_cnt <= 16'd0;
                end else if (eol) begin
                    xb <= {XB_BITS{1'b0}};
                    x_in_blk <= 5'd0;
                    y  <= y + 10'd1;
                    if (last_line_of_row) begin
                        line_in_blk <= 6'd0;
                        if (y != Y_LAST)
                            yb <= yb + 1'b1;
                    end else begin
                        line_in_blk <= line_in_blk + 1'b1;
                    end
                end else begin
                    if (last_beat_of_blk) begin
                        x_in_blk <= 5'd0;
                        xb <= xb + 1'b1;
                    end else begin
                        x_in_blk <= x_in_blk + 1'b1;
                    end
                end
            end

            // ---- frame report (delayed past the finalize drain) -----------
            if (fe_latch) begin
                O_frame_done    <= 1'b1;
                if (blk_cnt > BLK_MAX) begin
                    // Global lighting change: report nothing rather than
                    // claiming the whole frame is a moving object.
                    O_any_motion    <= 1'b0;
                    O_motion_blocks <= 16'd0;
                    O_box_x0 <= 16'd0;
                    O_box_y0 <= 16'd0;
                    O_box_x1 <= 16'd0;
                    O_box_y1 <= 16'd0;
                end else begin
                    O_any_motion    <= any_blk;
                    O_motion_blocks <= blk_cnt;
                    O_box_x0 <= min_xb * BLK;
                    O_box_y0 <= min_yb * BLK;
                    O_box_x1 <= (max_xb + 1'b1) * BLK - 1'b1;
                    O_box_y1 <= (max_yb + 1'b1) * BLK - 1'b1;
                end
                warm    <= 1'b1;
            end
        end
    end

endmodule

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_frame_stats
//
// Per-frame statistics engine tapped on the 96-bit ISP stream
// (4 x RGB888 pixels per beat, tuser = frame start, tlast = line end).
//
// Produces once per frame:
//   * a 64-bin luma histogram (readable by a downstream Otsu engine)
//   * R/G/B/luma sums and the pixel count for AE / AWB feedback
//
// The histogram uses two ping-pong bank sets. The current frame is binned into
// one set while the other holds the previous frame for readout, and the set
// needed by the next frame is emptied during vertical blanking. This removes
// both the four-pixel write hazard and cross-frame accumulation. Luma is the
// 1/4 + 1/2 + 1/4 approximation, so no multiplier is required.
//----------------------------------------------------------------------------
module vis_frame_stats #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter BIN_BITS   = 6,      // 64 luma bins
    parameter CNT_BITS   = 20,     // per-bank count width
    parameter SUM_BITS   = 32      // channel / luma accumulator width
)(
    input                       I_clk,
    input                       I_rst_n,

    // 96-bit video stream
    input                       I_tuser,
    input                       I_tlast,
    input                       I_tvalid,
    input      [95:0]           I_tdata,

    // Histogram read port (combinational). Serves the completed frame.
    input      [BIN_BITS-1:0]   I_hist_rd_addr,
    output     [CNT_BITS:0]     O_hist_rd_data,

    // Per-frame results (valid while O_frame_done is high)
    output reg                  O_frame_done,
    output reg [SUM_BITS-1:0]   O_r_sum,
    output reg [SUM_BITS-1:0]   O_g_sum,
    output reg [SUM_BITS-1:0]   O_b_sum,
    output reg [SUM_BITS-1:0]   O_luma_sum,
    output reg [CNT_BITS:0]     O_pixel_cnt
);

    localparam BINS = (1 << BIN_BITS);
    localparam [15:0] LAST_LINE = IMG_HEIGHT - 1;

    // ----- pixel slices -----------------------------------------------------
    wire [7:0] r0 = I_tdata[95:88];
    wire [7:0] g0 = I_tdata[87:80];
    wire [7:0] b0 = I_tdata[79:72];

    wire [7:0] r1 = I_tdata[71:64];
    wire [7:0] g1 = I_tdata[63:56];
    wire [7:0] b1 = I_tdata[55:48];

    wire [7:0] r2 = I_tdata[47:40];
    wire [7:0] g2 = I_tdata[39:32];
    wire [7:0] b2 = I_tdata[31:24];

    wire [7:0] r3 = I_tdata[23:16];
    wire [7:0] g3 = I_tdata[15:8];
    wire [7:0] b3 = I_tdata[7:0];

    // ----- 1/4 + 1/2 + 1/4 luma (max 253, always fits in 8 bits) ------------
    function [7:0] lum;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        begin
            lum = {2'b00, r[7:2]} + {1'b0, g[7:1]} + {2'b00, b[7:2]};
        end
    endfunction

    wire [7:0] y0 = lum(r0, g0, b0);
    wire [7:0] y1 = lum(r1, g1, b1);
    wire [7:0] y2 = lum(r2, g2, b2);
    wire [7:0] y3 = lum(r3, g3, b3);

    wire [BIN_BITS-1:0] bin0 = y0[7:8-BIN_BITS];
    wire [BIN_BITS-1:0] bin1 = y1[7:8-BIN_BITS];
    wire [BIN_BITS-1:0] bin2 = y2[7:8-BIN_BITS];
    wire [BIN_BITS-1:0] bin3 = y3[7:8-BIN_BITS];

    // ----- ping-pong histogram banks ----------------------------------------
    (* ram_style = "distributed" *) reg [CNT_BITS-1:0] bankA0 [0:BINS-1];
    (* ram_style = "distributed" *) reg [CNT_BITS-1:0] bankA1 [0:BINS-1];
    (* ram_style = "distributed" *) reg [CNT_BITS-1:0] bankA2 [0:BINS-1];
    (* ram_style = "distributed" *) reg [CNT_BITS-1:0] bankA3 [0:BINS-1];
    (* ram_style = "distributed" *) reg [CNT_BITS-1:0] bankB0 [0:BINS-1];
    (* ram_style = "distributed" *) reg [CNT_BITS-1:0] bankB1 [0:BINS-1];
    (* ram_style = "distributed" *) reg [CNT_BITS-1:0] bankB2 [0:BINS-1];
    (* ram_style = "distributed" *) reg [CNT_BITS-1:0] bankB3 [0:BINS-1];

    wire [CNT_BITS-1:0] ra0 = bankA0[I_hist_rd_addr];
    wire [CNT_BITS-1:0] ra1 = bankA1[I_hist_rd_addr];
    wire [CNT_BITS-1:0] ra2 = bankA2[I_hist_rd_addr];
    wire [CNT_BITS-1:0] ra3 = bankA3[I_hist_rd_addr];
    wire [CNT_BITS-1:0] rb0 = bankB0[I_hist_rd_addr];
    wire [CNT_BITS-1:0] rb1 = bankB1[I_hist_rd_addr];
    wire [CNT_BITS-1:0] rb2 = bankB2[I_hist_rd_addr];
    wire [CNT_BITS-1:0] rb3 = bankB3[I_hist_rd_addr];

    // ----- control ----------------------------------------------------------
    reg        wr_set;        // bank set written during the current frame
    reg        rd_set;        // bank set exposed to the Otsu engine
    reg        clr_set;       // bank set being cleared
    reg        clr_active;
    reg [BIN_BITS-1:0] clr_addr;
    reg [7:0]  init_cnt;      // power-on clear of both sets (128 cycles)

    wire [CNT_BITS:0] sumA = {1'b0, ra0} + {1'b0, ra1} + {1'b0, ra2} + {1'b0, ra3};
    wire [CNT_BITS:0] sumB = {1'b0, rb0} + {1'b0, rb1} + {1'b0, rb2} + {1'b0, rb3};

    assign O_hist_rd_data = rd_set ? sumB : sumA;

    wire init_busy = ~init_cnt[7];

    // ----- frame position ---------------------------------------------------
    reg [15:0] line_cnt;

    wire sof = I_tvalid & I_tuser;
    wire eol = I_tvalid & I_tlast;
    wire eof = eol & (line_cnt == LAST_LINE);
    wire bin_en = I_tvalid & ~clr_active & ~init_busy;

    // ----- running accumulators --------------------------------------------
    reg [SUM_BITS-1:0] r_acc;
    reg [SUM_BITS-1:0] g_acc;
    reg [SUM_BITS-1:0] b_acc;
    reg [SUM_BITS-1:0] y_acc;
    reg [CNT_BITS:0]   p_acc;

    wire [SUM_BITS-1:0] r_beat = r0 + r1 + r2 + r3;
    wire [SUM_BITS-1:0] g_beat = g0 + g1 + g2 + g3;
    wire [SUM_BITS-1:0] b_beat = b0 + b1 + b2 + b3;
    wire [SUM_BITS-1:0] y_beat = y0 + y1 + y2 + y3;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            wr_set       <= 1'b0;
            rd_set       <= 1'b1;
            clr_set      <= 1'b0;
            clr_active   <= 1'b0;
            clr_addr     <= {BIN_BITS{1'b0}};
            init_cnt     <= 8'd0;
            line_cnt     <= 16'd0;
            r_acc        <= {SUM_BITS{1'b0}};
            g_acc        <= {SUM_BITS{1'b0}};
            b_acc        <= {SUM_BITS{1'b0}};
            y_acc        <= {SUM_BITS{1'b0}};
            p_acc        <= {(CNT_BITS+1){1'b0}};
            O_frame_done <= 1'b0;
            O_r_sum      <= {SUM_BITS{1'b0}};
            O_g_sum      <= {SUM_BITS{1'b0}};
            O_b_sum      <= {SUM_BITS{1'b0}};
            O_luma_sum   <= {SUM_BITS{1'b0}};
            O_pixel_cnt  <= {(CNT_BITS+1){1'b0}};
        end else begin
            O_frame_done <= 1'b0;

            if (init_busy)
                init_cnt <= init_cnt + 8'd1;

            if (clr_active) begin
                if (clr_addr == BINS - 1) begin
                    clr_active <= 1'b0;
                    clr_addr   <= {BIN_BITS{1'b0}};
                end else
                    clr_addr <= clr_addr + 1'b1;
            end

            // Power-on clear walks both sets; afterwards the runtime clear
            // engine empties the next frame's set during vertical blanking.
            if (init_busy) begin
                if (init_cnt[6]) begin
                    bankB0[init_cnt[BIN_BITS-1:0]] <= {CNT_BITS{1'b0}};
                    bankB1[init_cnt[BIN_BITS-1:0]] <= {CNT_BITS{1'b0}};
                    bankB2[init_cnt[BIN_BITS-1:0]] <= {CNT_BITS{1'b0}};
                    bankB3[init_cnt[BIN_BITS-1:0]] <= {CNT_BITS{1'b0}};
                end else begin
                    bankA0[init_cnt[BIN_BITS-1:0]] <= {CNT_BITS{1'b0}};
                    bankA1[init_cnt[BIN_BITS-1:0]] <= {CNT_BITS{1'b0}};
                    bankA2[init_cnt[BIN_BITS-1:0]] <= {CNT_BITS{1'b0}};
                    bankA3[init_cnt[BIN_BITS-1:0]] <= {CNT_BITS{1'b0}};
                end
            end else if (clr_active) begin
                if (clr_set) begin
                    bankB0[clr_addr] <= {CNT_BITS{1'b0}};
                    bankB1[clr_addr] <= {CNT_BITS{1'b0}};
                    bankB2[clr_addr] <= {CNT_BITS{1'b0}};
                    bankB3[clr_addr] <= {CNT_BITS{1'b0}};
                end else begin
                    bankA0[clr_addr] <= {CNT_BITS{1'b0}};
                    bankA1[clr_addr] <= {CNT_BITS{1'b0}};
                    bankA2[clr_addr] <= {CNT_BITS{1'b0}};
                    bankA3[clr_addr] <= {CNT_BITS{1'b0}};
                end
            end

            // Histogram binning: four banks updated in parallel.
            if (bin_en) begin
                if (wr_set) begin
                    bankB0[bin0] <= bankB0[bin0] + 1'b1;
                    bankB1[bin1] <= bankB1[bin1] + 1'b1;
                    bankB2[bin2] <= bankB2[bin2] + 1'b1;
                    bankB3[bin3] <= bankB3[bin3] + 1'b1;
                end else begin
                    bankA0[bin0] <= bankA0[bin0] + 1'b1;
                    bankA1[bin1] <= bankA1[bin1] + 1'b1;
                    bankA2[bin2] <= bankA2[bin2] + 1'b1;
                    bankA3[bin3] <= bankA3[bin3] + 1'b1;
                end
            end

            // Frame accumulators and bank rotation.
            if (I_tvalid) begin
                if (sof) begin
                    r_acc    <= r_beat;
                    g_acc    <= g_beat;
                    b_acc    <= b_beat;
                    y_acc    <= y_beat;
                    p_acc    <= 4;
                    line_cnt <= 16'd0;
                end else begin
                    r_acc <= r_acc + r_beat;
                    g_acc <= g_acc + g_beat;
                    b_acc <= b_acc + b_beat;
                    y_acc <= y_acc + y_beat;
                    p_acc <= p_acc + 4;
                    if (eol)
                        line_cnt <= line_cnt + 1'b1;
                end

                if (eof) begin
                    O_r_sum      <= r_acc + r_beat;
                    O_g_sum      <= g_acc + g_beat;
                    O_b_sum      <= b_acc + b_beat;
                    O_luma_sum   <= y_acc + y_beat;
                    O_pixel_cnt  <= p_acc + 4;
                    O_frame_done <= 1'b1;
                    // Publish the finished set, write the other one next frame,
                    // and empty it during the coming vertical blanking.
                    rd_set     <= wr_set;
                    wr_set     <= ~wr_set;
                    clr_set    <= ~wr_set;
                    clr_active <= 1'b1;
                    clr_addr   <= {BIN_BITS{1'b0}};
                end
            end
        end
    end

endmodule

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_otsu_thresh
//
// Sequential Otsu threshold over the 64-bin histogram produced by
// vis_frame_stats. Called once per frame during vertical blanking, so it is
// free to spend thousands of cycles and use a single serial divider instead of
// a wide parallel one.
//
// Between-class variance for a split at bin t is
//     sigma_b^2(t) = W0 * W1 * (mu0 - mu1)^2
// which is what this module maximises. Class means are kept in FRAC-bit
// fixed point and the class weights are scaled down before the multiply so the
// score fits comfortably in 48 bits.
//----------------------------------------------------------------------------
module vis_otsu_thresh #(
    parameter BIN_BITS  = 6,      // histogram bins
    parameter HIST_BITS = 21,     // histogram bin width from vis_frame_stats
    parameter CNT_BITS  = 21,     // pixel count width
    parameter SUM_BITS  = 28,     // weighted-sum width
    parameter FRAC      = 4,      // fractional bits for class means
    parameter WSHIFT    = 8       // class-weight prescale before the multiply
)(
    input                       I_clk,
    input                       I_rst_n,

    input                       I_start,        // 1-cycle pulse
    output     [BIN_BITS-1:0]   O_hist_addr,    // histogram read address
    input      [HIST_BITS-1:0]  I_hist_data,    // histogram bin value

    output reg [7:0]            O_thresh,       // luma threshold 0..255
    output reg                  O_valid,        // 1-cycle pulse when done
    output reg                  O_busy
);

    localparam BINS   = (1 << BIN_BITS);
    localparam ND     = 32;                     // divider width

    localparam S_IDLE  = 4'd0,
               S_P1    = 4'd1,
               S_LOAD  = 4'd2,
               S_P2    = 4'd3,
               S_DIV   = 4'd4,
               S_MU    = 4'd5,
               S_WS    = 4'd6,
               S_SQ    = 4'd7,
               S_CMP   = 4'd8,
               S_UPD   = 4'd9,
               S_DONE  = 4'd10,
               S_P2B   = 4'd11,
               S_P1B   = 4'd12,
               S_P2C   = 4'd13;

    reg [3:0]  state;
    reg [BIN_BITS-1:0] addr;

    reg [CNT_BITS-1:0] n_total;
    reg [SUM_BITS-1:0] s_total;

    reg [CNT_BITS-1:0] w0;
    reg [SUM_BITS-1:0] s0;

    reg [CNT_BITS-1:0] cand_w0;
    reg [CNT_BITS-1:0] cand_w1;

    // S_P2 is split in two so that no single cycle carries a multiplier plus a
    // subtractor chain: S_P2 does the multiply/add, S_P2B the subtractions.
    reg [CNT_BITS-1:0] w0_c;
    reg [SUM_BITS-1:0] s0_c;
    // Registered multiplier results, so no cycle carries a multiplier plus an
    // adder in series.
    reg [SUM_BITS-1:0] p1_r;
    reg [SUM_BITS-1:0] s0_m;
    wire [CNT_BITS-1:0] w1_c = n_total - w0_c;
    wire [SUM_BITS-1:0] s1_c = s_total - s0_c;

    reg [47:0]            best_score;
    reg [BIN_BITS-1:0]    best_t;

    reg [ND-1:0] num0, num1, rem0, rem1, q0, q1;
    reg [ND-1:0] den0, den1;
    reg [5:0]    di;

    wire [HIST_BITS-1:0] bin_val = I_hist_data;

    // The histogram read port is address-combinational, so the value present
    // on I_hist_data always matches the current addr without a pipeline skew.
    assign O_hist_addr = addr;

    // ----- pass 1: totals ---------------------------------------------------
    wire [SUM_BITS-1:0] p1_term = addr * bin_val;

    // ----- pass 2: candidate ------------------------------------------------
    wire [CNT_BITS-1:0] w0_n = w0 + bin_val;
    wire [SUM_BITS-1:0] s0_n = s0 + addr * bin_val;
    wire [CNT_BITS-1:0] w1_n = n_total - w0_n;
    wire [SUM_BITS-1:0] s1_n = s_total - s0_n;

    // ----- serial restore divider (two instances, run in lock step) ---------
    wire [ND-1:0] sh0 = {rem0[ND-2:0], num0[di]};
    wire [ND-1:0] sh1 = {rem1[ND-2:0], num1[di]};
    wire          ge0 = (den0 != {ND{1'b0}}) && (sh0 >= den0);
    wire          ge1 = (den1 != {ND{1'b0}}) && (sh1 >= den1);
    wire [ND-1:0] nx0 = ge0 ? (sh0 - den0) : sh0;
    wire [ND-1:0] nx1 = ge1 ? (sh1 - den1) : sh1;

    // ----- score pipeline ---------------------------------------------------
    // One multiplier per stage. Chaining diff*diff and the class-weight product
    // in a single cycle did not close timing on the 9.66 ns ISP clock, and this
    // whole engine only runs during vertical blanking, so latency is free.
    reg [17:0] mu0_r, mu1_r;
    reg [17:0] diff_r;
    reg [31:0] diff_sq_r;
    reg [23:0] ws0_r, ws1_r;
    reg [31:0] ws_r;
    reg [47:0] score_r;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            state       <= S_IDLE;
            addr        <= {BIN_BITS{1'b0}};
            n_total     <= {CNT_BITS{1'b0}};
            s_total     <= {SUM_BITS{1'b0}};
            w0          <= {CNT_BITS{1'b0}};
            s0          <= {SUM_BITS{1'b0}};
            cand_w0     <= {CNT_BITS{1'b0}};
            cand_w1     <= {CNT_BITS{1'b0}};
            w0_c        <= {CNT_BITS{1'b0}};
            s0_c        <= {SUM_BITS{1'b0}};
            p1_r        <= {SUM_BITS{1'b0}};
            s0_m        <= {SUM_BITS{1'b0}};
            best_score  <= 48'd0;
            best_t      <= {BIN_BITS{1'b0}};
            num0 <= 0; num1 <= 0; rem0 <= 0; rem1 <= 0; q0 <= 0; q1 <= 0;
            den0 <= 0; den1 <= 0;
            di          <= 6'd0;
            mu0_r <= 18'd0; mu1_r <= 18'd0; diff_r <= 18'd0;
            diff_sq_r <= 32'd0; ws0_r <= 24'd0; ws1_r <= 24'd0;
            ws_r <= 32'd0; score_r <= 48'd0;
            O_thresh    <= 8'd0;
            O_valid     <= 1'b0;
            O_busy      <= 1'b0;
        end else begin
            O_valid <= 1'b0;

            case (state)
                S_IDLE: begin
                    if (I_start) begin
                        n_total <= {CNT_BITS{1'b0}};
                        s_total <= {SUM_BITS{1'b0}};
                        addr    <= {BIN_BITS{1'b0}};
                        O_busy  <= 1'b1;
                        state   <= S_P1;
                    end
                end

                S_P1: begin
                    n_total <= n_total + bin_val;
                    s_total <= s_total + p1_r;   // product of the previous addr
                    p1_r    <= p1_term;          // product of the current addr
                    if (addr == BINS - 1)
                        state <= S_P1B;
                    else
                        addr <= addr + 1'b1;
                end

                S_P1B: begin
                    s_total <= s_total + p1_r;   // product of the last addr
                    state   <= S_LOAD;
                end

                S_LOAD: begin
                    addr       <= {BIN_BITS{1'b0}};
                    w0         <= {CNT_BITS{1'b0}};
                    s0         <= {SUM_BITS{1'b0}};
                    best_score <= 48'd0;
                    best_t     <= {BIN_BITS{1'b0}};
                    state      <= S_P2;
                end

                S_P2: begin
                    w0_c <= w0_n;
                    s0_m <= addr * bin_val;
                    state <= S_P2C;
                end

                S_P2C: begin
                    s0_c <= s0 + s0_m;
                    state <= S_P2B;
                end

                S_P2B: begin
                    num0 <= {{(ND-SUM_BITS){1'b0}}, s0_c} << FRAC;
                    num1 <= {{(ND-SUM_BITS){1'b0}}, s1_c} << FRAC;
                    den0 <= {{(ND-CNT_BITS){1'b0}}, w0_c};
                    den1 <= {{(ND-CNT_BITS){1'b0}}, w1_c};
                    rem0 <= {ND{1'b0}};
                    rem1 <= {ND{1'b0}};
                    q0   <= {ND{1'b0}};
                    q1   <= {ND{1'b0}};
                    di   <= ND - 1;
                    cand_w0 <= w0_c;
                    cand_w1 <= w1_c;
                    state <= S_DIV;
                end

                S_DIV: begin
                    rem0   <= nx0;
                    rem1   <= nx1;
                    q0[di] <= ge0;
                    q1[di] <= ge1;
                    if (di == 6'd0)
                        state <= S_MU;
                    else
                        di <= di - 6'd1;
                end

                // Latch the class means and the prescaled class weights.
                S_MU: begin
                    mu0_r <= q0[17:0];
                    mu1_r <= q1[17:0];
                    ws0_r <= cand_w0 >> WSHIFT;
                    ws1_r <= cand_w1 >> WSHIFT;
                    state <= S_WS;
                end

                // |mu0 - mu1| and the weight product (one multiplier).
                S_WS: begin
                    diff_r <= (mu0_r >= mu1_r) ? (mu0_r - mu1_r)
                                               : (mu1_r - mu0_r);
                    ws_r   <= ws0_r * ws1_r;
                    state  <= S_SQ;
                end

                // (mu0 - mu1)^2 (one multiplier).
                S_SQ: begin
                    diff_sq_r <= diff_r * diff_r;
                    state     <= S_CMP;
                end

                // W0 * W1 * (mu0 - mu1)^2, registered so the comparator below
                // has the whole next cycle to itself.
                S_CMP: begin
                    score_r <= ws_r * diff_sq_r;
                    state   <= S_UPD;
                end

                S_UPD: begin
                    if (score_r > best_score) begin
                        best_score <= score_r;
                        best_t     <= addr;
                    end
                    // Advance the running class statistics for the next split.
                    w0 <= w0_c;
                    s0 <= s0_c;
                    if (addr == BINS - 1)
                        state <= S_DONE;
                    else begin
                        addr  <= addr + 1'b1;
                        state <= S_P2;
                    end
                end

                S_DONE: begin
                    // Report the bin centre, not the bin lower edge.
                    O_thresh <= (best_t << (8 - BIN_BITS)) +
                                (1 << (8 - BIN_BITS - 1));
                    O_valid  <= 1'b1;
                    O_busy   <= 1'b0;
                    state    <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_glyph_rom -- 16x16 glyphs, generated (SimHei for CJK, Consolas for
// digits). Index map:
//  0 模 1 式 2 一 3 ： 4 原 5 始 6 图 7 像 8 二 9 边 10 缘 11 检 12 测 13 三
//  14 值 15 化 16 四 17 运 18 动 19 五 20 色 21 彩 22 增 23 强
//  24 自 25 适 26 应  27..36 = 0..9  37 . 38 m 39 s  40 灰 41 度
//----------------------------------------------------------------------------
module vis_glyph_rom (
    input      [5:0]  I_char_idx,
    input      [3:0]  I_row,
    output reg [15:0] O_bits
);
    always @(*) begin
        case ({I_char_idx, I_row})
            10'h000: O_bits = 16'h08A0;
            10'h001: O_bits = 16'h09F8;
            10'h002: O_bits = 16'h08A0;
            10'h003: O_bits = 16'h1DF8;
            10'h004: O_bits = 16'h0908;
            10'h005: O_bits = 16'h0DF8;
            10'h006: O_bits = 16'h1BF8;
            10'h007: O_bits = 16'h0840;
            10'h008: O_bits = 16'h09F0;
            10'h009: O_bits = 16'h0890;
            10'h00A: O_bits = 16'h0B08;
            10'h00B: O_bits = 16'h0000;
            10'h00C: O_bits = 16'h0000;
            10'h00D: O_bits = 16'h0000;
            10'h00E: O_bits = 16'h0000;
            10'h00F: O_bits = 16'h0000;
            10'h010: O_bits = 16'h0080;
            10'h011: O_bits = 16'h00B0;
            10'h012: O_bits = 16'h00A0;
            10'h013: O_bits = 16'h1FF8;
            10'h014: O_bits = 16'h0080;
            10'h015: O_bits = 16'h1F40;
            10'h016: O_bits = 16'h0440;
            10'h017: O_bits = 16'h0440;
            10'h018: O_bits = 16'h0428;
            10'h019: O_bits = 16'h1F28;
            10'h01A: O_bits = 16'h1018;
            10'h01B: O_bits = 16'h0000;
            10'h01C: O_bits = 16'h0000;
            10'h01D: O_bits = 16'h0000;
            10'h01E: O_bits = 16'h0000;
            10'h01F: O_bits = 16'h0000;
            10'h020: O_bits = 16'h0000;
            10'h021: O_bits = 16'h0000;
            10'h022: O_bits = 16'h0000;
            10'h023: O_bits = 16'h0000;
            10'h024: O_bits = 16'h0000;
            10'h025: O_bits = 16'h1FF8;
            10'h026: O_bits = 16'h0000;
            10'h027: O_bits = 16'h0000;
            10'h028: O_bits = 16'h0000;
            10'h029: O_bits = 16'h0000;
            10'h02A: O_bits = 16'h0000;
            10'h02B: O_bits = 16'h0000;
            10'h02C: O_bits = 16'h0000;
            10'h02D: O_bits = 16'h0000;
            10'h02E: O_bits = 16'h0000;
            10'h02F: O_bits = 16'h0000;
            10'h030: O_bits = 16'h0000;
            10'h031: O_bits = 16'h0000;
            10'h032: O_bits = 16'h0000;
            10'h033: O_bits = 16'h0000;
            10'h034: O_bits = 16'h0000;
            10'h035: O_bits = 16'h0C00;
            10'h036: O_bits = 16'h0C00;
            10'h037: O_bits = 16'h0000;
            10'h038: O_bits = 16'h0C00;
            10'h039: O_bits = 16'h0C00;
            10'h03A: O_bits = 16'h0000;
            10'h03B: O_bits = 16'h0000;
            10'h03C: O_bits = 16'h0000;
            10'h03D: O_bits = 16'h0000;
            10'h03E: O_bits = 16'h0000;
            10'h03F: O_bits = 16'h0000;
            10'h040: O_bits = 16'h0000;
            10'h041: O_bits = 16'h0FF8;
            10'h042: O_bits = 16'h0880;
            10'h043: O_bits = 16'h0BF0;
            10'h044: O_bits = 16'h0A10;
            10'h045: O_bits = 16'h0BF0;
            10'h046: O_bits = 16'h0BF0;
            10'h047: O_bits = 16'h0840;
            10'h048: O_bits = 16'h0A50;
            10'h049: O_bits = 16'h1278;
            10'h04A: O_bits = 16'h15C8;
            10'h04B: O_bits = 16'h0000;
            10'h04C: O_bits = 16'h0000;
            10'h04D: O_bits = 16'h0000;
            10'h04E: O_bits = 16'h0000;
            10'h04F: O_bits = 16'h0000;
            10'h050: O_bits = 16'h0840;
            10'h051: O_bits = 16'h0840;
            10'h052: O_bits = 16'h0890;
            10'h053: O_bits = 16'h1F18;
            10'h054: O_bits = 16'h13F8;
            10'h055: O_bits = 16'h1400;
            10'h056: O_bits = 16'h14F8;
            10'h057: O_bits = 16'h0D08;
            10'h058: O_bits = 16'h0D08;
            10'h059: O_bits = 16'h0AF8;
            10'h05A: O_bits = 16'h1008;
            10'h05B: O_bits = 16'h0000;
            10'h05C: O_bits = 16'h0000;
            10'h05D: O_bits = 16'h0000;
            10'h05E: O_bits = 16'h0000;
            10'h05F: O_bits = 16'h0000;
            10'h060: O_bits = 16'h0000;
            10'h061: O_bits = 16'h1FF8;
            10'h062: O_bits = 16'h1108;
            10'h063: O_bits = 16'h12C8;
            10'h064: O_bits = 16'h1588;
            10'h065: O_bits = 16'h11C8;
            10'h066: O_bits = 16'h1738;
            10'h067: O_bits = 16'h10C8;
            10'h068: O_bits = 16'h1188;
            10'h069: O_bits = 16'h1008;
            10'h06A: O_bits = 16'h1FF8;
            10'h06B: O_bits = 16'h0000;
            10'h06C: O_bits = 16'h0000;
            10'h06D: O_bits = 16'h0000;
            10'h06E: O_bits = 16'h0000;
            10'h06F: O_bits = 16'h0000;
            10'h070: O_bits = 16'h0080;
            10'h071: O_bits = 16'h09E0;
            10'h072: O_bits = 16'h0A20;
            10'h073: O_bits = 16'h1FF8;
            10'h074: O_bits = 16'h1A48;
            10'h075: O_bits = 16'h09B0;
            10'h076: O_bits = 16'h0B70;
            10'h077: O_bits = 16'h09D0;
            10'h078: O_bits = 16'h0AB0;
            10'h079: O_bits = 16'h0928;
            10'h07A: O_bits = 16'h0AC0;
            10'h07B: O_bits = 16'h0000;
            10'h07C: O_bits = 16'h0000;
            10'h07D: O_bits = 16'h0000;
            10'h07E: O_bits = 16'h0000;
            10'h07F: O_bits = 16'h0000;
            10'h080: O_bits = 16'h0000;
            10'h081: O_bits = 16'h0000;
            10'h082: O_bits = 16'h0FF0;
            10'h083: O_bits = 16'h0000;
            10'h084: O_bits = 16'h0000;
            10'h085: O_bits = 16'h0000;
            10'h086: O_bits = 16'h0000;
            10'h087: O_bits = 16'h0000;
            10'h088: O_bits = 16'h0000;
            10'h089: O_bits = 16'h1FF8;
            10'h08A: O_bits = 16'h0000;
            10'h08B: O_bits = 16'h0000;
            10'h08C: O_bits = 16'h0000;
            10'h08D: O_bits = 16'h0000;
            10'h08E: O_bits = 16'h0000;
            10'h08F: O_bits = 16'h0000;
            10'h090: O_bits = 16'h00C0;
            10'h091: O_bits = 16'h0880;
            10'h092: O_bits = 16'h0880;
            10'h093: O_bits = 16'h03F8;
            10'h094: O_bits = 16'h0088;
            10'h095: O_bits = 16'h1890;
            10'h096: O_bits = 16'h0890;
            10'h097: O_bits = 16'h0910;
            10'h098: O_bits = 16'h0A30;
            10'h099: O_bits = 16'h1400;
            10'h09A: O_bits = 16'h13F8;
            10'h09B: O_bits = 16'h0000;
            10'h09C: O_bits = 16'h0000;
            10'h09D: O_bits = 16'h0000;
            10'h09E: O_bits = 16'h0000;
            10'h09F: O_bits = 16'h0000;
            10'h0A0: O_bits = 16'h0880;
            10'h0A1: O_bits = 16'h08F0;
            10'h0A2: O_bits = 16'h1510;
            10'h0A3: O_bits = 16'h15E0;
            10'h0A4: O_bits = 16'h1BF8;
            10'h0A5: O_bits = 16'h0890;
            10'h0A6: O_bits = 16'h1370;
            10'h0A7: O_bits = 16'h1CB0;
            10'h0A8: O_bits = 16'h0370;
            10'h0A9: O_bits = 16'h1DAC;
            10'h0AA: O_bits = 16'h0260;
            10'h0AB: O_bits = 16'h0000;
            10'h0AC: O_bits = 16'h0000;
            10'h0AD: O_bits = 16'h0000;
            10'h0AE: O_bits = 16'h0000;
            10'h0AF: O_bits = 16'h0000;
            10'h0B0: O_bits = 16'h0840;
            10'h0B1: O_bits = 16'h0840;
            10'h0B2: O_bits = 16'h08A0;
            10'h0B3: O_bits = 16'h1F10;
            10'h0B4: O_bits = 16'h0BE8;
            10'h0B5: O_bits = 16'h0C50;
            10'h0B6: O_bits = 16'h1B50;
            10'h0B7: O_bits = 16'h2950;
            10'h0B8: O_bits = 16'h0890;
            10'h0B9: O_bits = 16'h0820;
            10'h0BA: O_bits = 16'h09D0;
            10'h0BB: O_bits = 16'h0000;
            10'h0BC: O_bits = 16'h0000;
            10'h0BD: O_bits = 16'h0000;
            10'h0BE: O_bits = 16'h0000;
            10'h0BF: O_bits = 16'h0000;
            10'h0C0: O_bits = 16'h0008;
            10'h0C1: O_bits = 16'h1BC8;
            10'h0C2: O_bits = 16'h0468;
            10'h0C3: O_bits = 16'h0568;
            10'h0C4: O_bits = 16'h1568;
            10'h0C5: O_bits = 16'h1568;
            10'h0C6: O_bits = 16'h0D68;
            10'h0C7: O_bits = 16'h0928;
            10'h0C8: O_bits = 16'h1188;
            10'h0C9: O_bits = 16'h12C8;
            10'h0CA: O_bits = 16'h0458;
            10'h0CB: O_bits = 16'h0000;
            10'h0CC: O_bits = 16'h0000;
            10'h0CD: O_bits = 16'h0000;
            10'h0CE: O_bits = 16'h0000;
            10'h0CF: O_bits = 16'h0000;
            10'h0D0: O_bits = 16'h0000;
            10'h0D1: O_bits = 16'h0FF0;
            10'h0D2: O_bits = 16'h0000;
            10'h0D3: O_bits = 16'h0000;
            10'h0D4: O_bits = 16'h0000;
            10'h0D5: O_bits = 16'h0FE0;
            10'h0D6: O_bits = 16'h0000;
            10'h0D7: O_bits = 16'h0000;
            10'h0D8: O_bits = 16'h0000;
            10'h0D9: O_bits = 16'h1FF8;
            10'h0DA: O_bits = 16'h0000;
            10'h0DB: O_bits = 16'h0000;
            10'h0DC: O_bits = 16'h0000;
            10'h0DD: O_bits = 16'h0000;
            10'h0DE: O_bits = 16'h0000;
            10'h0DF: O_bits = 16'h0000;
            10'h0E0: O_bits = 16'h0040;
            10'h0E1: O_bits = 16'h0440;
            10'h0E2: O_bits = 16'h0BF0;
            10'h0E3: O_bits = 16'h09F0;
            10'h0E4: O_bits = 16'h1910;
            10'h0E5: O_bits = 16'h19F0;
            10'h0E6: O_bits = 16'h0910;
            10'h0E7: O_bits = 16'h09F0;
            10'h0E8: O_bits = 16'h09F0;
            10'h0E9: O_bits = 16'h0910;
            10'h0EA: O_bits = 16'h0AE8;
            10'h0EB: O_bits = 16'h0000;
            10'h0EC: O_bits = 16'h0000;
            10'h0ED: O_bits = 16'h0000;
            10'h0EE: O_bits = 16'h0000;
            10'h0EF: O_bits = 16'h0000;
            10'h0F0: O_bits = 16'h0400;
            10'h0F1: O_bits = 16'h0480;
            10'h0F2: O_bits = 16'h0488;
            10'h0F3: O_bits = 16'h0890;
            10'h0F4: O_bits = 16'h18A0;
            10'h0F5: O_bits = 16'h18C0;
            10'h0F6: O_bits = 16'h0980;
            10'h0F7: O_bits = 16'h0B80;
            10'h0F8: O_bits = 16'h0888;
            10'h0F9: O_bits = 16'h0888;
            10'h0FA: O_bits = 16'h0878;
            10'h0FB: O_bits = 16'h0000;
            10'h0FC: O_bits = 16'h0000;
            10'h0FD: O_bits = 16'h0000;
            10'h0FE: O_bits = 16'h0000;
            10'h0FF: O_bits = 16'h0000;
            10'h100: O_bits = 16'h0000;
            10'h101: O_bits = 16'h1FF8;
            10'h102: O_bits = 16'h1288;
            10'h103: O_bits = 16'h1288;
            10'h104: O_bits = 16'h1288;
            10'h105: O_bits = 16'h1288;
            10'h106: O_bits = 16'h1468;
            10'h107: O_bits = 16'h1808;
            10'h108: O_bits = 16'h1008;
            10'h109: O_bits = 16'h1FF8;
            10'h10A: O_bits = 16'h1008;
            10'h10B: O_bits = 16'h0000;
            10'h10C: O_bits = 16'h0000;
            10'h10D: O_bits = 16'h0000;
            10'h10E: O_bits = 16'h0000;
            10'h10F: O_bits = 16'h0000;
            10'h110: O_bits = 16'h0000;
            10'h111: O_bits = 16'h09F0;
            10'h112: O_bits = 16'h0800;
            10'h113: O_bits = 16'h0000;
            10'h114: O_bits = 16'h1BF8;
            10'h115: O_bits = 16'h08A0;
            10'h116: O_bits = 16'h0910;
            10'h117: O_bits = 16'h0970;
            10'h118: O_bits = 16'h0A80;
            10'h119: O_bits = 16'h1E00;
            10'h11A: O_bits = 16'h11F8;
            10'h11B: O_bits = 16'h0000;
            10'h11C: O_bits = 16'h0000;
            10'h11D: O_bits = 16'h0000;
            10'h11E: O_bits = 16'h0000;
            10'h11F: O_bits = 16'h0000;
            10'h120: O_bits = 16'h0020;
            10'h121: O_bits = 16'h0020;
            10'h122: O_bits = 16'h1F20;
            10'h123: O_bits = 16'h00F8;
            10'h124: O_bits = 16'h1F28;
            10'h125: O_bits = 16'h0828;
            10'h126: O_bits = 16'h0A48;
            10'h127: O_bits = 16'h0948;
            10'h128: O_bits = 16'h1F48;
            10'h129: O_bits = 16'h0088;
            10'h12A: O_bits = 16'h0198;
            10'h12B: O_bits = 16'h0000;
            10'h12C: O_bits = 16'h0000;
            10'h12D: O_bits = 16'h0000;
            10'h12E: O_bits = 16'h0000;
            10'h12F: O_bits = 16'h0000;
            10'h130: O_bits = 16'h0000;
            10'h131: O_bits = 16'h1FF8;
            10'h132: O_bits = 16'h0100;
            10'h133: O_bits = 16'h0100;
            10'h134: O_bits = 16'h0100;
            10'h135: O_bits = 16'h07E0;
            10'h136: O_bits = 16'h0220;
            10'h137: O_bits = 16'h0220;
            10'h138: O_bits = 16'h0220;
            10'h139: O_bits = 16'h1FF8;
            10'h13A: O_bits = 16'h0000;
            10'h13B: O_bits = 16'h0000;
            10'h13C: O_bits = 16'h0000;
            10'h13D: O_bits = 16'h0000;
            10'h13E: O_bits = 16'h0000;
            10'h13F: O_bits = 16'h0000;
            10'h140: O_bits = 16'h0200;
            10'h141: O_bits = 16'h0200;
            10'h142: O_bits = 16'h05C0;
            10'h143: O_bits = 16'h0840;
            10'h144: O_bits = 16'h1FF0;
            10'h145: O_bits = 16'h0890;
            10'h146: O_bits = 16'h0FF0;
            10'h147: O_bits = 16'h0810;
            10'h148: O_bits = 16'h0800;
            10'h149: O_bits = 16'h0808;
            10'h14A: O_bits = 16'h07F0;
            10'h14B: O_bits = 16'h0000;
            10'h14C: O_bits = 16'h0000;
            10'h14D: O_bits = 16'h0000;
            10'h14E: O_bits = 16'h0000;
            10'h14F: O_bits = 16'h0000;
            10'h150: O_bits = 16'h0000;
            10'h151: O_bits = 16'h1F18;
            10'h152: O_bits = 16'h14B0;
            10'h153: O_bits = 16'h1540;
            10'h154: O_bits = 16'h0208;
            10'h155: O_bits = 16'h0410;
            10'h156: O_bits = 16'h1FE0;
            10'h157: O_bits = 16'h0508;
            10'h158: O_bits = 16'h0D98;
            10'h159: O_bits = 16'h14A0;
            10'h15A: O_bits = 16'h04C0;
            10'h15B: O_bits = 16'h0000;
            10'h15C: O_bits = 16'h0000;
            10'h15D: O_bits = 16'h0000;
            10'h15E: O_bits = 16'h0000;
            10'h15F: O_bits = 16'h0000;
            10'h160: O_bits = 16'h0010;
            10'h161: O_bits = 16'h0890;
            10'h162: O_bits = 16'h09F8;
            10'h163: O_bits = 16'h09D8;
            10'h164: O_bits = 16'h09D8;
            10'h165: O_bits = 16'h09F8;
            10'h166: O_bits = 16'h0800;
            10'h167: O_bits = 16'h09F8;
            10'h168: O_bits = 16'h1FF8;
            10'h169: O_bits = 16'h0108;
            10'h16A: O_bits = 16'h01F8;
            10'h16B: O_bits = 16'h0000;
            10'h16C: O_bits = 16'h0000;
            10'h16D: O_bits = 16'h0000;
            10'h16E: O_bits = 16'h0000;
            10'h16F: O_bits = 16'h0000;
            10'h170: O_bits = 16'h0000;
            10'h171: O_bits = 16'h1DF0;
            10'h172: O_bits = 16'h0510;
            10'h173: O_bits = 16'h1CE0;
            10'h174: O_bits = 16'h1040;
            10'h175: O_bits = 16'h11F8;
            10'h176: O_bits = 16'h1D48;
            10'h177: O_bits = 16'h05F8;
            10'h178: O_bits = 16'h0450;
            10'h179: O_bits = 16'h0478;
            10'h17A: O_bits = 16'h1B84;
            10'h17B: O_bits = 16'h0000;
            10'h17C: O_bits = 16'h0000;
            10'h17D: O_bits = 16'h0000;
            10'h17E: O_bits = 16'h0000;
            10'h17F: O_bits = 16'h0000;
            10'h180: O_bits = 16'h0100;
            10'h181: O_bits = 16'h0100;
            10'h182: O_bits = 16'h0FF0;
            10'h183: O_bits = 16'h0810;
            10'h184: O_bits = 16'h0FF0;
            10'h185: O_bits = 16'h0810;
            10'h186: O_bits = 16'h0810;
            10'h187: O_bits = 16'h0FF0;
            10'h188: O_bits = 16'h0810;
            10'h189: O_bits = 16'h0FF0;
            10'h18A: O_bits = 16'h0810;
            10'h18B: O_bits = 16'h0000;
            10'h18C: O_bits = 16'h0000;
            10'h18D: O_bits = 16'h0000;
            10'h18E: O_bits = 16'h0000;
            10'h18F: O_bits = 16'h0000;
            10'h190: O_bits = 16'h0010;
            10'h191: O_bits = 16'h11E0;
            10'h192: O_bits = 16'h0840;
            10'h193: O_bits = 16'h03F8;
            10'h194: O_bits = 16'h0040;
            10'h195: O_bits = 16'h09F0;
            10'h196: O_bits = 16'h0A10;
            10'h197: O_bits = 16'h0A10;
            10'h198: O_bits = 16'h09F0;
            10'h199: O_bits = 16'h0C00;
            10'h19A: O_bits = 16'h13F8;
            10'h19B: O_bits = 16'h0000;
            10'h19C: O_bits = 16'h0000;
            10'h19D: O_bits = 16'h0000;
            10'h19E: O_bits = 16'h0000;
            10'h19F: O_bits = 16'h0000;
            10'h1A0: O_bits = 16'h0080;
            10'h1A1: O_bits = 16'h0080;
            10'h1A2: O_bits = 16'h0FF8;
            10'h1A3: O_bits = 16'h0800;
            10'h1A4: O_bits = 16'h0890;
            10'h1A5: O_bits = 16'h0C90;
            10'h1A6: O_bits = 16'h0A50;
            10'h1A7: O_bits = 16'h0920;
            10'h1A8: O_bits = 16'h1020;
            10'h1A9: O_bits = 16'h1040;
            10'h1AA: O_bits = 16'h17B8;
            10'h1AB: O_bits = 16'h0000;
            10'h1AC: O_bits = 16'h0000;
            10'h1AD: O_bits = 16'h0000;
            10'h1AE: O_bits = 16'h0000;
            10'h1AF: O_bits = 16'h0000;
            10'h1B0: O_bits = 16'h0000;
            10'h1B1: O_bits = 16'h0000;
            10'h1B2: O_bits = 16'h0000;
            10'h1B3: O_bits = 16'h0000;
            10'h1B4: O_bits = 16'h0F00;
            10'h1B5: O_bits = 16'h0900;
            10'h1B6: O_bits = 16'h1080;
            10'h1B7: O_bits = 16'h1180;
            10'h1B8: O_bits = 16'h1680;
            10'h1B9: O_bits = 16'h1880;
            10'h1BA: O_bits = 16'h1080;
            10'h1BB: O_bits = 16'h0900;
            10'h1BC: O_bits = 16'h0F00;
            10'h1BD: O_bits = 16'h0000;
            10'h1BE: O_bits = 16'h0000;
            10'h1BF: O_bits = 16'h0000;
            10'h1C0: O_bits = 16'h0000;
            10'h1C1: O_bits = 16'h0000;
            10'h1C2: O_bits = 16'h0000;
            10'h1C3: O_bits = 16'h0000;
            10'h1C4: O_bits = 16'h0200;
            10'h1C5: O_bits = 16'h0600;
            10'h1C6: O_bits = 16'h0A00;
            10'h1C7: O_bits = 16'h0200;
            10'h1C8: O_bits = 16'h0200;
            10'h1C9: O_bits = 16'h0200;
            10'h1CA: O_bits = 16'h0200;
            10'h1CB: O_bits = 16'h0200;
            10'h1CC: O_bits = 16'h0F80;
            10'h1CD: O_bits = 16'h0000;
            10'h1CE: O_bits = 16'h0000;
            10'h1CF: O_bits = 16'h0000;
            10'h1D0: O_bits = 16'h0000;
            10'h1D1: O_bits = 16'h0000;
            10'h1D2: O_bits = 16'h0000;
            10'h1D3: O_bits = 16'h0000;
            10'h1D4: O_bits = 16'h0F00;
            10'h1D5: O_bits = 16'h1080;
            10'h1D6: O_bits = 16'h0080;
            10'h1D7: O_bits = 16'h0080;
            10'h1D8: O_bits = 16'h0100;
            10'h1D9: O_bits = 16'h0300;
            10'h1DA: O_bits = 16'h0600;
            10'h1DB: O_bits = 16'h0C00;
            10'h1DC: O_bits = 16'h1F80;
            10'h1DD: O_bits = 16'h0000;
            10'h1DE: O_bits = 16'h0000;
            10'h1DF: O_bits = 16'h0000;
            10'h1E0: O_bits = 16'h0000;
            10'h1E1: O_bits = 16'h0000;
            10'h1E2: O_bits = 16'h0000;
            10'h1E3: O_bits = 16'h0000;
            10'h1E4: O_bits = 16'h1E00;
            10'h1E5: O_bits = 16'h0100;
            10'h1E6: O_bits = 16'h0100;
            10'h1E7: O_bits = 16'h0100;
            10'h1E8: O_bits = 16'h0E00;
            10'h1E9: O_bits = 16'h0100;
            10'h1EA: O_bits = 16'h0100;
            10'h1EB: O_bits = 16'h0100;
            10'h1EC: O_bits = 16'h1E00;
            10'h1ED: O_bits = 16'h0000;
            10'h1EE: O_bits = 16'h0000;
            10'h1EF: O_bits = 16'h0000;
            10'h1F0: O_bits = 16'h0000;
            10'h1F1: O_bits = 16'h0000;
            10'h1F2: O_bits = 16'h0000;
            10'h1F3: O_bits = 16'h0000;
            10'h1F4: O_bits = 16'h0600;
            10'h1F5: O_bits = 16'h0600;
            10'h1F6: O_bits = 16'h0A00;
            10'h1F7: O_bits = 16'h1A00;
            10'h1F8: O_bits = 16'h1200;
            10'h1F9: O_bits = 16'h2200;
            10'h1FA: O_bits = 16'h3F80;
            10'h1FB: O_bits = 16'h0200;
            10'h1FC: O_bits = 16'h0200;
            10'h1FD: O_bits = 16'h0000;
            10'h1FE: O_bits = 16'h0000;
            10'h1FF: O_bits = 16'h0000;
            10'h200: O_bits = 16'h0000;
            10'h201: O_bits = 16'h0000;
            10'h202: O_bits = 16'h0000;
            10'h203: O_bits = 16'h0000;
            10'h204: O_bits = 16'h1F00;
            10'h205: O_bits = 16'h1000;
            10'h206: O_bits = 16'h1000;
            10'h207: O_bits = 16'h1000;
            10'h208: O_bits = 16'h1E00;
            10'h209: O_bits = 16'h0100;
            10'h20A: O_bits = 16'h0100;
            10'h20B: O_bits = 16'h0300;
            10'h20C: O_bits = 16'h1E00;
            10'h20D: O_bits = 16'h0000;
            10'h20E: O_bits = 16'h0000;
            10'h20F: O_bits = 16'h0000;
            10'h210: O_bits = 16'h0000;
            10'h211: O_bits = 16'h0000;
            10'h212: O_bits = 16'h0000;
            10'h213: O_bits = 16'h0000;
            10'h214: O_bits = 16'h0700;
            10'h215: O_bits = 16'h0800;
            10'h216: O_bits = 16'h1000;
            10'h217: O_bits = 16'h1700;
            10'h218: O_bits = 16'h1880;
            10'h219: O_bits = 16'h1080;
            10'h21A: O_bits = 16'h1080;
            10'h21B: O_bits = 16'h1980;
            10'h21C: O_bits = 16'h0F00;
            10'h21D: O_bits = 16'h0000;
            10'h21E: O_bits = 16'h0000;
            10'h21F: O_bits = 16'h0000;
            10'h220: O_bits = 16'h0000;
            10'h221: O_bits = 16'h0000;
            10'h222: O_bits = 16'h0000;
            10'h223: O_bits = 16'h0000;
            10'h224: O_bits = 16'h1F80;
            10'h225: O_bits = 16'h0080;
            10'h226: O_bits = 16'h0100;
            10'h227: O_bits = 16'h0100;
            10'h228: O_bits = 16'h0200;
            10'h229: O_bits = 16'h0200;
            10'h22A: O_bits = 16'h0400;
            10'h22B: O_bits = 16'h0400;
            10'h22C: O_bits = 16'h0C00;
            10'h22D: O_bits = 16'h0000;
            10'h22E: O_bits = 16'h0000;
            10'h22F: O_bits = 16'h0000;
            10'h230: O_bits = 16'h0000;
            10'h231: O_bits = 16'h0000;
            10'h232: O_bits = 16'h0000;
            10'h233: O_bits = 16'h0000;
            10'h234: O_bits = 16'h0F00;
            10'h235: O_bits = 16'h1080;
            10'h236: O_bits = 16'h1080;
            10'h237: O_bits = 16'h1980;
            10'h238: O_bits = 16'h0600;
            10'h239: O_bits = 16'h1980;
            10'h23A: O_bits = 16'h1080;
            10'h23B: O_bits = 16'h1080;
            10'h23C: O_bits = 16'h0F00;
            10'h23D: O_bits = 16'h0000;
            10'h23E: O_bits = 16'h0000;
            10'h23F: O_bits = 16'h0000;
            10'h240: O_bits = 16'h0000;
            10'h241: O_bits = 16'h0000;
            10'h242: O_bits = 16'h0000;
            10'h243: O_bits = 16'h0000;
            10'h244: O_bits = 16'h0F00;
            10'h245: O_bits = 16'h1980;
            10'h246: O_bits = 16'h1080;
            10'h247: O_bits = 16'h1080;
            10'h248: O_bits = 16'h1180;
            10'h249: O_bits = 16'h0E80;
            10'h24A: O_bits = 16'h0080;
            10'h24B: O_bits = 16'h0100;
            10'h24C: O_bits = 16'h0E00;
            10'h24D: O_bits = 16'h0000;
            10'h24E: O_bits = 16'h0000;
            10'h24F: O_bits = 16'h0000;
            10'h250: O_bits = 16'h0000;
            10'h251: O_bits = 16'h0000;
            10'h252: O_bits = 16'h0000;
            10'h253: O_bits = 16'h0000;
            10'h254: O_bits = 16'h0000;
            10'h255: O_bits = 16'h0000;
            10'h256: O_bits = 16'h0000;
            10'h257: O_bits = 16'h0000;
            10'h258: O_bits = 16'h0000;
            10'h259: O_bits = 16'h0000;
            10'h25A: O_bits = 16'h0000;
            10'h25B: O_bits = 16'h0600;
            10'h25C: O_bits = 16'h0600;
            10'h25D: O_bits = 16'h0000;
            10'h25E: O_bits = 16'h0000;
            10'h25F: O_bits = 16'h0000;
            10'h260: O_bits = 16'h0000;
            10'h261: O_bits = 16'h0000;
            10'h262: O_bits = 16'h0000;
            10'h263: O_bits = 16'h0000;
            10'h264: O_bits = 16'h0000;
            10'h265: O_bits = 16'h0000;
            10'h266: O_bits = 16'h3F80;
            10'h267: O_bits = 16'h3680;
            10'h268: O_bits = 16'h2480;
            10'h269: O_bits = 16'h2480;
            10'h26A: O_bits = 16'h2480;
            10'h26B: O_bits = 16'h2480;
            10'h26C: O_bits = 16'h2480;
            10'h26D: O_bits = 16'h0000;
            10'h26E: O_bits = 16'h0000;
            10'h26F: O_bits = 16'h0000;
            10'h270: O_bits = 16'h0000;
            10'h271: O_bits = 16'h0000;
            10'h272: O_bits = 16'h0000;
            10'h273: O_bits = 16'h0000;
            10'h274: O_bits = 16'h0000;
            10'h275: O_bits = 16'h0000;
            10'h276: O_bits = 16'h0F00;
            10'h277: O_bits = 16'h1000;
            10'h278: O_bits = 16'h1000;
            10'h279: O_bits = 16'h0E00;
            10'h27A: O_bits = 16'h0100;
            10'h27B: O_bits = 16'h0100;
            10'h27C: O_bits = 16'h1E00;
            10'h27D: O_bits = 16'h0000;
            10'h27E: O_bits = 16'h0000;
            10'h27F: O_bits = 16'h0000;
            10'h280: O_bits = 16'h0200;
            10'h281: O_bits = 16'h0200;
            10'h282: O_bits = 16'h1FF8;
            10'h283: O_bits = 16'h0240;
            10'h284: O_bits = 16'h0440;
            10'h285: O_bits = 16'h0650;
            10'h286: O_bits = 16'h0A50;
            10'h287: O_bits = 16'h18E0;
            10'h288: O_bits = 16'h10A0;
            10'h289: O_bits = 16'h0310;
            10'h28A: O_bits = 16'h0E08;
            10'h28B: O_bits = 16'h0000;
            10'h28C: O_bits = 16'h0000;
            10'h28D: O_bits = 16'h0000;
            10'h28E: O_bits = 16'h0000;
            10'h28F: O_bits = 16'h0000;
            10'h290: O_bits = 16'h0080;
            10'h291: O_bits = 16'h0088;
            10'h292: O_bits = 16'h0F70;
            10'h293: O_bits = 16'h0920;
            10'h294: O_bits = 16'h0FF8;
            10'h295: O_bits = 16'h0920;
            10'h296: O_bits = 16'h10C0;
            10'h297: O_bits = 16'h17E0;
            10'h298: O_bits = 16'h1120;
            10'h299: O_bits = 16'h10C0;
            10'h29A: O_bits = 16'h1F38;
            10'h29B: O_bits = 16'h0000;
            10'h29C: O_bits = 16'h0000;
            10'h29D: O_bits = 16'h0000;
            10'h29E: O_bits = 16'h0000;
            10'h29F: O_bits = 16'h0000;
            default: O_bits = 16'h0000;
        endcase
    end
endmodule

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

`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_top
//
// Display-path select for the 96-bit ISP stream (4 x RGB888 per beat).
//
//   I_mode = 0  camera passthrough
//   I_mode = 1  Scharr edge detection, black background with white edges
//
// I_mode is a level, not a toggle: it is driven straight from the debounced
// SW1 input, so holding the switch up keeps edge detection on.
//
// vis_edge_96 registers its output, so the two branches have different
// latency. Only one branch is selected at a time and the downstream DDR writer
// only needs valid data with aligned tuser/tlast, so the constant latency
// difference is harmless.
//----------------------------------------------------------------------------
module vis_top #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    // Fixed threshold for the non-adaptive edge mode. Scharr weights sum to
    // 16. 260 is the compromise between the noisy 220 setting and the sparse
    // 300 setting; the Gaussian prefilter and NMS suppress texture noise.
    parameter [14:0] EDGE_TH = 15'd260
)(
    input               I_clk,
    input               I_rst_n,

    input               I_tuser,
    input               I_tlast,
    input               I_tvalid,
    input      [95:0]   I_tdata,
    input               I_tready,

    input      [2:0]    I_mode,       // 0 pass, 1 edge, 2 binarize, 3 motion, 4 colour

    output     [95:0]   O_tdata,
    output              O_tlast,
    output              O_tuser,
    output              O_tvalid,
    output              O_tready,

    // Motion results. The box is drawn in the display clock domain (see
    // vis_text_osd_disp) so it does not jitter; the alarm drives the LEDs.
    output              O_motion_alarm,
    output              O_motion_any,
    output     [15:0]   O_box_x0,
    output     [15:0]   O_box_y0,
    output     [15:0]   O_box_x1,
    output     [15:0]   O_box_y1,
    output              O_low_light
);

    wire [95:0] gauss_tdata;
    wire        gauss_tlast, gauss_tuser, gauss_tvalid, gauss_tready;

    vis_gauss3_96 #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT)
    ) u_gauss (
        .I_clk    (I_clk),
        .I_rst_n  (I_rst_n),
        .I_tlast  (I_tlast),
        .I_tuser  (I_tuser),
        .I_tdata  (I_tdata),
        .I_tvalid (I_tvalid),
        .I_tready (I_tready),
        .O_tdata  (gauss_tdata),
        .O_tlast  (gauss_tlast),
        .O_tuser  (gauss_tuser),
        .O_tvalid (gauss_tvalid),
        .O_tready (gauss_tready)
    );

    wire [95:0] edge_tdata;
    wire        edge_tlast, edge_tuser, edge_tvalid, edge_tready;

    // The non-adaptive edge mode uses EDGE_TH. Otsu remains available for
    // diagnostics, but it is intentionally not connected to this mode.
    wire [7:0]  otsu_thresh;

    vis_edge_96 #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT)
    ) u_edge (
        .I_clk    (I_clk),
        .I_rst_n  (I_rst_n),
        .I_tlast  (gauss_tlast),
        .I_tuser  (gauss_tuser),
        .I_tdata  (gauss_tdata),
        .I_tvalid (gauss_tvalid),
        .I_tready (I_tready),
        .I_edge_th(EDGE_TH),
        .O_tlast  (edge_tlast),
        .O_tuser  (edge_tuser),
        .O_tdata  (edge_tdata),
        .O_tvalid (edge_tvalid),
        .O_tready (edge_tready)
    );

    // Grayscale output (mode 3). Fixed-threshold binarization was dropped: the
    // adaptive version in mode 5 supersedes it.
    wire [95:0] gry_tdata;
    wire        gry_tlast, gry_tuser, gry_tvalid, gry_tready;

    vis_gray u_gray (
        .I_clk    (I_clk),
        .I_rst_n  (I_rst_n),
        .I_tuser  (I_tuser),
        .I_tlast  (I_tlast),
        .I_tvalid (I_tvalid),
        .I_tdata  (I_tdata),
        .I_tready (I_tready),
        .O_tdata  (gry_tdata),
        .O_tlast  (gry_tlast),
        .O_tuser  (gry_tuser),
        .O_tvalid (gry_tvalid),
        .O_tready (gry_tready)
    );

    // Colour enhancement. Mode 5 selects it; the other passthrough modes stay
    // untouched so the earlier verified look is preserved.
    wire [95:0] enh_tdata;
    wire        enh_tlast, enh_tuser, enh_tvalid;
    vis_color_enh u_color (
        .I_clk    (I_clk),
        .I_rst_n  (I_rst_n),
        .I_tuser  (I_tuser),
        .I_tlast  (I_tlast),
        .I_tvalid (I_tvalid),
        .I_tdata  (I_tdata),
        .I_tready (I_tready),
        .O_tdata  (enh_tdata),
        .O_tlast  (enh_tlast),
        .O_tuser  (enh_tuser),
        .O_tvalid (enh_tvalid),
        .O_tready ()
    );

    // NOTE: the USM sharpening stage is DISABLED on purpose.
    // It was the critical path (SWNS -0.309 ns, and 41% of that path was
    // routing because the device sits at 76% LUT). Everything else in this
    // chain has been verified on the board, so sharpening is the one feature
    // to give up: it only affects perceived crispness, while the timing
    // failure risks the whole demo.
    // To re-enable: re-instantiate vis_sharpen (see the git history / the
    // standalone vis_sharpen.v) and point pass_tdata at its output.
    wire sel_edge = (I_mode == 3'd1);
    wire sel_gray = (I_mode == 3'd2);

    // Contrast stays in the raw-image path. The zero-offset brightness stage
    // was removed: it only added a 12-channel register pipe and did not change
    // any pixel value.
    wire [95:0] ct_tdata;
    wire        ct_tlast, ct_tuser, ct_tvalid, ct_tready;

    vis_contrast #(
        .GAIN (4'd5)             // 1.25x; 4=1.0x, 6=1.5x, 8=2.0x
    ) u_contrast (
        .I_clk    (I_clk),
        .I_rst_n  (I_rst_n),
        .I_tlast  (enh_tlast),
        .I_tuser  (enh_tuser),
        .I_tdata  (enh_tdata),
        .I_tvalid (enh_tvalid),
        .I_tready (ct_tready),
        .O_tlast  (ct_tlast),
        .O_tuser  (ct_tuser),
        .O_tdata  (ct_tdata),
        .O_tvalid (ct_tvalid),
        .O_tready (I_tready)
    );

    wire [95:0] pass_tdata = ct_tdata;

    // ------------------------------------------------------------------
    // Adaptive binarization (mode 5). Same binarizer as mode 3, but its
    // threshold comes from a per-frame Otsu engine instead of a constant, so
    // strong or dim lighting no longer breaks the black/white split. The Otsu
    // engine only runs during vertical blanking, so it costs no pixel-path
    // latency.
    // ------------------------------------------------------------------
    wire [5:0]  otsu_hist_addr;
    wire [20:0] otsu_hist_data;
    wire        otsu_stats_done;
    wire [31:0] stats_luma_sum;

    vis_frame_stats #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT)
    ) u_stats (
        .I_clk          (I_clk),
        .I_rst_n        (I_rst_n),
        .I_tuser        (I_tuser),
        .I_tlast        (I_tlast),
        .I_tvalid       (I_tvalid),
        .I_tdata        (I_tdata),
        .I_hist_rd_addr (otsu_hist_addr),
        .O_hist_rd_data (otsu_hist_data),
        .O_frame_done   (otsu_stats_done),
        .O_r_sum        (),
        .O_g_sum        (),
        .O_b_sum        (),
        .O_luma_sum     (stats_luma_sum),
        .O_pixel_cnt    ()
    );

    reg low_light;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            low_light <= 1'b0;
        else if(otsu_stats_done)
            low_light <= ((stats_luma_sum >> 20) < 8'd35);
    end
    assign O_low_light = low_light;

    vis_otsu_thresh #(
        .BIN_BITS  (6),
        .HIST_BITS (21)
    ) u_otsu (
        .I_clk       (I_clk),
        .I_rst_n     (I_rst_n),
        .I_start     (otsu_stats_done),
        .O_hist_addr (otsu_hist_addr),
        .I_hist_data (otsu_hist_data),
        .O_thresh    (otsu_thresh),
        .O_valid     (),
        .O_busy      ()
    );

    wire [95:0] adbin_tdata;
    wire        adbin_tlast, adbin_tuser, adbin_tvalid, adbin_tready;
    vis_binarize u_adbin (
        .I_clk    (I_clk),
        .I_rst_n  (I_rst_n),
        .I_tuser  (I_tuser),
        .I_tlast  (I_tlast),
        .I_tvalid (I_tvalid),
        .I_tdata  (I_tdata),
        .I_tready (I_tready),
        .I_thresh (otsu_thresh),
        .O_tdata  (adbin_tdata),
        .O_tlast  (adbin_tlast),
        .O_tuser  (adbin_tuser),
        .O_tvalid (adbin_tvalid),
        .O_tready (adbin_tready)
    );

    // Mode 5 shows the adaptively corrected colour picture, NOT a binarized
    // one. Dropping this branch also leaves the Otsu engine without a consumer,
    // so the synthesiser removes it and the LUT count falls.
    // Digital automatic brightness (mode 5). The per-frame luma sum walks a
    // 2-bit gain up in dim light and down in bright light, so the mode really
    // adapts rather than just carrying an "adaptive" label.
    wire [95:0] ag_tdata;
    wire        ag_tlast, ag_tuser, ag_tvalid, ag_tready;
    wire [1:0]  ag_level;
    wire        sel_ag = (I_mode == 3'd4);

    vis_auto_gain u_autogain (
        .I_clk        (I_clk),
        .I_rst_n      (I_rst_n),
        .I_tuser      (enh_tuser),
        .I_tlast      (enh_tlast),
        .I_tvalid     (enh_tvalid),
        .I_tdata      (enh_tdata),
        .I_tready     (I_tready),
        .I_luma_sum   (stats_luma_sum),
        .I_frame_done (otsu_stats_done),
        .O_tdata      (ag_tdata),
        .O_tlast      (ag_tlast),
        .O_tuser      (ag_tuser),
        .O_tvalid     (ag_tvalid),
        .O_tready     (ag_tready),
        .O_level      (ag_level)
    );

    wire [95:0] sel_tdata  = sel_ag    ? ag_tdata     :
                             sel_gray  ? gry_tdata    :
                             sel_edge  ? edge_tdata   : pass_tdata;
    wire        sel_tlast  = sel_ag    ? ag_tlast     :
                             sel_gray  ? gry_tlast    :
                             sel_edge  ? edge_tlast   : ct_tlast;
    wire        sel_tuser  = sel_ag    ? ag_tuser     :
                             sel_gray  ? gry_tuser    :
                             sel_edge  ? edge_tuser   : ct_tuser;
    wire        sel_tvalid = sel_ag    ? ag_tvalid    :
                             sel_gray  ? gry_tvalid   :
                             sel_edge  ? edge_tvalid  : ct_tvalid;

    // Motion detector. It always runs; mode 4 only decides whether the box is
    // shown. The video itself stays a passthrough in that mode.
    vis_motion_grid #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT),
        .BLK        (20)
    ) u_motion (
        .I_clk           (I_clk),
        .I_rst_n         (I_rst_n),
        .I_tuser         (I_tuser),
        .I_tlast         (I_tlast),
        .I_tvalid        (I_tvalid),
        .I_tdata         (I_tdata),
        .O_frame_done    (),
        .O_any_motion    (O_motion_any),
        .O_box_x0        (O_box_x0),
        .O_box_y0        (O_box_y0),
        .O_box_x1        (O_box_x1),
        .O_box_y1        (O_box_y1),
        .O_motion_blocks ()
    );

    // Reaching the outermost block band counts as hitting the frame edge.
    wire at_left   = (O_box_x0 == 16'd0);
    wire at_top    = (O_box_y0 == 16'd0);
    wire at_right  = (O_box_x1 >= (IMG_WIDTH  - 16));
    wire at_bottom = (O_box_y1 >= (IMG_HEIGHT - 16));

    assign O_motion_alarm = O_motion_any &&
                            (at_left | at_top | at_right | at_bottom);

    // The mode banner is drawn in the display clock domain instead - see
    // vis_text_osd_disp - so its position does not jitter with the MIPI
    // frame start.
    assign O_tdata  = sel_tdata;
    assign O_tlast  = sel_tlast;
    assign O_tuser  = sel_tuser;
    assign O_tvalid = sel_tvalid;
    assign O_tready = I_tready;

endmodule
