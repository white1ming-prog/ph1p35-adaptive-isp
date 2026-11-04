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
