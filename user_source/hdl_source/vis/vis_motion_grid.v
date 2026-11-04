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
