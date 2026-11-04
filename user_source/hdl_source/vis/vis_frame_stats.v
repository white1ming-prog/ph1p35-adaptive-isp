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
