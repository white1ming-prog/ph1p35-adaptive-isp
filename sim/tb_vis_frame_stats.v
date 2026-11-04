`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// tb_vis_frame_stats
//
// Frame 1 uses a gradient where R = G = B = 4*i for pixel index i, so the
// 1/4 + 1/2 + 1/4 luma is exactly 4*i and every one of the 64 bins receives
// exactly one pixel. Frame 2 is all-black, which catches cross-frame
// accumulation: the published histogram must be {64, 0, 0, ...}.
//----------------------------------------------------------------------------
module tb_vis_frame_stats;

    localparam IMG_WIDTH  = 16;
    localparam IMG_HEIGHT = 4;
    localparam BIN_BITS   = 6;
    localparam CNT_BITS   = 20;
    localparam SUM_BITS   = 32;
    localparam BINS       = (1 << BIN_BITS);
    localparam BEATS_LINE = IMG_WIDTH / 4;

    reg                 clk = 1'b0;
    reg                 rst_n = 1'b0;
    reg                 tuser = 1'b0;
    reg                 tlast = 1'b0;
    reg                 tvalid = 1'b0;
    reg  [95:0]         tdata = 96'd0;
    reg  [BIN_BITS-1:0] rd_addr = {BIN_BITS{1'b0}};

    wire [CNT_BITS:0]   rd_data;
    wire                frame_done;
    wire [SUM_BITS-1:0] r_sum, g_sum, b_sum, y_sum;
    wire [CNT_BITS:0]   pix_cnt;

    integer errors = 0;
    integer mode;
    integer m, k, i;
    integer b;
    reg [7:0] v;
    reg [95:0] d;

    vis_frame_stats #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT),
        .BIN_BITS   (BIN_BITS),
        .CNT_BITS   (CNT_BITS),
        .SUM_BITS   (SUM_BITS)
    ) dut (
        .I_clk          (clk),
        .I_rst_n        (rst_n),
        .I_tuser        (tuser),
        .I_tlast        (tlast),
        .I_tvalid       (tvalid),
        .I_tdata        (tdata),
        .I_hist_rd_addr (rd_addr),
        .O_hist_rd_data (rd_data),
        .O_frame_done   (frame_done),
        .O_r_sum        (r_sum),
        .O_g_sum        (g_sum),
        .O_b_sum        (b_sum),
        .O_luma_sum     (y_sum),
        .O_pixel_cnt    (pix_cnt)
    );

    always #5 clk = ~clk;   // 100 MHz behavioural clock

    // Pixel value for a given index and frame mode.
    function [7:0] pval;
        input integer idx;
        input integer md;
        begin
            pval = (md == 0) ? (idx * 4) : 8'd0;
        end
    endfunction

    // One valid beat, held for a single clock.
    task send_beat;
        input [95:0] dat;
        input        usr;
        input        lst;
        begin
            @(negedge clk);
            tdata  = dat;
            tuser  = usr;
            tlast  = lst;
            tvalid = 1'b1;
            @(negedge clk);
            tvalid = 1'b0;
            tuser  = 1'b0;
            tlast  = 1'b0;
        end
    endtask

    task drive_frame;
        input integer md;
        begin
            for (m = 0; m < IMG_HEIGHT * BEATS_LINE; m = m + 1) begin
                for (k = 0; k < 4; k = k + 1) begin
                    v = pval(4*m + k, md);
                    case (k)
                        0: d[95:72] = {v, v, v};
                        1: d[71:48] = {v, v, v};
                        2: d[47:24] = {v, v, v};
                        3: d[23:0]  = {v, v, v};
                    endcase
                end
                send_beat(d, (m == 0), ((m % BEATS_LINE) == BEATS_LINE - 1));
            end
            // Vertical blanking: long enough for the 64-cycle bank clear.
            repeat (120) @(negedge clk);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        // Let the power-on clear of both bank sets finish.
        repeat (160) @(negedge clk);

        // ---------------- frame 1 : gradient ------------------------------
        fork
            drive_frame(0);
            begin
                @(posedge frame_done);
                if (pix_cnt !== 64) begin
                    errors = errors + 1;
                    $display("FAIL frame1 pixel count: got %0d expected 64", pix_cnt);
                end
                if (r_sum !== 32'd8064 || g_sum !== 32'd8064 ||
                    b_sum !== 32'd8064 || y_sum !== 32'd8064) begin
                    errors = errors + 1;
                    $display("FAIL frame1 sums: R=%0d G=%0d B=%0d Y=%0d expected 8064",
                             r_sum, g_sum, b_sum, y_sum);
                end
                for (i = 0; i < BINS; i = i + 1) begin
                    rd_addr = i[BIN_BITS-1:0];
                    #1;
                    if (rd_data !== 21'd1) begin
                        errors = errors + 1;
                        $display("FAIL frame1 hist[%0d]=%0d expected 1", i, rd_data);
                    end
                end
            end
        join

        // ---------------- frame 2 : all black -----------------------------
        fork
            drive_frame(1);
            begin
                @(posedge frame_done);
                if (pix_cnt !== 64 || r_sum !== 0 || y_sum !== 0) begin
                    errors = errors + 1;
                    $display("FAIL frame2 totals: cnt=%0d R=%0d Y=%0d",
                             pix_cnt, r_sum, y_sum);
                end
                for (i = 0; i < BINS; i = i + 1) begin
                    rd_addr = i[BIN_BITS-1:0];
                    #1;
                    b = (i == 0) ? 21'd64 : 21'd0;
                    if (rd_data !== b) begin
                        errors = errors + 1;
                        $display("FAIL frame2 hist[%0d]=%0d expected %0d", i, rd_data, b);
                    end
                end
            end
        join

        if (errors == 0)
            $display("RESULT: PASS");
        else
            $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    // Safety net so a hang does not run forever.
    initial begin
        #500000;
        $display("RESULT: TIMEOUT");
        $finish;
    end

endmodule
