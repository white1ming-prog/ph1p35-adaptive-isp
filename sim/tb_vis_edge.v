`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// tb_vis_edge
//
// Drives vis_edge_96 with a synthetic frame. Line y carries exactly one
// vertical edge at column y, so all four group offsets are exercised.
//
// Scharr 在阶跃两侧的 f-1 / f 两列产生相同幅值。方向 NMS 比较左右邻后，
// 只保留更靠左的 f-1，因此一个阶跃得到单像素线：
//   - 每行第 0 组的第一列 (列 0) 没有左邻居, 恒被屏蔽; 最后一列同理
//   - 前两行没有完整的三行窗口, 恒黑
// 期望值由 sim/ref_edge_model.py 给出的周期精确参考模型核对过 (本机 vsim
// 没有 license, 只能用该模型代替仿真)。
//----------------------------------------------------------------------------
module tb_vis_edge;

    localparam IMG_WIDTH  = 64;
    localparam IMG_HEIGHT = 16;
    localparam GRP_W      = IMG_WIDTH / 4;
    localparam TH         = 15'd60;

    reg         clk = 1'b0;
    reg         rst_n = 1'b0;

    reg  [95:0] i_tdata  = 96'd0;
    reg         i_tvalid = 1'b0;
    reg         i_tlast  = 1'b0;
    reg         i_tuser  = 1'b0;
    reg  [14:0] i_th     = TH;

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
        .I_edge_th(i_th),
        .O_tlast  (o_tlast),
        .O_tuser  (o_tuser),
        .O_tdata  (o_tdata),
        .O_tvalid (o_tvalid),
        .O_tready (o_tready)
    );

    // ---- stimulus ---------------------------------------------------------
    integer i, y, g, p;
    reg [7:0] v;

    function [23:0] grey;
        input [7:0] l;
        begin
            grey = {l, l, l};
        end
    endfunction

    integer f;

    initial begin
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        @(negedge clk);

        // One frame per edge column. Inside a frame every line has the same
        // edge column, otherwise the row-to-row difference would create
        // horizontal gradients and fake edges.
        for (f = 0; f < 16; f = f + 1) begin
            for (i = 0; i < IMG_HEIGHT * GRP_W; i = i + 1) begin
                y = i / GRP_W;
                g = i % GRP_W;

                i_tvalid = 1'b1;
                i_tuser  = (i == 0);
                i_tlast  = (g == GRP_W - 1);

                i_tdata = 96'd0;
                for (p = 0; p < 4; p = p + 1) begin
                    v = ((g * 4 + p) < f) ? 8'd10 : 8'd220;
                    i_tdata[24*(3-p) +: 24] = grey(v);   // p=0 is the leftmost pixel = tdata[95:72]
                end

                @(negedge clk);
            end

            // idle gap between frames: the module delays the stream by three
            // beats, so without a gap the last beats of one frame would be
            // counted with the first beats of the next one.
            i_tvalid = 1'b0;
            i_tlast  = 1'b0;
            i_tuser  = 1'b0;
            i_tdata  = 96'd0;
            repeat (5) @(negedge clk);
        end

        i_tvalid = 1'b0;
        i_tlast  = 1'b0;
        i_tuser  = 1'b0;
        i_tdata  = 96'd0;

        repeat (12) @(negedge clk);
        $display("");
        $display("RESULT: %s  lines=%0d beats=%0d tuser=%0d tlast=%0d mismatch=%0d",
                 (fails == 0) ? "PASS" : "FAIL",
                 lines, beats, n_tuser, n_tlast, fails);
        $finish;
    end

    // ---- checker ----------------------------------------------------------
    integer lines = 0, beats = 0, n_tuser = 0, n_tlast = 0, fails = 0;
    integer outb = 0, nwhite = 0, base, ec, lif, pc;
    reg [63:0] mask = 64'd0, exp = 64'd0;
    reg [23:0] px;

    always @(posedge clk) begin
        if (o_tvalid === 1'b1) begin
            beats = beats + 1;
            if (o_tuser) n_tuser = n_tuser + 1;

            // dedicated loop variable: the stimulus block uses p as well
            for (pc = 0; pc < 4; pc = pc + 1) begin
                px = o_tdata[24*(3-pc) +: 24];  // pc=0 is the leftmost pixel
                if (px !== 24'd0) begin
                    nwhite = nwhite + 1;
                    mask[outb*4 + pc] = 1'b1;
                end
            end

            if (o_tlast) begin
                ec   = lines / IMG_HEIGHT;              // edge column of this frame
                lif  = lines % IMG_HEIGHT;              // line within the frame

                exp = 64'd0;
                // 前两行没有完整 3x3 窗口; 列 0 没有左邻居, 最后一列没有右邻居,
                // 都被窗口有效性屏蔽掉。NMS 后只保留阶跃左侧一列。
                if (lif >= 2) begin
                    if ((ec >= 1) && (ec != IMG_WIDTH - 1))
                        exp[ec - 1] = 1'b1;
                end
                if (mask !== exp) fails = fails + 1;

                if (mask !== exp)
                    $display("frame %2d line %2d (edge@%2d): mask=%h exp=%h  %s",
                             ec, lif, ec, mask, exp,
                             (mask === exp) ? "ok" : "MISMATCH");

                lines   = lines + 1;
                n_tlast = n_tlast + 1;
                mask    = 64'd0;
                nwhite  = 0;
                outb    = 0;
            end else begin
                outb = outb + 1;
            end
        end
    end

endmodule
