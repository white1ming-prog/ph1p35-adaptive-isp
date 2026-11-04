`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// tb_vis_motion_grid
//
// Small grid (64x32 pixels, 16x16 blocks -> 4x2 blocks of 64). Frame 1 is a
// flat background and only fills the reference buffer. Frame 2 brightens the
// block at (xb=1, yb=0), so exactly one block must be reported as moving with
// a box of x 16..31, y 0..15.
//----------------------------------------------------------------------------
module tb_vis_motion_grid;

    localparam IMG_WIDTH  = 64;
    localparam IMG_HEIGHT = 32;
    localparam BLK        = 16;
    localparam BEATS      = IMG_WIDTH / 4;

    reg          clk = 1'b0;
    reg          rst_n = 1'b0;
    reg          tuser = 1'b0;
    reg          tlast = 1'b0;
    reg          tvalid = 1'b0;
    reg  [95:0]  tdata = 96'd0;

    wire         frame_done;
    wire         any_motion;
    wire [15:0]  box_x0, box_y0, box_x1, box_y1;
    wire [15:0]  motion_blocks;

    integer errors = 0;
    integer m, j, ky, kx;
    integer mode;
    integer x, yy;
    reg [7:0] v;
    reg [95:0] d;

    vis_motion_grid #(
        .IMG_WIDTH  (IMG_WIDTH),
        .IMG_HEIGHT (IMG_HEIGHT),
        .BLK        (BLK),
        .DIFF_TH    (8'd10)
    ) dut (
        .I_clk           (clk),
        .I_rst_n         (rst_n),
        .I_tuser         (tuser),
        .I_tlast         (tlast),
        .I_tvalid        (tvalid),
        .I_tdata         (tdata),
        .O_frame_done    (frame_done),
        .O_any_motion    (any_motion),
        .O_box_x0        (box_x0),
        .O_box_y0        (box_y0),
        .O_box_x1        (box_x1),
        .O_box_y1        (box_y1),
        .O_motion_blocks (motion_blocks)
    );

    always #5 clk = ~clk;

    function [7:0] pixel_value;
        input integer xp;
        input integer yp;
        input integer md;
        begin
            if (md == 1 && xp >= 16 && xp <= 31 && yp >= 0 && yp <= 15)
                pixel_value = 8'd204;
            else
                pixel_value = 8'd52;
        end
    endfunction

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
            for (yy = 0; yy < IMG_HEIGHT; yy = yy + 1) begin
                for (m = 0; m < BEATS; m = m + 1) begin
                    for (j = 0; j < 4; j = j + 1) begin
                        x = m * 4 + j;
                        v = pixel_value(x, yy, md);
                        case (j)
                            0: d[95:72] = {v, v, v};
                            1: d[71:48] = {v, v, v};
                            2: d[47:24] = {v, v, v};
                            3: d[23:0]  = {v, v, v};
                        endcase
                    end
                    send_beat(d, (m == 0 && yy == 0), (m == BEATS - 1));
                end
            end
            repeat (40) @(negedge clk);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);

        // ---- frame 1: flat background, warms the reference buffer --------
        fork
            drive_frame(0);
            begin
                @(posedge frame_done);
                if (any_motion !== 1'b0) begin
                    errors = errors + 1;
                    $display("FAIL frame1: motion reported during warm-up");
                end else begin
                    $display("  frame1 warm-up ok (any_motion=0)");
                end
            end
        join

        // ---- frame 2: one block changes ---------------------------------
        fork
            drive_frame(1);
            begin
                @(posedge frame_done);
                if (any_motion !== 1'b1) begin
                    errors = errors + 1;
                    $display("FAIL frame2: any_motion=%0d expected 1", any_motion);
                end
                if (motion_blocks !== 16'd1) begin
                    errors = errors + 1;
                    $display("FAIL frame2: motion_blocks=%0d expected 1", motion_blocks);
                end
                if (box_x0 !== 16'd16 || box_x1 !== 16'd31 ||
                    box_y0 !== 16'd0  || box_y1 !== 16'd15) begin
                    errors = errors + 1;
                    $display("FAIL frame2: box=(%0d,%0d)-(%0d,%0d) expected (16,0)-(31,15)",
                             box_x0, box_y0, box_x1, box_y1);
                end else begin
                    $display("  frame2 box ok: (%0d,%0d)-(%0d,%0d), blocks=%0d",
                             box_x0, box_y0, box_x1, box_y1, motion_blocks);
                end
            end
        join

        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #500000;
        $display("RESULT: TIMEOUT");
        $finish;
    end

endmodule
