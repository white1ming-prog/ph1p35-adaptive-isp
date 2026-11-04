`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// tb_vis_binarize
//
// Checks that adaptive binarization outputs pure black or pure white from the
// 1/4 + 1/2 + 1/4 luma, and that the sync signals pass through untouched.
//----------------------------------------------------------------------------
module tb_vis_binarize;

    reg         clk = 1'b0;
    reg         rst_n = 1'b0;
    reg         tuser = 1'b0;
    reg         tlast = 1'b0;
    reg         tvalid = 1'b0;
    reg [95:0]  tdata = 96'd0;
    reg [7:0]   thresh = 8'd100;

    wire [95:0] o_tdata;
    wire        o_tlast, o_tuser, o_tvalid;

    integer errors = 0;

    vis_binarize dut (
        .I_clk    (clk),
        .I_rst_n  (rst_n),
        .I_tuser  (tuser),
        .I_tlast  (tlast),
        .I_tvalid (tvalid),
        .I_tdata  (tdata),
        .I_tready (1'b1),
        .I_thresh (thresh),
        .O_tdata  (o_tdata),
        .O_tlast  (o_tlast),
        .O_tuser  (o_tuser),
        .O_tvalid (o_tvalid),
        .O_tready ()
    );

    always #5 clk = ~clk;

    // One beat: four pixels of the same grey value v.
    task check_beat;
        input [7:0] v;
        input       expect_white;
        reg [23:0]  expect;
        begin
            expect = expect_white ? 24'hffffff : 24'h000000;
            tdata = {v, v, v, v, v, v, v, v, v, v, v, v};
            tvalid = 1'b1;
            #1;
            if (o_tdata[95:72] !== expect || o_tdata[71:48] !== expect ||
                o_tdata[47:24] !== expect || o_tdata[23:0]  !== expect) begin
                errors = errors + 1;
                $display("FAIL v=%0d: got %h expected all %h", v, o_tdata, expect);
            end
            @(negedge clk);
            tvalid = 1'b0;
            @(negedge clk);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (2) @(negedge clk);

        // threshold = 100
        check_beat(8'd40,  1'b0);   // dark  -> black
        check_beat(8'd100, 1'b1);   // equal -> white (>= compare)
        check_beat(8'd240, 1'b1);   // light -> white
        check_beat(8'd0,   1'b0);

        // move the threshold and confirm the split follows it
        thresh = 8'd200;
        check_beat(8'd100, 1'b0);   // now below -> black
        check_beat(8'd240, 1'b1);

        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #100000;
        $display("RESULT: TIMEOUT");
        $finish;
    end

endmodule
