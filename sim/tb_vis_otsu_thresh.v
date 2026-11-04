`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// tb_vis_otsu_thresh
//
// Feeds synthetic 64-bin histograms into the Otsu engine and checks that the
// returned split actually separates the two modes. A behavioural Otsu is also
// evaluated with real arithmetic and printed for reference.
//----------------------------------------------------------------------------
module tb_vis_otsu_thresh;

    localparam BIN_BITS  = 6;
    localparam HIST_BITS = 21;
    localparam BINS      = (1 << BIN_BITS);

    reg                     clk = 1'b0;
    reg                     rst_n = 1'b0;
    reg                     start = 1'b0;

    wire [BIN_BITS-1:0]     hist_addr;
    reg  [HIST_BITS-1:0]    hist_data;
    wire [7:0]              thresh;
    wire                    valid;
    wire                    busy;

    reg [HIST_BITS-1:0] hist_mem [0:BINS-1];

    integer errors = 0;
    integer i, t;
    integer case_id;

    // Behavioural reference values.
    real   n, stot, w0, s0, w1, s1, mu0, mu1, sig, best;
    integer best_t;

    vis_otsu_thresh #(
        .BIN_BITS  (BIN_BITS),
        .HIST_BITS (HIST_BITS)
    ) dut (
        .I_clk       (clk),
        .I_rst_n     (rst_n),
        .I_start     (start),
        .O_hist_addr (hist_addr),
        .I_hist_data (hist_data),
        .O_thresh    (thresh),
        .O_valid     (valid),
        .O_busy      (busy)
    );

    always #5 clk = ~clk;

    // Combinational histogram lookup, matching vis_frame_stats' read port.
    always @(*) hist_data = hist_mem[hist_addr];

    task clear_hist;
        begin
            for (i = 0; i < BINS; i = i + 1)
                hist_mem[i] = {HIST_BITS{1'b0}};
        end
    endtask

    task run_reference;
        begin
            n = 0.0;  stot = 0.0;
            for (t = 0; t < BINS; t = t + 1) begin
                n    = n + hist_mem[t];
                stot = stot + t * hist_mem[t];
            end
            w0 = 0.0; s0 = 0.0; best = -1.0; best_t = 0;
            for (t = 0; t < BINS; t = t + 1) begin
                w0 = w0 + hist_mem[t];
                s0 = s0 + t * hist_mem[t];
                w1 = n - w0;
                s1 = stot - s0;
                if (w0 > 0.0 && w1 > 0.0) begin
                    mu0 = s0 / w0;
                    mu1 = s1 / w1;
                    sig = w0 * w1 * (mu0 - mu1) * (mu0 - mu1);
                    if (sig > best) begin
                        best = sig;
                        best_t = t;
                    end
                end
            end
            $display("  reference Otsu bin = %0d, raw luma = %0d", best_t, best_t * 4);
        end
    endtask

    task run_case;
        input integer cid;
        input integer lo_bin;
        input integer hi_bin;
        begin
            case_id = cid;
            clear_hist;
            for (t = lo_bin; t <= lo_bin + 4; t = t + 1) hist_mem[t] = 21'd8000;
            for (t = hi_bin; t <= hi_bin + 4; t = t + 1) hist_mem[t] = 21'd8000;

            run_reference;

            @(negedge clk);
            start = 1'b1;
            @(negedge clk);
            start = 1'b0;

            // Wait for completion (the engine needs ~2 * 64 * 32 cycles).
            i = 0;
            while (!valid && i < 20000) begin
                @(negedge clk);
                i = i + 1;
            end

            if (!valid) begin
                errors = errors + 1;
                $display("FAIL case%0d: no O_valid", cid);
            end else begin
                // A valid Otsu split places every low-mode bin (lo_bin..lo_bin+4)
                // in class 0 and every high-mode bin in class 1, so the returned
                // bin may be anywhere from lo_bin+4 to hi_bin-1.
                if (thresh < (lo_bin + 4) * 4 + 2 ||
                    thresh > (hi_bin - 1) * 4 + 2) begin
                    errors = errors + 1;
                    $display("FAIL case%0d: threshold %0d outside valid split (%0d..%0d)",
                             cid, thresh, (lo_bin + 4) * 4 + 2, (hi_bin - 1) * 4 + 2);
                end else begin
                    $display("  case%0d threshold = %0d (bin %0d)", cid, thresh, thresh >> 2);
                end
            end

            // One more cycle so the engine returns to idle.
            @(negedge clk);
            if (busy) begin
                errors = errors + 1;
                $display("FAIL case%0d: still busy after O_valid", cid);
            end
        end
    endtask

    initial begin
        start = 1'b0;
        clear_hist;
        rst_n = 1'b0;
        repeat (4) @(negedge clk);
        rst_n = 1'b1;
        repeat (4) @(negedge clk);

        run_case(1, 8, 48);
        run_case(2, 20, 40);

        if (errors == 0) $display("RESULT: PASS");
        else             $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #2000000;
        $display("RESULT: TIMEOUT");
        $finish;
    end

endmodule
