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
