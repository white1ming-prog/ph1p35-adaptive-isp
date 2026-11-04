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
