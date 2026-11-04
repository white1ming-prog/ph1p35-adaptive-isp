`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_sharpen
//
// Unsharp mask on the 96-bit ISP stream (4 x RGB888 per beat).
//
//   detail = Y - blur3x3(Y)          (the high frequency part)
//   out_c  = c + detail/2            (clipped)
//
// Only the LUMA is blurred, so the line buffers are 8 bits wide instead of 96,
// and the same detail is added back to R, G and B. Human vision is far more
// sensitive to luminance sharpness than to chroma sharpness, so the visual
// result is the same for a fraction of the BRAM.
//
// The blur kernel is the separable-free Gaussian [1 2 1; 2 4 2; 1 2 1], whose
// weights sum to 16. That makes the divide a >>4 shift - no divider and no
// multiplier anywhere. 2x and 4x terms are shifts as well.
//
// Two luma samples are produced per beat (each one the average of a pixel
// pair), which halves the line buffer depth and still gives 640 detail samples
// per line.
//----------------------------------------------------------------------------
module vis_sharpen #(
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

    function [7:0] lum;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        begin
            lum = {2'b00, r[7:2]} + {1'b0, g[7:1]} + {2'b00, b[7:2]};
        end
    endfunction

    wire [7:0] y0 = lum(I_tdata[95:88], I_tdata[87:80], I_tdata[79:72]);
    wire [7:0] y1 = lum(I_tdata[71:64], I_tdata[63:56], I_tdata[55:48]);
    wire [7:0] y2 = lum(I_tdata[47:40], I_tdata[39:32], I_tdata[31:24]);
    wire [7:0] y3 = lum(I_tdata[23:16], I_tdata[15:8],  I_tdata[7:0]);

    // ---- stage 1: register luma, RGB and the stream control ----------------
    reg [7:0]  r0, r1, r2, r3;
    reg [95:0] rgb1;
    reg        v1, u1, l1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            r0 <= 8'd0; r1 <= 8'd0; r2 <= 8'd0; r3 <= 8'd0;
            rgb1 <= 96'd0;
            v1 <= 1'b0; u1 <= 1'b0; l1 <= 1'b0;
        end else begin
            r0 <= y0; r1 <= y1; r2 <= y2; r3 <= y3;
            rgb1 <= I_tdata;
            v1 <= I_tvalid; u1 <= I_tuser; l1 <= I_tlast;
        end
    end

    // Two luma samples per beat, each the average of a pixel pair.
    wire [8:0] ps0 = {1'b0,r0} + {1'b0,r1};
    wire [8:0] ps1 = {1'b0,r2} + {1'b0,r3};
    wire [7:0] p0  = ps0[8:1];
    wire [7:0] p1  = ps1[8:1];

    localparam GRP = (IMG_WIDTH >> 2);

    (* ram_style = "block" *) reg [15:0] line1_mem [0:GRP-1];
    (* ram_style = "block" *) reg [15:0] line2_mem [0:GRP-1];

    reg [10:0] x_grp;
    reg [10:0] y_pos;

    wire [15:0] top_word = line2_mem[x_grp];
    wire [15:0] mid_word = line1_mem[x_grp];
    wire [15:0] bot_word = {p0,p1};

    wire [7:0] top0 = top_word[15:8];
    wire [7:0] top1 = top_word[7:0];
    wire [7:0] mid0 = mid_word[15:8];
    wire [7:0] mid1 = mid_word[7:0];

    reg [7:0] top_prev2, top_prev1;
    reg [7:0] mid_prev2, mid_prev1;
    reg [7:0] bot_prev2, bot_prev1;

    // ---- stage 2: 3x3 gaussian blur of the two samples --------------------
    // Window for sample 0:            Window for sample 1:
    //   tp2 tp1 top0                    tp1 top0 top1
    //   mp2 mp1 mid0                    mp1 mid0 mid1
    //   bp2 bp1 p0                      bp1 p0   p1
    // Weight 4 sits on the centre, weight 2 on the edge neighbours.
    // Balanced adder trees. Written as a left-associative chain this became
    // ten adders in series (SWNS -0.314 ns), so the terms are paired up here
    // instead: two levels to build each half, then two more to combine.
    wire [8:0]  c0a = {1'b0,top_prev2}  + {1'b0,top0};
    wire [8:0]  c0b = {1'b0,bot_prev2}  + {1'b0,p0};
    wire [9:0]  cor0 = {1'b0,c0a} + {1'b0,c0b};

    wire [8:0]  e0a = {1'b0,top_prev1}  + {1'b0,mid_prev2};
    wire [8:0]  e0b = {1'b0,mid0}       + {1'b0,bot_prev1};
    wire [9:0]  edg0 = {1'b0,e0a} + {1'b0,e0b};

    wire [11:0] bs0 = ({2'b0,cor0} + {1'b0,edg0,1'b0}) + {1'b0,mid_prev1,2'b0};

    wire [8:0]  c1a = {1'b0,top_prev1}  + {1'b0,top1};
    wire [8:0]  c1b = {1'b0,bot_prev1}  + {1'b0,p1};
    wire [9:0]  cor1 = {1'b0,c1a} + {1'b0,c1b};

    wire [8:0]  e1a = {1'b0,top0}       + {1'b0,mid_prev1};
    wire [8:0]  e1b = {1'b0,mid1}       + {1'b0,p0};
    wire [9:0]  edg1 = {1'b0,e1a} + {1'b0,e1b};

    wire [11:0] bs1 = ({2'b0,cor1} + {1'b0,edg1,1'b0}) + {1'b0,mid0,2'b0};

    wire [7:0] blur0 = bs0[11:4];
    wire [7:0] blur1 = bs1[11:4];

    // Window validity: the first two rows and the first column lack
    // neighbours. Declared here because the stage-2 register below samples it.
    wire window_valid = (y_pos >= 11'd2) && (x_grp != 11'd0) && !u1;

    // ---- stage 2: register the blur, the pixels and the control -----------
    // The blur tree and the sharpening arithmetic in one cycle did not fit:
    // at 76% LUT the router stretched the path so that 40% of the delay was
    // wire. Splitting the two halves lets each one be placed compactly.
    reg [7:0]  blur0_r, blur1_r, p0_r, p1_r;
    reg [95:0] rgb2;
    reg        wv2, v2, u2, l2;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            blur0_r <= 8'd0; blur1_r <= 8'd0;
            p0_r    <= 8'd0; p1_r    <= 8'd0;
            rgb2    <= 96'd0;
            wv2 <= 1'b0; v2 <= 1'b0; u2 <= 1'b0; l2 <= 1'b0;
        end else begin
            blur0_r <= blur0; blur1_r <= blur1;
            p0_r    <= p0;    p1_r    <= p1;
            rgb2    <= rgb1;
            wv2 <= window_valid;
            v2  <= v1; u2 <= u1; l2 <= l1;
        end
    end

    // detail = current - blurred (signed), halved with an arithmetic shift
    wire signed [8:0] det0 = $signed({1'b0,p0_r}) - $signed({1'b0,blur0_r});
    wire signed [8:0] det1 = $signed({1'b0,p1_r}) - $signed({1'b0,blur1_r});
    wire signed [8:0] add0 = det0 >>> 1;
    wire signed [8:0] add1 = det1 >>> 1;

    function [7:0] sharp;
        input [7:0] c;
        input signed [8:0] h;
        reg signed [10:0] t;
        begin
            t = $signed({3'b000,c}) + h;
            if (t < 11'sd0)
                sharp = 8'd0;
            else if (t > 11'sd255)
                sharp = 8'd255;
            else
                sharp = t[7:0];
        end
    endfunction

    // ---- stage 3: registered output ---------------------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
            x_grp    <= 11'd0;
            y_pos    <= 11'd0;
            top_prev2 <= 8'd0; top_prev1 <= 8'd0;
            mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
            bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
        end else begin
            O_tlast  <= l2;
            O_tuser  <= u2;
            O_tvalid <= v2;

            if(v2) begin
                if(wv2)
                    O_tdata <= { sharp(rgb2[95:88], add0), sharp(rgb2[87:80], add0),
                                 sharp(rgb2[79:72], add0),
                                 sharp(rgb2[71:64], add0), sharp(rgb2[63:56], add0),
                                 sharp(rgb2[55:48], add0),
                                 sharp(rgb2[47:40], add1), sharp(rgb2[39:32], add1),
                                 sharp(rgb2[31:24], add1),
                                 sharp(rgb2[23:16], add1), sharp(rgb2[15:8],  add1),
                                 sharp(rgb2[7:0],   add1) };
                else
                    O_tdata <= rgb2;      // no full window yet: pass through
            end else
                O_tdata <= 96'd0;

            // line buffers and window position, on the stage-1 control
            if(v1) begin
                line2_mem[x_grp] <= mid_word;
                line1_mem[x_grp] <= bot_word;

                top_prev2 <= top0; top_prev1 <= top1;
                mid_prev2 <= mid0; mid_prev1 <= mid1;
                bot_prev2 <= p0;   bot_prev1 <= p1;

                if(u1) begin
                    x_grp <= 11'd1;
                    y_pos <= 11'd0;
                end else if(x_grp == (GRP - 1)) begin
                    x_grp <= 11'd0;
                    if(y_pos == (IMG_HEIGHT - 1))
                        y_pos <= 11'd0;
                    else
                        y_pos <= y_pos + 1'b1;
                    top_prev2 <= 8'd0; top_prev1 <= 8'd0;
                    mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
                    bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
                end else begin
                    x_grp <= x_grp + 1'b1;
                end
            end
        end
    end

endmodule
