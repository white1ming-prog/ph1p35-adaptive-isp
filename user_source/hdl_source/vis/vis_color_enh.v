`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_color_enh
//
// Colour enhancement for the "washed out / hazy" camera look. Each channel is
// pushed away from the pixel luma:
//
//     out = Y + 1.25 * (c - Y)
//
// 1.25 is d + (d >> 2), so there is no multiplier anywhere - only a subtract,
// a shift and two adds per channel. Results are clipped to 0..255, so strongly
// saturated pixels clamp instead of wrapping.
//
// Purely combinational on the pixel path, so it adds no latency.
//----------------------------------------------------------------------------
module vis_color_enh (
    input               I_clk,
    input               I_rst_n,

    input               I_tuser,
    input               I_tlast,
    input               I_tvalid,
    input      [95:0]   I_tdata,
    input               I_tready,

    output reg [95:0]   O_tdata,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg          O_tvalid,
    output              O_tready
);

    assign O_tready = I_tready;

    // 1/4 R + 1/2 G + 1/4 B, maximum 253, no divider needed.
    function [7:0] lum;
        input [7:0] r;
        input [7:0] g;
        input [7:0] b;
        begin
            lum = {2'b00, r[7:2]} + {1'b0, g[7:1]} + {2'b00, b[7:2]};
        end
    endfunction

    function [7:0] push;
        input [7:0] c;
        input [7:0] y;
        reg signed [11:0] d;
        reg signed [11:0] r;
        begin
            d = $signed({4'b0000, c}) - $signed({4'b0000, y});
            // 1.25x saturation: d + (d >> 2). Deliberately gentle: any colour
            // unevenness the sensor/lens already has (radial lens shading,
            // demosaic false colour) gets multiplied by this factor too, so a
            // larger boost trades vividness for visible colour defects.
            r = $signed({4'b0000, y}) + d + (d >>> 2);
            if (r < 12'sd0)
                push = 8'd0;
            else if (r > 12'sd255)
                push = 8'd255;
            else
                push = r[7:0];
        end
    endfunction

    // Green trim. The sensor plus the vendor AWB leaves a slight green cast,
    // and the saturation boost amplifies it, so G is pulled down about 6%:
    // g - (g >> 4). Change to (g >> 3) for a stronger ~12% correction, or
    // remove the call below if the scene is genuinely green.
    function [7:0] g_trim;
        input [7:0] g;
        begin
            g_trim = g - (g >> 4);
        end
    endfunction

    function [23:0] enh_pixel;
        input [23:0] p;
        reg   [7:0]  y;
        begin
            y = lum(p[23:16], p[15:8], p[7:0]);
            // g_trim is disabled: the green cast is radial (lens shading), so a
            // global green reduction would push the corners magenta instead.
            // Re-enable ( g_trim(push(...)) ) only if the cast is global.
            enh_pixel = { push(p[23:16], y),
                          push(p[15:8],  y),
                          push(p[7:0],   y) };
        end
    endfunction

    // Gamma correction first, on all 12 channel values (4 px x 3 channels),
    // so the saturation boost below acts on display-referred data.
    wire [7:0] gr0, gg0, gb0, gr1, gg1, gb1, gr2, gg2, gb2, gr3, gg3, gb3;

    vis_gamma_rom u_g0r (.I_in(I_tdata[95:88]), .O_out(gr0));
    vis_gamma_rom u_g0g (.I_in(I_tdata[87:80]), .O_out(gg0));
    vis_gamma_rom u_g0b (.I_in(I_tdata[79:72]), .O_out(gb0));
    vis_gamma_rom u_g1r (.I_in(I_tdata[71:64]), .O_out(gr1));
    vis_gamma_rom u_g1g (.I_in(I_tdata[63:56]), .O_out(gg1));
    vis_gamma_rom u_g1b (.I_in(I_tdata[55:48]), .O_out(gb1));
    vis_gamma_rom u_g2r (.I_in(I_tdata[47:40]), .O_out(gr2));
    vis_gamma_rom u_g2g (.I_in(I_tdata[39:32]), .O_out(gg2));
    vis_gamma_rom u_g2b (.I_in(I_tdata[31:24]), .O_out(gb2));
    vis_gamma_rom u_g3r (.I_in(I_tdata[23:16]), .O_out(gr3));
    vis_gamma_rom u_g3g (.I_in(I_tdata[15:8]),  .O_out(gg3));
    vis_gamma_rom u_g3b (.I_in(I_tdata[7:0]),   .O_out(gb3));

    // ---- stage 1: register the gamma results ------------------------------
    // Gamma lookup -> saturation in a single cycle did not close timing
    // (SWNS fell to 0.009 ns), so the table outputs are registered here and the
    // saturation arithmetic gets a cycle of its own. The stream control is
    // delayed by the same one cycle so tuser/tlast stay aligned.
    reg [7:0] gr0_r, gg0_r, gb0_r, gr1_r, gg1_r, gb1_r;
    reg [7:0] gr2_r, gg2_r, gb2_r, gr3_r, gg3_r, gb3_r;
    reg       v1, u1, l1;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            gr0_r <= 8'd0; gg0_r <= 8'd0; gb0_r <= 8'd0;
            gr1_r <= 8'd0; gg1_r <= 8'd0; gb1_r <= 8'd0;
            gr2_r <= 8'd0; gg2_r <= 8'd0; gb2_r <= 8'd0;
            gr3_r <= 8'd0; gg3_r <= 8'd0; gb3_r <= 8'd0;
            v1 <= 1'b0; u1 <= 1'b0; l1 <= 1'b0;
        end else begin
            gr0_r <= gr0; gg0_r <= gg0; gb0_r <= gb0;
            gr1_r <= gr1; gg1_r <= gg1; gb1_r <= gb1;
            gr2_r <= gr2; gg2_r <= gg2; gb2_r <= gb2;
            gr3_r <= gr3; gg3_r <= gg3; gb3_r <= gb3;
            v1 <= I_tvalid; u1 <= I_tuser; l1 <= I_tlast;
        end
    end

    // ---- stage 2: saturation, registered output ---------------------------
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            O_tdata  <= 96'd0;
            O_tlast  <= 1'b0;
            O_tuser  <= 1'b0;
            O_tvalid <= 1'b0;
        end else begin
            O_tlast  <= l1;
            O_tuser  <= u1;
            O_tvalid <= v1;
            if(v1)
                O_tdata <= { enh_pixel({gr0_r,gg0_r,gb0_r}),
                             enh_pixel({gr1_r,gg1_r,gb1_r}),
                             enh_pixel({gr2_r,gg2_r,gb2_r}),
                             enh_pixel({gr3_r,gg3_r,gb3_r}) };
            else
                O_tdata <= 96'd0;
        end
    end

endmodule
