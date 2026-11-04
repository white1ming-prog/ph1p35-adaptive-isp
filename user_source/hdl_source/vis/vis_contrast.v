`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_contrast
//
// Contrast stretch around mid-grey:
//
//     out = 128 + (in - 128) * GAIN
//
// GAIN is expressed in quarter steps and implemented as shifts and adds, so no
// multiplier is inferred:
//     GAIN=4 -> 1.00x   GAIN=5 -> 1.25x   GAIN=6 -> 1.50x   GAIN=8 -> 2.00x
//
// The reference design (ui_Contrast_adj) does the same maths with an 8x8
// multiplier per channel; at 78% LUT and 30 free DSPs that does not fit, and
// quarter-step gain is plenty for a display tuning control.
//
// Two-stage pipeline; tuser/tlast/tvalid are delayed to match.
//----------------------------------------------------------------------------
module vis_contrast #(
    parameter [3:0] GAIN = 4'd5        // 4=1.0x, 5=1.25x, 6=1.5x, 8=2.0x
)(
    input               I_clk,
    input               I_rst_n,

    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    output              I_tready,

    output              O_tlast,
    output              O_tuser,
    output     [95:0]   O_tdata,
    output              O_tvalid,
    input               O_tready
);

    reg [1:0] I_tlast_r, I_tuser_r, I_tvalid_r;

    wire [23:0] p0 = I_tdata[23:0];
    wire [23:0] p1 = I_tdata[47:24];
    wire [23:0] p2 = I_tdata[71:48];
    wire [23:0] p3 = I_tdata[95:72];

    wire [7:0] c [0:11];
    assign c[0]  = p0[23:16];
    assign c[1]  = p0[15:8];
    assign c[2]  = p0[7:0];
    assign c[3]  = p1[23:16];
    assign c[4]  = p1[15:8];
    assign c[5]  = p1[7:0];
    assign c[6]  = p2[23:16];
    assign c[7]  = p2[15:8];
    assign c[8]  = p2[7:0];
    assign c[9]  = p3[23:16];
    assign c[10] = p3[15:8];
    assign c[11] = p3[7:0];

    reg  signed [9:0]  cent [0:11];    // (in - 128)
    reg  signed [11:0] scal [0:11];    // (in-128) * GAIN / 4
    wire signed [11:0] sum  [0:11];    // scaled + 128
    wire [7:0]         o    [0:11];

    genvar i;
    generate
        for (i = 0; i < 12; i = i + 1) begin : g_chan
            // stage 1: centre around mid-grey
            always @(posedge I_clk or negedge I_rst_n) begin
                if(!I_rst_n)
                    cent[i] <= 10'sd0;
                else if(I_tvalid)
                    cent[i] <= $signed({2'b00, c[i]}) - 10'sd128;
            end

            // stage 2: multiply by the quarter step, via shifts
            //   GAIN=4 -> d
            //   GAIN=5 -> d + d/4
            //   GAIN=6 -> d + d/2
            //   GAIN=8 -> d*2
            always @(posedge I_clk or negedge I_rst_n) begin
                if(!I_rst_n)
                    scal[i] <= 12'sd0;
                else if(I_tvalid_r[0]) begin
                    case (GAIN)
                        4'd4: scal[i] <= cent[i];
                        4'd5: scal[i] <= cent[i] + (cent[i] >>> 2);
                        4'd6: scal[i] <= cent[i] + (cent[i] >>> 1);
                        4'd7: scal[i] <= cent[i] + (cent[i] >>> 1) + (cent[i] >>> 2);
                        4'd8: scal[i] <= cent[i] <<< 1;
                        default: scal[i] <= cent[i];
                    endcase
                end
            end

            assign sum[i] = scal[i] + 12'sd128;

            // clip: below 0 or above 255
            assign o[i] = sum[i][11] ? 8'd0 :
                          (sum[i] > 12'sd255) ? 8'd255 : sum[i][7:0];
        end
    endgenerate

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            {I_tlast_r, I_tuser_r, I_tvalid_r} <= 6'b0;
        else begin
            I_tlast_r  <= {I_tlast_r[0],  I_tlast};
            I_tuser_r  <= {I_tuser_r[0],  I_tuser};
            I_tvalid_r <= {I_tvalid_r[0], I_tvalid};
        end
    end

    assign O_tlast  = I_tlast_r[1];
    assign O_tuser  = I_tuser_r[1];
    assign O_tvalid = I_tvalid_r[1];
    assign I_tready = O_tready;

    assign O_tdata = { o[9], o[10], o[11],
                       o[6], o[7],  o[8],
                       o[3], o[4],  o[5],
                       o[0], o[1],  o[2] };

endmodule
