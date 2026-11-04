`timescale 1ns / 1ps
//----------------------------------------------------------------------------
// vis_brightness
//
// Brightness offset on the 96-bit RGB stream (4 pixels per beat). Each channel
// gets +BRIGHT_ADD - BRIGHT_SUB and is clipped to 0..255. No multiplier, so it
// costs almost nothing.
//
// Ported from the MiLianKe ui_Brightness_adj reference; the arithmetic and the
// clipping idiom (sign bit for negative, bit 8 for overflow) are kept as-is so
// the behaviour matches the vendor demo.
//----------------------------------------------------------------------------
module vis_brightness #(
    parameter [7:0] BRIGHT_ADD   = 8'd0,   // 0..255
    parameter [7:0] BRIGHT_SUB   = 8'd0    // 0..255
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

    reg I_tlast_r, I_tuser_r, I_tvalid_r;

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

    reg  [9:0] d [0:11];
    wire [7:0] o [0:11];

    genvar i;
    generate
        for (i = 0; i < 12; i = i + 1) begin : g_chan
            always @(posedge I_clk or negedge I_rst_n) begin
                if(!I_rst_n)
                    d[i] <= 10'd0;
                else if(I_tvalid)
                    d[i] <= {2'b00, c[i]} + {2'b00, BRIGHT_ADD} - {2'b00, BRIGHT_SUB};
            end

            assign o[i] = d[i][9] ? 8'd0 : (d[i][8] ? 8'd255 : d[i][7:0]);
        end
    endgenerate

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)
            {I_tlast_r, I_tuser_r, I_tvalid_r} <= 3'b0;
        else begin
            I_tlast_r  <= I_tlast;
            I_tuser_r  <= I_tuser;
            I_tvalid_r <= I_tvalid;
        end
    end

    assign O_tlast  = I_tlast_r;
    assign O_tuser  = I_tuser_r;
    assign O_tvalid = I_tvalid_r;
    assign I_tready = O_tready;

    assign O_tdata = { o[9], o[10], o[11],
                       o[6], o[7],  o[8],
                       o[3], o[4],  o[5],
                       o[0], o[1],  o[2] };

endmodule
