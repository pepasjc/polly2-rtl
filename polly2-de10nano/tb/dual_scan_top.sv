// dual_scan_top.sv - the CRT_DUAL display path of sys_top: the stock 1080p
// spg (HDMI) and the 15 kHz CRT spg (LINE_MODE 3) sharing one Avalon read
// port through vbuf_arbiter. The C++ tb (dual_scan_tb.cpp) models the DDR.
module dual_scan_top (
    input  wire         clk_hdmi,
    input  wire         clk_crt,
    input  wire         avl_clk,
    input  wire         reset,

    input  wire [31:0]  fb_base,
    input  wire [13:0]  fb_stride,
    input  wire [1:0]   fb_depth,
    input  wire [3:0]   crt_ctrl,

    output wire         avl_read,
    output wire [27:0]  avl_address,
    output wire [7:0]   avl_burstcount,
    input  wire         avl_waitrequest,
    input  wire [127:0] avl_readdata,
    input  wire         avl_readdatavalid,

    output wire [23:0]  hdmi_rgb,
    output wire         hdmi_de,
    output wire         hdmi_vs,
    output wire         hdmi_underrun,

    output wire [23:0]  crt_rgb,
    output wire         crt_hs,
    output wire         crt_vs,
    output wire         crt_de,
    output wire         crt_underrun
);

wire [27:0]  h_addr, c_addr;
wire  [7:0]  h_bc, c_bc;
wire         h_rd, c_rd, h_wait, c_wait, h_valid, c_valid;
wire [127:0] h_data, c_data;

spg spg_hdmi (
    .clk(clk_hdmi), .reset(reset),
    .fb_base(fb_base), .fb_stride(fb_stride), .fb_line_dbl(1'b0), .fb_pix_dbl(1'b0),
    .fb_split(1'b1), .fb_disp_half(1'b0), .fb_depth(fb_depth), .fb_concat(3'd0),
    .fb_enable(1'b1), .crt_ctrl(4'd0), .fb_top_base(32'd0), .fb_bot_base(32'd0),
    .avl_clk(avl_clk), .avl_read(h_rd), .avl_address(h_addr), .avl_burstcount(h_bc),
    .avl_waitrequest(h_wait), .avl_readdata(h_data), .avl_readdatavalid(h_valid),
    .red(hdmi_rgb[23:16]), .green(hdmi_rgb[15:8]), .blue(hdmi_rgb[7:0]),
    .hsync(), .vsync(hdmi_vs), .de(hdmi_de), .vblank(), .border(),
    .src_line(), .vblank_in(), .vblank_out(), .underrun(hdmi_underrun)
);

spg #(
    .H_ACTIVE(1440), .H_FP(38), .H_SYNC(124), .H_BP(114),
    .V_ACTIVE(240),  .V_FP(4),  .V_SYNC(3),   .V_BP(15),
    .LINE_MODE(3), .FIELD_SWAP(0)
) spg_crt (
    .clk(clk_crt), .reset(reset),
    .fb_base(fb_base), .fb_stride(fb_stride), .fb_line_dbl(1'b0), .fb_pix_dbl(1'b0),
    .fb_split(1'b1), .fb_disp_half(1'b0), .fb_depth(fb_depth), .fb_concat(3'd0),
    .fb_enable(1'b1), .crt_ctrl(crt_ctrl), .fb_top_base(32'd0), .fb_bot_base(32'd0),
    .avl_clk(avl_clk), .avl_read(c_rd), .avl_address(c_addr), .avl_burstcount(c_bc),
    .avl_waitrequest(c_wait), .avl_readdata(c_data), .avl_readdatavalid(c_valid),
    .red(crt_rgb[23:16]), .green(crt_rgb[15:8]), .blue(crt_rgb[7:0]),
    .hsync(crt_hs), .vsync(crt_vs), .de(crt_de), .vblank(), .border(),
    .src_line(), .vblank_in(), .vblank_out(), .underrun(crt_underrun)
);

vbuf_arbiter arb (
    .clk(avl_clk), .reset(reset),
    .m0_read(c_rd), .m0_address(c_addr), .m0_burstcount(c_bc), .m0_waitrequest(c_wait),
    .m0_readdata(c_data), .m0_readdatavalid(c_valid),
    .m1_read(h_rd), .m1_address(h_addr), .m1_burstcount(h_bc), .m1_waitrequest(h_wait),
    .m1_readdata(h_data), .m1_readdatavalid(h_valid),
    .s_read(avl_read), .s_address(avl_address), .s_burstcount(avl_burstcount),
    .s_waitrequest(avl_waitrequest), .s_readdata(avl_readdata), .s_readdatavalid(avl_readdatavalid)
);

endmodule
