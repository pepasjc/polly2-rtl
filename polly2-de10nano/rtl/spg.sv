// spg.sv - fixed-mode display controller, loosely modelled on the Dreamcast
// (HOLLY) spg, but generating the FINAL 1080p raster directly at the HDMI
// pixel clock. Replaces ascal on the HDMI path:
//
//  - 1920x1080 CEA-861 timing (2200 x 1125 totals, both syncs positive).
//    At a 148.352 MHz pixel clock this is 1080p59.94 (the DC VGA field
//    rate); at 148.5 MHz it is exactly 60.000 Hz.
//  - The 640-wide framebuffer in DDR3 is displayed pixel- and line-doubled
//    (2x nearest) as a 1280x960 window centred in the frame (x 320..1599,
//    y 60..1019 - the same window the stripped ascal used). fb_line_dbl
//    displays a 640x240 source at 4x vertically instead (FB_R_CTRL
//    fb_line_double / the 240p menu option); fb_pix_dbl a 320-wide source
//    at 4x horizontally (VO_CONTROL pixel_double) - the two compose.
//  - All four FB_R_CTRL fb_depth read formats, with refsw2 Present()
//    semantics (fb_concat appended below the 5/6-bit channels):
//      0: 0555 RGB, 2 bytes/px     1: 565 RGB, 2 bytes/px
//      2: 888 RGB packed, 3 bytes/px, STRAIGHT B,G,R at byte 3n (the FB
//         write master's packmode-4 layout; refsw2 Present's odd/even
//         fetch quirk is NOT copied - see the read path note)
//      3: 0888 RGB, 4 bytes/px
//    fb_enable=0 blanks the game window (borders/bands unaffected).
//  - FB_R_SOF is honoured PER LINE: fb_base/fb_disp_half are re-sampled
//    for every source-line fetch request ("render base"), while the
//    line-to-line advance is a separately accumulated "render offset"
//    (0 at source line 0, +fb_stride per line). A mid-frame base change
//    therefore takes effect from the next requested line, with the offset
//    accumulation undisturbed - refsw2's continuous addr walk. The base is
//    adopted through a 2-sample stability filter (it originates in another
//    clock domain); the sub-beat byte offset of each line's base is kept
//    per line buffer for the read side.
//  - fb_split replicates the Dreamcast 32-bit-view VRAM layout (minicast's
//    pvr_map32 rule): FB 32-bit word W lives at DDR byte W*8, in the LOW
//    (fb_disp_half=0, bank 0) or HIGH (fb_disp_half=1, bank 1) 32-bit half
//    of each 64-bit word - physical byte = W*8 + bank*4 (half re-sampled
//    per line with the base, it is SOF bit 22). A line is then fetched at 2x the FB-view byte count (2x
//    overfetch - still trivial bandwidth). fb_stride stays in FB-view
//    bytes in both modes; the DDR advance doubles internally.
//  - Two optional border bands: 640x30 RGB565 LINEAR framebuffers (stride
//    fixed at 1280 bytes) displayed 2x-doubled as 1280x60, exactly filling
//    the top (lines 0..59) and bottom (lines 1020..1079) borders,
//    x-centred like the game window. Bands are host OSD surfaces: always
//    565 with MSB-replicated expansion, independent of the game fb_depth.
//    fb_top_base / fb_bot_base are DDR BYTE addresses, 128-byte aligned,
//    sampled once per frame at the start of vertical blanking (line 1080);
//    0 disables a band (black border).
//  - DDR access is a 128-bit Avalon read master, intended for the vbuf
//    port on sysmem_lite that ascal used to own (16-byte word addresses).
//    Fetches are beat-aligned: the FB base's low 4 bits (split: bit 3
//    only) become a byte offset applied on the read side, and a misaligned
//    base just costs one extra beat. Bursts are serialized: a new command
//    is issued only after every beat of the previous one has arrived
//    (deliberately conservative w.r.t. the real-DDR3 read-beat desync
//    behaviour seen on hardware). A request landing mid-fetch (severe
//    starvation) is queued, not lost.
//  - Two line buffers ("current" / "next"): while one source line is
//    displayed (two output lines), the next is burst-read into the other.
//    Prefetch slack is two output lines (~29.7 us) per line; worst case
//    (split 0888, misaligned) is 321 beats - still no DMA engine and no
//    tight arbiter deadline. Storage is four 32-bit banks of 512: the
//    line's raw FB-view bytes, beat-aligned (split-mode half-selection is
//    the only write-side transformation), FB 32-bit word N -> bank N[1:0],
//    address {buffer, N[9:2]}.
//  - Read side: a source pixel at FB-view byte b reads the two adjacent
//    32-bit words containing bytes b..b+3 (per-bank addresses form a
//    sliding 4-word window), funnels the four bytes down and converts per
//    fb_depth. Addresses are pre-computed one pixel ahead so the RAM sees
//    only registered addresses.
//  - 15 kHz modes (LINE_MODE 1/2): one source line per output line, buffer
//    parity = output-line parity; the request for row r fires on row r-2
//    right after its window ends (that row's buffer is the one reused), so a
//    fetch has ~1.2 output lines (~79 us) before row r. With a 480-line
//    source, output row r of field f shows source line s = 2r+f (f = 0 in
//    240p; in 480i f = field ^ FIELD_SWAP ^ CRT_CTRL[0]).
//  - 15 kHz vertical filter (crt_ctrl[2:1], the CRT_CTRL register): off
//    shows L[s] raw; 2-tap = (L[s] + L[s+1] + 1) >> 1; 3-tap =
//    (L[s-1] + 2*L[s] + L[s+1] + 2) >> 2 - per 8-bit channel after depth
//    expansion, rounded half up, neighbours clamped to source lines
//    0..SRC_H-1. Each output line's fetch request then becomes a GROUP:
//    the centre line s into the banks above, then s+1 and s-1 (same render
//    base, +-1 stride) into two more bank sets at the same buffer parity.
//    The shared base means a shared sub-beat offset, so all sets are read
//    at the same address and only the funnel/convert is replicated.
//    Bypassed for fb_line_dbl (240-line) sources. VFILT_TAPS = 2 builds
//    without the s-1 set (3-tap then falls back to 2-tap), 0 without any.
//
// All video outputs are registered and mutually aligned (2 clk latency
// from the internal counters). Border pixels are black. Byte 0 of each
// 32-bit FB word is the lowest FB address (little-endian), matching the
// peel_core 16bpp tile writeback.

module spg
#(
	// CEA-861 1080p timing
	parameter H_ACTIVE = 1920,
	parameter H_FP     = 88,
	parameter H_SYNC   = 44,
	parameter H_BP     = 148,   // H total 2200
	parameter V_ACTIVE = 1080,
	parameter V_FP     = 4,
	parameter V_SYNC   = 5,
	parameter V_BP     = 36,    // V total 1125
	// source framebuffer (doubled to SRC_W*2 x SRC_H*2 on screen)
	parameter SRC_W    = 640,
	parameter SRC_H    = 480,
	// LINE_MODE 0: stock - line-doubled 1080p (SRC_H*2 window, border bands).
	// LINE_MODE 1: 15 kHz 240p - one source line per output line; a 480-line
	//              source shows its even lines, a 240-line source every line.
	// LINE_MODE 2: 15 kHz 480i - like 1, but field 1 shows the odd lines and
	//              runs 263 lines with its vsync half a line later (CEA 480i).
	// In both 15 kHz modes the window fills all V_ACTIVE lines (no bands).
	parameter LINE_MODE = 0,
	parameter FIELD_SWAP = 0,        // 480i: 1 = field 0 shows the odd lines
	// 15 kHz vertical filter hardware (see header): 3 = 2- and 3-tap,
	// 2 = 2-tap only, 0 = none. No effect in LINE_MODE 0.
	parameter VFILT_TAPS = 3
)
(
	input  wire        clk,          // 1080p pixel clock (148.352 / 148.5 MHz)
	input  wire        reset,        // video-domain reset

	// Framebuffer location. fb_base/fb_disp_half are sampled once PER LINE
	// (at the line's fetch request, through a 2-sample stability filter);
	// everything else once per frame at the top of the raster. fb_base is
	// a byte address; bits [3:0] (split: bit [3], [2:0] must be 0) select
	// the starting byte within its 16-byte beat. fb_stride is in FB-view
	// bytes, a multiple of 16.
	input  wire [31:0] fb_base,      // BYTE address of the top-left pixel
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [13:0] fb_stride,    // FB-view BYTEs per source line (SRC_W * bytes/px)
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        fb_line_dbl,  // 240p source: 4x vertical instead of 2x
	input  wire        fb_pix_dbl,   // VO_CONTROL.pixel_double: 320-wide source, 4x horizontal
	input  wire        fb_split,     // Dreamcast split-VRAM layout (see header)
	input  wire        fb_disp_half, // split: which 32-bit half of each 64-bit word
	input  wire [1:0]  fb_depth,     // FB_R_CTRL.fb_depth: 0=0555 1=565 2=888 3=0888
	input  wire [2:0]  fb_concat,    // FB_R_CTRL.fb_concat (low bits of 5/6-bit channels)
	input  wire        fb_enable,    // FB_R_CTRL.fb_enable: 0 = game window black

	// CRT_CTRL (pvr_mmio, another clock domain - synchronised here and
	// adopted once per field, at the start of vertical blanking):
	// [0] 480i field swap, xor'ed with FIELD_SWAP; [2:1] 15 kHz vertical
	// filter 0 = off, 1 = 2-tap, 2 = 3-tap, 3 = 3-tap. Unused in LINE_MODE 0.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [2:0]  crt_ctrl,
	/* verilator lint_on UNUSEDSIGNAL */

	// Border bands (see header). BYTE addresses, 128-byte aligned, 0 = off;
	// sampled once per frame at the start of vertical blanking.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [31:0] fb_top_base,  // 640x30 linear RGB565 above the window
	input  wire [31:0] fb_bot_base,  // 640x30 linear RGB565 below the window
	/* verilator lint_on UNUSEDSIGNAL */

	// 128-bit Avalon read master (sysmem vbuf port), avl_clk domain
	input  wire         avl_clk,
	output reg          avl_read,
	output reg  [27:0]  avl_address,    // 16-byte word address
	output reg  [7:0]   avl_burstcount,
	input  wire         avl_waitrequest,
	input  wire [127:0] avl_readdata,
	input  wire         avl_readdatavalid,

	// Video out (registered, aligned; syncs active high)
	output reg  [7:0]  red,
	output reg  [7:0]  green,
	output reg  [7:0]  blue,
	output reg         hsync,
	output reg         vsync,
	output reg         de,           // full 1920x1080 active area
	output reg         vblank,       // output-raster vertical blank (ascal o_vbl)
	output reg         border,       // active raster but outside the image window (ascal o_brd)

	// Raster status / frame pacing (video domain)
	output reg  [9:0]  src_line,     // source line currently displayed
	output reg         vblank_in,    // 1-clk pulse: image window finished
	output reg         vblank_out,   // 1-clk pulse: image window starts
	output reg         underrun      // sticky: a line was displayed before its fetch finished
);

// NOTE: no SystemVerilog size casts anywhere in this file - Quartus
// Standard 17.0 does not support N'(expr).
/* verilator lint_off WIDTHTRUNC */
localparam [11:0] H_TOTAL  = H_ACTIVE + H_FP + H_SYNC + H_BP;
localparam [10:0] V_TOTAL  = V_ACTIVE + V_FP + V_SYNC + V_BP;
localparam [11:0] HS_BEG   = H_ACTIVE + H_FP;
localparam [11:0] HS_END   = H_ACTIVE + H_FP + H_SYNC;
localparam [10:0] VS_BEG   = V_ACTIVE + V_FP;
localparam [10:0] VS_END   = V_ACTIVE + V_FP + V_SYNC;
localparam [11:0] H_ACT    = H_ACTIVE;
localparam [10:0] V_ACT    = V_ACTIVE;
localparam [11:0] X0       = (H_ACTIVE - SRC_W*2)/2;        // 320
localparam [11:0] X1       = (H_ACTIVE - SRC_W*2)/2 + SRC_W*2; // 1600
localparam        M15K     = (LINE_MODE != 0);
localparam        ILACE    = (LINE_MODE == 2);
localparam [10:0] Y0       = M15K ? 11'd0 : (V_ACTIVE - SRC_H*2)/2;        // 60
localparam [10:0] Y1       = M15K ? V_ACTIVE : (V_ACTIVE - SRC_H*2)/2 + SRC_H*2; // 1020
localparam [10:0] V_TOTAL1 = V_TOTAL + (ILACE ? 11'd1 : 11'd0);  // 480i field 1: 263
localparam [11:0] HALF_LN  = H_TOTAL / 2;
localparam [10:0] LOOK     = 11'd2;                  // fetch-ahead, output lines
// Request point within the line. 1080p: hcnt 2 (see the look cone note).
// 15 kHz (one buffer per output line, parity = row parity): the target
// buffer's previous occupant is the CURRENT line, so the request fires just
// after the window ends - ~1.2 lines of fetch time for up to 3 lines of
// vertical-filter group, instead of the 1 line of a request at hcnt 2 of
// the following line.
localparam [11:0] REQ_H    = M15K ? (H_ACTIVE - SRC_W*2)/2 + SRC_W*2 + 16 : 2;
localparam        VF2      = M15K && (VFILT_TAPS >= 2); // s+1 tap bank set
localparam        VF3      = M15K && (VFILT_TAPS >= 3); // s-1 tap bank set
localparam  [9:0] SRC_LAST = SRC_H - 1;                 // filter clamp line

localparam        BURST_LEN = 16;        // 128-bit beats per burst (256 bytes)
localparam [7:0]  BC_FULL   = BURST_LEN; // full burstcount
localparam [4:0]  BB_FULL   = BURST_LEN; // full per-burst beat count
localparam [8:0]  B9_FULL   = BURST_LEN;
localparam [27:0] ADDR_STEP = BURST_LEN; // address advance per full burst

// border bands: 640x30 linear, stride 1280 bytes = 80 16-byte words/beats
localparam [10:0] BAND_ADV   = 11'd80;
localparam [8:0]  BAND_BEATS = 9'd80;

// fetch/display region encoding ({region, src line} keys the request dedup)
localparam [1:0] RGN_GAME = 2'd0;
localparam [1:0] RGN_TOP  = 2'd1;
localparam [1:0] RGN_BOT  = 2'd2;
/* verilator lint_on WIDTHTRUNC */

//////////////////////////////////////////////////////////////////////////
// Output raster counters
//////////////////////////////////////////////////////////////////////////

reg [11:0] hcnt = 12'd0;
reg [10:0] vcnt = 11'd0;
reg        field = 1'b0;          // 480i field (always 0 otherwise)
wire [10:0] vtot_cur = field ? V_TOTAL1 : V_TOTAL;

always @(posedge clk or posedge reset) begin
	if (reset) begin
		hcnt  <= 12'd0;
		vcnt  <= 11'd0;
		field <= 1'b0;
	end
	else begin
		if (hcnt == H_TOTAL - 12'd1) begin
			hcnt <= 12'd0;
			if (vcnt == vtot_cur - 11'd1) begin
				vcnt  <= 11'd0;
				if (ILACE) field <= ~field;
			end
			else vcnt <= vcnt + 11'd1;
		end
		else hcnt <= hcnt + 12'd1;
	end
end
// CRT_CTRL: 2-flop synchronizer, adopted once per field at the start of
// vertical blanking (line V_ACT: after the field's last fetch request and
// before the next field's first one at V_TOTAL-2, so a write never tears a
// field), and only when two consecutive samples agree (multi-bit bus).
reg  [2:0] crt_s1  = 3'd0;
reg  [2:0] crt_s2  = 3'd0;
reg        fsw_lat = 1'b0;   // runtime field swap (480i only)
reg  [1:0] vf_lat  = 2'd0;   // vertical filter: 0 off, 1 2-tap, 2 3-tap
always @(posedge clk) begin
	crt_s1 <= crt_ctrl;
	crt_s2 <= crt_s1;
	if (vcnt == V_ACT && hcnt == 12'd0 && crt_s2 == crt_s1) begin
		fsw_lat <= ILACE && crt_s2[0];
		vf_lat  <= (!VF2 || crt_s2[2:1] == 2'd0) ? 2'd0
		         : (!VF3 || crt_s2[2:1] == 2'd1) ? 2'd1 : 2'd2;
	end
end

// field whose lines a source-line mapping uses (FIELD_SWAP / CRT_CTRL[0]
// flip parity)
wire field_src = ILACE ? (field ^ (FIELD_SWAP != 0) ^ fsw_lat) : 1'b0;

wire img_v = (vcnt >= Y0) && (vcnt < Y1);

// Per-frame latch of the quasi-static config (stable for the Avalon domain:
// the first fetch request is issued Y0-2 lines later). The BASE is not here
// - it rides in each line's request payload.
reg [10:0] adv_lat   = 11'd0;   // 16-byte words to advance per source line
reg        dbl_lat   = 1'b0;
reg        pd_lat    = 1'b0;
reg        split_lat = 1'b0;
reg  [1:0] dep_lat   = 2'd0;
reg  [2:0] cat_lat   = 3'd0;
reg        en_lat    = 1'b0;

// Band latches: sampled at the start of vertical blanking (line V_ACT), so
// they are stable before the top band's first request (2 lines before the
// raster wraps) and its display at line 0.
reg [27:0] top_base_lat = 28'd0;
reg [27:0] bot_base_lat = 28'd0;
reg        top_en_lat   = 1'b0;
reg        bot_en_lat   = 1'b0;

// line-doubling shift: output lines per source line = 2 (480) or 4 (240p)
wire [1:0] vshift = dbl_lat ? 2'd2 : 2'd1;

// Per-line base stability filter: {fb_disp_half, fb_base} originates in
// another clock domain and may change mid-frame; adopt a value only after
// it has been sampled identical on two consecutive clocks, so a request
// can never latch a torn word (writes settle in well under a line).
reg [32:0] fbs_q      = 33'd0;
reg [32:0] fbs_stable = 33'd0;

always @(posedge clk) begin
	fbs_q <= {fb_disp_half, fb_base};
	if (fbs_q == {fb_disp_half, fb_base}) fbs_stable <= fbs_q;
end

//////////////////////////////////////////////////////////////////////////
// Fetch requests (video domain -> Avalon domain)
//////////////////////////////////////////////////////////////////////////

// Two output lines ahead of the raster, decide which SOURCE line (of which
// region) will be needed and request it once. Buffer parity = source line
// LSB, so the line being written is never the line being displayed - this
// holds across region seams too (each region restarts at src 0 an even
// number of output lines after the previous region's last line began).
reg        req_toggle = 1'b0;
reg        req_sof    = 1'b0;   // first line of its region: reset the render offset
reg        req_buf    = 1'b0;
reg  [1:0] req_region = RGN_GAME;
reg [27:0] req_base   = 28'd0;  // render base: this line's fb_base, in beats
reg  [8:0] req_beats  = 9'd0;   // beats to fetch (game lines; bands use BAND_BEATS)
reg        req_half   = 1'b0;   // this line's fb_disp_half
reg        req_step2  = 1'b0;   // advance 2 strides (15 kHz line skip)
reg        req_init1  = 1'b0;   // first line starts at +1 stride (480i odd field)
reg  [1:0] req_filt   = 2'd0;   // vertical filter group: 0 centre only, 1 +s+1, 2 +s+1,s-1
reg        req_first  = 1'b0;   // centre is source line 0 (s-1 clamps to s)
reg        req_last   = 1'b0;   // centre is source line SRC_H-1 (s+1 clamps to s)
reg [11:0] last_req   = 12'hFFF;   // {region, src}
reg  [1:0] cnt_req    = 2'd0;

// sub-beat byte offset of each buffer's line, for the read side
reg  [3:0] line_roff [0:1];
initial begin line_roff[0] = 4'd0; line_roff[1] = 4'd0; end
// filter mode each buffer's line group was fetched with, for the read side
reg  [1:0] line_filt [0:1];
initial begin line_filt[0] = 2'd0; line_filt[1] = 2'd0; end

// wraps only for the top band (1080p) / the game window (15 kHz), whose
// first request lands 2 lines before the raster does (V_TOTAL-2 -> line 0;
// 15 kHz: rows 0-1 of a field are thus requested before the per-frame
// config latch at line 0 - a config change fully settles one field later)
wire [10:0] y_look_raw = vcnt + LOOK;
wire        look_wrap  = (y_look_raw >= vtot_cur);
wire [10:0] y_look     = look_wrap ? y_look_raw - vtot_cur : y_look_raw;
wire        look_fld   = ILACE ? (field_src ^ look_wrap) : 1'b0;

wire        look_top  = (y_look < Y0) && top_en_lat;
wire        look_game = (y_look >= Y0) && (y_look < Y1);
wire        look_bot  = (y_look >= Y1) && (y_look < V_ACT) && bot_en_lat;
wire        look_in   = look_top || look_game || look_bot;
wire  [1:0] look_rgn  = look_top ? RGN_TOP : look_game ? RGN_GAME : RGN_BOT;
wire [10:0] look_rel  = look_top  ? y_look
                      : look_game ? y_look - Y0
                                  : y_look - Y1;
wire  [1:0] look_vsh  = look_game ? vshift : 2'd1;   // bands are always 2x
/* verilator lint_off UNUSEDSIGNAL */
wire [10:0] look_shf = look_rel >> look_vsh;
/* verilator lint_on UNUSEDSIGNAL */
// 15 kHz: one source line per output line (every 2nd line of a 480 source,
// the field parity picking even/odd in 480i; every line of a 240p source)
wire  [9:0] look_src = !M15K ? look_shf[9:0]
                     : dbl_lat ? look_rel[9:0]
                               : {look_rel[8:0], look_fld};
wire [11:0] look_key = {look_rgn, look_src};
// vertical filter group of this line (vf_lat is 0 outside the 15 kHz
// modes); bypassed for 240-line sources, where a row IS one source line
wire  [1:0] look_filt = (look_game && !dbl_lat) ? vf_lat : 2'd0;

// game-line fetch length: whole beats covering the FB-view bytes of one
// line (raw layout doubles in split mode), +1 for the 888 packed quirk
// (one byte past SRC_W*3 can be read), +1 when the base is mid-beat
wire  [3:0] look_roff  = split_lat ? {1'b0, fbs_stable[3], 2'b00}
                                   : fbs_stable[3:0];
/* verilator lint_off WIDTHTRUNC */
wire  [8:0] bb_lin     = pd_lat
                       ? ((dep_lat == 2'd2) ? SRC_W*3/32     // 60 (960 bytes)
                        : (dep_lat == 2'd3) ? SRC_W/8        // 80
                                            : SRC_W/16)      // 40
                       : ((dep_lat == 2'd2) ? SRC_W*3/16     // 120 (1920 bytes)
                        : (dep_lat == 2'd3) ? SRC_W/4        // 160
                                            : SRC_W/8);      // 80
/* verilator lint_on WIDTHTRUNC */
wire  [8:0] bb_geo     = split_lat ? {bb_lin[7:0], 1'b0} : bb_lin;
wire  [8:0] look_beats = bb_geo + {8'd0, look_roff != 4'd0};

// look_* are LINE-CONSTANT (vcnt only changes when hcnt wraps): register the
// whole lookahead cone per clock (copies are settled from hcnt==1 on) and
// fire the request at hcnt==2 instead of hcnt==0, so the vcnt+2 wrap adder /
// region subtracts / 12-bit dedup compare never gate the req_* enables
// combinationally (they were an STA-failing family into req_base). The two
// pixel clocks of extra latency are nothing against the ~2-output-line
// fetch-ahead margin, and the per-line config latches at hcnt==0 have
// settled a full cycle before the registered copies are consumed.
reg        look_in_r    = 1'b0;
reg [11:0] look_key_r   = 12'hFFF;
reg  [1:0] look_rgn_r   = 2'd0;
reg        look_sof_r   = 1'b0;
reg        look_buf_r   = 1'b0;
reg        look_game_r  = 1'b0;
reg  [3:0] look_roff_r  = 4'd0;
reg  [8:0] look_beats_r = 9'd0;
reg        look_step2_r = 1'b0;   // 15 kHz skip: advance 2 source lines per request
reg        look_init1_r = 1'b0;   // 480i odd field: region starts 1 source line in
reg  [1:0] look_filt_r  = 2'd0;
reg        look_first_r = 1'b0;
reg        look_last_r  = 1'b0;
always @(posedge clk) begin
	look_filt_r  <= look_filt;
	look_first_r <= (look_src == 10'd0);
	look_last_r  <= (look_src == SRC_LAST);
	look_in_r    <= look_in;
	look_key_r   <= look_key;
	look_rgn_r   <= look_rgn;
	look_sof_r   <= M15K ? (look_rel == 11'd0) : (look_src == 10'd0);
	look_buf_r   <= M15K ? look_rel[0] : look_src[0];
	look_step2_r <= M15K && !dbl_lat;
	look_init1_r <= M15K && !dbl_lat && look_fld;
	look_game_r  <= look_game;
	look_roff_r  <= look_roff;
	look_beats_r <= look_beats;
end

always @(posedge clk or posedge reset) begin
	if (reset) begin
		req_toggle <= 1'b0;
		last_req   <= 12'hFFF;
		cnt_req    <= 2'd0;
	end
	else begin
		if (vcnt == 11'd0 && hcnt == 12'd0) begin
			// DDR bytes per line double in split mode (2 DDR bytes per FB byte)
			adv_lat   <= fb_split ? fb_stride[13:3] : {1'b0, fb_stride[13:4]};
			dbl_lat   <= fb_line_dbl;
			pd_lat    <= fb_pix_dbl;
			split_lat <= fb_split;
			dep_lat   <= fb_depth;
			cat_lat   <= fb_concat;
			en_lat    <= fb_enable;
		end

		if (vcnt == V_ACT && hcnt == 12'd0) begin
			top_base_lat <= fb_top_base[31:4];   // [6:4] zero: 128B aligned
			bot_base_lat <= fb_bot_base[31:4];
			top_en_lat   <= (fb_top_base[31:7] != 25'd0);
			bot_en_lat   <= (fb_bot_base[31:7] != 25'd0);
			last_req     <= 12'hFFF;
		end

		if (hcnt == REQ_H && look_in_r && look_key_r != last_req) begin
			req_sof    <= look_sof_r;
			req_buf    <= look_buf_r;
			req_region <= look_rgn_r;
			req_base   <= fbs_stable[31:4];
			req_half   <= fbs_stable[32];
			req_beats  <= look_beats_r;
			req_step2  <= look_step2_r;
			req_init1  <= look_init1_r;
			req_filt   <= look_filt_r;
			req_first  <= look_first_r;
			req_last   <= look_last_r;
			line_roff[look_buf_r] <= look_game_r ? look_roff_r : 4'd0;
			line_filt[look_buf_r] <= look_filt_r;
			last_req   <= look_key_r;
			cnt_req    <= cnt_req + 2'd1;
			req_toggle <= ~req_toggle;   // payload above is stable when this lands
		end
	end
end

//////////////////////////////////////////////////////////////////////////
// Avalon fetch FSM (avl_clk domain)
//////////////////////////////////////////////////////////////////////////

reg  [2:0] rt_sync     = 3'd0;
reg  [1:0] rst_sync    = 2'd0;
reg        fetching    = 1'b0;
reg        pending     = 1'b0;   // request arrived while still fetching
reg [27:0] line_off    = 28'd0;  // render offset: beats from the base, this region
reg  [8:0] w           = 9'd0;   // beat index within the line
reg  [8:0] beats_left  = 9'd0;   // beats still expected for the line
reg  [4:0] burst_beats = 5'd0;   // beats still expected for the current burst
reg        done_toggle = 1'b0;
reg        cur_split   = 1'b0;   // this fetch's layout: only game can be split
reg        cur_half    = 1'b0;   // ... and only game selects a 32-bit half

// 15 kHz vertical filter group: after the centre line (tap 0, the line an
// unfiltered request fetches) come s+1 (tap 1) and s-1 (tap 2), each a full
// line fetch of the same length into its own bank set at the same buffer
// parity. The group's payload is latched at its start; the next tap's
// start address is a registered base +- one stride (clamped at the source
// edges), settled long before the current line's last beat.
reg  [1:0] cur_tap     = 2'd0;   // line of the group being fetched
reg        grp_buf     = 1'b0;   // target buffer (the payload's req_buf moves on
                                 // with the next request, which a long fetch
                                 // can overlap)
reg  [1:0] grp_filt    = 2'd0;
reg        grp_first   = 1'b0;
reg        grp_last    = 1'b0;
reg  [8:0] grp_beats   = 9'd0;
reg [27:0] grp_addr    = 28'd0;  // centre line's start address
reg [27:0] tap_addr    = 28'd0;  // next tap line's start address

wire req_edge = rt_sync[2] ^ rt_sync[1];
wire req_game = (req_region == RGN_GAME);
wire tap_more = (VF2 && cur_tap == 2'd0 && grp_filt != 2'd0) ||
                (VF3 && cur_tap == 2'd1 && grp_filt == 2'd2);

// requests never interleave regions: line_off accumulates within one region
// and req_sof zeroes it at the region's first line. Bands are still
// base+offset - only their base is the per-frame band latch.
wire [27:0] band_base = (req_region == RGN_TOP) ? top_base_lat : bot_base_lat;

always @(posedge avl_clk) begin : fetch_fsm
	reg [27:0] na;
	reg [27:0] no;
	reg  [8:0] rem;

	// verilator lint_off SYNCASYNCNET
	rst_sync <= {rst_sync[0], reset};
	// verilator lint_on SYNCASYNCNET
	rt_sync  <= {rt_sync[1:0], req_toggle};

	if (avl_read && !avl_waitrequest) avl_read <= 1'b0;   // command accepted

	tap_addr <= (cur_tap == 2'd0) ? grp_addr + (grp_last  ? 28'd0 : {17'd0, adv_lat})
	                              : grp_addr - (grp_first ? 28'd0 : {17'd0, adv_lat});

	if (req_edge && fetching) pending <= 1'b1;

	if ((req_edge || pending) && !fetching) begin
		pending        <= 1'b0;
		no = req_sof ? (req_init1 ? {17'd0, adv_lat} : 28'd0)
		             : line_off + (!req_game ? {17'd0, BAND_ADV}
		                          : req_step2 ? {16'd0, adv_lat, 1'b0}
		                                      : {17'd0, adv_lat});
		line_off       <= no;
		na = (req_game ? req_base : band_base) + no;
		avl_address    <= na;
		avl_burstcount <= BC_FULL;         // every fetch is >= 80 beats: first burst is full
		avl_read       <= 1'b1;
		burst_beats    <= BB_FULL;
		beats_left     <= req_game ? req_beats : BAND_BEATS;
		cur_split      <= req_game & split_lat;
		cur_half       <= req_half;
		w              <= 9'd0;
		fetching       <= 1'b1;
		cur_tap        <= 2'd0;
		grp_buf        <= req_buf;
		grp_addr       <= na;
		grp_filt       <= req_game ? req_filt : 2'd0;
		grp_first      <= req_first;
		grp_last       <= req_last;
		grp_beats      <= req_beats;
	end
	else if (fetching && avl_readdatavalid) begin
		w           <= w + 9'd1;
		beats_left  <= beats_left - 9'd1;
		burst_beats <= burst_beats - 5'd1;
		if (burst_beats == 5'd1) begin                    // last beat of this burst
			rem = beats_left - 9'd1;
			if (rem == 9'd0 && tap_more) begin
				// line done, the group's next tap line starts (same
				// serialization as a burst: the command follows the
				// previous burst's last beat)
				cur_tap        <= cur_tap + 2'd1;
				avl_address    <= tap_addr;
				avl_burstcount <= BC_FULL;
				avl_read       <= 1'b1;
				burst_beats    <= BB_FULL;
				beats_left     <= grp_beats;
				w              <= 9'd0;
			end
			else if (rem == 9'd0) begin
				fetching    <= 1'b0;
				done_toggle <= ~done_toggle;
			end
			else begin
				// every burst except a shorter final one is BURST_LEN long
				avl_address    <= avl_address + ADDR_STEP;
				avl_burstcount <= (rem >= B9_FULL) ? BC_FULL : rem[7:0];
				burst_beats    <= (rem >= B9_FULL) ? BB_FULL : rem[4:0];
				avl_read       <= 1'b1;
			end
		end
	end

	if (rst_sync[1]) begin
		fetching <= 1'b0;
		avl_read <= 1'b0;
		pending  <= 1'b0;
	end
end

//////////////////////////////////////////////////////////////////////////
// Line buffers: four 32-bit banks of 512 (2 buffers x 161 beat-words),
// dual clock. The stored stream is the line's raw FB-view bytes starting
// at its beat-aligned fetch address: FB-view 32-bit word N of the stream
// -> bank N[1:0], address {buffer, N[9:2]}.
//
// Write side, beat w (128 bits): the stream words in this beat are
//   linear: n = 4w + j, j = 0..3, data = beat[32j +: 32]
//   split : n = 2w + h, h = 0..1, data = the selected 32-bit half of
//           beat[64h +: 64]
// (No head/tail trimming: a misaligned base's padding bytes are stored
// and skipped by the read side's byte offset.)
//
// 15 kHz vertical filter: two more bank sets of the same shape hold the
// group's s+1 (tap 1) and s-1 (tap 2) lines; the write side just steers by
// cur_tap, the read side shares the centre set's registered address.
//////////////////////////////////////////////////////////////////////////

wire [10:0] base_n = cur_split ? {1'b0, w, 1'b0} : {w, 2'b00};

wire  [9:0] w0_pre;   // display-side window word (declared ahead of use)
wire        rbuf;
wire [31:0] rq [0:3];
wire [31:0] rqp [0:3];   // tap 1: s+1
wire [31:0] rqm [0:3];   // tap 2: s-1
wire        tap_we = fetching && avl_readdatavalid;

generate
genvar gb;
for (gb = 0; gb < 4; gb = gb + 1) begin : bank
	reg [31:0] mem [0:511];
	reg  [8:0] radr = 9'd0;
	reg [31:0] q = 32'd0;

	// candidate offset for this bank: (gb - base_n) mod 4
	/* verilator lint_off WIDTHTRUNC */
	localparam [1:0] GB = gb;
	/* verilator lint_on WIDTHTRUNC */
	wire  [1:0] o2   = GB - base_n[1:0];
	wire        ok   = cur_split ? (o2 < 2'd2) : 1'b1;
	wire [10:0] n    = base_n + {9'd0, o2};

	wire [63:0] h64  = o2[0] ? avl_readdata[127:64] : avl_readdata[63:0];
	wire [31:0] w32  = o2[1] ? (o2[0] ? avl_readdata[127:96] : avl_readdata[95:64])
	                         : (o2[0] ? avl_readdata[63:32]  : avl_readdata[31:0]);
	// pvr_map32: bank 0 (half=0) = LOW 32 bits, bank 1 = HIGH
	wire [31:0] wd   = cur_split ? (cur_half ? h64[63:32] : h64[31:0]) : w32;

	always @(posedge avl_clk) begin
		if (tap_we && ok && cur_tap == 2'd0) mem[{grp_buf, n[9:2]}] <= wd;
	end

	// read side: this bank holds the unique word of the sliding window
	// w0..w0+3 whose index is GB mod 4 - one address bump when GB has
	// already wrapped past w0 (constantly false for bank 3, by design)
	/* verilator lint_off CMPCONST */
	wire bump = (GB < w0_pre[1:0]);
	/* verilator lint_on CMPCONST */
	always @(posedge clk) begin
		radr <= {rbuf, w0_pre[9:2] + (bump ? 8'd1 : 8'd0)};
		q    <= mem[radr];
	end
	assign rq[gb] = q;

	// vertical filter tap sets (15 kHz builds only)
	if (VF2) begin : tap_p
		reg [31:0] mem_p [0:511];
		reg [31:0] q_p = 32'd0;
		always @(posedge avl_clk) begin
			if (tap_we && ok && cur_tap == 2'd1) mem_p[{grp_buf, n[9:2]}] <= wd;
		end
		always @(posedge clk) q_p <= mem_p[radr];
		assign rqp[gb] = q_p;
	end
	else begin : no_tap_p
		assign rqp[gb] = 32'd0;
	end
	if (VF3) begin : tap_m
		reg [31:0] mem_m [0:511];
		reg [31:0] q_m = 32'd0;
		always @(posedge avl_clk) begin
			if (tap_we && ok && cur_tap == 2'd2) mem_m[{grp_buf, n[9:2]}] <= wd;
		end
		always @(posedge clk) q_m <= mem_m[radr];
		assign rqm[gb] = q_m;
	end
	else begin : no_tap_m
		assign rqm[gb] = 32'd0;
	end
end
endgenerate

//////////////////////////////////////////////////////////////////////////
// Display read path (3-stage: address, RAM read, byte funnel + depth
// convert). Addresses are computed for hcnt+1 so the RAMs see registered
// addresses only; RGB still emerges 2 clocks after the counters, and
// syncs/de/border are piped by 2 to stay aligned.
//////////////////////////////////////////////////////////////////////////

// active display region this line: game window or one of the border bands
wire        top_v = (vcnt < Y0) && top_en_lat;
wire        bot_v = (vcnt >= Y1) && (vcnt < V_ACT) && bot_en_lat;
wire        band_v = top_v || bot_v;

wire [10:0] y_rel   = vcnt - Y0;             // game-relative (underrun check)
wire [10:0] d_rel   = top_v ? vcnt : bot_v ? vcnt - Y1 : y_rel;
wire  [1:0] d_vsh   = band_v ? 2'd1 : vshift;
/* verilator lint_off UNUSEDSIGNAL */
wire [10:0] y_shf   = d_rel >> d_vsh;
wire [11:0] x_pre   = hcnt + 12'd2 - X0;     // lookahead: pixel needed in 2 clks
/* verilator lint_on UNUSEDSIGNAL */
wire  [9:0] src_cur = !M15K ? y_shf[9:0]
                    : dbl_lat ? y_rel[9:0]
                              : {y_rel[8:0], field_src};
wire        dbuf    = M15K ? y_rel[0] : src_cur[0];   // line-buffer parity

// Line-constant display selects, REGISTERED: vcnt only changes when hcnt
// wraps, so per-clock copies are settled from hcnt==1 on - hundreds of
// clocks before the window starts at X0. This keeps the vcnt band compares,
// the region subtract chain and the line_roff lookup out of the per-pixel
// address cone (they headed spg's worst STA family, landing in the bank
// RAMs' read-address registers).
reg        band_v_r   = 1'b0;
reg  [1:0] disp_dep_r = 2'd0;
reg  [3:0] roff_r     = 4'd0;
reg        rbuf_r     = 1'b0;
reg        pd_sel_r   = 1'b0;
reg  [1:0] filt_r     = 2'd0;   // vertical filter of this line's group
always @(posedge clk) begin
	band_v_r   <= band_v;
	disp_dep_r <= band_v ? 2'd1 : dep_lat;   // bands read as 16bpp
	roff_r     <= band_v ? 4'd0 : line_roff[dbuf];
	rbuf_r     <= dbuf;
	pd_sel_r   <= pd_lat && !band_v;
	filt_r     <= band_v ? 2'd0 : line_filt[dbuf];
end
assign rbuf = rbuf_r;

// FB-view byte address of the lookahead pixel within the stored stream:
// roff (the line base's sub-beat offset) + n * bytes-per-pixel. Packed 888
// is STRAIGHT B,G,R at byte 3n - the layout the FB write master's packmode
// 4 emits (refsw2's mode-4 writer). refsw2 Present()'s odd/even +-1 fetch
// quirk is deliberately NOT copied: it contradicts refsw2's own writer
// (overlapping pixels) and rotates channels across the write->scanout
// round trip (SC pre-intro: red logo showed green, teal water pink, plus
// a 1-px comb from the odd/even split - while the core render was right).
// /2: source pixel 0..639; pixel_double game lines: /4, source 0..319
// (bands are always 640 wide at 2x)
//
// The byte address is REGISTERED (b_pre_r) with the lookahead deepened to
// hcnt+2, so the bank RAMs' address registers see only the short bump/
// increment cone off a register slice. Overall RGB alignment vs the
// counters is unchanged: the extra register stage is exactly absorbed by
// the extra pixel of lookahead.
wire  [9:0] n_pre    = pd_sel_r ? x_pre[11:2] : x_pre[10:1];
reg  [11:0] b_pre_r  = 12'd0;
always @(posedge clk) begin
	b_pre_r <=
		((disp_dep_r == 2'd2) ? ({1'b0, n_pre, 1'b0} + {2'b00, n_pre})   // 3n
		:(disp_dep_r == 2'd3) ? {n_pre, 2'b00}                           // 4n
		                      : {1'b0, n_pre, 1'b0})                     // 2n
		+ {8'd0, roff_r};
end
assign w0_pre = b_pre_r[11:2];

wire        img_h   = (hcnt >= X0) && (hcnt < X1);

wire de_c  = (hcnt < H_ACT) && (vcnt < V_ACT);
wire hs_c  = (hcnt >= HS_BEG) && (hcnt < HS_END);
// VS edges must coincide with the HS leading edge (CEA-861; same equation
// as ascal's o_vsv): rise at HS_BEG of line VS_BEG, fall at HS_BEG of line
// VS_END. Toggling at hcnt==0 instead puts the VS edge 2008 px before HS,
// which some HDMI sinks reject as an unsupported mode.
// 480i field 1: the whole vsync pulse sits half a line later (edges at
// HS_BEG + H_TOTAL/2, i.e. mid-line of the following line) -> 262.5-line fields
wire        vs_half = ILACE && field;
wire [10:0] vs_b    = vs_half ? VS_BEG + 11'd1 : VS_BEG;
wire [10:0] vs_e    = vs_half ? VS_END + 11'd1 : VS_END;
wire [11:0] vs_x    = vs_half ? HS_BEG + HALF_LN - H_TOTAL : HS_BEG;
wire vs_c  = (vcnt == vs_b && hcnt >= vs_x) ||
             (vcnt >  vs_b && vcnt <  vs_e) ||
             (vcnt == vs_e && hcnt <  vs_x);
wire vbl_c = (vcnt >= V_ACT);
wire img_c = img_h && (img_v || band_v);

reg [1:0] s1_w0lo = 2'd0, s2_w0lo = 2'd0;   // window rotation, piped with the RAM
reg [1:0] s1_blo  = 2'd0, s2_blo  = 2'd0;   // byte offset within the window
reg [4:0] pipe1   = 5'd0;   // {img, de, hs, vs, vbl}

// Stage-2 pixel extraction from one bank set's four RAM words: rotate the
// window, funnel the pixel's bytes down, convert to {R8, G8, B8}.
function [23:0] px_rgb;
	input [31:0] q0, q1, q2, q3;   // bank 0..3 read data
	input  [1:0] w0lo;             // window rotation
	input  [1:0] blo;              // byte offset within the window
	input        band;             // bands: 565, MSB-replicated
	input  [1:0] dep;
	input  [2:0] cat;
	reg   [31:0] wlo, whi;
	reg   [63:0] s64;
	reg   [31:0] p32;
	reg   [15:0] p16;
	begin
		case (w0lo)
			2'd0:    begin wlo = q0; whi = q1; end
			2'd1:    begin wlo = q1; whi = q2; end
			2'd2:    begin wlo = q2; whi = q3; end
			default: begin wlo = q3; whi = q0; end
		endcase
		s64 = {whi, wlo} >> {blo, 3'b000};
		p32 = s64[31:0];
		p16 = p32[15:0];
		if (band)
			px_rgb = {p16[15:11], p16[15:13], p16[10:5], p16[10:9], p16[4:0], p16[4:2]};
		else case (dep)
			2'd0:    px_rgb = {p16[14:10], cat, p16[9:5], cat, p16[4:0], cat};         // 0555 + fb_concat
			2'd1:    px_rgb = {p16[15:11], cat, p16[10:5], cat[2:1], p16[4:0], cat};  // 565 + fb_concat
			default: px_rgb = p32[23:0];   // 888 packed / 0888: R,G,B = bytes 2,1,0
		endcase
	end
endfunction

always @(posedge clk) begin
	// stage 0/1 companions of the RAM pipeline in the generate above
	s1_w0lo <= w0_pre[1:0];
	s1_blo  <= b_pre_r[1:0];
	s2_w0lo <= s1_w0lo;
	s2_blo  <= s1_blo;
	pipe1   <= {img_c, de_c, hs_c, vs_c, vbl_c};

	// stage 2: rotate the window, funnel the pixel's bytes down, convert.
	// dep_lat/cat_lat/en_lat/band_v_r/filt_r are line-constant, so using
	// them "late" is safe (the affected edge pixels are border-black).
	begin : lane_mux
		reg [23:0] cx, cp, cm;       // centre line s, s+1, s-1
		reg  [8:0] a2r, a2g, a2b;    // 2-tap sums
		reg  [9:0] a3r, a3g, a3b;    // 3-tap sums
		reg  [7:0] r8, g8, b8;
		cx = px_rgb(rq[0],  rq[1],  rq[2],  rq[3],  s2_w0lo, s2_blo, band_v_r, dep_lat, cat_lat);
		// 15 kHz vertical filter taps (one base per group, so every set
		// shares the window rotation and byte offset); rounding half up.
		// Constant zero - hence pruned - without the tap sets (LINE_MODE 0).
		cp = px_rgb(rqp[0], rqp[1], rqp[2], rqp[3], s2_w0lo, s2_blo, 1'b0, dep_lat, cat_lat);
		cm = px_rgb(rqm[0], rqm[1], rqm[2], rqm[3], s2_w0lo, s2_blo, 1'b0, dep_lat, cat_lat);
		// 2-tap: (s + s+1 + 1) >> 1
		a2r = {1'b0, cx[23:16]} + {1'b0, cp[23:16]} + 9'd1;
		a2g = {1'b0, cx[15:8]}  + {1'b0, cp[15:8]}  + 9'd1;
		a2b = {1'b0, cx[7:0]}   + {1'b0, cp[7:0]}   + 9'd1;
		// 3-tap: (s-1 + 2s + s+1 + 2) >> 2
		a3r = {2'd0, cm[23:16]} + {1'b0, cx[23:16], 1'b0} + {2'd0, cp[23:16]} + 10'd2;
		a3g = {2'd0, cm[15:8]}  + {1'b0, cx[15:8],  1'b0} + {2'd0, cp[15:8]}  + 10'd2;
		a3b = {2'd0, cm[7:0]}   + {1'b0, cx[7:0],   1'b0} + {2'd0, cp[7:0]}   + 10'd2;
		if (VF3 && filt_r == 2'd2)
			{r8, g8, b8} = {a3r[9:2], a3g[9:2], a3b[9:2]};
		else if (VF2 && filt_r != 2'd0)
			{r8, g8, b8} = {a2r[8:1], a2g[8:1], a2b[8:1]};
		else
			{r8, g8, b8} = cx;
		if (pipe1[4] && (band_v_r || en_lat)) begin
			red   <= r8;
			green <= g8;
			blue  <= b8;
		end
		else begin
			red   <= 8'd0;
			green <= 8'd0;
			blue  <= 8'd0;
		end
	end
	de     <= pipe1[3];
	hsync  <= pipe1[2];
	vsync  <= pipe1[1];
	vblank <= pipe1[0];
	border <= pipe1[3] & ~pipe1[4];
end

//////////////////////////////////////////////////////////////////////////
// Raster status, frame pacing pulses, underrun detect
//////////////////////////////////////////////////////////////////////////

reg [2:0] dt_sync  = 3'd0;
reg [1:0] cnt_done = 2'd0;

always @(posedge clk or posedge reset) begin
	if (reset) begin
		dt_sync    <= 3'd0;
		cnt_done   <= 2'd0;
		underrun   <= 1'b0;
		vblank_in  <= 1'b0;
		vblank_out <= 1'b0;
		src_line   <= 10'd0;
	end
	else begin
		dt_sync <= {dt_sync[1:0], done_toggle};
		if (dt_sync[2] ^ dt_sync[1]) cnt_done <= cnt_done + 2'd1;

		src_line <= img_v ? src_cur : 10'd0;

		vblank_in  <= (vcnt == Y1 && hcnt == 12'd0);
		vblank_out <= (vcnt == Y0 && hcnt == 12'd0);

		// Just before the window pixels of the FIRST output line of each
		// source line: the fetch for this line (issued 2 output lines ago)
		// must have completed; only the next line's fetch may be in flight.
		if (img_v && hcnt == X0 - 12'd4 && (M15K || y_rel[0] == 1'b0)
		    && (cnt_req - cnt_done) >= 2'd2) underrun <= 1'b1;
	end
end

endmodule
