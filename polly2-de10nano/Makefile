# polly2-de10nano - display/DDR-path testbenches (Verilator)
#
#   make            - build + run all testbenches
#   make spg-sync   - spg.sv sync-geometry check: HS/VS periods and widths,
#                     VS edges aligned to the HS leading edge (CEA-861),
#                     DE exactly 1920x1080 per frame
#   make pvr-burst  - simplex_pvr_top DDR masters vs core reset: an in-flight
#                     write burst completes under reset (a cut burst desyncs
#                     the HPS f2sdram bridge until the fabric is reloaded),
#                     stale read beats are swallowed without reaching the core
#   make clean

VERILATOR ?= verilator

NJOBS := $(shell nproc 2>/dev/null || echo 4)

VFLAGS = -Wno-WIDTH -Wno-UNUSEDSIGNAL -j $(NJOBS) --build-jobs $(NJOBS) \
         -CFLAGS "-O3"

TSP_PKG = ../polly2-core/rtl/gen/tsp_pkg.sv
# polly2-core video-out (framebuffer write master) instantiated by simplex_pvr_top
VO_RTL  = ../polly2-core/rtl/vo.sv

all: spg-sync pvr-burst spg-loopback audio-i2s spg-bands spg-depth spg-240p spg-15k fb-write render-irq

#   make fb-write - FB write master formats: every FB_W_CTRL packmode
#                   (0555K/565/4444/1555/888/0888K/8888, refsw2 dither
#                   quantization) x 32-bit split area / dense 64-bit
#                   render-to-texture mirror (FB_W_SOF1[24]); exact DDR
#                   byte-map compare incl. byte enables
fb-write: obj_dir/fb_write_tb
	./obj_dir/fb_write_tb

obj_dir/fb_write_tb: rtl/simplex_pvr_top.sv $(VO_RTL) tb/peel_core_stub.sv tb/fb_write_tb.cpp $(TSP_PKG)
	$(VERILATOR) --cc --exe --build $(VFLAGS) \
	    -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINMISSING \
	    -I../polly2-core/rtl/gen \
	    --top-module simplex_pvr_top \
	    $(TSP_PKG) tb/peel_core_stub.sv rtl/simplex_pvr_top.sv $(VO_RTL) tb/fb_write_tb.cpp \
	    -o fb_write_tb

#   make spg-depth - FB_R_CTRL read formats + per-line FB_R_SOF: all four
#                    fb_depth modes (linear + split, misaligned bases,
#                    fb_concat, the packed-888 fetch quirk) against a
#                    refsw2 Present() software model; mid-frame SOF change
#                    lands line-exactly; fb_enable=0 blanks the window
spg-depth: obj_dir/spg_depth_tb
	./obj_dir/spg_depth_tb

obj_dir/spg_depth_tb: rtl/spg.sv tb/spg_depth_tb.cpp
	$(VERILATOR) --cc --exe --build $(VFLAGS) rtl/spg.sv tb/spg_depth_tb.cpp -o spg_depth_tb

#   make spg-240p  - 240p (line-doubled) display: a 640x240 source shown at
#                    4x vertically (four output lines per source line, last
#                    source line 239), every fb_depth, linear + split,
#                    composed with pixel_double, bands still 2x
spg-240p: obj_dir/spg_240p_tb
	./obj_dir/spg_240p_tb

obj_dir/spg_240p_tb: rtl/spg.sv tb/spg_240p_tb.cpp
	$(VERILATOR) --cc --exe --build $(VFLAGS) rtl/spg.sv tb/spg_240p_tb.cpp -o spg_240p_tb

#   make render-irq - irq_stretch (render-done -> f2h IRQ1): a done edge
#                     gives exactly one 64-cycle pulse (a held level does
#                     not retrigger), edges mid-stretch coalesce without
#                     the line dropping, a later edge gives a fresh pulse
render-irq: obj_dir/irq_stretch_tb
	./obj_dir/irq_stretch_tb

obj_dir/irq_stretch_tb: rtl/irq_stretch.sv tb/irq_stretch_tb.cpp
	$(VERILATOR) --cc --exe --build $(VFLAGS) rtl/irq_stretch.sv tb/irq_stretch_tb.cpp -o irq_stretch_tb

#   make spg-bands - border bands: per-pixel compare of full frames (top
#                    band / game window / bottom band / black borders),
#                    band enable/disable latching at vblank, no underrun
spg-bands: obj_dir/spg_bands_tb
	./obj_dir/spg_bands_tb

obj_dir/spg_bands_tb: rtl/spg.sv tb/spg_bands_tb.cpp
	$(VERILATOR) --cc --exe --build $(VFLAGS) rtl/spg.sv tb/spg_bands_tb.cpp -o spg_bands_tb

spg-sync: obj_dir/spg_sync_tb
	./obj_dir/spg_sync_tb

#   make spg-15k   - 15 kHz CRT modes on the CRT build's 1716 x 262 raster
#                    (LINE_MODE 1 = 240p, 2 = 480i): line/field timing, 480i
#                    half-line vsync + measured top-field parity, every pixel
#                    vs a model over fb_depth / pixel_double / 240-line
#                    sources, CRT_CTRL field swap and 2/3-tap vertical
#                    filter (exact model incl. edge clamp and rounding),
#                    CRT_CTRL adopted at field boundaries only, no underrun
#                    at the 0888 3-tap worst case; plus a 480i build with
#                    FIELD_SWAP=1 and VFILT_TAPS=2 (2-tap-only hardware)
SPG15K_V = mkdir -p obj_dir && $(VERILATOR) --cc --exe --build $(VFLAGS) -Wno-UNSIGNED \
           -GH_ACTIVE=1440 -GH_FP=38 -GH_SYNC=124 -GH_BP=114 \
           -GV_ACTIVE=240 -GV_FP=4 -GV_SYNC=3 -GV_BP=15

spg-15k: obj_dir/spg15k_1/spg_15k_tb obj_dir/spg15k_2/spg_15k_tb obj_dir/spg15k_2s/spg_15k_tb
	./obj_dir/spg15k_1/spg_15k_tb
	./obj_dir/spg15k_2/spg_15k_tb
	./obj_dir/spg15k_2s/spg_15k_tb

obj_dir/spg15k_1/spg_15k_tb: rtl/spg.sv tb/spg_15k_tb.cpp
	$(SPG15K_V) -GLINE_MODE=1 -CFLAGS -DLMODE=1 --Mdir obj_dir/spg15k_1 \
	    rtl/spg.sv tb/spg_15k_tb.cpp -o spg_15k_tb

obj_dir/spg15k_2/spg_15k_tb: rtl/spg.sv tb/spg_15k_tb.cpp
	$(SPG15K_V) -GLINE_MODE=2 -CFLAGS -DLMODE=2 --Mdir obj_dir/spg15k_2 \
	    rtl/spg.sv tb/spg_15k_tb.cpp -o spg_15k_tb

obj_dir/spg15k_2s/spg_15k_tb: rtl/spg.sv tb/spg_15k_tb.cpp
	$(SPG15K_V) -GLINE_MODE=2 -GFIELD_SWAP=1 -GVFILT_TAPS=2 \
	    -CFLAGS "-DLMODE=2 -DFSWAP=1 -DVFT=2" --Mdir obj_dir/spg15k_2s \
	    rtl/spg.sv tb/spg_15k_tb.cpp -o spg_15k_tb

#   make audio-i2s - MMIO AUDIO_DATA -> 2048-entry async FIFO -> I2S:
#                    framing/order/one-shot delivery, blocking-write
#                    backpressure when full, non-audio accesses unaffected,
#                    full-drain integrity (no loss or duplication)
audio-i2s: obj_dir/audio_i2s_tb
	./obj_dir/audio_i2s_tb

obj_dir/audio_i2s_tb: rtl/pvr_mmio.sv rtl/audio_i2s.sv tb/audio_tb_top.sv tb/audio_i2s_tb.cpp
	$(VERILATOR) --cc --exe --build $(VFLAGS) \
	    -Wno-PINCONNECTEMPTY --top-module audio_tb_top \
	    rtl/pvr_mmio.sv rtl/audio_i2s.sv tb/audio_tb_top.sv tb/audio_i2s_tb.cpp \
	    -o audio_i2s_tb

obj_dir/spg_sync_tb: rtl/spg.sv tb/spg_sync_tb.cpp
	$(VERILATOR) --cc --exe --build $(VFLAGS) rtl/spg.sv tb/spg_sync_tb.cpp -o spg_sync_tb

pvr-burst: obj_dir/pvr_burst_tb
	./obj_dir/pvr_burst_tb

#   make spg-loopback - stream a full frame through the FB write master in
#                       tile order, scan it out through spg, compare every
#                       window pixel (write layout vs scanout layout)
spg-loopback: obj_dir/spg_loopback_tb
	./obj_dir/spg_loopback_tb

obj_dir/spg_loopback_tb: rtl/simplex_pvr_top.sv $(VO_RTL) rtl/spg.sv tb/spg_loopback_top.sv tb/peel_core_stub.sv tb/spg_loopback_tb.cpp $(TSP_PKG)
	$(VERILATOR) --cc --exe --build $(VFLAGS) \
	    -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINMISSING \
	    -I../polly2-core/rtl/gen \
	    --top-module spg_loopback_top \
	    $(TSP_PKG) tb/peel_core_stub.sv rtl/simplex_pvr_top.sv $(VO_RTL) rtl/spg.sv \
	    tb/spg_loopback_top.sv tb/spg_loopback_tb.cpp \
	    -o spg_loopback_tb

obj_dir/pvr_burst_tb: rtl/simplex_pvr_top.sv $(VO_RTL) tb/peel_core_stub.sv tb/pvr_burst_tb.cpp $(TSP_PKG)
	$(VERILATOR) --cc --exe --build $(VFLAGS) \
	    -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINMISSING \
	    -I../polly2-core/rtl/gen \
	    --top-module simplex_pvr_top \
	    $(TSP_PKG) tb/peel_core_stub.sv rtl/simplex_pvr_top.sv $(VO_RTL) tb/pvr_burst_tb.cpp \
	    -o pvr_burst_tb

clean:
	rm -rf obj_dir

.PHONY: all spg-sync pvr-burst spg-loopback audio-i2s spg-bands spg-depth spg-240p spg-15k fb-write render-irq clean
