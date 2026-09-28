// spg 15 kHz check (LINE_MODE 1 = 240p, 2 = 480i), built once per mode by
// `make spg-15k` with the CRT build's 1716 x 262 raster and -DLMODE=n
// (optional -DFSWAP=1 / -DVFT=2 mirror -GFIELD_SWAP=1 / -GVFILT_TAPS=2):
//  - raster: 1716-clk lines, 240 active lines per field; 480i vsync phase
//    alternating 0/858 (the half-line-late vsync ends DUT field 1) with
//    262.5-line field spacing, 240p 262. The CRT's top field is measured
//    (shorter vsync -> first-active-line distance) and reported with the
//    source parity it carries;
//  - every active pixel of every checked field against a C++ model, split
//    VRAM (fb_split=1, half 0), over a list of cases: fb_depth 0555/565/888/
//    0888 (+fb_concat), a misaligned base (odd 32-bit word: +1 beat),
//    pixel_double, 240-line (fb_line_dbl) sources;
//  - crt_ctrl (the CRT_CTRL register): [0] field swap flips the 480i parity
//    (240p ignores it); [2:1] vertical filter, per 8-bit channel after
//    depth expansion, output row r showing source line s = 2r + f (480i:
//    f = DUT field ^ FIELD_SWAP ^ swap; 240p: f = 0), neighbours clamped to
//    source lines 0..479, rounding half up:
//      0 off   : L[s]                    (the unchanged, unfiltered path)
//      1 2-tap : (L[s] + L[s+1] + 1) >> 1
//      2 3-tap : (L[s-1] + 2 L[s] + L[s+1] + 2) >> 2      (3 = reserved = 2)
//    240-line sources bypass the filter (row r = source line r);
//  - a CRT_CTRL write mid-field only lands at the next field (no tearing);
//  - no underrun anywhere, incl. the worst case (0888, 640 wide, misaligned
//    base, 3-tap = 3 x 321 beats per output line) with the Avalon side at
//    100 MHz vs the 27 MHz pixel clock, 24-cycle read latency and random
//    waitrequest/readdatavalid gaps - and a final stress run with the
//    Avalon clock at the PIXEL clock (1:1, no latency, as the old tb).
#include "Vspg.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <deque>
#include <vector>
#include <algorithm>
#ifndef LMODE
#define LMODE 2
#endif
#ifndef FSWAP
#define FSWAP 0
#endif
#ifndef VFT
#define VFT 3
#endif
static const int H_TOT = 1716, V_ACT = 240, X0 = 80, X1 = 1360, SRC_H = 480;
static Vspg* dut; static int errors = 0;
static uint8_t pat(uint64_t a) { uint32_t v = (uint32_t)a * 2654435761u; return (uint8_t)(v >> 24); }

// ---- surface config: drives the DUT and the model ----
struct Cfg {
    const char* name;
    int depth, concat, pd, dbl, crt;
    uint32_t base_fb;   // FB-view byte address of line 0 (split: fb_base = 2x)
    int mid_crt;        // >= 0: CRT_CTRL write at row 100 of the first checked field
    int nfields;        // checked fields
    int stress;         // Avalon at the pixel clock, no latency
};
static Cfg g;
static int exp_crt = 0;   // CRT_CTRL in effect for the field being displayed

static uint64_t f2d(uint64_t A) { return (A >> 2) * 8 + (A & 3); }   // split, half 0
static uint8_t fb8(uint64_t A) { return pat(f2d(A)); }
static int bpp(int d) { return d == 2 ? 3 : d == 3 ? 4 : 2; }
static void src_rgb(int s, int n, int* c) {
    uint64_t F = g.base_fb + (uint64_t)s * ((g.pd ? 320 : 640) * bpp(g.depth));
    if (g.depth <= 1) {
        uint16_t p = (uint16_t)(fb8(F + 2 * n) | (fb8(F + 2 * n + 1) << 8));
        if (g.depth == 0) { c[0] = ((p >> 10) & 0x1F) << 3 | g.concat; c[1] = ((p >> 5) & 0x1F) << 3 | g.concat; }
        else              { c[0] = ((p >> 11) & 0x1F) << 3 | g.concat; c[1] = ((p >> 5) & 0x3F) << 2 | (g.concat >> 1); }
        c[2] = (p & 0x1F) << 3 | g.concat;
    } else {   // 888 packed / 0888: B,G,R at byte bpp*n
        uint64_t f = F + (uint64_t)bpp(g.depth) * n;
        c[2] = fb8(f); c[1] = fb8(f + 1); c[0] = fb8(f + 2);
    }
}
static int filt_mode(int crt) {
    int m = (crt >> 1) & 3;
    if (m == 3) m = 2;              // reserved -> 3-tap
    if (VFT < 3 && m == 2) m = 1;   // no s-1 bank set: 2-tap
    if (VFT < 2) m = 0;
    return m;
}
static int src_line(int row, int dut_field) {
    int f = (LMODE == 2) ? (dut_field ^ FSWAP ^ (exp_crt & 1)) : 0;
    return 2 * row + f;
}
static void expect_px(int x, int row, int dut_field, int* c) {
    c[0] = c[1] = c[2] = 0;
    if (x < X0 || x >= X1) return;
    int n = g.pd ? (x - X0) >> 2 : (x - X0) >> 1;
    if (g.dbl) { src_rgb(row, n, c); return; }   // 240-line source: bypass
    int s = src_line(row, dut_field), m = filt_mode(exp_crt);
    int a[3], b[3];
    src_rgb(s, n, c);
    if (m == 0) return;
    src_rgb(std::min(s + 1, SRC_H - 1), n, b);
    if (m == 1) { for (int i = 0; i < 3; i++) c[i] = (c[i] + b[i] + 1) >> 1; return; }
    src_rgb(std::max(s - 1, 0), n, a);
    for (int i = 0; i < 3; i++) c[i] = (a[i] + 2 * c[i] + b[i] + 2) >> 2;
}

// ---- dual-clock Avalon DDR model ----
static uint32_t lfsr = 0xBEEF;
static int rnd4() { lfsr = (lfsr >> 1) ^ (-(int)(lfsr & 1) & 0xB400u); return lfsr & 3; }
struct Beat { uint64_t wa, ready; };
static std::deque<Beat> owed;
static const uint64_t CLK_PS = 37037;              // 27 MHz pixel clock
static uint64_t avl_ps = 10000, ddr_lat = 24;      // 100 MHz Avalon, read latency (avl cycles)
static uint64_t t_clk = 0, t_avl = 0, avl_cyc = 0;
static long long t = 0;                            // pixel clocks
static void avl_inputs() {
    dut->avl_waitrequest = (rnd4() == 0);
    if (dut->avl_read && !dut->avl_waitrequest)
        for (uint32_t b = 0; b < dut->avl_burstcount; b++)
            owed.push_back({(uint64_t)dut->avl_address + b, avl_cyc + ddr_lat});
    dut->avl_readdatavalid = 0;
    if (!owed.empty() && owed.front().ready <= avl_cyc && rnd4() != 1) {
        uint64_t wa = owed.front().wa; owed.pop_front();
        for (int i = 0; i < 4; i++) { uint32_t v = 0; for (int k = 0; k < 4; k++) v |= (uint32_t)pat(wa * 16 + 4 * i + k) << (8 * k); dut->avl_readdata[i] = v; }
        dut->avl_readdatavalid = 1;
    }
}
static void tick() {   // run through the next pixel-clock rising edge
    for (;;) {
        uint64_t tn = std::min(t_clk, t_avl);
        bool c = (t_clk == tn), a = (t_avl == tn);
        if (c) dut->clk = 0;
        if (a) dut->avl_clk = 0;
        dut->eval();
        if (a) avl_inputs();
        if (c) dut->clk = 1;
        if (a) dut->avl_clk = 1;
        dut->eval();
        if (a) { t_avl += avl_ps; avl_cyc++; }
        if (c) { t_clk += CLK_PS; t++; return; }
    }
}

static const uint32_t B0 = 0x00200000;
static bool apply(const Cfg& c) {   // true when a per-frame (non-CRT_CTRL) input changed
    bool ch = c.depth != g.depth || c.concat != g.concat || c.pd != g.pd || c.dbl != g.dbl ||
              c.base_fb != g.base_fb || c.stress != g.stress;
    g = c;
    dut->fb_base = g.base_fb * 2; dut->fb_stride = (g.pd ? 320 : 640) * bpp(g.depth);
    dut->fb_depth = g.depth; dut->fb_concat = g.concat; dut->fb_pix_dbl = g.pd; dut->fb_line_dbl = g.dbl;
    dut->crt_ctrl = g.crt;
    if (g.stress) { avl_ps = CLK_PS; ddr_lat = 0; t_avl = t_clk; }
    return ch;
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv); dut = new Vspg;
    const Cfg cases[] = {
        // name                          dep cat pd dbl crt base     mid nf stress
        {"565 off",                        1, 0, 0, 0, 0, B0,      -1, 2, 0},
        {"565 off swap",                   1, 0, 0, 0, 1, B0,      -1, 2, 0},
        {"565 2tap",                       1, 0, 0, 0, 2, B0,      -1, 2, 0},
        {"565 2tap swap",                  1, 0, 0, 0, 3, B0,      -1, 2, 0},
        {"565 3tap",                       1, 0, 0, 0, 4, B0,      -1, 2, 0},
        {"565 3tap swap",                  1, 0, 0, 0, 5, B0,      -1, 2, 0},
        {"565 rsv(=3tap)",                 1, 0, 0, 0, 6, B0,      -1, 2, 0},
        {"565 rsv swap",                   1, 0, 0, 0, 7, B0,      -1, 2, 0},
        {"565+cat3 3tap swap",             1, 3, 0, 0, 5, B0,      -1, 2, 0},
        {"0555+cat5 2tap",                 0, 5, 0, 0, 2, B0,      -1, 2, 0},
        {"0555+cat5 3tap swap",            0, 5, 0, 0, 5, B0,      -1, 2, 0},
        {"0888 off",                       3, 0, 0, 0, 0, B0,      -1, 2, 0},
        {"0888 off swap",                  3, 0, 0, 0, 1, B0,      -1, 2, 0},
        {"0888 2tap",                      3, 0, 0, 0, 2, B0,      -1, 2, 0},
        {"0888 2tap swap",                 3, 0, 0, 0, 3, B0,      -1, 2, 0},
        {"0888 3tap",                      3, 0, 0, 0, 4, B0,      -1, 2, 0},
        {"0888 3tap swap",                 3, 0, 0, 0, 5, B0,      -1, 2, 0},
        {"888 2tap odd-base",              2, 0, 0, 0, 3, B0 + 4,  -1, 2, 0},
        {"888 3tap odd-base",              2, 0, 0, 0, 4, B0 + 4,  -1, 2, 0},
        {"0888 3tap odd-base (worst)",     3, 0, 0, 0, 5, B0 + 4,  -1, 2, 0},
        {"565 pd 3tap",                    1, 0, 1, 0, 4, B0,      -1, 2, 0},
        {"0888 pd 2tap swap",              3, 0, 1, 0, 3, B0,      -1, 2, 0},
        {"dbl 565 off",                    1, 0, 0, 1, 0, B0,      -1, 2, 0},
        {"dbl 565 3tap (bypass)",          1, 0, 0, 1, 4, B0,      -1, 2, 0},
        {"dbl 0888 2tap swap (bypass)",    3, 0, 0, 1, 3, B0,      -1, 2, 0},
        {"mid-field write off->3tap",      1, 0, 0, 0, 0, B0,       4, 3, 0},
        {"mid-field write 3tap->swap",     1, 0, 0, 0, 4, B0,       1, 3, 0},
        {"1:1 stress 0888 3tap odd-base",  3, 0, 0, 0, 4, B0 + 4,  -1, 2, 1},
    };
    const int NC = sizeof(cases) / sizeof(cases[0]);

    dut->fb_split = 1; dut->fb_disp_half = 0; dut->fb_enable = 1;
    dut->fb_top_base = 0; dut->fb_bot_base = 0;
    g = cases[0]; apply(cases[0]);
    dut->reset = 1; for (int i = 0; i < 20; i++) tick(); dut->reset = 0;

    int ci = 0, case_field = 0, case_err = 0, skip = 1; bool checked = false;
    int field_idx = 0, cx = 0, cy = -1, prev_de = 0, prev_hs = 0, prev_vs = 0; long gap = 1000000;
    long long last_hs = -1, hs_ref = -1, last_vs = -1;
    std::vector<long long> vs_times; std::vector<int> vs_phase, vs_field;
    long long vs2pic[2] = {-1, -1};   // vsync -> first active line, per DUT field
    bool under_seen = false;
    while (ci < NC && t < 200000000LL) {
        tick();
        if (dut->hsync && !prev_hs) {
            if (last_hs >= 0 && t - last_hs != H_TOT) { if (errors < 10) printf("H period %lld at t=%lld\n", t - last_hs, t); errors++; }
            last_hs = t; if (hs_ref < 0) hs_ref = t;
        }
        if (dut->vsync && !prev_vs) {
            vs_times.push_back(t); vs_phase.push_back((int)(((t - hs_ref) % H_TOT + H_TOT) % H_TOT));
            vs_field.push_back((field_idx - 1) & 1); last_vs = t;
        }
        if (dut->de) {
            if (!prev_de) {
                cx = 0; if (gap > 10000) cy = 0; else cy++;
                if (cy == 0 && last_vs >= 0 && field_idx >= 2) vs2pic[field_idx & 1] = t - last_vs;
            }
            gap = 0;
            if (checked && case_field == 0 && g.mid_crt >= 0 && cy == 100 && cx == 0) dut->crt_ctrl = g.crt = g.mid_crt;
            if (checked && cy >= 0) {
                int e[3]; expect_px(cx, cy, field_idx & 1, e);
                if (dut->red != e[0] || dut->green != e[1] || dut->blue != e[2]) {
                    if (errors < 12) printf("  [%s] field %d (%d,%d) src %d crt %d: got %02x%02x%02x want %02x%02x%02x\n",
                                            g.name, field_idx, cx, cy, g.dbl ? cy : src_line(cy, field_idx & 1), exp_crt,
                                            dut->red, dut->green, dut->blue, e[0], e[1], e[2]);
                    errors++; case_err++;
                }
            }
            cx++;
        } else {
            gap++;
            if (prev_de && cy == V_ACT - 1) {   // end of a field's active area
                field_idx++;
                if (checked) case_field++;
                if (dut->underrun && !under_seen) { printf("  UNDERRUN during/before [%s]\n", g.name); under_seen = true; errors++; case_err++; }
                if (case_field >= g.nfields) {
                    printf("%-32s %s\n", g.name, case_err ? "FAIL" : "ok");
                    if (++ci < NC) { skip = apply(cases[ci]) ? 1 : 0; case_field = 0; case_err = 0; }
                }
                checked = (skip == 0); if (skip > 0) skip--;
                exp_crt = g.crt;   // adopted by the DUT at line V_ACT, just ahead
            }
        }
        prev_de = dut->de; prev_hs = dut->hsync; prev_vs = dut->vsync;
    }
    if (ci < NC) { printf("timeout in case %d\n", ci); errors++; }

    printf("vsync (lines/phase): ");
    for (size_t i = 1; i < vs_times.size() && i < 9; i++) printf("%.1f/%d ", (vs_times[i] - vs_times[i - 1]) / (double)H_TOT, vs_phase[i]);
    printf("...\n");
    for (size_t i = 2; i < vs_times.size(); i++) {
        double lines = (vs_times[i] - vs_times[i - 1]) / (double)H_TOT;
        double want = (LMODE == 2) ? 262.5 : 262.0;
        if (lines != want) { printf("vsync spacing %.2f != %.1f\n", lines, want); errors++; }
        // the half-line-late vsync is DUT field 1's (field_idx parity == DUT field)
        int want_ph = (LMODE == 2 && vs_field[i]) ? H_TOT / 2 : 0;
        if (vs_phase[i] != want_ph) { printf("vsync %zu phase %d != %d\n", i, vs_phase[i], want_ph); errors++; }
    }
    if (LMODE == 2) {
        int top = vs2pic[1] < vs2pic[0] ? 1 : 0;
        printf("480i: vsync->picture %.3f / %.3f lines (DUT field 0 / 1): top field = DUT field %d,\n"
               "      it shows the %s source lines with CRT_CTRL[0]=0, %s with CRT_CTRL[0]=1 (FIELD_SWAP=%d)\n",
               vs2pic[0] / (double)H_TOT, vs2pic[1] / (double)H_TOT, top,
               ((top ^ FSWAP) & 1) ? "ODD" : "EVEN", ((top ^ FSWAP ^ 1) & 1) ? "ODD" : "EVEN", FSWAP);
        if (vs2pic[0] - vs2pic[1] != H_TOT / 2 && vs2pic[1] - vs2pic[0] != H_TOT / 2) { printf("fields not half a line apart\n"); errors++; }
    }
    if (dut->underrun && !under_seen) { printf("UNDERRUN\n"); errors++; }
    printf("LINE_MODE %d FIELD_SWAP %d VFILT_TAPS %d: %d fields, %s (%d errors)\n", LMODE, FSWAP, VFT, field_idx, errors ? "FAIL" : "PASS", errors);
    dut->final(); delete dut; return errors ? 1 : 0;
}
