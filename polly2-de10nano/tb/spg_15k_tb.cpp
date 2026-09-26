// spg 15 kHz (LINE_MODE 1 = 240p, 2 = 480i) check: 1716-clk lines, 240
// active lines/field, 480i half-line vsync phase + 262.5-line field spacing,
// every active pixel vs a model (split VRAM, 565), fb_line_dbl sources, no underrun.
#include "Vspg.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <deque>
#include <vector>
#ifndef LMODE
#define LMODE 2
#endif
static const int H_TOT = 1716, H_ACT = 1440, V_ACT = 240, X0 = 80, X1 = 1360;
static const int SRC_W = 640;
static Vspg* dut; static int errors = 0;
static uint8_t pat(uint64_t a) { uint32_t v = (uint32_t)a * 2654435761u; return (uint8_t)(v >> 24); }
static const uint64_t BASE_FB = 0x00200000; static int g_dbl = 0;
static uint64_t f2d(uint64_t A) { return (A >> 2) * 8 + (A & 3); }         // split, half 0
static uint16_t fb16(uint64_t A) { return (uint16_t)(pat(f2d(A)) | (pat(f2d(A + 1)) << 8)); }
static void expect_px(int x, int s, uint8_t* r, uint8_t* g, uint8_t* b) {
    *r = *g = *b = 0; if (x < X0 || x >= X1) return;
    int n = (x - X0) >> 1; uint16_t p = fb16(BASE_FB + (uint64_t)s * 1280 + 2 * n);
    *r = (uint8_t)(((p >> 11) & 0x1F) << 3); *g = (uint8_t)(((p >> 5) & 0x3F) << 2); *b = (uint8_t)((p & 0x1F) << 3);
}
static uint32_t lfsr = 0xBEEF;
static int rnd4() { lfsr = (lfsr >> 1) ^ (-(int)(lfsr & 1) & 0xB400u); return lfsr & 3; }
static std::deque<uint64_t> owed; static long long t = 0;
static void tick() {
    dut->clk = 0; dut->avl_clk = 0; dut->eval();
    dut->avl_waitrequest = (rnd4() == 0);
    if (dut->avl_read && !dut->avl_waitrequest)
        for (uint32_t b = 0; b < dut->avl_burstcount; b++) owed.push_back((uint64_t)dut->avl_address + b);
    dut->avl_readdatavalid = 0;
    if (!owed.empty() && rnd4() != 1) {
        uint64_t wa = owed.front(); owed.pop_front();
        for (int i = 0; i < 4; i++) { uint32_t v = 0; for (int k = 0; k < 4; k++) v |= (uint32_t)pat(wa * 16 + 4 * i + k) << (8 * k); dut->avl_readdata[i] = v; }
        dut->avl_readdatavalid = 1;
    }
    dut->clk = 1; dut->avl_clk = 1; dut->eval(); t++;
}
int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv); dut = new Vspg;
    dut->fb_base = (uint32_t)(BASE_FB * 2); dut->fb_stride = 1280; dut->fb_split = 1; dut->fb_disp_half = 0;
    dut->fb_depth = 1; dut->fb_concat = 0; dut->fb_enable = 1; dut->fb_pix_dbl = 0; dut->fb_line_dbl = 0;
    dut->fb_top_base = 0; dut->fb_bot_base = 0;
    dut->reset = 1; for (int i = 0; i < 20; i++) tick(); dut->reset = 0;
    int field_idx = 0, cx = 0, cy = -1, prev_de = 0, prev_hs = 0, prev_vs = 0; long gap = 1000000;
    long long last_hs = -1, last_vs = -1, hs_ref = -1; int lines_in_field = 0;
    std::vector<long long> vs_times; std::vector<int> vs_phase, active_lines;
    const int FIELDS = 12;
    while (field_idx < FIELDS && t < 30000000LL) {
        if (field_idx == 6 && cy == 0 && cx == 0 && !g_dbl) {}  // placeholder
        tick();
        if (dut->hsync && !prev_hs) { if (last_hs >= 0 && t - last_hs != H_TOT) { if (errors < 10) printf("H period %lld at t=%lld\n", t - last_hs, t); errors++; } last_hs = t; if (hs_ref < 0) hs_ref = t; }
        if (dut->vsync && !prev_vs) { vs_times.push_back(t); vs_phase.push_back((int)(((t - hs_ref) % H_TOT + H_TOT) % H_TOT)); }
        if (dut->de) {
            if (!prev_de) { cx = 0; if (gap > 10000) cy = 0; else cy++; }
            gap = 0;
            if (field_idx >= 2 && field_idx != 7 && cy >= 0) {
                int f = (LMODE == 2) ? (field_idx & 1) : 0;
                int s = g_dbl ? cy : 2 * cy + f;
                uint8_t er, eg, eb; expect_px(cx, s, &er, &eg, &eb);
                if (dut->red != er || dut->green != eg || dut->blue != eb) {
                    if (errors < 10) printf("field %d (%d,%d) src %d: got %02x%02x%02x want %02x%02x%02x\n", field_idx, cx, cy, s, dut->red, dut->green, dut->blue, er, eg, eb);
                    errors++;
                }
            }
            cx++;
        } else {
            gap++;
            if (prev_de && cy == V_ACT - 1) {
                active_lines.push_back(cy + 1); field_idx++;
                if (field_idx == 7) { g_dbl = 1; dut->fb_line_dbl = 1; }   // switch to a 240-line source (latched next field)
                if (field_idx == 8) { g_dbl = 1; }
            }
        }
        prev_de = dut->de; prev_hs = dut->hsync; prev_vs = dut->vsync;
    }
    // 240-line-source switch lands one field late (per-field latch): re-check handled by skipping field 7
    for (size_t i = 0; i < active_lines.size(); i++) if (active_lines[i] != V_ACT) { printf("field %zu active lines %d\n", i, active_lines[i]); errors++; }
    printf("vsync: ");
    for (size_t i = 1; i < vs_times.size(); i++) printf("%.1f/%d ", (vs_times[i] - vs_times[i - 1]) / (double)H_TOT, vs_phase[i]);
    printf("\n");
    for (size_t i = 2; i < vs_times.size(); i++) {
        double lines = (vs_times[i] - vs_times[i - 1]) / (double)H_TOT;
        double want = (LMODE == 2) ? 262.5 : 262.0;
        if (lines != want) { printf("vsync spacing %.2f != %.1f\n", lines, want); errors++; }
        if (LMODE == 2 && vs_phase[i] == vs_phase[i - 1]) { printf("vsync phase not alternating\n"); errors++; }
    }
    if (dut->underrun) { printf("UNDERRUN\n"); errors++; }
    printf("LINE_MODE %d: %d fields, %s (%d errors)\n", LMODE, field_idx, errors ? "FAIL" : "PASS", errors);
    delete dut; return errors ? 1 : 0;
}
