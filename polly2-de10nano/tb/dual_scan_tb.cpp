// dual_scan_tb: the CRT_DUAL display path (dual_scan_top.sv) - 1080p HDMI spg
// at 148.5 MHz and the 15 kHz CRT spg at 27 MHz fetching through one
// vbuf_arbiter from a DDR model at 100 MHz (24-cycle latency, random
// waitrequest / readdatavalid gaps):
//  (a bad parity guess shows as every pixel of every other 480i field)
//  - every active CRT pixel against a model (565, split VRAM), in 240p and
//    480i (CRT_CTRL[3]), so sharing the port never corrupts or starves it;
//  - neither scanout underruns, the HDMI one keeps producing frames.
#include "Vdual_scan_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <deque>
#include <algorithm>

static Vdual_scan_top* dut;
static int errors = 0;
static uint8_t pat(uint64_t a) { uint32_t v = (uint32_t)a * 2654435761u; return (uint8_t)(v >> 24); }

static const uint32_t BASE_FB = 0x00200000;          // FB-view byte address
static const int X0 = 80, X1 = 1360, V_ACT = 240;    // CRT game window
static uint64_t f2d(uint64_t A) { return (A >> 2) * 8 + (A & 3); }   // split, half 0
static uint8_t fb8(uint64_t A) { return pat(f2d(A)); }
static void expect_crt(int x, int row, int field, bool p240, int* c) {
    c[0] = c[1] = c[2] = 0;
    if (x < X0 || x >= X1) return;
    int n = (x - X0) >> 1;
    int s = 2 * row + (p240 ? 0 : field);
    uint64_t F = BASE_FB + (uint64_t)s * 1280;
    uint16_t p = (uint16_t)(fb8(F + 2 * n) | (fb8(F + 2 * n + 1) << 8));
    c[0] = ((p >> 11) & 0x1F) << 3; c[1] = ((p >> 5) & 0x3F) << 2; c[2] = (p & 0x1F) << 3;
}

// ---- three clocks, scheduled in ps ----
static const uint64_t P_HDMI = 6734, P_CRT = 37037, P_AVL = 10000;
static uint64_t t_h = 0, t_c = 0, t_a = 0, avl_cyc = 0;
static uint32_t lfsr = 0xACE1;
static int rnd4() { lfsr = (lfsr >> 1) ^ (-(int)(lfsr & 1) & 0xB400u); return lfsr & 3; }
struct Beat { uint64_t wa, ready; };
static std::deque<Beat> owed;
static void avl_inputs() {
    dut->avl_waitrequest = (rnd4() == 0);
    if (dut->avl_read && !dut->avl_waitrequest)
        for (uint32_t b = 0; b < dut->avl_burstcount; b++)
            owed.push_back({(uint64_t)dut->avl_address + b, avl_cyc + 24});
    dut->avl_readdatavalid = 0;
    if (!owed.empty() && owed.front().ready <= avl_cyc && rnd4() != 1) {
        uint64_t wa = owed.front().wa; owed.pop_front();
        for (int i = 0; i < 4; i++) { uint32_t v = 0; for (int k = 0; k < 4; k++) v |= (uint32_t)pat(wa * 16 + 4 * i + k) << (8 * k); dut->avl_readdata[i] = v; }
        dut->avl_readdatavalid = 1;
    }
}
// advance to the next edge of any clock; returns which clocks rose
static int step() {
    uint64_t tn = std::min(t_h, std::min(t_c, t_a));
    bool h = t_h == tn, c = t_c == tn, a = t_a == tn;
    if (h) dut->clk_hdmi = 0;
    if (c) dut->clk_crt = 0;
    if (a) dut->avl_clk = 0;
    dut->eval();
    if (a) avl_inputs();
    if (h) dut->clk_hdmi = 1;
    if (c) dut->clk_crt = 1;
    if (a) dut->avl_clk = 1;
    dut->eval();
    if (h) t_h += P_HDMI;
    if (c) t_c += P_CRT;
    if (a) { t_a += P_AVL; avl_cyc++; }
    return (h ? 1 : 0) | (c ? 2 : 0);
}

int main(int argc, char** argv) {
    Verilated::commandArgs(argc, argv);
    dut = new Vdual_scan_top;
    dut->fb_base = BASE_FB * 2; dut->fb_stride = 1280; dut->fb_depth = 1;   // 565
    dut->reset = 1;
    for (int i = 0; i < 200; i++) step();
    dut->reset = 0;

    // mode: 240p for the first part, then 480i (adopted at the next field)
    bool p240 = true; dut->crt_ctrl = 8;
    int field_idx = 0, cx = 0, cy = -1, prev_de = 0, hdmi_frames = 0, prev_hvs = 0;
    long gap = 1000000;
    int checked_fields = 0, case_fields = 0;
    bool checking = false;
    int fpar = -1;                                  // 480i field phase, learnt
    while (checked_fields < 8) {
        int r = step();
        if (r & 1) {
            if (dut->hdmi_vs && !prev_hvs) hdmi_frames++;
            prev_hvs = dut->hdmi_vs;
        }
        if (!(r & 2)) continue;
        if (dut->crt_de) {
            if (!prev_de) { cx = 0; if (gap > 10000) cy = 0; else cy++; }
            gap = 0;
            if (checking && cy >= 0) {
                // 480i: the DUT's field phase after the 240p->480i switch is
                // arbitrary (field order itself is spg_15k_tb's job) - learn it
                // from the first pixel of the first checked 480i field, then
                // every later field must alternate from it
                if (!p240 && fpar < 0 && cx == X0 && cy == 0) {
                    int a[3]; expect_crt(cx, cy, field_idx & 1, false, a);
                    int R = (dut->crt_rgb >> 16) & 0xFF, G = (dut->crt_rgb >> 8) & 0xFF, B = dut->crt_rgb & 0xFF;
                    fpar = (R == a[0] && G == a[1] && B == a[2]) ? 0 : 1;
                }
                int e[3]; expect_crt(cx, cy, (field_idx & 1) ^ (fpar > 0), p240, e);
                int R = (dut->crt_rgb >> 16) & 0xFF, G = (dut->crt_rgb >> 8) & 0xFF, B = dut->crt_rgb & 0xFF;
                if (R != e[0] || G != e[1] || B != e[2]) {
                    if (errors < 10) printf("  CRT %s field %d (%d,%d): got %02x%02x%02x want %02x%02x%02x\n",
                                            p240 ? "240p" : "480i", field_idx, cx, cy, R, G, B, e[0], e[1], e[2]);
                    errors++;
                }
            }
            cx++;
        } else {
            gap++;
            if (prev_de && cy == V_ACT - 1) {      // end of a field's active area
                field_idx++;
                if (checking) { checked_fields++; case_fields++; }
                if (dut->crt_underrun) { printf("CRT UNDERRUN by field %d\n", field_idx); errors++; break; }
                if (dut->hdmi_underrun) { printf("HDMI UNDERRUN by field %d\n", field_idx); errors++; break; }
                checking = field_idx >= 2;           // first fields: pipelines filling
                if (p240 && case_fields >= 4) {      // switch to 480i: skip the switch field
                    p240 = false; dut->crt_ctrl = 0; checking = false; case_fields = 0;
                }
                else if (!p240 && !checking) checking = true;
            }
        }
        prev_de = dut->crt_de;
    }
    printf("CRT fields checked %d (last %s), HDMI frames %d, underrun crt=%d hdmi=%d\n",
           checked_fields, p240 ? "240p" : "480i", hdmi_frames, dut->crt_underrun, dut->hdmi_underrun);
    if (hdmi_frames < 4) { printf("HDMI produced only %d frames\n", hdmi_frames); errors++; }
    printf(errors ? "FAIL (%d errors)\n" : "PASS\n", errors);
    dut->final(); delete dut;
    return errors ? 1 : 0;
}
