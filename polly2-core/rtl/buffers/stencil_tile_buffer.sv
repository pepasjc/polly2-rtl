//
// stencil_tile_buffer - the MODIFIER VOLUME stencil plane for one 32x32 tile, plus
// the per-copy INV images the spanner reads.
//
// refsw2 keeps a 3-bit stencil per pixel (refsw_tile.cpp stencilBuffer) and this
// module is that byte, banked for the raster:
//
//   bit0 INV   "this pixel is inside the accumulated shadow volume". The only bit
//              TSP ever sees: refsw2 RenderParamTags computes
//                  InVolume = (stencil & 0b001) && tag.shadow
//              and, in CHEAP-SHADOW mode (FPU_SHAD_SCALE.intensity_shadow), scales
//              the interpolated base/offset colour by FPU_SHAD_SCALE.scale_factor.
//   bit1 FLIP  per-volume parity. A modifier-volume triangle that passes the depth
//              test TOGGLES it (refsw2 PixelFlush_isp RM_MODIFIER: `*stencil ^= 0b0010`).
//   bit2 SUM   "a modvol triangle touched this pixel in the current volume", set
//              alongside every flip (`*stencil |= 0b100`). It is what makes the
//              summarize a no-op on untouched pixels.
//
// VOLUME END (refsw2 RenderTriangle, after RM_MODIFIER rasterization): a triangle
// whose ISP word carries VolumeMode != 0 closes the volume and SUMMARIZES the whole
// tile - SummarizeStencilOr for VolumeMode 1 ("inside last"), SummarizeStencilAnd
// for 2 ("outside last"):
//
//   if (SUM) { INV = or ? (INV | FLIP) : (INV & FLIP); }   FLIP = 0; SUM = 0;
//
// FLIP can only be set together with SUM, so clearing both unconditionally is the
// same transform on untouched pixels (which keep their INV) with no extra mux.
// VolumeMode 0 is a plain non-last polygon: accumulate parity, no summarize.
//
// STORAGE - TWO simple-dual-port block RAMs:
//
//  * the WORKING plane (u_ram): ONE tile, 1024/LANES entries x 3*LANES bits - the
//    whole LANES-pixel raster chunk in one word, so the write needs no per-lane
//    byte enable and no banking. It is ISP-PRIVATE: only the raster RMW, the
//    CLEAR/zero walk and the summarize walk touch it, all serialized by peel_core's
//    barriers, so it needs no copies at all. Like refsw2's stencil byte it simply
//    persists from one region entry to the next (a z_keep=1 entry inherits INV).
//
//  * the INV IMAGES (u_img): COPIES tile images x 1 bit/pixel, LANES bits wide -
//    the ISP->TSP handoff of the only bit TSP needs. The copy index works exactly
//    like u_taginvw's (wr_buf = the producer copy the ISP is filling, rd_buf = the
//    consumer copy the spanner is reading), so INV rides the same per-copy credit
//    as the tags it qualifies: the ISP may run TI_COPIES-1 passes ahead of the
//    spanner without a stencil stall, and the working plane never has to be
//    copied or ping-ponged. INV only CHANGES at a summarize, so the summarize walk
//    writes the new INV of every chunk into image[wr_buf] alongside the working
//    plane, and the CLEAR/zero walk zeroes image[wr_buf] with it - image[wr_buf]
//    therefore always equals the working INV for as long as wr_buf is unchanged.
//    peel_core only lets the spanner USE an image that a summarize produced in the
//    OM phase of the pass being shaded (its per-copy ti_mv flag), so the images of
//    copies handed by other passes may hold anything.
//
// Chunk addr = {y[4:0], x[4:BB]}; image addr = {copy, chunk addr}. The spanner's
// 4-wide aligned group is a contiguous slice of one image chunk.
//
// The raster path is a READ-MODIFY-WRITE across the same stage A / stage B pair as
// peel_tile_buffer: stage A presents the chunk read, stage B (next cycle) XORs the
// passing lanes' FLIP and writes the chunk back - untouched lanes are carried from
// the read, exactly as peel_tile_buffer carries its unwritten fields. Consecutive
// raster chunks are always different addresses (the sweep steps x by LANES, then
// rows), and back-to-back triangles are separated by the POP+CORNER pair, so the
// read never collides with the previous cycle's write.
//
// Working plane read clients  (at most one/cycle): raster stage A | summarize walk.
// Working plane write clients (at most one/cycle): raster stage B | CLEAR/zero walk |
//                                                  summarize walk.
// Image write clients: CLEAR/zero walk | summarize walk (same cycle as the working
// plane write).  Image read client: the spanner only.
// The module asserts the exclusions in sim.
//
module stencil_tile_buffer #(
    parameter integer LANES  = 8,
    parameter integer COPIES = 1                 // INV images (1, 2, 4, 8, ...)
) (
    input                       clk,
    input                       reset,

    // ---- INV image copy select (see STORAGE) ----
    input      [(COPIES>1 ? $clog2(COPIES) : 1)-1:0] wr_buf,  // CLEAR / summarize
    input      [(COPIES>1 ? $clog2(COPIES) : 1)-1:0] rd_buf,  // spanner read

    // ---- RASTER stage A: present the chunk read (mirrors peel_tile_buffer) ----
    input                       ras_a_valid,
    input      [4:0]            ras_a_y,
    input      [4:0]            ras_a_x,      // chunk base (LANES-aligned)

    // ---- RASTER stage B: flip FLIP + set SUM on the lanes that passed ----
    // mv_we[l] = peel_tile_buffer's b_mv_we[l] (inside & the forced-GE depth test).
    input                       ras_b_valid,
    input      [LANES-1:0]      mv_we,
    input      [4:0]            b_y,
    input      [4:0]            b_x,

    // ---- CLEAR / ZERO walk: write {0,0,0} to the chunk at clr_addr ----
    // Used by the tile CLEAR (refsw ClearBuffers stencilValue=0) and by the
    // PT/TL peel walks (refsw PeelBuffers/PeelBuffersPTInitial also zero it, so a
    // translucent pass never inherits the opaque pass's shadow mask). Zeroes the
    // image[wr_buf] chunk too.
    input                       clr_valid,
    input      [10-$clog2(LANES)-1:0] clr_addr,

    // ---- SUMMARIZE RMW walk (read-ahead cursor / delayed write, like PeelBuffers) ----
    // The write also lands the chunk's new INV in image[wr_buf].
    input                       sum_rd_valid,
    input      [10-$clog2(LANES)-1:0] sum_rd_addr,
    input                       sum_wr_valid,
    input      [10-$clog2(LANES)-1:0] sum_wr_addr,
    input                       sum_and,      // 0 = OR (VolumeMode 1), 1 = AND (VolumeMode 2)

    // ---- SPANNER: 4-wide ALIGNED read of image[rd_buf] (group = x & ~3), 1-cyc ----
    input                       rd4_valid,
    input      [9:0]            rd4_group,
    output     [3:0]            g4_inv        // per-lane INV bit (lane l = pixel group|l)
);
    localparam integer BANK_BITS = $clog2(LANES);        // 3 for 8, 2 for 4
    localparam integer AW        = 10 - BANK_BITS;       // chunk-address width (7 / 8)
    localparam integer NCH       = 1 << AW;              // chunks per tile
    localparam integer SW        = 3;                    // {SUM, FLIP, INV} per lane
    localparam integer W         = SW * LANES;
    localparam integer CB        = (COPIES > 1) ? $clog2(COPIES) : 0;  // copy-select bits
    localparam integer IAW       = AW + CB;              // image address width

    // per-lane field offsets inside the packed chunk word
    localparam integer F_INV  = 0;
    localparam integer F_FLIP = 1;
    localparam integer F_SUM  = 2;

`ifndef SYNTHESIS
    initial
        if (COPIES & (COPIES - 1))
            $error("stencil_tile_buffer: COPIES must be a power of two (got %0d)", COPIES);
`endif

    // ==================== WORKING plane (ISP-private) ====================
    reg              we;
    reg  [AW-1:0]    waddr;
    reg  [W-1:0]     wdata;
    wire             re = ras_a_valid | sum_rd_valid;
    wire [AW-1:0]    raddr = ras_a_valid ? {ras_a_y, ras_a_x[4:BANK_BITS]} : sum_rd_addr;
    wire [W-1:0]     q;
    // WRITE PORT PIPELINE (the taginvw_tile_buffer REG_WRITE idea). The raster flip's
    // data is mv_we, i.e. peel_tile_buffer's M10K read -> depth compare - and this
    // RAM is ONE block fed by all LANES compares, so it cannot sit next to all of
    // them. One register on the whole write port ({we, waddr, wdata} together) cuts
    // that into compare -> FF and FF -> this M10K. The write lands one cycle later,
    // which is safe because no client re-reads a chunk within two cycles of writing
    // it: raster chunks of one triangle are all distinct, consecutive triangles are
    // >= 3 issue cycles apart (POP + CORNER), the summarize walk reads chunk N+1
    // while writing chunk N, and peel_core's barriers put many cycles between a walk
    // and the next raster read. The sim check at the bottom asserts it.
    reg              we_q;
    reg  [AW-1:0]    waddr_q;
    reg  [W-1:0]     wdata_q;
    always @(posedge clk) begin
        if (reset) we_q <= 1'b0;
        else       we_q <= we;
        waddr_q <= waddr;
        wdata_q <= wdata;
    end
    bram_sdp #(.W(W), .D(NCH)) u_ram (
        .clk(clk), .we(we_q), .waddr(waddr_q), .din(wdata_q),
        .re(re), .raddr(raddr), .q(q));

    // the summarized INV of the chunk being written back (also the image write data)
    reg  [LANES-1:0] sum_inv;
    integer cw;
    always @(*) begin
        for (cw = 0; cw < LANES; cw = cw + 1)
            sum_inv[cw] = q[SW*cw + F_SUM] ? (sum_and ? (q[SW*cw + F_INV] & q[SW*cw + F_FLIP])
                                                      : (q[SW*cw + F_INV] | q[SW*cw + F_FLIP]))
                                           :  q[SW*cw + F_INV];
    end

    // -------------------- WRITE port mux --------------------
    always @(*) begin
        we    = 1'b0;
        waddr = '0;
        wdata = '0;

        if (clr_valid) begin                       // CLEAR / zero walk
            we    = 1'b1;
            waddr = clr_addr;
            // wdata stays all-zero: INV/FLIP/SUM cleared for every lane.
        end else if (sum_wr_valid) begin           // SummarizeStencilOr / ...And
            we    = 1'b1;
            waddr = sum_wr_addr;
            for (cw = 0; cw < LANES; cw = cw + 1) begin
                wdata[SW*cw + F_INV]  = sum_inv[cw];
                wdata[SW*cw + F_FLIP] = 1'b0;      // (FLIP set => SUM set, so this is
                wdata[SW*cw + F_SUM]  = 1'b0;      //  the `&= 0b001` of the reference)
            end
        end else if (ras_b_valid) begin            // modvol accept: flip parity
            we    = 1'b1;
            waddr = {b_y, b_x[4:BANK_BITS]};
            for (cw = 0; cw < LANES; cw = cw + 1) begin
                wdata[SW*cw + F_INV]  =  q[SW*cw + F_INV];        // untouched by a flip
                wdata[SW*cw + F_FLIP] =  q[SW*cw + F_FLIP] ^ mv_we[cw];
                wdata[SW*cw + F_SUM]  =  q[SW*cw + F_SUM]  | mv_we[cw];
            end
        end
    end

    // ==================== INV images (ISP -> spanner handoff) ====================
    // copy select as an address OFFSET (copy << AW), so COPIES==1 degenerates to a
    // constant 0 with no copy bits at all (same idiom as taginvw_tile_buffer).
    wire [IAW-1:0] wbase = (COPIES > 1) ? (IAW'(wr_buf) << AW) : {IAW{1'b0}};
    wire [IAW-1:0] rbase = (COPIES > 1) ? (IAW'(rd_buf) << AW) : {IAW{1'b0}};
    wire           img_we    = clr_valid | sum_wr_valid;
    wire [IAW-1:0] img_waddr = wbase | IAW'(clr_valid ? clr_addr : sum_wr_addr);
    wire [LANES-1:0] img_din = clr_valid ? {LANES{1'b0}} : sum_inv;
    wire [IAW-1:0] img_raddr = rbase | IAW'({rd4_group[9:5], rd4_group[4:BANK_BITS]});
    wire [LANES-1:0] img_q;
    bram_sdp #(.W(LANES), .D(NCH * COPIES)) u_img (
        .clk(clk), .we(img_we), .waddr(img_waddr), .din(img_din),
        .re(rd4_valid), .raddr(img_raddr), .q(img_q));

    // -------------------- SPANNER 4-wide group output --------------------
    // The aligned group {g..g+3} is a contiguous 4-lane slice of the chunk: the whole
    // chunk when LANES==4, the g[2]-selected half when LANES==8. Latch the half select
    // with the read so it tracks the registered q (same trick as taginvw_tile_buffer).
    localparam integer G4B = (BANK_BITS > 2) ? BANK_BITS - 2 : 1;
    reg [G4B-1:0] g4_half_r;
    always @(posedge clk) begin
        if (reset) g4_half_r <= '0;
        else if (rd4_valid) g4_half_r <= (BANK_BITS > 2) ? rd4_group[2 +: G4B] : '0;
    end
    genvar gl;
    generate
      for (gl = 0; gl < 4; gl = gl + 1) begin : g4lane
        assign g4_inv[gl] = img_q[4*g4_half_r + gl];
      end
    endgenerate

`ifndef SYNTHESIS
    always @(posedge clk) if (!reset) begin
        if ((clr_valid + sum_wr_valid + ras_b_valid) > 1)
            $error("stencil_tile_buffer: multiple WRITE clients (%b%b%b)",
                   clr_valid, sum_wr_valid, ras_b_valid);
        if ((ras_a_valid + sum_rd_valid) > 1)
            $error("stencil_tile_buffer: multiple READ clients (%b%b)",
                   ras_a_valid, sum_rd_valid);
        // the delayed write lands on the same edge as this read: the RAM is
        // read-first, so a same-address hit would return the stale chunk
        if (we_q && re && (waddr_q == raddr))
            $error("stencil_tile_buffer: delayed write collides with a read of chunk %0d",
                   raddr);
    end
`endif
endmodule
