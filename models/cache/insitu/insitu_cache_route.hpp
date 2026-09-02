// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful programmable xbar routing + MSB rotation (structural model, Step 6).
//
// Header-only transcription of the pure routing/rotation datapath of `tcdm_cache_interco.sv`
// (and the 2:1 bypass arbitration of `reqrsp_xbar.sv` used as the coalescer|scalar merge). This is
// the REAL any-core→any-bank routing that replaces the Knuth-hashed N→M approximation in
// `insitu_cache_interco.cpp`. The structural facts it captures:
//
//   - REQUEST ROUTING (tcdm_cache_interco.sv:219-267). The BankSel field sits at
//     addr[dynamic_offset +: log2(NumCache)]; the TileID field directly above it. Three runtime
//     partition modes (num_private_cache ∈ {0, NumCache/2, NumCache}):
//       * all-private / single-tile → local, sel = BankSel (full field, no fold)
//       * all-shared (priv=0)        → local iff addr_tile==tile_id; else remote slot (tile % NumRemotePort)
//       * mixed                      → private req (addr>=private_start) folds BankSel % num_private;
//                                      shared req folds num_private + (BankSel % num_shared), or remote
//   - RESPONSE ROUTING (:273-284). A response returns to its originating core_id, unless its tile_id
//     differs from this tile → it leaves on the remote slot (tile_id % NumRemotePort) — the same slot
//     the request arrived on (the group xbar pins source-tile→slot).
//   - MSB ADDRESS ROTATION (:357-405). After the xbar each bank sees only its own requests; the N
//     routing bits (BankSel, +TileID for shared banks) immediately above dynamic_offset are ROTATED to
//     the MSB so the cache's tag/index logic never sees them (and no tag SRAM is wasted on constant
//     zeros). The refill unit applies the inverse before issuing to the NoC — `unrotate_addr()` here.
//   - 2:1 BYPASS (reqrsp_xbar.sv, NumInp=2/NumOut=1). The coalescer-aggregate path and the scalar
//     bypass arbitrate to one cache port; responses demux by the `bypass_coalescer` user bit.
//
// SCOPE (pure combinational routing/rotation — the structural essential). DEFERRED to the
// timing-calibration phase (with the per-cycle component integration): the 1-cycle input spill
// register (PipeReg/spill_register), the fall-through rsp register, and the per-output round-robin
// arbiter inside `stream_xbar` — those set the per-cycle *timing* (a +1 spill, RR fairness); this
// models WHICH output a request routes to, WHICH input a response returns to, and the exact rotated
// address a bank stores. Validated standalone; the spill/RR timing + wiring land with calibration.

#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace insitu {

// No-allocate alias bit (ManyRVData rlc_am doc/RLC_HW.md §2): the per-access permission bit is
// address-borne on the CORE side — software reads payload through `addr | (1<<30)`, an alias of
// the DRAM PMA ([0x8000_0000, 0xA000_0000) aliases at [0xC000_0000, 0xE000_0000), disjoint from
// every real mapping above 0xC003_0000 for the addresses the kernel actually aliases). The tile
// xbar STRIPS the bit at its ingress and carries the attribute as IoReq::noalloc from there on —
// the model's stand-in for the RTL's sideband wire, required because the MSB rotation owns the
// top address bits between xbar and bank. Consumers outside the datapath (pcstat attribution)
// mask it with this constant.
inline constexpr uint64_t noalloc_alias_bit = 1ull << 30;

// Routed request: which xbar output, and (if remote) the target tile.
struct ReqRoute {
    bool     local       = true;   // stays on local banks (vs forwarded to a remote tile)
    uint32_t sel         = 0;      // xbar output index: [0..NumCache) local bank | NumCache+slot remote
    uint32_t remote_tile = 0;      // target tile id when !local
};

// tcdm_cache_interco.sv:363-385 — how many routing bits a given LOCAL bank port rotates.
// FREE function: the single source of truth shared by the xbar (rotate on request), the cache
// core (unrotate on L2 egress), and the peripheral-side setters (E3 runtime repartition) — so
// they can never drift apart.
inline uint32_t bits_to_rotate(uint32_t bank_port, uint32_t num_private_cache, uint32_t num_cache,
                               uint32_t cache_bank_bits, uint32_t tile_bits) {
    if (num_private_cache == 0)          return cache_bank_bits + tile_bits;   // all-shared
    if (num_private_cache == num_cache)  return cache_bank_bits;               // all-private
    return (bank_port < num_private_cache) ? cache_bank_bits                   // mixed: private
                                           : (cache_bank_bits + tile_bits);    // mixed: shared
}

// One placed region (ManyRVData rlc_am doc/RLC_HW.md §1): an address window whose TileID and
// BankSel routing fields sit at CONFIGURED bit positions instead of the flat
// [dyn_offset +: bank][.. +: tile] slice — so one tile owns one contiguous block (TileID at a
// coarse bit, e.g. 19) while consecutive lines inside it still round-robin that tile's banks
// (BankSel at the cacheline bit). Addresses matching no region keep today's flat behavior, so
// every existing kernel is unaffected. Priority-encoded, up to four, matching the RTL sketch's
// register interface; here they are elaboration-time configuration (the RTL writes them once
// from one core and broadcasts — the model just starts with them written).
//
// The MSB rotation is deliberately UNTOUCHED for region-matched requests: it is a bijection on
// the full address applied identically on the bank's ingress and inverted on its L2 egress, so
// which bank a request routed to does not affect correctness — and for a region the varying
// intra-block bits are exactly the ones that land in the set index, so there is no capacity
// collapse either.
struct PlacedRegion {
    bool     en         = false;
    uint64_t base       = 0;
    uint64_t size       = 0;
    uint32_t tile_shift = 0;   // TileID  at addr[tile_shift +: log2(tiles)]
    uint32_t bank_shift = 0;   // BankSel at addr[bank_shift +: log2(banks)]
};

// Geometry + runtime CSRs of one tile's tcdm_cache_interco. All widths in bits; addresses byte-addressed.
struct RouteGeom {
    uint32_t num_cache       = 4;    // NumCache — local cache banks per tile
    uint32_t num_remote_port = 0;    // NumRemotePort — remote-out slots (0 → single-tile)
    uint32_t num_cores       = 4;    // NumCores — local input ports
    uint32_t num_tiles       = 1;    // NumTiles
    uint32_t dyn_offset      = 6;    // dynamic_offset_i — BankSel field LSB (log2 cacheline by default)
    uint32_t addr_width      = 32;   // AddrWidth
    uint64_t private_start   = 0;    // private_start_addr_i (mixed mode boundary)
    // derived
    uint32_t cache_bank_bits = 2;    // log2(NumCache)
    uint32_t tile_id_width   = 0;    // TileIDWidth
    uint32_t tile_bits       = 0;    // log2(NumTotCache/NumCache) == log2(NumTiles)

    void init(uint32_t n_cache, uint32_t n_remote, uint32_t n_cores, uint32_t n_tiles,
              uint32_t dynamic_offset, uint32_t addr_w, uint64_t priv_start) {
        num_cache = n_cache; num_remote_port = n_remote; num_cores = n_cores;
        num_tiles = n_tiles; dyn_offset = dynamic_offset; addr_width = addr_w; private_start = priv_start;
        cache_bank_bits = log2_up(n_cache);
        tile_bits       = log2_up(n_tiles);
        tile_id_width   = tile_bits;     // TileIDWidth == log2(NumTotCache/NumCache) by construction
    }

    static uint32_t log2_up(uint32_t v) { uint32_t b = 0; while ((1u << b) < v) b++; return b; }

    // Placed regions (see PlacedRegion above). Empty by default: flat routing everywhere.
    static constexpr uint32_t max_regions = 4;
    PlacedRegion regions[max_regions];

    // Parse "base:size:tile_shift:bank_shift[,...]" (numbers via strtoull base 0, so 0x works).
    // Returns how many regions were configured; a malformed clause stops the parse.
    uint32_t parse_regions(const char *spec) {
        uint32_t n = 0;
        if (spec == nullptr) return 0;
        while (*spec != '\0' && n < max_regions) {
            char *end = nullptr;
            const uint64_t base = strtoull(spec, &end, 0);
            if (end == spec || *end != ':') break;
            spec = end + 1;
            const uint64_t size = strtoull(spec, &end, 0);
            if (end == spec || *end != ':') break;
            spec = end + 1;
            const uint64_t tsh = strtoull(spec, &end, 0);
            if (end == spec || *end != ':') break;
            spec = end + 1;
            const uint64_t bsh = strtoull(spec, &end, 0);
            if (end == spec) break;
            regions[n].en = true;
            regions[n].base = base;
            regions[n].size = size;
            regions[n].tile_shift = (uint32_t)tsh;
            regions[n].bank_shift = (uint32_t)bsh;
            ++n;
            spec = (*end == ',') ? end + 1 : end;
        }
        return n;
    }

    uint32_t addr_bank(uint64_t addr) const {
        return cache_bank_bits ? (uint32_t)((addr >> dyn_offset) & (((uint64_t)1 << cache_bank_bits) - 1)) : 0;
    }
    uint32_t addr_tile(uint64_t addr) const {
        return tile_id_width ? (uint32_t)((addr >> (dyn_offset + cache_bank_bits)) & (((uint64_t)1 << tile_id_width) - 1)) : 0;
    }

    // The effective {BankSel, TileID} of an address: a matching placed region's configured bit
    // positions, else the flat slice. This is THE routing decision both xbars consult.
    void routing_fields(uint64_t addr, uint32_t &bank, uint32_t &tile) const {
        for (uint32_t i = 0; i < max_regions; ++i) {
            const PlacedRegion &r = regions[i];
            if (r.en && (addr - r.base) < r.size) {
                bank = cache_bank_bits
                     ? (uint32_t)((addr >> r.bank_shift) & (((uint64_t)1 << cache_bank_bits) - 1)) : 0;
                tile = tile_id_width
                     ? (uint32_t)((addr >> r.tile_shift) & (((uint64_t)1 << tile_id_width) - 1)) : 0;
                return;
            }
        }
        bank = addr_bank(addr);
        tile = addr_tile(addr);
    }

    // tcdm_cache_interco.sv:219-267 — request input-side routing.
    ReqRoute route_request(uint64_t addr, uint32_t tile_id, uint32_t num_private_cache) const {
        ReqRoute r;
        uint32_t ab = 0;
        uint32_t at = 0;
        routing_fields(addr, ab, at);
        const uint32_t num_shared = num_cache - num_private_cache;
        if (num_private_cache == num_cache || num_tiles == 1) {        // all-private / single-tile
            r.local = true; r.sel = ab;
        } else if (num_private_cache == 0) {                           // all-shared
            r.local = (at == tile_id);
            r.sel = r.local ? ab : (num_cache + (num_remote_port ? (at % num_remote_port) : 0));
            r.remote_tile = at;
        } else {                                                       // mixed
            const bool is_private = (addr >= private_start);
            if (is_private) {
                r.local = true; r.sel = (num_private_cache ? (ab % num_private_cache) : 0);
            } else {
                r.local = (at == tile_id);
                r.sel = r.local ? (num_private_cache + (num_shared ? (ab % num_shared) : 0))
                                : (num_cache + (num_remote_port ? (at % num_remote_port) : 0));
                r.remote_tile = at;
            }
        }
        return r;
    }

    // tcdm_cache_interco.sv:273-284 — response output-side routing → xbar input index.
    uint32_t route_response(uint32_t rsp_core_id, uint32_t rsp_tile_id, uint32_t tile_id) const {
        if (rsp_tile_id != tile_id)
            return num_cores + (num_remote_port ? (rsp_tile_id % num_remote_port) : 0);
        return rsp_core_id;
    }

    // tcdm_cache_interco.sv:363-385 — how many routing bits a given LOCAL bank port rotates.
    uint32_t bits_to_rotate(uint32_t bank_port, uint32_t num_private_cache) const {
        return insitu::bits_to_rotate(bank_port, num_private_cache, num_cache, cache_bank_bits, tile_bits);
    }

    // tcdm_cache_interco.sv:387-404 — rotate the N routing bits (just above dyn_offset) to the MSB.
    uint64_t rotate_addr(uint64_t addr, uint32_t n) const {
        const uint64_t lower     = addr & (((uint64_t)1 << dyn_offset) - 1);
        const uint64_t rot_field = (addr >> dyn_offset) & (((uint64_t)1 << n) - 1);
        const uint64_t upper     = addr >> (dyn_offset + n);
        return lower | (upper << dyn_offset) | (rot_field << (addr_width - n));
    }

    // Inverse of rotate_addr — applied by the refill unit before issuing to the NoC (:334-336).
    uint64_t unrotate_addr(uint64_t addr_rot, uint32_t n) const {
        const uint64_t lower     = addr_rot & (((uint64_t)1 << dyn_offset) - 1);
        const uint64_t rot_field = addr_rot >> (addr_width - n);
        const uint64_t upper     = (addr_rot >> dyn_offset) & ((((uint64_t)1 << (addr_width - n - dyn_offset)) - 1));
        return lower | (rot_field << dyn_offset) | (upper << (dyn_offset + n));
    }
};

// reqrsp_xbar.sv as a 2:1 bypass (NumInp=2 {0:coalescer-aggregate, 1:scalar-bypass}, NumOut=1).
// Functionally: both inputs target output 0; a response demuxes back by the bypass_coalescer bit.
struct BypassXbar {
    // Arbitrate the two valid inputs to the single cache port; returns the granted input (0/1) or -1.
    // Round-robin (LockIn): alternate priority each grant (the stream_xbar RR pointer).
    int grant(bool in0_valid, bool in1_valid) {
        int g = -1;
        if (in0_valid && in1_valid)       { g = rr_; rr_ ^= 1; }
        else if (in0_valid)               { g = 0; }
        else if (in1_valid)               { g = 1; }
        return g;
    }
    // Response routing: a response with bypass_coalescer=false returns to input 0 (coalescer-aggregate),
    // true → input 1 (the scalar bypass).
    static int route_response(bool bypass_coalescer) { return bypass_coalescer ? 1 : 0; }
private:
    int rr_ = 0;
};

} // namespace insitu
