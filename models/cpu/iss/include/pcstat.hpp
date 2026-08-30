// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// Per-PC load/store attribution (CACHEPOOL_PC_STATS=1): every scalar LSU request (loads,
// stores, atomics) is attributed to the PC of the issuing instruction with its completion
// latency and its routed destination (same tile / same group / other group / outside the
// cached window). Classification reuses the EXACT xbar routing datapath
// (cache/insitu/insitu_cache_route.hpp), configured from the same CACHEPOOL_* environment
// the target reads — so it can never disagree with where the request actually went.
//
// Process-global, merged across cores on the fly (GVSoC single-thread engine; a mutex keeps
// multi-thread runs safe), dumped once at process exit to stderr:
//   [PC-STATS] pc=%08x n= w= a= lat= max= t= g= r= x=
//     n   total attributed requests    w  writes among them   a  atomics among them
//     lat summed latency (cycles)      max worst single latency
//     t/g/r  destination same-tile / same-group / other-group   x  outside cached window
// Parse/rank/symbolize with rlc_am/script/pc_stats.py. Zero overhead when the env is unset.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <mutex>
#include <set>
#include <unordered_map>

#include "cache/insitu/insitu_cache_route.hpp"

namespace pcstat {

struct Stat {
    uint64_t n = 0, w = 0, a = 0, lat = 0, max = 0;
    uint64_t d[4] = {0, 0, 0, 0};   // tile / group / remote / external
};

class Registry {
public:
    static Registry &get() {
        static Registry inst;
        return inst;
    }
    bool enabled() const { return enabled_; }

    // CACHEPOOL_PC_STATS_CORE=N (or "N,M,..."): attribute only these harts, so one
    // core's profile is not drowned by 255 others (a team leader, the UE core, ...).
    inline bool core_selected(uint32_t hart) const {
        return !core_filter_ || sel_.count(hart) != 0;
    }

    void note(uint32_t pc, bool is_write, bool is_amo, uint64_t addr, uint32_t my_tile,
              uint64_t latency) {
        int cls = 3;   // external (outside the cached window)
        if (addr - cached_base_ < cached_size_) {
            const insitu::ReqRoute r = geom_.route_request(addr, my_tile, 0);
            const uint32_t dest = r.local ? my_tile : r.remote_tile;
            cls = (dest == my_tile) ? 0 : (dest / tiles_per_group_ == my_tile / tiles_per_group_) ? 1 : 2;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        Stat &s = map_[pc];
        s.n++;
        if (is_write) s.w++;
        if (is_amo) s.a++;
        s.lat += latency;
        if (latency > s.max) s.max = latency;
        s.d[cls]++;
    }

private:
    Registry() {
        const char *en = getenv("CACHEPOOL_PC_STATS");
        enabled_ = (en != nullptr && en[0] != '\0' && en[0] != '0');
        if (!enabled_) return;
        const auto envu = [](const char *name, uint64_t dflt) {
            const char *v = getenv(name);
            return (v && v[0]) ? strtoull(v, nullptr, 0) : dflt;
        };
        const uint32_t nb_tile = (uint32_t)envu("CACHEPOOL_NB_TILE", 1);
        const uint32_t bpt     = (uint32_t)envu("CACHEPOOL_BANKS_PER_TILE", 4);
        cores_per_tile_        = (uint32_t)envu("CACHEPOOL_CORES_PER_TILE", 4);
        tiles_per_group_       = (uint32_t)envu("CACHEPOOL_TILES_PER_GROUP", 16);
        if (tiles_per_group_ == 0) tiles_per_group_ = 1;
        // Cached DRAM window the L1 fronts (RTL cachepool_pkg: 512 MiB at 0x8000_0000).
        cached_base_ = envu("CACHEPOOL_PC_STATS_CACHED_BASE", 0x80000000ull);
        cached_size_ = envu("CACHEPOOL_PC_STATS_CACHED_SIZE", 0x20000000ull);
        geom_.init(/*n_cache*/bpt, /*n_remote*/1, /*n_cores*/cores_per_tile_,
                   /*n_tiles*/nb_tile, /*dyn_offset*/6, /*addr_w*/32, /*priv_start*/0);
        const char *regions = getenv("CACHEPOOL_L1_REGIONS");
        if (regions != nullptr) geom_.parse_regions(regions);
        if (const char *cf = getenv("CACHEPOOL_PC_STATS_CORE")) {
            for (const char *p = cf; *p; ) {
                char *end = nullptr;
                const unsigned long v = strtoul(p, &end, 0);
                if (end == p) break;
                sel_.insert((uint32_t)v);
                p = (*end == ',') ? end + 1 : end;
            }
            core_filter_ = !sel_.empty();
        }
    }

public:
    // Called from IssWrapper::stop() (every core stops after the engine halts, so the table is
    // complete); the guard makes the first caller dump and the other 255 no-ops. atexit is NOT
    // used: the engine's teardown can leave this DSO's atexit handlers unexecuted.
    void dump_at_stop() {
        if (!enabled_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (dumped_) return;
        dumped_ = true;

        uint64_t tot_n = 0, tot_lat = 0;
        for (const auto &kv : map_) { tot_n += kv.second.n; tot_lat += kv.second.lat; }
        fprintf(stderr, "[PC-STATS-TOTAL] pcs=%zu n=%llu lat=%llu\n",
                map_.size(), (unsigned long long)tot_n, (unsigned long long)tot_lat);
        for (const auto &kv : map_) {
            const Stat &s = kv.second;
            fprintf(stderr,
                    "[PC-STATS] pc=%08x n=%llu w=%llu a=%llu lat=%llu max=%llu t=%llu g=%llu r=%llu x=%llu\n",
                    kv.first, (unsigned long long)s.n, (unsigned long long)s.w,
                    (unsigned long long)s.a, (unsigned long long)s.lat,
                    (unsigned long long)s.max, (unsigned long long)s.d[0],
                    (unsigned long long)s.d[1], (unsigned long long)s.d[2],
                    (unsigned long long)s.d[3]);
        }
    }

private:
    bool enabled_ = false;
    bool dumped_ = false;
    bool core_filter_ = false;
    std::set<uint32_t> sel_;
    uint32_t cores_per_tile_ = 4, tiles_per_group_ = 16;
    uint64_t cached_base_ = 0x80000000ull, cached_size_ = 0x20000000ull;
    insitu::RouteGeom geom_;
    std::mutex mutex_;
    std::unordered_map<uint32_t, Stat> map_;
};

}   // namespace pcstat
