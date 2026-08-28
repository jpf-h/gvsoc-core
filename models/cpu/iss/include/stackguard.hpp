// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// Stack-overflow guard (CACHEPOOL_STACK_GUARD="base:size", hex): any scalar
// load/store/atomic touching [base, base+size) aborts the simulation with the
// issuing PC, core and address. The band sits just below the per-core SPM
// stack window, where an overflowing stack frame lands first — overflow there
// otherwise writes silently into uncached DRAM and corrupts whatever lives at
// those addresses (observed in practice; this trades silent corruption for an
// immediate, located abort). The cachepool target arms it by default; set
// CACHEPOOL_STACK_GUARD=0 to disable. Zero cost when unset.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace stackguard {

class Guard {
public:
    static const Guard &get() {
        static Guard inst;
        return inst;
    }
    inline bool hits(uint64_t addr, uint64_t size) const {
        // [addr, addr+size) intersects [base_, base_+size_)?
        return size_ != 0 && addr < base_ + size_ && addr + size > base_;
    }
    __attribute__((noreturn))
    void trip(uint32_t pc, uint32_t core, uint64_t addr, uint64_t size, bool is_write) const {
        fprintf(stderr,
                "[STACK-GUARD] FATAL: core %u %s %u bytes at 0x%08llx (guard band 0x%08llx..0x%08llx) "
                "pc=0x%08x — stack overflow past the SPM window. Resolve the pc with addr2line/nm.\n",
                core, is_write ? "wrote" : "read", (unsigned)size,
                (unsigned long long)addr, (unsigned long long)base_,
                (unsigned long long)(base_ + size_), pc);
        abort();
    }

private:
    Guard() {
        const char *env = getenv("CACHEPOOL_STACK_GUARD");
        if (env == nullptr || env[0] == '\0' || (env[0] == '0' && env[1] == '\0')) return;
        unsigned long long b = 0, s = 0;
        if (sscanf(env, "%llx:%llx", &b, &s) == 2) { base_ = b; size_ = s; }
    }
    uint64_t base_ = 0, size_ = 0;
};

}   // namespace stackguard
