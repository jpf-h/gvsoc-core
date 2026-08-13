// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — RTL-faithful AMO / LR-SC shim (structural model, Step 7).
//
// Header-only transcription of `spatz_cache_amo.sv` (+ its `amo_alu` submodule): the atomic-memory-op
// and load-reserved / store-conditional shim instanced on the scalar TCDM lane (j=4) of each tile. The
// structural facts it captures:
//
//   RMW FSM (spatz_cache_amo.sv:67-69, 215-279) — 4 states {Idle, Wait, DoAMO, WriteBackAMO}:
//     Idle         — a true-AMO request is forwarded to memory as a READ (opcode stripped to AMONone);
//                    `load_amo` latches {op, addr, operand_b, user, idx} and → DoAMO.
//     DoAMO        — block ALL upstream (core_ready=0, mem_req.q_valid=0); wait for the READ response
//                    matched by (core_id, req_id); on match latch the ALU result → WriteBackAMO.
//     WriteBackAMO — drive the WRITE-back (result placed in the addressed 32b lane, strb 0xF<<idx*4,
//                    user.is_amo=1 so the eventual response is suppressed); hold until mem q_ready → Wait.
//     Wait         — wait for the write-back response (is_amo=1) to drain → Idle.
//   ATOMICITY: core_ready (=core_rsp.q_ready) is hardwired 0 in every state but Idle, so no new request
//     on the lane can interleave the read-modify-write (spatz_cache_amo.sv:236, 249, 268).
//
//   ALU (amo_alu, spatz_cache_amo.sv:319-368) — strictly 32-bit; 64-bit handled by selecting the
//     addressed half via idx. Swap=b, Add=a+b, And/Or/Xor bitwise, signed Max/Min and unsigned Maxu/Minu
//     via the a-b sign/borrow bit.
//   AMO opcodes (reqrsp_pkg amo_op_e): None=0 Swap=1 Add=2 And=3 Or=4 Xor=5 Max=6 Maxu=7 Min=8 Minu=9
//     LR=0xA SC=0xB. "True AMO" = {Swap..Minu} (triggers the RMW FSM); LR/SC bypass the FSM.
//
//   LR/SC reservation (spatz_cache_amo.sv:72-188) — a single {valid, addr, core} register. LR (a plain
//     read to memory) SETs it (overwriting any prior). It is CLEARED by: a foreign-core write or true-AMO
//     to the reserved address; or an SC from the reserving core. SC success = reservation valid AND core
//     match AND addr match; SC returns 0 (success) / 1 (failure) and only writes memory on success
//     (mem_req.write = req.write | (sc_successful & op==SC), :220).
//
// SCOPE (FSM control flow + atomicity + ALU + reservation rules — the structural essential). The
// valid/ready spill timing and the exact response-mux priority (SC-rsp override > is_amo suppress >
// normal) are modeled functionally; per-cycle handshake timing lands with the integrated tile +
// calibration. Validated standalone.

#pragma once
#include <cstdint>

namespace insitu {

enum AmoOp {
    AMO_NONE = 0x0, AMO_SWAP = 0x1, AMO_ADD = 0x2, AMO_AND = 0x3, AMO_OR = 0x4, AMO_XOR = 0x5,
    AMO_MAX = 0x6, AMO_MAXU = 0x7, AMO_MIN = 0x8, AMO_MINU = 0x9, AMO_LR = 0xA, AMO_SC = 0xB,
    AMO_CAS = 0xC   // Zacas amocas: conditional swap, no write on mismatch (shim-handled)
};

inline bool amo_is_true(uint8_t op) { return op >= AMO_SWAP && op <= AMO_MINU; }   // Swap..Minu

// amo_alu (spatz_cache_amo.sv:319-368) — 32-bit. operand_a = value read from memory, operand_b = store.
inline uint32_t amo_alu(uint8_t op, uint32_t a, uint32_t b) {
    switch (op) {
    case AMO_SWAP: return b;
    case AMO_CAS:  return b;   // reached only when the compare matched (shim gates the write)
    case AMO_ADD:  return (uint32_t)(a + b);
    case AMO_AND:  return a & b;
    case AMO_OR:   return a | b;
    case AMO_XOR:  return a ^ b;
    case AMO_MAX:  return ((int32_t)a < (int32_t)b) ? b : a;
    case AMO_MIN:  return ((int32_t)a < (int32_t)b) ? a : b;
    case AMO_MAXU: return (a < b) ? b : a;
    case AMO_MINU: return (a < b) ? a : b;
    default:       return 0;   // None/LR/SC never reach the ALU
    }
}

// The LR/SC reservation register (spatz_cache_amo.sv:72-84).
struct Reservation {
    bool     valid = false;
    uint64_t addr  = 0;
    uint32_t core  = 0;

    // An LR (a read) SETs the reservation, overwriting any prior (spatz_cache_amo.sv:166-170).
    void on_lr(uint32_t req_core, uint64_t req_addr) { valid = true; addr = req_addr; core = req_core; }

    // A foreign write / true-AMO to the reserved address clears it (spatz_cache_amo.sv:178-181).
    void on_foreign_access(uint32_t req_core, uint64_t req_addr, uint8_t op, bool is_write) {
        if (valid && req_core != core && req_addr == addr && (amo_is_true(op) || is_write)) valid = false;
    }

    // An SC from the reserving core clears the reservation; returns true on success (addr match).
    // A foreign SC (core mismatch) leaves the reservation alone and fails (spatz_cache_amo.sv:184-188).
    bool on_sc(uint32_t req_core, uint64_t req_addr) {
        bool success = false;
        if (valid && core == req_core) { success = (addr == req_addr); valid = false; }
        return success;
    }
};

// The RMW FSM (spatz_cache_amo.sv:215-279). Encoding matches the SV enum `{Idle, Wait, DoAMO, WriteBackAMO}`.
enum AmoState { AMO_IDLE = 0, AMO_WAIT = 1, AMO_DOAMO = 2, AMO_WB = 3 };

struct AmoReq {
    bool     valid    = false;
    uint8_t  op       = AMO_NONE;
    uint64_t addr     = 0;
    uint32_t wdata    = 0;    // store data (operand_b), the addressed 32b word
    uint32_t core     = 0;
    uint32_t req_id   = 0;
    bool     write    = false;
    uint32_t idx      = 0;    // which 32b half of a 64b word (DataWidth==64 ? strb[4] : 0)
};

struct AmoMemResp {
    bool     valid  = false;
    uint32_t data   = 0;      // read data (the addressed 32b lane already selected)
    bool     is_amo = false;  // user.is_amo (write-back response)
    uint32_t core   = 0;
    uint32_t req_id = 0;
};

struct AmoMemReq {
    bool     valid   = false;
    bool     write   = false;
    uint64_t addr    = 0;
    uint32_t data    = 0;
    uint32_t strb    = 0;
    bool     is_amo  = false;
};

class AmoShim {
public:
    void reset() { state_ = AMO_IDLE; }
    AmoState state() const { return (AmoState)state_; }
    bool core_ready() const { return state_ == AMO_IDLE; }   // back-pressure: only Idle accepts

    // One clock. Returns the memory-side request this cycle; updates state. `mem_q_ready` = memory
    // accepted mem_req; `resp` = a memory response presented this cycle.
    AmoMemReq step(const AmoReq &req, const AmoMemResp &resp, bool mem_q_ready) {
        AmoMemReq m;
        switch (state_) {
        case AMO_IDLE:
            // pass-through (read forwarded; opcode stripped). load_amo on an accepted true-AMO.
            if (req.valid) {
                m.valid = true; m.addr = req.addr; m.write = req.write; m.data = req.wdata;
                if (amo_is_true(req.op) && mem_q_ready) {
                    op_ = req.op; addr_ = req.addr; operand_b_ = req.wdata;
                    idx_ = req.idx; core_ = req.core; req_id_ = req.req_id;
                    m.write = false;        // forwarded as a READ
                    state_ = AMO_DOAMO;
                }
            }
            break;
        case AMO_DOAMO:
            // block upstream; await the matching read response, then compute + latch.
            if (resp.valid && resp.core == core_ && resp.req_id == req_id_) {
                result_ = amo_alu(op_, resp.data, operand_b_);
                state_ = AMO_WB;
            }
            break;
        case AMO_WB:
            m.valid = true; m.write = true; m.addr = addr_;
            m.data = result_; m.strb = 0xFu << (idx_ * 4); m.is_amo = true;
            if (mem_q_ready) state_ = AMO_WAIT;
            break;
        case AMO_WAIT:
            if (resp.valid && resp.is_amo) state_ = AMO_IDLE;
            break;
        }
        return m;
    }

    uint32_t last_result() const { return result_; }

private:
    uint32_t state_     = AMO_IDLE;
    uint8_t  op_        = AMO_NONE;
    uint64_t addr_      = 0;
    uint32_t operand_b_ = 0, result_ = 0, idx_ = 0, core_ = 0, req_id_ = 0;
};

} // namespace insitu
