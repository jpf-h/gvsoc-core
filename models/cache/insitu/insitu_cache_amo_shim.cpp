// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — AMO / LR-SC shim (spatz_cache_amo), instanced on the scalar lane (j=4) of each cell.
//
// Wraps the validated `insitu_cache_amo.hpp` (ALU + LR/SC reservation) in an event shell that sits
// between the tile's scalar-lane crossbar and the cache core. The RTL puts spatz_cache_amo only on lane
// NrTCDMPortsPerCore-1 (cachepool_tile.sv:658), one per controller; the Spatz VLSU lanes bypass it.
//
// Handling by IoReqOpcode (engine io.hpp: READ=0 WRITE=1 LR=2 SC=3 SWAP=4 ADD=5 XOR=6 AND=7 OR=8 MIN=9
// MAX=10 MINU=11 MAXU=12):
//   - READ / WRITE  → pass straight through to the core (req_forward; response auto-routes upstream).
//                     A WRITE to the reserved address clears the reservation.
//   - LR            → set the reservation, present as a plain READ to the core (the load value returns).
//   - SC            → success (reservation valid + addr match): write the store to the core, then return
//                     0; failure: return 1 WITHOUT writing. (RISC-V SC semantics.)
//   - true AMO      → read-modify-write: read the line word from the core, compute amo_alu(op, old, b),
//                     write the result back, then return the OLD value. The lane is atomic: the scalar
//                     LSU is single-outstanding, so at most one AMO/SC is in flight (no queue needed).
//
// SCOPE: the open-loop calib issues only READ/WRITE, so this validates the PASS-THROUGH path
// (data_err=0, scalar lane unchanged). The LR/SC/AMO paths use the standalone-validated amo.hpp and are
// exercised closed-loop (Spatz). Cross-LANE reservation clearing (a VLSU write to the reserved addr,
// which bypasses this shim) and the DMA-vs-AMO conflict detector are deferred (the reservation here is
// per-bank: same-address LR/SC route to the same bank → same shim, so same-address LR/SC are correct).

#include <cstdint>
#include <cstring>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include "insitu_cache_amo.hpp"

using namespace insitu;

namespace {
// engine IoReqOpcode → amo.hpp AmoOp (the two enums differ).
inline uint8_t opcode_to_amo(vp::IoReqOpcode op)
{
    switch (op) {
    case vp::LR:   return AMO_LR;
    case vp::SC:   return AMO_SC;
    case vp::SWAP: return AMO_SWAP;
    case vp::ADD:  return AMO_ADD;
    case vp::XOR:  return AMO_XOR;
    case vp::AND:  return AMO_AND;
    case vp::OR:   return AMO_OR;
    case vp::MIN:  return AMO_MIN;
    case vp::MAX:  return AMO_MAX;
    case vp::MINU: return AMO_MINU;
    case vp::MAXU: return AMO_MAXU;
    case vp::CAS:  return AMO_CAS;
    default:       return AMO_NONE;
    }
}
}

class InsituCacheAmo : public vp::Component
{
public:
    explicit InsituCacheAmo(vp::ComponentConf &conf);
    void stop() override {
        fprintf(stderr, "[INSITU-AMO %s] rmw=%lu lat_sum=%lu\n", this->get_path().c_str(),
                (unsigned long)n_rmw_, (unsigned long)lat_rmw_sum_);
        vp::Component::stop();
    }

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static void resp_handler(vp::Block *__this, vp::IoReq *req);

    Reservation res_;
    uint32_t word_bytes_;

    // single in-flight AMO/SC transaction (the scalar lane is single-outstanding → atomic).
    enum Phase { IDLE, AMO_READ, AMO_WRITE, SC_WRITE };
    Phase    phase_ = IDLE;
    // Synchronous-slave cache (cachepool run_request_sync): the sub-read/sub-write to the core complete
    // in-call (req returns IO_REQ_OK, not PENDING), so the whole RMW/SC resolves INSIDE req_handler. In that
    // case the shim must return IO_REQ_OK (the result is already written into the upstream req's data) and
    // must NOT call resp() — calling resp() and then returning PENDING double-completes the request (SIGABRT).
    // On the async cache (Spatz controller) the sub-ops return PENDING and resp() fires later (PENDING path).
    bool     in_sync_call_ = false;
    bool     sync_completed_ = false;
    vp::IoReq *orig_ = nullptr;   // the held upstream AMO/SC request
    uint8_t  amo_op_ = AMO_NONE;
    uint64_t amo_addr_ = 0;
    uint32_t old_val_ = 0, b_val_ = 0, cas_cmp_ = 0;
    uint8_t  scratch_buf_[8];     // value buffer for the scratch read/write
    vp::IoReq scratch_;

    // B3 RMW lane occupancy (spatz_cache_amo.sv: core_ready=0 in DoAMO/WriteBackAMO/Wait — the
    // bank-shared scalar lane is HELD from RMW accept until the write-back drains, ~15-20 cy on a
    // hit, full refill time on a miss). rmw_busy_until_ = the cycle the window ends; any new request
    // on the lane (any opcode) waits it out. rmw_write_rtt_cycles_ = the write-back RTT knob.
    int64_t  rmw_busy_until_ = 0;
    int64_t  rmw_read_lat_ = 0;   // the scratch read's latency (captured before scratch_ re-init)
    int32_t  rmw_write_rtt_cycles_ = 8;
    // Cap on one op's window contribution (see the .py). The requester's own
    // latency is never capped — only the hold the NEXT requests wait out.
    // Uncapped chaining by measured latencies double-charges queueing (the
    // waiters were already stamped their wait) and feeds back: delayed
    // requests bunch, downstream latencies rise, windows grow — observed as
    // megacycle stalls on AMO-dense phases (rlc_am doc/PLAN.md item 77).
    int64_t  rmw_window_cap_cycles_ = 128;

    vp::IoSlave  input_;
    vp::IoMaster output_;
    vp::Trace trace_;
    // diagnostics: op count + total stamped latency (dumped at stop())
    uint64_t n_rmw_ = 0, lat_rmw_sum_ = 0;
};

InsituCacheAmo::InsituCacheAmo(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    word_bytes_ = cfg->get_child_int("word_bytes");
    if (word_bytes_ < 1) word_bytes_ = 4;
    // B3: the write-back RTT of an RMW (the part after the scratch read). -1 → default 8
    // (read-hit ~10 + 1 + 8 ≈ 19 cy end-to-end, matching the RTL's ~15-20 cy).
    int32_t wrtt = cfg->get_child_int("amo_rmw_write_rtt_cycles");
    if (wrtt >= 0) rmw_write_rtt_cycles_ = wrtt;
    int32_t wcap = cfg->get_child_int("amo_rmw_window_cap_cycles");
    if (wcap >= 0) rmw_window_cap_cycles_ = wcap;

    input_.set_req_meth(&InsituCacheAmo::req_handler);
    this->new_slave_port("input", &input_);
    output_.set_resp_meth(&InsituCacheAmo::resp_handler);
    this->new_master_port("out", &output_);

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheAmo word=%u\n", word_bytes_);
}

vp::IoReqStatus InsituCacheAmo::req_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheAmo *_this = static_cast<InsituCacheAmo *>(__this);
    const vp::IoReqOpcode op = req->get_opcode();
    const uint64_t addr = req->get_addr();
    const uint32_t core = (uint32_t)req->get_initiator();

    // B3: a new request on this lane (any opcode) waits out the previous RMW's occupancy window.
    const int64_t now = _this->clock.get_cycles();
    if (now < _this->rmw_busy_until_) req->inc_latency(_this->rmw_busy_until_ - now);

    // --- plain accesses: pass through ---
    if (op == vp::READ) {
        return _this->output_.req_forward(req);
    }
    if (op == vp::WRITE) {
        _this->res_.on_foreign_access(core, addr, AMO_NONE, /*is_write=*/true);
        return _this->output_.req_forward(req);
    }

    // --- LR: set reservation, present as a plain READ (the load value returns to the core) ---
    if (op == vp::LR) {
        _this->res_.on_lr(core, addr);
        req->set_opcode(vp::READ);
        // CRITICAL, same convention as the AMO write-back below (lsu.cpp:547-549, mirrored by
        // memory.cpp's handle_atomic, which passes get_second_data() as its out_data for LR too):
        // for an atomic, get_data() aliases rs2 and get_second_data() aliases rd. A plain READ
        // writes into get_data() -- so forwarding LR untouched put the loaded value in rs2 and left
        // rd stale.
        if (req->get_second_data() != nullptr) {
            req->set_data(req->get_second_data());
        }
        return _this->output_.req_forward(req);
    }

    // --- SC: success → write + return 0; failure → return 1, no write ---
    if (op == vp::SC) {
        const bool ok = _this->res_.on_sc(core, addr);
        if (!ok) {
            uint8_t *dst = req->get_second_data() != nullptr ? req->get_second_data() : req->get_data();
            if (dst != nullptr) { uint32_t one = 1; memcpy(dst, &one, _this->word_bytes_ <= 4 ? _this->word_bytes_ : 4); }
            return vp::IO_REQ_OK;   // SC fail: no memory write, synchronous response (data=1)
        }
        // success: write the store to the core, then return 0 on the write-back response.
        _this->phase_ = SC_WRITE;
        _this->orig_  = req;
        _this->scratch_.init();
        _this->scratch_.set_addr(addr);
        _this->scratch_.set_size(req->get_size());
        _this->scratch_.set_is_write(true);
        _this->scratch_.set_data(req->get_data());   // the SC store data
        _this->in_sync_call_ = true; _this->sync_completed_ = false;
        vp::IoReqStatus st = _this->output_.req(&_this->scratch_);
        if (st == vp::IO_REQ_OK) { resp_handler(_this, &_this->scratch_); }
        _this->in_sync_call_ = false;
        return _this->sync_completed_ ? vp::IO_REQ_OK : vp::IO_REQ_PENDING;
    }

    // --- true AMO: read-modify-write ---
    _this->phase_   = AMO_READ;
    _this->orig_    = req;
    _this->amo_op_  = opcode_to_amo(op);
    _this->amo_addr_= addr;
    memcpy(&_this->b_val_, req->get_data(), 4);       // the store operand
    // amocas (Zacas): the compare value travels in second_data — the very
    // register the old value is returned through (rv32a.hpp) — so it is read
    // here, before the completion path overwrites it.
    if (_this->amo_op_ == AMO_CAS && req->get_second_data() != nullptr)
        memcpy(&_this->cas_cmp_, req->get_second_data(), 4);
    _this->scratch_.init();
    _this->scratch_.set_addr(addr);
    _this->scratch_.set_size(req->get_size());
    _this->scratch_.set_is_write(false);
    _this->scratch_.set_data(_this->scratch_buf_);
    _this->in_sync_call_ = true; _this->sync_completed_ = false;
    vp::IoReqStatus st = _this->output_.req(&_this->scratch_);
    if (st == vp::IO_REQ_OK) { resp_handler(_this, &_this->scratch_); }
    _this->in_sync_call_ = false;
    return _this->sync_completed_ ? vp::IO_REQ_OK : vp::IO_REQ_PENDING;
}

void InsituCacheAmo::resp_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheAmo *_this = static_cast<InsituCacheAmo *>(__this);
    if (req != &_this->scratch_) return;   // not our scratch (shouldn't happen)

    if (_this->phase_ == AMO_READ) {
        memcpy(&_this->old_val_, _this->scratch_buf_, 4);
        _this->rmw_read_lat_ = _this->scratch_.get_full_latency();   // B3: before scratch_ re-init
        // amocas: a mismatch performs no write at all (Zacas). The old value
        // still returns through the completion path below; the lane pays the
        // read plus one cycle rather than the full write-back RTT.
        if (_this->amo_op_ == AMO_CAS && _this->old_val_ != _this->cas_cmp_) {
            vp::IoReq *orig = _this->orig_;
            uint8_t *result_dst = (orig != nullptr && orig->get_second_data() != nullptr)
                                  ? orig->get_second_data() : (orig != nullptr ? orig->get_data() : nullptr);
            if (result_dst != nullptr) memcpy(result_dst, &_this->old_val_, 4);
            _this->phase_ = IDLE;
            _this->orig_ = nullptr;
            if (orig != nullptr) {
                const int64_t total = _this->rmw_read_lat_ + 1;
                orig->inc_latency(total);
                const int64_t now2 = _this->clock.get_cycles();
                const int64_t hold = total < _this->rmw_window_cap_cycles_
                                     ? total : _this->rmw_window_cap_cycles_;
                _this->rmw_busy_until_ = (_this->rmw_busy_until_ > now2 ? _this->rmw_busy_until_ : now2) + hold;
                _this->rmw_read_lat_ = 0;
                _this->n_rmw_++; _this->lat_rmw_sum_ += (uint64_t)orig->get_full_latency();
            }
            if (_this->in_sync_call_) _this->sync_completed_ = true;
            else if (orig != nullptr) orig->get_resp_port()->resp(orig);
            return;
        }
        const uint32_t newv = amo_alu(_this->amo_op_, _this->old_val_, _this->b_val_);
        memcpy(_this->scratch_buf_, &newv, 4);
        _this->phase_ = AMO_WRITE;
        _this->scratch_.init();
        _this->scratch_.set_addr(_this->amo_addr_);
        _this->scratch_.set_size(4);
        _this->scratch_.set_is_write(true);
        _this->scratch_.set_data(_this->scratch_buf_);
        vp::IoReqStatus st = _this->output_.req(&_this->scratch_);
        if (st == vp::IO_REQ_OK) { resp_handler(_this, &_this->scratch_); }
        return;
    }

    // AMO_WRITE done → return the OLD value; SC_WRITE done → return 0 (success).
    // CRITICAL: the ISS AMO convention (lsu.cpp:547-549, mirrored by memory.cpp's atomics) is operand =
    // get_data(), RESULT = get_second_data() (which aliases the destination register). The old value MUST be
    // written to get_second_data() — writing it to get_data() (the previous behaviour) left rd unchanged, so
    // the core's beqz/bnez acquire check read stale register garbage and the spin lock never resolved.
    vp::IoReq *orig = _this->orig_;
    uint8_t *result_dst = (orig != nullptr && orig->get_second_data() != nullptr)
                          ? orig->get_second_data() : (orig != nullptr ? orig->get_data() : nullptr);
    if (result_dst != nullptr) {
        if (_this->phase_ == AMO_WRITE) memcpy(result_dst, &_this->old_val_, 4);
        else /* SC_WRITE */            { uint32_t zero = 0; memcpy(result_dst, &zero, 4); }
    }
    _this->phase_ = IDLE;
    _this->orig_ = nullptr;
    if (orig != nullptr) {
        // B3: the requester sees the full RMW round trip (scratch read + 1 + write-back), and the
        // lane stays busy for the same window (core_ready=0 until the write-back drains). A scratch
        // read MISS already carries the refill latency, so a missing RMW costs ~refill+write ≈ the
        // RTL's "full refill time on a miss". SC has no separate read: write + RTT. The window
        // CHAINS: it starts after the previous RMW's window (the wait stamped at entry) — setting
        // busy = now + total would let overlapping windows shrink the serialization.
        const int64_t total = (_this->rmw_read_lat_ > 0)
            ? (_this->rmw_read_lat_ + 1 + _this->scratch_.get_full_latency())
            : (_this->scratch_.get_full_latency() + _this->rmw_write_rtt_cycles_);
        orig->inc_latency(total);
        const int64_t now = _this->clock.get_cycles();
        const int64_t hold = total < _this->rmw_window_cap_cycles_
                             ? total : _this->rmw_window_cap_cycles_;
        _this->rmw_busy_until_ = (_this->rmw_busy_until_ > now ? _this->rmw_busy_until_ : now) + hold;
        _this->rmw_read_lat_ = 0;
        _this->n_rmw_++; _this->lat_rmw_sum_ += (uint64_t)orig->get_full_latency();
    }
    if (_this->in_sync_call_) {
        // resolved inside req_handler (sync cache): tell it to return IO_REQ_OK; do NOT resp() (the result
        // is already in orig's data buffer). resp()+PENDING would double-complete the upstream request.
        _this->sync_completed_ = true;
    } else if (orig != nullptr) {
        orig->get_resp_port()->resp(orig);
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheAmo(config);
}
