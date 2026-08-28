// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// The RLC DMA engine (ManyRVData rlc_am doc/RLC_HW.md §3): one engine at the
// PHY, N submission rings it round-robins across. The whole hardware contract
// is two registers per ring — a write-only submit (chain-head address) and a
// read-only retired count — plus three requirements: a ring's submissions
// retire IN ORDER, the engine holds RLC_DMA_RING_SLOTS outstanding
// submissions per ring (no acknowledgement path), and descriptor lookahead is
// the engine's own (not modeled yet — the walk here is sequential; the
// read-ahead hides descriptor latency and would only make the engine faster).
//
// Model attachment (flat cachepool topology):
//  - `input`  : the register file, on the SoC narrow AXI at the peripheral
//               block (0xC002_0000). Ring r: submit W @ r*8, retired R @ r*8+4.
//  - `desc`   : descriptor reads. Descriptors are ordinary write-back cached
//               stores in the tiles' arenas, so DRAM is stale for them — the
//               engine must read THROUGH the shared L1, like any requester
//               (RTL: "descriptor fetches across tiles"). Bound into the
//               cluster's core-0 ingress (the scheduler core's lane).
//  - `mem`    : payload reads, straight to DRAM on the wide/refill AXI — the
//               "engine at the PHY reads DRAM directly" story, valid because
//               payload writes are write-through no-allocate (DRAM current).
//
// Descriptor reads serialize (the chain pointer is a true dependency — the
// 2-entry read-ahead would hide ONE such latency; not modeled, conservative).
// Payload reads PIPELINE: issued one per cycle (the wide AXI's own occupancy
// model paces the stream), and a chain retires only once its last read has
// completed — the engine streams, it does not stop-and-wait per line. Nothing is copied anywhere — the
// value read is discarded; the memory TRAFFIC and the retire timing are the
// model. Stats dumped at stop(): [RLC-DMA] chains= desc= lines= busy=.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <queue>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

class RlcDma : public vp::Component
{
public:
    RlcDma(vp::ComponentConf &conf);
    void reset(bool active) override;
    void stop() override
    {
        if (cnt_chains_ | cnt_desc_)
            fprintf(stderr, "[RLC-DMA %s] chains=%lu desc=%lu payload_lines=%lu busy_cycles=%lu\n",
                    this->get_path().c_str(), (unsigned long)cnt_chains_,
                    (unsigned long)cnt_desc_, (unsigned long)cnt_lines_,
                    (unsigned long)cnt_busy_);
        vp::Component::stop();
    }

private:
    // 16 B scatter-gather element (drivers/dma.h). Packed; `chain`==0 ends the chain.
    struct __attribute__((packed)) Descriptor
    {
        uint8_t flags;
        uint8_t header[5];
        uint32_t source;
        uint16_t length;
        uint32_t chain;
    };

    enum class State { Idle, DescIssue, DescWait, PayloadIssue, PayloadWait };

    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req);
    static void resp_handler(vp::Block *__this, vp::IoReq *req);
    static void fsm_handler(vp::Block *__this, vp::ClockEvent *event);
    void fsm();
    void schedule(int64_t cycles)
    {
        if (!fsm_event_->is_enqueued())
            this->event_enqueue(fsm_event_, cycles > 0 ? cycles : 1);
    }
    bool pick_ring();   // round-robin scan; loads current_head_, returns found

    vp::IoSlave input_itf_;
    vp::IoMaster desc_itf_, mem_itf_;
    vp::ClockEvent *fsm_event_ = nullptr;
    vp::Trace trace_;

    uint32_t nb_rings_ = 16;
    uint32_t line_bytes_ = 64;
    std::vector<std::queue<uint32_t>> pending_;   // submitted chain heads per ring
    std::vector<uint32_t> retired_;

    State state_ = State::Idle;
    uint32_t rr_ = 0;              // ring being served
    uint32_t current_head_ = 0;    // descriptor address being walked
    Descriptor desc_buf_ = {};
    uint32_t payload_next_ = 0, payload_last_ = 0;   // line addresses
    uint8_t line_buf_[64];
    vp::IoReq walk_req_;
    bool req_pending_ = false;

    uint64_t cnt_chains_ = 0, cnt_desc_ = 0, cnt_lines_ = 0, cnt_busy_ = 0;
    int64_t busy_since_ = -1;
    int64_t chain_done_cycle_ = 0;   // completion horizon of the chain's issued payload reads
};

RlcDma::RlcDma(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    if (cfg->get("nb_rings") != nullptr) nb_rings_ = cfg->get_child_int("nb_rings");
    if (cfg->get("line_bytes") != nullptr) line_bytes_ = cfg->get_child_int("line_bytes");

    input_itf_.set_req_meth(&RlcDma::req_handler);
    this->new_slave_port("input", &input_itf_);
    desc_itf_.set_resp_meth(&RlcDma::resp_handler);
    this->new_master_port("desc", &desc_itf_);
    mem_itf_.set_resp_meth(&RlcDma::resp_handler);
    this->new_master_port("mem", &mem_itf_);
    fsm_event_ = this->event_new(&RlcDma::fsm_handler);
    this->traces.new_trace("trace", &trace_, vp::DEBUG);

    pending_.resize(nb_rings_);
    retired_.assign(nb_rings_, 0);
}

void RlcDma::reset(bool active)
{
    if (active)
    {
        for (auto &q : pending_) while (!q.empty()) q.pop();
        std::fill(retired_.begin(), retired_.end(), 0);
        state_ = State::Idle;
        req_pending_ = false;
        busy_since_ = -1;
    }
}

vp::IoReqStatus RlcDma::req_handler(vp::Block *__this, vp::IoReq *req)
{
    RlcDma *_this = static_cast<RlcDma *>(__this);
    const uint32_t offset = (uint32_t)req->get_addr();
    const uint32_t ring = offset >> 3;
    if (ring >= _this->nb_rings_ || req->get_size() != 4)
        return vp::IO_REQ_INVALID;

    if (req->get_is_write() && (offset & 7u) == 0)
    {
        uint32_t head = 0;
        memcpy(&head, req->get_data(), 4);
        _this->trace_.msg(vp::Trace::LEVEL_DEBUG, "submit ring=%u head=0x%x\n", ring, head);
        _this->pending_[ring].push(head);
        if (_this->state_ == State::Idle) _this->schedule(1);
        return vp::IO_REQ_OK;
    }
    if (!req->get_is_write() && (offset & 7u) == 4)
    {
        memcpy(req->get_data(), &_this->retired_[ring], 4);
        return vp::IO_REQ_OK;
    }
    return vp::IO_REQ_INVALID;
}

bool RlcDma::pick_ring()
{
    for (uint32_t i = 0; i < nb_rings_; i++)
    {
        const uint32_t r = (rr_ + 1 + i) % nb_rings_;
        if (!pending_[r].empty())
        {
            rr_ = r;
            current_head_ = pending_[r].front();
            return true;
        }
    }
    return false;
}

void RlcDma::fsm_handler(vp::Block *__this, vp::ClockEvent *event)
{
    static_cast<RlcDma *>(__this)->fsm();
}

void RlcDma::resp_handler(vp::Block *__this, vp::IoReq *req)
{
    // Asynchronous completion of the one in-flight read: resume the walk.
    RlcDma *_this = static_cast<RlcDma *>(__this);
    _this->req_pending_ = false;
    _this->schedule(1);
}

void RlcDma::fsm()
{
    if (req_pending_) return;   // resumed by resp_handler

    switch (state_)
    {
    case State::Idle:
        if (!pick_ring())
        {
            if (busy_since_ >= 0)
            {
                cnt_busy_ += this->clock.get_cycles() - busy_since_;
                busy_since_ = -1;
            }
            return;
        }
        if (busy_since_ < 0) busy_since_ = this->clock.get_cycles();
        chain_done_cycle_ = this->clock.get_cycles();
        state_ = State::DescIssue;
        [[fallthrough]];

    case State::DescIssue:
    {
        walk_req_.init();
        walk_req_.set_addr(current_head_);
        walk_req_.set_size(sizeof(Descriptor));
        walk_req_.set_is_write(false);
        walk_req_.set_data((uint8_t *)&desc_buf_);
        cnt_desc_++;
        state_ = State::DescWait;
        const vp::IoReqStatus st = desc_itf_.req(&walk_req_);
        if (st == vp::IO_REQ_OK) { schedule((int64_t)walk_req_.get_full_latency()); return; }
        if (st == vp::IO_REQ_PENDING || st == vp::IO_REQ_DENIED) { req_pending_ = true; return; }
        // INVALID: drop the chain (software bug); retire so the ring is not wedged.
        trace_.msg(vp::Trace::LEVEL_WARNING, "invalid descriptor read at 0x%x\n", current_head_);
        desc_buf_.chain = 0; desc_buf_.length = 0; desc_buf_.source = 0;
        schedule(1);
        return;
    }

    case State::DescWait:
    {
        // Descriptor is in desc_buf_; queue its payload lines.
        if (desc_buf_.source != 0 && desc_buf_.length != 0)
        {
            const uint32_t mask = ~(line_bytes_ - 1u);
            payload_next_ = desc_buf_.source & mask;
            payload_last_ = (desc_buf_.source + desc_buf_.length - 1u) & mask;
            state_ = State::PayloadIssue;
        }
        else
        {
            state_ = State::PayloadWait;   // no payload: go decide chain/next
        }
        schedule(1);
        return;
    }

    case State::PayloadIssue:
    {
        walk_req_.init();
        walk_req_.set_addr(payload_next_);
        walk_req_.set_size(line_bytes_);
        walk_req_.set_is_write(false);
        walk_req_.set_data(line_buf_);
        cnt_lines_++;
        const bool last = payload_next_ == payload_last_;
        payload_next_ += line_bytes_;
        if (last) state_ = State::PayloadWait;
        const vp::IoReqStatus st = mem_itf_.req(&walk_req_);
        if (st == vp::IO_REQ_OK)
        {
            // Pipelined: next issue next cycle; the chain completes when the
            // LAST read completes (tracked, awaited before retiring).
            const int64_t done = this->clock.get_cycles() + (int64_t)walk_req_.get_full_latency();
            if (done > chain_done_cycle_) chain_done_cycle_ = done;
            schedule(1);
            return;
        }
        if (st == vp::IO_REQ_PENDING || st == vp::IO_REQ_DENIED) { req_pending_ = true; return; }
        trace_.msg(vp::Trace::LEVEL_WARNING, "invalid payload read at 0x%x\n", payload_next_ - line_bytes_);
        schedule(1);
        return;
    }

    case State::PayloadWait:
    {
        // Descriptor fully transferred: follow the chain.
        if (desc_buf_.chain != 0)
        {
            current_head_ = desc_buf_.chain;
            state_ = State::DescIssue;
        }
        else
        {
            // Chain fully issued: wait for its last payload read to complete,
            // then retire IN ORDER on this ring and round-robin onward.
            const int64_t now = this->clock.get_cycles();
            if (chain_done_cycle_ > now) { schedule(chain_done_cycle_ - now); chain_done_cycle_ = now; return; }
            pending_[rr_].pop();
            retired_[rr_]++;
            cnt_chains_++;
            state_ = State::Idle;
        }
        schedule(1);
        return;
    }
    }
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new RlcDma(config);
}
