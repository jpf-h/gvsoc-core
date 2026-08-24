// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — structural per-port-class crossbar (one tcdm_cache_interco lane).
//
// The RTL-faithful replacement for the hashed InsituCacheInterco. The tile instantiates ONE of these
// PER port-class j (NrTCDMPortsPerCore=5 of them), each routing that lane's core ports to the cache
// banks BY ADDRESS using the real `insitu_cache_route.hpp` logic (BankSel/TileID field extraction,
// the three partition modes, MSB address rotation, remote-tile slot routing) — instead of the single
// flat Knuth-style hash. See cachepool_tile.sv:604-652 (gen_cache_xbar) and prompt/
// insitu_cache_structural_tile_plan_2026-06-18.md.
//
// Ports: in_{0..NumInputs-1} = NrCores local core ports (this lane) + NumRemotePort remote-in slots;
//        out_{0..NumOutputs-1} = NumCache local banks + NumRemotePort remote-out slots.
// Response routing: GVSoC carries the originating slave's response port on the IoReq, so a LOCAL
// forward returns to the originating core automatically (no explicit demux needed in single-tile).
// Remote response routing (by tile_id) lands with the group composite.
//
// SCOPE (Phase A1): single-tile local routing validated first. MSB rotation is behind `enable_rotation`
// (default off — the bank then uses a strided subset of its sets, functionally correct, no aliasing;
// faithful rotation + the matching refill un-rotation land in Phase A2). Remote slots are present but
// inactive when num_tiles==1 (route_request always returns local).

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include "insitu_cache_route.hpp"

using namespace insitu;

class InsituCacheXbar : public vp::Component
{
public:
    explicit InsituCacheXbar(vp::ComponentConf &conf);
    void stop() override {
        if (na_size_ != 0 && (cnt_sb_hit_ | cnt_sb_fill_ | cnt_pf_))
            fprintf(stderr, "[INSITU-XBAR %s] sb_hit=%lu sb_fill=%lu sb_wait_sum=%lu pf=%lu pf_late=%lu inval=%lu\n",
                    this->get_path().c_str(), (unsigned long)cnt_sb_hit_,
                    (unsigned long)cnt_sb_fill_, (unsigned long)cnt_sb_wait_,
                    (unsigned long)cnt_pf_, (unsigned long)cnt_pf_late_,
                    (unsigned long)cnt_inval_);
        vp::Component::stop();
    }

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req, int input_id);
    // E3 runtime partition config (csr-id in addr: 0=num_private, 1=private_start, 2=dyn_offset).
    static vp::IoReqStatus config_handler(vp::Block *__this, vp::IoReq *req);

    // --- REQUESTER-SIDE STREAM BUFFER (rlc_am doc/RLC_HW.md §2, drivers/hint.h) ---
    // One line buffer per LOCAL core port of this lane, at the requesting tile — where
    // drivers/hint.h places it — rather than at the home bank, where every requester
    // (the DMA engine's payload stream included) shared one line and evicted each other.
    // A read in the no-allocate window is served from the port's buffer when it holds the
    // line; a miss fetches the line ONCE through the normal routed path (the home bank
    // refills from L2 without installing) and keeps it. A posted prefetch — a store of the
    // target address to the configured port word — starts that fetch without blocking the
    // core; the fill's cost is modeled as buffer readiness (`ready_cycle`), so a demand
    // read arriving early pays only the residue. Window writes from this tile invalidate
    // matching lines in every local buffer; cross-tile writes are invisible by design —
    // hint.h's contract demands the bytes be stable for the duration of the read.
    struct StreamBuf {
        uint64_t line = ~0ull;
        int64_t  ready_cycle = 0;
        std::vector<uint8_t> data;
    };
    int64_t fill_line(uint64_t line_addr, uint8_t *dst);
    uint32_t route_out(uint64_t addr, uint64_t *routed_addr);

    RouteGeom geom_;
    uint32_t  num_inputs_, num_outputs_, num_cache_, num_remote_port_;
    uint32_t  num_cores_;
    uint32_t  tile_id_, num_private_cache_;
    int32_t   xbar_latency_cycles_;
    bool      enable_rotation_;
    bool      forward_initiator_;

    uint64_t  na_base_ = 0, na_size_ = 0, pf_addr_ = 0;
    uint32_t  line_bytes_ = 64;
    std::vector<StreamBuf> bufs_;
    uint64_t  cnt_sb_hit_ = 0, cnt_sb_fill_ = 0, cnt_sb_wait_ = 0;
    uint64_t  cnt_pf_ = 0, cnt_pf_late_ = 0, cnt_inval_ = 0;

    std::vector<vp::IoSlave *>  inputs_;
    std::vector<vp::IoMaster *> outputs_;
    vp::IoSlave config_;
    vp::IoReq split_subreq_;
    vp::IoReq fill_req_;
    vp::Trace trace_;
};

InsituCacheXbar::InsituCacheXbar(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    num_inputs_         = cfg->get_child_int("num_inputs");
    num_outputs_        = cfg->get_child_int("num_outputs");
    num_cache_          = cfg->get_child_int("num_cache");
    num_remote_port_    = cfg->get_child_int("num_remote_port");
    tile_id_            = cfg->get_child_int("tile_id");
    num_private_cache_  = cfg->get_child_int("num_private_cache");
    xbar_latency_cycles_= cfg->get_child_int("xbar_latency_cycles");
    enable_rotation_    = cfg->get_child_bool("enable_rotation");
    forward_initiator_  = cfg->get_child_bool("forward_initiator");

    // private_start_addr can exceed INT32_MAX (e.g. 0x80000000/0xA0000000): read it through the
    // 64-bit path — get_child_int narrows to `int` and sign-extends, which silently routes every
    // private-range access to the shared banks (found by the E3.5 partition gate).
    const uint64_t priv_start = cfg->get("private_start_addr")
        ? (uint64_t)cfg->get("private_start_addr")->get_int() : 0;
    geom_.init(/*n_cache*/num_cache_, /*n_remote*/num_remote_port_,
               /*n_cores*/cfg->get_child_int("num_cores"), /*n_tiles*/cfg->get_child_int("num_tiles"),
               /*dyn_offset*/cfg->get_child_int("dynamic_offset"), /*addr_w*/cfg->get_child_int("addr_width"),
                /*priv_start*/priv_start);
    geom_.parse_regions(cfg->get_child_str("regions").c_str());

    num_cores_ = cfg->get_child_int("num_cores");
    {
        const int lb = cfg->get_child_int("line_bytes");
        line_bytes_ = (lb > 0) ? (uint32_t)lb : 64u;
        const std::string na = cfg->get_child_str("noalloc");
        unsigned long b = 0, s = 0, p = 0;
        if (!na.empty() && sscanf(na.c_str(), "%lx:%lx:%lx", &b, &s, &p) >= 2) {
            na_base_ = b;
            na_size_ = s;
            pf_addr_ = p;   // 0 when the third field is absent: no prefetch port.
        }
        if (na_size_ != 0) {
            bufs_.resize(num_cores_);
            for (auto &buf : bufs_) buf.data.assign(line_bytes_, 0);
        }
    }

    inputs_.resize(num_inputs_);
    outputs_.resize(num_outputs_);
    for (uint32_t i = 0; i < num_inputs_; ++i) {
        inputs_[i] = new vp::IoSlave();
        inputs_[i]->set_req_meth_muxed(&InsituCacheXbar::req_handler, (int)i);
        this->new_slave_port("in_" + std::to_string(i), inputs_[i]);
    }
    for (uint32_t o = 0; o < num_outputs_; ++o) {
        outputs_[o] = new vp::IoMaster();
        this->new_master_port("out_" + std::to_string(o), outputs_[o]);
    }

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO,
        "InsituCacheXbar in=%u out=%u cache=%u remote=%u tile=%u priv=%u rot=%d\n",
        num_inputs_, num_outputs_, num_cache_, num_remote_port_, tile_id_,
        num_private_cache_, (int)enable_rotation_);

    // E3: runtime partition config (the peripheral broadcasts on the partition-commit writes).
    config_.set_req_meth(&InsituCacheXbar::config_handler);
    this->new_slave_port("config", &config_);
}

vp::IoReqStatus InsituCacheXbar::config_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheXbar *_this = static_cast<InsituCacheXbar *>(__this);
    uint32_t value = 0;
    if (req->get_data() != nullptr) memcpy(&value, req->get_data(), req->get_size() < 4 ? req->get_size() : 4);
    const uint32_t csr = (uint32_t)req->get_addr();
    switch (csr) {
        case 0:  // L1D_PRIVATE: num_private_cache (per-call into route_request/bits_to_rotate).
            _this->num_private_cache_ = value;
            break;
        case 1:  // L1D_ADDR: the private_start boundary.
            _this->geom_.private_start = (uint64_t)value;
            break;
        case 2:  // XBAR_OFFSET: the BankSel field LSB (routing granularity).
            _this->geom_.dyn_offset = value;
            break;
        default:
            _this->trace_.msg(vp::Trace::LEVEL_WARNING, "xbar config: unknown csr-id %u\n", csr);
            break;
    }
    _this->trace_.msg(vp::Trace::LEVEL_INFO, "xbar config csr=%u value=0x%x\n", csr, value);
    return vp::IO_REQ_OK;
}

uint32_t InsituCacheXbar::route_out(uint64_t addr, uint64_t *routed_addr)
{
    ReqRoute r = geom_.route_request(addr, tile_id_, num_private_cache_);
    uint32_t out_id = r.sel;
    if (out_id >= num_outputs_) out_id = num_outputs_ - 1;   // safety clamp
    *routed_addr = addr;
    // MSB address rotation toward a LOCAL bank (hide the routing bits from tag/index). Off in Phase A1.
    // When enabled, the matching refill un-rotation must run on the tile's refill egress.
    if (enable_rotation_ && r.local && out_id < num_cache_)
        *routed_addr = geom_.rotate_addr(addr, geom_.bits_to_rotate(out_id, num_private_cache_));
    return out_id;
}

// One whole-line fetch through the normal routed path, synchronously; the home bank serves a
// window read from L2 without installing (insitu_cache_core.cpp). Returns the fetch's latency
// in cycles, or -1 when the path could not complete synchronously (the caller falls back to
// forwarding the original request untouched).
int64_t InsituCacheXbar::fill_line(uint64_t line_addr, uint8_t *dst)
{
    uint64_t routed = line_addr;
    const uint32_t out_id = route_out(line_addr, &routed);
    fill_req_.init();
    fill_req_.set_addr(routed);
    fill_req_.set_size(line_bytes_);
    fill_req_.set_is_write(false);
    fill_req_.set_data(dst);
    if (outputs_[out_id]->req_forward(&fill_req_) != vp::IO_REQ_OK) return -1;
    return (int64_t)fill_req_.get_full_latency();
}

vp::IoReqStatus InsituCacheXbar::req_handler(vp::Block *__this, vp::IoReq *req, int input_id)
{
    InsituCacheXbar *_this = static_cast<InsituCacheXbar *>(__this);
    const uint64_t addr = req->get_addr();

    if (_this->na_size_ != 0 && (uint32_t)input_id < _this->num_cores_) {
        // Posted prefetch: a store of the target address to the port word. Never forwarded —
        // the word exists only to carry the hint. The fill runs through the normal path now
        // (consuming real refill bandwidth at the home bank) but its cost becomes buffer
        // readiness rather than core stall: the store returns immediately.
        if (_this->pf_addr_ != 0 && addr == _this->pf_addr_ && req->get_is_write()) {
            uint32_t target = 0;
            if (req->get_data() != nullptr && req->get_size() >= 4)
                memcpy(&target, req->get_data(), 4);
            if (target != 0 && (uint64_t)target - _this->na_base_ < _this->na_size_) {
                StreamBuf &buf = _this->bufs_[input_id];
                const uint64_t line = (uint64_t)target & ~((uint64_t)_this->line_bytes_ - 1);
                if (buf.line != line) {
                    const int64_t lat = _this->fill_line(line, buf.data.data());
                    if (lat >= 0) {
                        buf.line = line;
                        buf.ready_cycle = _this->clock.get_cycles() + lat;
                        _this->cnt_pf_++;
                    }
                }
            }
            req->inc_latency(1);
            return vp::IO_REQ_OK;
        }

        if (addr - _this->na_base_ < _this->na_size_) {
            if (req->get_is_write()) {
                // Same-tile window write: drop every local buffer holding the line, then fall
                // through to the ordinary path (the bank writes through without allocating).
                const uint64_t line = addr & ~((uint64_t)_this->line_bytes_ - 1);
                for (auto &buf : _this->bufs_)
                    if (buf.line == line) { buf.line = ~0ull; _this->cnt_inval_++; }
            } else {
                const uint64_t line = addr & ~((uint64_t)_this->line_bytes_ - 1);
                const uint32_t off = (uint32_t)(addr & (_this->line_bytes_ - 1));
                uint32_t n = (uint32_t)req->get_size();
                if (off + n <= _this->line_bytes_) {
                    StreamBuf &buf = _this->bufs_[input_id];
                    const int64_t now = _this->clock.get_cycles();
                    if (buf.line == line) {
                        const int64_t wait = (buf.ready_cycle > now) ? buf.ready_cycle - now : 0;
                        if (wait > 0) { _this->cnt_sb_wait_ += (uint64_t)wait; _this->cnt_pf_late_++; }
                        if (req->get_data() != nullptr)
                            memcpy(req->get_data(), buf.data.data() + off, n);
                        req->inc_latency(_this->xbar_latency_cycles_ + 1 + wait);
                        _this->cnt_sb_hit_++;
                        return vp::IO_REQ_OK;
                    }
                    const int64_t lat = _this->fill_line(line, buf.data.data());
                    if (lat >= 0) {
                        buf.line = line;
                        buf.ready_cycle = now + lat;
                        if (req->get_data() != nullptr)
                            memcpy(req->get_data(), buf.data.data() + off, n);
                        req->inc_latency(_this->xbar_latency_cycles_ + 1 + lat);
                        _this->cnt_sb_fill_++;
                        return vp::IO_REQ_OK;
                    }
                    // Fill failed (asynchronous path): fall through to plain forwarding.
                }
            }
        }
    }

    uint64_t routed = addr;
    const uint32_t out_id = _this->route_out(addr, &routed);

    if (_this->forward_initiator_) req->set_initiator(input_id);
    if (_this->xbar_latency_cycles_ > 0) req->inc_latency(_this->xbar_latency_cycles_);
    req->set_addr(routed);

    _this->trace_.msg(vp::Trace::LEVEL_TRACE, "route in=%d addr=0x%lx -> out=%u\n",
                      input_id, (unsigned long)addr, out_id);
    return _this->outputs_[out_id]->req_forward(req);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheXbar(config);
}
