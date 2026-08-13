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

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req, int input_id);
    // E3 runtime partition config (csr-id in addr: 0=num_private, 1=private_start, 2=dyn_offset).
    static vp::IoReqStatus config_handler(vp::Block *__this, vp::IoReq *req);

    RouteGeom geom_;
    uint32_t  num_inputs_, num_outputs_, num_cache_, num_remote_port_;
    uint32_t  tile_id_, num_private_cache_;
    int32_t   xbar_latency_cycles_;
    bool      enable_rotation_;
    bool      forward_initiator_;

    std::vector<vp::IoSlave *>  inputs_;
    std::vector<vp::IoMaster *> outputs_;
    vp::IoSlave config_;
    vp::IoReq split_subreq_;
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

vp::IoReqStatus InsituCacheXbar::req_handler(vp::Block *__this, vp::IoReq *req, int input_id)
{
    InsituCacheXbar *_this = static_cast<InsituCacheXbar *>(__this);
    const uint64_t addr = req->get_addr();

    ReqRoute r = _this->geom_.route_request(addr, _this->tile_id_, _this->num_private_cache_);
    uint32_t out_id = r.sel;
    if (out_id >= _this->num_outputs_) out_id = _this->num_outputs_ - 1;   // safety clamp

    if (_this->forward_initiator_) req->set_initiator(input_id);
    if (_this->xbar_latency_cycles_ > 0) req->inc_latency(_this->xbar_latency_cycles_);

    // MSB address rotation toward a LOCAL bank (hide the routing bits from tag/index). Off in Phase A1.
    // When enabled, the matching refill un-rotation must run on the tile's refill egress.
    if (_this->enable_rotation_ && r.local && out_id < _this->num_cache_) {
        const uint32_t n = _this->geom_.bits_to_rotate(out_id, _this->num_private_cache_);
        req->set_addr(_this->geom_.rotate_addr(addr, n));
    }

    _this->trace_.msg(vp::Trace::LEVEL_TRACE, "route in=%d addr=0x%lx -> out=%u local=%d\n",
                      input_id, (unsigned long)addr, out_id, (int)r.local);
    return _this->outputs_[out_id]->req_forward(req);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheXbar(config);
}
