// SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
// SPDX-License-Identifier: Apache-2.0
//
// InSitu cache — inter-tile remote crossbar (cachepool_remote_xbar), one per port-class.
//
// The group instances NrTCDMPortsPerCore (=5) of these (cachepool_group.sv:399-432), one per port-class.
// Each routes a cross-tile request emitted on a SOURCE tile's remote-out slot to the TARGET tile's
// remote-in slot, by the TileID field of the address — extending the shared L1 across tiles (any core in
// tile A can reach a bank homed in tile B). The GVSoC response auto-routes back along the preserved
// resp-port chain, so the RTL's source-tile-mod-N slot pinning is a timing/contention detail, not needed
// for functional correctness here; this models WHICH target tile a request lands on.
//
// num_tiles inputs (one source-tile remote-out per port-class) × num_tiles outputs (one target-tile
// remote-in). Route: out = addr_tile = addr[dynamic_offset + log2(NumCache) +: TileIDWidth] (route.hpp).

#include <cstdint>
#include <string>
#include <vector>

#include <vp/vp.hpp>
#include <vp/itf/io.hpp>

#include "insitu_cache_route.hpp"

using namespace insitu;

class InsituCacheRemoteXbar : public vp::Component
{
public:
    explicit InsituCacheRemoteXbar(vp::ComponentConf &conf);

private:
    static vp::IoReqStatus req_handler(vp::Block *__this, vp::IoReq *req, int input_id);
    // E3: dyn_offset only (csr-id 2).
    static vp::IoReqStatus config_handler(vp::Block *__this, vp::IoReq *req);

    RouteGeom geom_;
    uint32_t  num_tiles_, nrpc_, n_slots_;
    int32_t   hop_latency_cycles_;
    std::vector<vp::IoSlave *>  inputs_;
    std::vector<vp::IoMaster *> outputs_;
    vp::IoSlave config_;    // E3: dyn_offset only (csr-id 2)
    vp::Trace trace_;
};

InsituCacheRemoteXbar::InsituCacheRemoteXbar(vp::ComponentConf &conf) : vp::Component(conf)
{
    auto *cfg = this->get_js_config();
    num_tiles_          = cfg->get_child_int("num_tiles");
    nrpc_               = cfg->get_child_int("num_remote_port_core");
    if (nrpc_ < 1) nrpc_ = 1;
    n_slots_            = num_tiles_ * nrpc_;    // NumInp = NumOut = NumTiles * NumRemotePortCore
    hop_latency_cycles_ = cfg->get_child_int("hop_latency_cycles");

    geom_.init(/*n_cache*/cfg->get_child_int("num_cache"), /*n_remote*/nrpc_,
               /*n_cores*/cfg->get_child_int("num_cores"), /*n_tiles*/num_tiles_,
               /*dyn_offset*/cfg->get_child_int("dynamic_offset"), /*addr_w*/cfg->get_child_int("addr_width"),
               /*priv_start*/0);
    geom_.parse_regions(cfg->get_child_str("regions").c_str());

    inputs_.resize(n_slots_);
    outputs_.resize(n_slots_);
    for (uint32_t i = 0; i < n_slots_; i++) {
        inputs_[i] = new vp::IoSlave();
        inputs_[i]->set_req_meth_muxed(&InsituCacheRemoteXbar::req_handler, (int)i);
        this->new_slave_port("in_" + std::to_string(i), inputs_[i]);
    }
    for (uint32_t o = 0; o < n_slots_; o++) {
        outputs_[o] = new vp::IoMaster();
        this->new_master_port("out_" + std::to_string(o), outputs_[o]);
    }

    this->traces.new_trace("trace", &this->trace_, vp::DEBUG);
    this->trace_.msg(vp::Trace::LEVEL_INFO, "InsituCacheRemoteXbar tiles=%u nrpc=%u\n", num_tiles_, nrpc_);

    // E3: dyn_offset only (the remote router never re-partitions banks, it re-extracts addr_tile).
    config_.set_req_meth(&InsituCacheRemoteXbar::config_handler);
    this->new_slave_port("config", &config_);
}

vp::IoReqStatus InsituCacheRemoteXbar::config_handler(vp::Block *__this, vp::IoReq *req)
{
    InsituCacheRemoteXbar *_this = static_cast<InsituCacheRemoteXbar *>(__this);
    if ((uint32_t)req->get_addr() == 2) {   // XBAR_OFFSET only
        uint32_t value = 0;
        if (req->get_data() != nullptr) memcpy(&value, req->get_data(), req->get_size() < 4 ? req->get_size() : 4);
        _this->geom_.dyn_offset = value;
    }
    return vp::IO_REQ_OK;
}

vp::IoReqStatus InsituCacheRemoteXbar::req_handler(vp::Block *__this, vp::IoReq *req, int input_id)
{
    InsituCacheRemoteXbar *_this = static_cast<InsituCacheRemoteXbar *>(__this);
    // Input slot = source_tile*nrpc + r. Route to the TARGET tile (by the address TileID); within the
    // target, pin the slot by source-tile-mod-N (RTL group.sv:285-295). The GVSoC response auto-routes
    // back along the preserved resp-port chain, so the exact slot is a timing/contention choice only.
    const uint32_t source = (uint32_t)input_id / _this->nrpc_;
    uint32_t bank_unused = 0;
    uint32_t target = 0;
    _this->geom_.routing_fields(req->get_addr(), bank_unused, target);
    if (target >= _this->num_tiles_) target = _this->num_tiles_ - 1;   // safety clamp
    const uint32_t out = target * _this->nrpc_ + (source % _this->nrpc_);
    if (_this->hop_latency_cycles_ > 0) req->inc_latency(_this->hop_latency_cycles_);
    _this->trace_.msg(vp::Trace::LEVEL_TRACE, "remote src=%u addr=0x%lx -> tile=%u slot=%u\n",
                      source, (unsigned long)req->get_addr(), target, out);
    return _this->outputs_[out]->req_forward(req);
}

extern "C" vp::Component *gv_new(vp::ComponentConf &config)
{
    return new InsituCacheRemoteXbar(config);
}
