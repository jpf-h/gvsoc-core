#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache — RTL-faithful structural cache core (Step 4, first runnable).

A per-cycle ClockEvent FSM transcription of insitu_cache_core.sv, consuming the Step-1 decode
(insitu_cache_decode.hpp) and Step-2 bank model (insitu_cache_bank_array.hpp). Same external port
interface as :class:`InsituCacheController` (i_INPUT / o_REFILL / o_EVICT / i_FLUSH) so the tile can
swap it in behind ``InsituCacheTileConfig.use_structural_core`` (open-loop / async only for now —
see insitu_cache_core.cpp scope note). See ``prompt/insitu_cache_structural_plan_2026-06-16.md``.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf

from cache.insitu.insitu_cache_config import InsituCacheControllerConfig


class InsituCacheCore(Component):
    """Structural per-cycle cache core. Drop-in for InsituCacheController (calib/structural mode)."""

    def __init__(self, parent: Component, name: str,
                 config: InsituCacheControllerConfig | None = None,
                 num_input_ports: int = 1,
                 rotate_bits: int = 0, rotate_dyn_offset: int = 0, rotate_addr_width: int = 32,
                 bank_index: int = 0, num_cache: int = 1, num_tiles: int = 1, num_private_cache: int = 0,
                 noalloc: str = ''):
        if config is None:
            config = InsituCacheControllerConfig()

        super().__init__(parent, name, config=config)

        self.add_sources(['cache/insitu/insitu_cache_core.cpp'])

        self._num_input_ports = num_input_ports
        self.add_properties({
            'cache_line_bytes': config.cache_line_bytes,
            'num_ways': config.num_ways,
            'num_sets': config.num_sets,
            'use_hash_way_select': config.use_hash_way_select,
            'functional_writethrough': config.functional_writethrough,
            'miss_penalty_cycles': config.miss_penalty_cycles,
            'refill_bank_write_cycles': config.refill_bank_write_cycles,
            'retr_fifo_depth': config.retr_fifo_depth,
            'miss_fifo_depth': config.miss_fifo_depth,
            'evic_fifo_depth': config.evic_fifo_depth,
            'bank_factor': config.bank_factor,
            # Multi-lane core port (RTL controller has a 5-wide core port). Default 1 = single 'input'
            # port (backward identical); the structural tile sets this to NrTCDMPortsPerCore.
            'num_input_ports': num_input_ports,
            # Synchronous-slave (closed-loop) mode + its analytic latency knobs. Default off → async path.
            'inline_sync_miss': config.inline_sync_miss,
            'hit_latency_cycles': config.hit_latency_cycles,
            'write_commit_cycles': config.write_commit_cycles,
            # Structural sync-slave overrides (separate from the async controller's decomposition).
            'structural_hit_latency_cycles': config.structural_hit_latency_cycles,
            'structural_miss_penalty_cycles': config.structural_miss_penalty_cycles,
            'structural_install_tail_cycles': config.structural_install_tail_cycles,
            'structural_write_hit_latency_cycles': getattr(config, 'structural_write_hit_latency_cycles', -1),
            # F1 flush walk (insitu_cache_tcdm_wrapper 7-state FSM): base = drain(21)+sets(256);
            # per dirty line a serialized downstream eviction.
            'flush_base_cycles': getattr(config, 'flush_base_cycles', 277),
            'flush_evict_cycles': getattr(config, 'flush_evict_cycles', 20),
            # E1 MSB-rotation inverse: how many routing bits the tile xbar rotated into the MSB for
            # THIS bank (route.hpp::bits_to_rotate), + the rotation geometry. 0 → l2_addr() identity.
            'rotate_bits': rotate_bits,
            'rotate_dyn_offset': rotate_dyn_offset,
            'rotate_addr_width': rotate_addr_width,
            # E3: this bank's identity (used by the config slave to repartition + recompute
            # rotate_bits from the shared table, and by run_flush to be partition-class-selective).
            'bank_index': bank_index,
            'num_cache': num_cache,
            'num_tiles': num_tiles,
            'num_private_cache': num_private_cache,
            # No-allocate window "base:size" over GLOBAL addresses (rlc_am doc/RLC_HW.md §2):
            # reads inside it are served through a single-line stream buffer without allocating
            # a way, writes go write-through only. Empty = disabled.
            'noalloc': noalloc,
        })

    def i_INPUT(self, port: int = 0) -> SlaveItf:
        """TCDM request input. port 0 = 'input' (default); lanes 1.. = 'input_{port}' (multi-lane core)."""
        return SlaveItf(self, 'input' if port == 0 else f'input_{port}', signature='io')

    def i_FLUSH(self) -> SlaveItf:
        """Flush/invalidate trigger (stub for now; full cache_sync FSM is Step 6)."""
        return SlaveItf(self, 'flush', signature='io')

    def o_REFILL(self, itf: SlaveItf):
        """Bind the line-fill request port to the next memory level."""
        self.itf_bind('refill', itf, signature='io')

    def o_EVICT(self, itf: SlaveItf):
        """Bind the dirty-writeback / functional-write port to the next memory level."""
        self.itf_bind('evict', itf, signature='io')
