#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache — inter-tile remote crossbar (cachepool_remote_xbar), one per port-class.

Routes a cross-tile request from a source tile's remote-out to the TARGET tile's remote-in by the
address's TileID field (extends the shared L1 across tiles). num_tiles inputs × num_tiles outputs.
See ``cachepool_group.sv:399-432``.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf


class InsituCacheRemoteXbar(Component):
    """One per-port-class inter-tile router: route by target tile-id."""

    def __init__(self, parent: Component, name: str, *,
                 num_tiles: int, num_cores: int, num_cache: int, num_remote_port_core: int = 1,
                 dynamic_offset: int = 6, addr_width: int = 32, hop_latency_cycles: int = 0,
                 regions: str = ''):
        super().__init__(parent, name)
        self.add_sources(['cache/insitu/insitu_cache_remote_xbar.cpp'])
        self.add_properties({
            'num_tiles': num_tiles,
            'num_cores': num_cores,
            'num_cache': num_cache,
            'num_remote_port_core': num_remote_port_core,
            'dynamic_offset': dynamic_offset,
            'addr_width': addr_width,
            'hop_latency_cycles': hop_latency_cycles,
            # Placed regions, same format/semantics as InsituCacheXbar (route.hpp). The remote
            # xbar must hold the same value: it routes by the same effective TileID.
            'regions': regions,
        })

    def i_INPUT(self, slot: int) -> SlaveItf:
        """Input slot = src_tile*num_remote_port_core + r (a source tile's remote-out)."""
        return SlaveItf(self, f'in_{slot}', signature='io')

    def o_OUTPUT(self, slot: int, itf: SlaveItf):
        """Output slot = tgt_tile*num_remote_port_core + r (bind to a target tile's remote-in)."""
        self.itf_bind(f'out_{slot}', itf, signature='io')
