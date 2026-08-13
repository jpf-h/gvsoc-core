#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache — structural per-port-class crossbar (one ``tcdm_cache_interco`` lane).

RTL-faithful replacement for the hashed ``InsituCacheInterco``: routes a single TCDM port-class j across
the cache banks BY ADDRESS using ``insitu_cache_route.hpp`` (BankSel/TileID extraction, partition modes,
MSB rotation, remote-tile slots). The structural tile instantiates ``NrTCDMPortsPerCore`` (=5) of these,
one per lane. See ``cachepool_tile.sv:604-652`` and ``prompt/insitu_cache_structural_tile_plan_2026-06-18.md``.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf


class InsituCacheXbar(Component):
    """One per-port-class crossbar: num_inputs (cores + remote-in) × num_outputs (banks + remote-out)."""

    def __init__(self, parent: Component, name: str, *,
                 num_cores: int, num_cache: int, num_remote_port: int = 0,
                 num_tiles: int = 1, tile_id: int = 0, num_private_cache: int | None = None,
                 dynamic_offset: int = 6, addr_width: int = 32, private_start_addr: int = 0,
                 xbar_latency_cycles: int = 0, enable_rotation: bool = False,
                 forward_initiator: bool = False, regions: str = ''):
        super().__init__(parent, name)

        self.add_sources(['cache/insitu/insitu_cache_xbar.cpp'])

        # All-private when single-tile (route.hpp forces local routing) unless overridden.
        if num_private_cache is None:
            num_private_cache = num_cache if num_tiles == 1 else 0

        self._num_inputs = num_cores + num_remote_port
        self._num_outputs = num_cache + num_remote_port

        self.add_properties({
            'num_inputs': self._num_inputs,
            'num_outputs': self._num_outputs,
            'num_cores': num_cores,
            'num_cache': num_cache,
            'num_remote_port': num_remote_port,
            'num_tiles': num_tiles,
            'tile_id': tile_id,
            'num_private_cache': num_private_cache,
            'dynamic_offset': dynamic_offset,
            'addr_width': addr_width,
            'private_start_addr': private_start_addr,
            'xbar_latency_cycles': xbar_latency_cycles,
            'enable_rotation': enable_rotation,
            'forward_initiator': forward_initiator,
            # Placed regions "base:size:tile_shift:bank_shift[,...]" — address windows whose
            # TileID/BankSel routing fields sit at configured bit positions (route.hpp). Empty =
            # flat routing everywhere (today's behavior).
            'regions': regions,
        })

    def i_INPUT(self, port: int) -> SlaveItf:
        """Input port ``port``: 0..num_cores-1 = local core lane ports; then remote-in slots."""
        return SlaveItf(self, f'in_{port}', signature='io')

    def o_OUTPUT(self, port: int, itf: SlaveItf):
        """Bind output ``port``: 0..num_cache-1 = local banks; then remote-out slots."""
        self.itf_bind(f'out_{port}', itf, signature='io')
