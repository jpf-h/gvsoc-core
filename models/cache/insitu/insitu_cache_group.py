#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu cache — multi-tile GROUP composite (cachepool_group.sv).

Assembles ``num_tiles`` structural ``InsituCacheTile`` instances + ``NrTCDMPortsPerCore`` per-port-class
inter-tile remote crossbars, giving a SHARED L1 across tiles: a core in tile A reaches a bank homed in
tile B by address (the tile's xbar emits the cross-tile request on its remote-out, the remote xbar routes
it to tile B by the address's TileID, and the GVSoC response auto-routes back). Topology
(cachepool_pkg.sv:29-67): Group(num_tiles) → Tile(cores × 5 TCDM ports, per-core cells).

    i_INPUT(global) ─► tile[t].in_(local)            (global = t*tile_ports + local)
    tile[t].remote_out[j] ─► rxbar[j].in[t] ─► (by TileID) ─► rxbar[j].out[T] ─► tile[T].remote_in[j]
    every tile.o_L2 ─► group o_L2 (fan-in)
"""

from __future__ import annotations

import copy

from gvsoc.systree import Component, SlaveItf

from cache.insitu.insitu_cache_config import InsituCacheTileConfig
from cache.insitu.insitu_cache_tile import InsituCacheTile
from cache.insitu.insitu_cache_remote_xbar import InsituCacheRemoteXbar


class InsituCacheGroup(Component):
    """``num_tiles`` structural tiles + per-port-class remote xbars (cross-tile shared L1)."""

    def __init__(self, parent: Component, name: str, config: InsituCacheTileConfig):
        super().__init__(parent, name)
        assert config.structural_tile, "InsituCacheGroup requires structural_tile=True per-tile config"
        assert config.num_tiles >= 1
        assert config.num_remote_port_core >= 1, "group needs num_remote_port_core>=1 for remote ports"

        self._config = config
        n_tiles = config.num_tiles
        n_ppc = config.tcdm_ports_per_core
        cores_per_tile = config.num_cores
        ctrl_per_tile = config.num_controllers
        self._tile_ports = cores_per_tile * n_ppc

        # --- per-tile structural tiles (each stamped with its tile_id) ---
        self._tiles = []
        for t in range(n_tiles):
            tcfg = copy.deepcopy(config)
            tcfg.tile_id = t
            self._tiles.append(InsituCacheTile(self, f'tile_{t}', config=tcfg))

        # --- per-port-class inter-tile remote crossbars (NumInp=NumOut = num_tiles * num_remote_port_core) ---
        n_remote = config.num_remote_port_core
        self._rxbars = []
        for j in range(n_ppc):
            self._rxbars.append(InsituCacheRemoteXbar(
                self, f'rxbar_{j}', num_tiles=n_tiles, num_cores=cores_per_tile,
                num_cache=ctrl_per_tile, num_remote_port_core=n_remote,
                dynamic_offset=config.interco.dynamic_offset, addr_width=config.addr_width,
                hop_latency_cycles=getattr(config, 'hop_latency_cycles', 0),
                regions=getattr(config, 'regions', '')))

        # --- remote wiring (slot = tile*num_remote_port_core + r) ---
        # tile[t].remote_out[j][r] → rxbar[j].in[t*nr + r]; rxbar[j].out[T*nr + r] → tile[T].remote_in[j][r].
        for j in range(n_ppc):
            for t in range(n_tiles):
                for r in range(n_remote):
                    self._tiles[t].o_REMOTE_OUT(j, r, self._rxbars[j].i_INPUT(t * n_remote + r))
            for tgt in range(n_tiles):
                for r in range(n_remote):
                    self._rxbars[j].o_OUTPUT(tgt * n_remote + r, self._tiles[tgt].i_REMOTE_IN(j, r))

        # --- group inputs → tile inputs ---
        for p in range(n_tiles * self._tile_ports):
            t = p // self._tile_ports
            lp = p % self._tile_ports
            self.bind(self, f'in_{p}', self._tiles[t], f'in_{lp}')

        # --- L2 fan-in: every tile's refill/evict → the group's single o_L2 ---
        for t in range(n_tiles):
            self.bind(self._tiles[t], 'l2', self, 'l2')

        # --- flush fan-out: one composite slave per (tile, controller) ---
        if ctrl_per_tile > 1 or n_tiles > 1:
            for t in range(n_tiles):
                for cb in range(ctrl_per_tile):
                    self.bind(self, f'flush_{t}_{cb}', self._tiles[t], f'flush_{cb}')
                    # E3: runtime partition config pass-throughs (peripheral → each cell/xbar/rxbar).
                    self.bind(self, f'config_core_{t}_{cb}', self._tiles[t], f'config_core_{cb}')
            for t in range(n_tiles):
                for j in range(n_ppc):
                    self.bind(self, f'config_xbar_{t}_{j}', self._tiles[t], f'config_xbar_{j}')
            for j in range(n_ppc):
                self.bind(self, f'config_rxbar_{j}', self._rxbars[j], 'config')

    # ---------- port factories ----------

    def num_input_ports(self) -> int:
        return self._config.num_tiles * self._tile_ports

    def i_INPUT(self, port: int) -> SlaveItf:
        """Global TCDM port: 0 ≤ port < num_tiles * num_cores * tcdm_ports_per_core."""
        return SlaveItf(self, f'in_{port}', signature='io')

    def i_FLUSH(self, tile: int, ctrl: int) -> SlaveItf:
        return SlaveItf(self, f'flush_{tile}_{ctrl}', signature='io')

    def i_CONFIG_CORE(self, tile: int, ctrl: int) -> SlaveItf:
        return SlaveItf(self, f'config_core_{tile}_{ctrl}', signature='io')

    def i_CONFIG_XBAR(self, tile: int, port_class: int) -> SlaveItf:
        return SlaveItf(self, f'config_xbar_{tile}_{port_class}', signature='io')

    def i_CONFIG_RXBAR(self, port_class: int) -> SlaveItf:
        return SlaveItf(self, f'config_rxbar_{port_class}', signature='io')

    def o_L2(self, itf: SlaveItf):
        """Fan-in of every tile's refill + eviction to a single external L2 slave."""
        self.itf_bind('l2', itf, signature='io')
