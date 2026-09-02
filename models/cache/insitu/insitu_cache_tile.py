#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""InSitu Cache tile — composite component.

Assembles one tile's worth of cache: a hashed interco on the TCDM side, N cache
controllers (one per core in the canonical config), N write-through coalescers (one per
controller), and a simple fan-in router on the L2 side.

Topology (default ``cachepool_512``):

    i_INPUT(0..19) ─► interco ─┬─► ctrl[0] ─┬─► REFILL ───────► l2_router ─► o_L2
                               ├─► ctrl[1] ─┼─► EVICT  ───────►
                               ├─► ctrl[2] ─┴─► WRITE_THROUGH ─► coal[0..3] ─►
                               └─► ctrl[3]

The tile's ``o_L2`` is a direct pass-through of the inner ``l2_router``'s default output,
so the external caller can bind to any downstream memory (SPM, DRAM, …) without the tile
imposing extra plumbing.
"""

from __future__ import annotations

from gvsoc.systree import Component, SlaveItf

from cache.insitu.insitu_cache_config import (
    InsituCacheTileConfig,
    InsituCacheParCoalescerConfig,
    make_cachepool_512_config,
)
from cache.insitu.insitu_cache_controller import InsituCacheController
from cache.insitu.insitu_cache_core import InsituCacheCore
from cache.insitu.insitu_cache_coalescer import InsituCacheCoalescer
from cache.insitu.insitu_cache_interco import InsituCacheInterco
from cache.insitu.insitu_cache_par_coalescer import InsituCacheParCoalescer
from cache.insitu.insitu_cache_xbar import InsituCacheXbar
from cache.insitu.insitu_cache_cell_coalescer import InsituCacheCellCoalescer
from cache.insitu.insitu_cache_amo_shim import InsituCacheAmo


class InsituCacheTile(Component):
    """One tile of the InSitu cache.

    Exposes ``i_INPUT(port)`` for each TCDM request port (total = num_cores *
    tcdm_ports_per_core) and a single ``o_L2`` master that fans in miss / evict /
    write-through traffic from all inner controllers.

    Parameters
    ----------
    parent, name : standard GVSoC systree args
    config : InsituCacheTileConfig | None
        Full tile config. If ``None``, :func:`make_cachepool_512_config` is used.
    """

    def __init__(self, parent: Component, name: str,
                 config: InsituCacheTileConfig | None = None):
        super().__init__(parent, name)

        if config is None:
            config = make_cachepool_512_config()
        self._config = config

        # RTL-faithful structural tile: 5 per-port-class crossbars + per-core multi-lane cells.
        if getattr(config, 'structural_tile', False):
            self._build_structural_tile(config)
            return

        n_ctrl = config.num_controllers
        n_ports = config.num_tcdm_ports

        # Keep interco fields consistent with tile topology.
        config.interco.num_inputs = n_ports
        config.interco.num_outputs = n_ctrl

        # Phase-1 structural mode: insert one par_coalescer between each interco output and its
        # controller. The interco becomes a pure router (defer_to_coalescer), and the coalescer
        # does the read-merge + output arbitration + interco_latency — extracting what
        # enable_input_coalesce did inline. Default off ⇒ monolithic interco (byte-identical).
        # Structural-rewrite switch: instantiate the RTL-faithful per-cycle InsituCachecore instead
        # of the cycle-approximate InsituCacheController (open-loop/async only for now).
        self._use_struct_core = config.use_structural_core
        self._use_struct_coal = config.use_structural_coalescer
        if self._use_struct_coal:
            config.interco.defer_to_coalescer = True
            # Mirror the interco's merge/arb knobs onto the coalescer so the structural path
            # reproduces the calibrated numbers; carry the merge intent from enable_input_coalesce.
            self._parcoal_cfg = InsituCacheParCoalescerConfig(
                interco_latency_cycles=config.interco.interco_latency_cycles,
                merge_same_line_reads=config.interco.enable_input_coalesce,
                cache_line_bytes=config.interco.cache_line_bytes,
                coalesce_max_latency=config.interco.coalesce_max_latency,
                output_accept_width=config.interco.output_accept_width,
                per_cycle_output_arb=config.interco.per_cycle_output_arb,
            )
            # The interco no longer merges inline (the coalescer does); keep the flag coherent.
            config.interco.enable_input_coalesce = False

        # -------- sub-components --------

        self._interco = InsituCacheInterco(self, 'interco', config=config.interco)

        self._ctrls = []
        self._coals = []
        self._parcoals = []
        for i in range(n_ctrl):
            if self._use_struct_core:
                ctrl = InsituCacheCore(self, f'ctrl_{i}', config=config.controller)
            else:
                ctrl = InsituCacheController(self, f'ctrl_{i}', config=config.controller)
            coal = InsituCacheCoalescer(self, f'coal_{i}', config=config.coalescer)
            self._ctrls.append(ctrl)
            self._coals.append(coal)
            if self._use_struct_coal:
                self._parcoals.append(
                    InsituCacheParCoalescer(self, f'parcoal_{i}', config=self._parcoal_cfg))

        # -------- wiring --------
        # TCDM inputs → interco inputs (pass-through this composite's slave ports)
        for p in range(n_ports):
            self.bind(self, f'in_{p}', self._interco, f'in_{p}')

        # Interco outputs → controllers; controllers' write-through → coalescers.
        # All cross-tile L2 traffic (refill + evict + write-through-flush) fans into a
        # single composite master port `l2` which the external caller binds via o_L2().
        # This follows the hierarchical_cache.py pattern (multiple inner masters → one
        # composite master → one external slave).
        for i in range(n_ctrl):
            # Interco output i → (par_coalescer i →) controller i.
            if self._use_struct_coal:
                self._interco.o_OUTPUT(i, self._parcoals[i].i_INPUT())
                self._parcoals[i].o_OUTPUT(self._ctrls[i].i_INPUT())
            else:
                self._interco.o_OUTPUT(i, self._ctrls[i].i_INPUT())
            # Write-through path exists only on the cycle-approximate controller; the structural
            # core folds writes into its evict/functional path (no separate WT port).
            if not self._use_struct_core:
                self._ctrls[i].o_WRITE_THROUGH(self._coals[i].i_INPUT())
                self.bind(self._coals[i], 'out', self, 'l2')

            # Fan-in to the tile's composite `l2` master. The binding framework
            # multiplexes the inner masters onto the single external destination.
            self.bind(self._ctrls[i], 'refill', self, 'l2')
            self.bind(self._ctrls[i], 'evict',  self, 'l2')

            # Flush: expose one composite slave port per controller so the cluster L1D-flush
            # peripheral can invalidate every controller (cache_sync flush-all). Only in the
            # multi-controller (cluster) config — the single-controller open-loop calib DUT has
            # no flush driver, so binding an externally-undriven composite port there is skipped.
            if n_ctrl > 1:
                self.bind(self, f'flush_{i}', self._ctrls[i], 'flush')

    # ---------- structural tile ----------

    def _build_structural_tile(self, config: InsituCacheTileConfig):
        """RTL-faithful tile (cachepool_tile.sv): NrTCDMPortsPerCore per-port-class crossbars routing
        any core's lane-j to any bank by address (shared L1), feeding per-core multi-lane cache cells.

        Phase A1: 5 xbars + N multi-lane InsituCacheCore cells (no rotation, no coalescer/AMO yet — those
        are A2/A3). Validates the per-port-class shared-L1 routing + end-to-end data correctness.
        """
        n_cores  = config.num_cores
        n_ppc    = config.tcdm_ports_per_core
        n_ctrl   = config.num_controllers
        n_remote = config.num_remote_port_core
        dyn_off  = config.interco.dynamic_offset

        # E1 MSB rotation. Per-bank rotated-bit count N (route.hpp::bits_to_rotate): the xbar moves
        # the N routing bits above dyn_offset to the MSB; each bank unrotates its L2-side egress by
        # the same N. Rotation is only well-defined when dyn_offset == log2(line) (it must not move
        # line-offset bits) — guard + fall back to off otherwise.
        en_rot = bool(getattr(config, 'enable_rotation', True))
        line_log2 = (config.controller.cache_line_bytes - 1).bit_length()
        if en_rot and dyn_off != line_log2:
            print(f'[insitu_cache_tile] WARNING: enable_rotation needs dyn_offset == log2(line) '
                  f'({dyn_off} != {line_log2}) — disabling rotation')
            en_rot = False
        # E3.5: the elaboration-time partition (default = xbar.py's all-private/all-shared rule).
        # config.num_private_cache (if set) overrides it — used by the calib TB, which has no
        # peripheral and therefore freezes the partition at build time.
        _n_priv_ovr = getattr(config, 'num_private_cache', None)
        n_priv   = (_n_priv_ovr if _n_priv_ovr is not None
                    else (n_ctrl if config.num_tiles == 1 else 0))
        _priv_start = getattr(config, 'private_start_addr', 0xA0000000)
        bank_bits = (n_ctrl - 1).bit_length()
        tile_bits = (config.num_tiles - 1).bit_length()
        def _rot_bits(bank_port):
            if not en_rot: return 0
            if n_priv == 0:          return bank_bits + tile_bits   # all-shared
            if n_priv == n_ctrl:     return bank_bits               # all-private
            return bank_bits if bank_port < n_priv else bank_bits + tile_bits

        # NrTCDMPortsPerCore per-port-class crossbars (one per lane j). Each: n_cores local lane-j ports
        # (+ remote-in) × n_ctrl local banks (+ remote-out), routed by address via insitu_cache_route.hpp.
        self._xbars = []
        for j in range(n_ppc):
            self._xbars.append(InsituCacheXbar(
                self, f'xbar_{j}',
                num_cores=n_cores, num_cache=n_ctrl, num_remote_port=n_remote,
                num_tiles=config.num_tiles, tile_id=config.tile_id,
                num_private_cache=n_priv,
                dynamic_offset=dyn_off, addr_width=config.addr_width,
                xbar_latency_cycles=getattr(config, 'xbar_latency_cycles', 0),
                enable_rotation=en_rot,
                private_start_addr=_priv_start,
                regions=getattr(config, 'regions', ''),
                hint_base=getattr(config, 'hint_base', 0),
                line_bytes=getattr(config.controller, 'cache_line_bytes', 64)))

        use_coal = getattr(config, 'cell_coalescer', False)
        n_vlsu = n_ppc - 1   # lanes 0..n_ppc-2 = Spatz VLSU; the last lane (n_ppc-1) = Snitch/FPU scalar

        # Per-core cache cells. A2 (cell_coalescer): the cell = par_coalescer(4 VLSU lanes) + the scalar
        # bypass → core's 2 inputs (matches cachepool_cache_ctrl). A1: the core takes all n_ppc lanes
        # directly (multi-lane port, no merge).
        self._ctrls = []
        self._cellcoals = []
        for cb in range(n_ctrl):
            self._ctrls.append(InsituCacheCore(
                self, f'ctrl_{cb}', config=config.controller,
                num_input_ports=(2 if use_coal else n_ppc),
                rotate_bits=_rot_bits(cb), rotate_dyn_offset=dyn_off,
                rotate_addr_width=config.addr_width,
                # E3: this bank's identity for runtime repartition (default matches the xbar).
                bank_index=cb, num_cache=n_ctrl, num_tiles=config.num_tiles,
                num_private_cache=n_priv))
            if use_coal:
                # C1: coalesce on the 16 B part (production folds PartSplit=4 → CoalescerDataWidth =
                # CacheLineWidth/4 = 128b, cachepool_cache_ctrl.sv:80-81), not the 64 B line.
                self._cellcoals.append(InsituCacheCellCoalescer(
                    self, f'coal_{cb}', num_inputs=n_vlsu,
                    cache_line_bytes=config.controller.cache_line_bytes, word_bytes=4,
                    part_bytes=config.controller.cache_line_bytes // 4))

        # Wiring. tile i_INPUT(p) → xbar[p % n_ppc].in_(p // n_ppc)
        # (RTL cache_req[j][cb] = unmerge_req[cb*NrTCDMPortsPerCore + j], tile.sv:541-551).
        for p in range(n_cores * n_ppc):
            self.bind(self, f'in_{p}', self._xbars[p % n_ppc], f'in_{p // n_ppc}')

        # Optional AMO/LR-SC shim on the scalar lane (j=n_ppc-1), one per cell (cachepool_tile.sv:658).
        use_amo = getattr(config, 'amo_lane', False)
        self._amos = []
        if use_amo:
            for cb in range(n_ctrl):
                self._amos.append(InsituCacheAmo(self, f'amo_{cb}', word_bytes=4))

        # xbar outputs → cells. The scalar lane (last) is the core's input 1 (with coalescer) or
        # input n_ppc-1 (A1), routed through the AMO shim when amo_lane is set.
        for cb in range(n_ctrl):
            scalar_core_in = 1 if use_coal else (n_ppc - 1)
            if use_coal:
                # VLSU lanes (xbar 0..n_vlsu-1) → coalescer[cb] → core input 0 (VLSU-aggregate).
                for j in range(n_vlsu):
                    self._xbars[j].o_OUTPUT(cb, self._cellcoals[cb].i_INPUT(j))
                self._cellcoals[cb].o_OUTPUT(self._ctrls[cb].i_INPUT(0))
            else:
                # A1: VLSU lanes → core inputs 0..n_ppc-2 directly (no merge).
                for j in range(n_ppc - 1):
                    self._xbars[j].o_OUTPUT(cb, self._ctrls[cb].i_INPUT(j))
            # scalar lane → (AMO shim →) core scalar input.
            if use_amo:
                self._xbars[n_ppc - 1].o_OUTPUT(cb, self._amos[cb].i_INPUT())
                self._amos[cb].o_OUTPUT(self._ctrls[cb].i_INPUT(scalar_core_in))
            else:
                self._xbars[n_ppc - 1].o_OUTPUT(cb, self._ctrls[cb].i_INPUT(scalar_core_in))

        # Refill + eviction (eviction rides the refill path) fan in to the tile's o_L2 master.
        for cb in range(n_ctrl):
            self.bind(self._ctrls[cb], 'refill', self, 'l2')
            self.bind(self._ctrls[cb], 'evict',  self, 'l2')
            if n_ctrl > 1:
                self.bind(self, f'flush_{cb}', self._ctrls[cb], 'flush')
                # E3: runtime partition config pass-through (peripheral → this cell).
                self.bind(self, f'config_core_{cb}', self._ctrls[cb], 'config')
        # E3: runtime partition config pass-through to each per-port-class xbar.
        if n_ctrl > 1:
            for j in range(n_ppc):
                self.bind(self, f'config_xbar_{j}', self._xbars[j], 'config')

        # Remote ports (cross-tile sharing): per port-class j, the xbar's remote-OUT slot (output index
        # n_ctrl) leaves the tile toward the group remote xbar; the remote-IN slot (input index n_cores)
        # receives requests for THIS tile's banks from other tiles. Exposed only when num_remote_port_core>0
        # (the group wires them); single-tile leaves them unbound (route_request never goes remote).
        self._n_remote = n_remote
        if n_remote > 0:
            for j in range(n_ppc):
                for r in range(n_remote):
                    # remote-out: xbar[j].out_(n_ctrl+r) → tile master 'remote_out_{j}_{r}' (to the group).
                    self.bind(self._xbars[j], f'out_{n_ctrl + r}', self, f'remote_out_{j}_{r}')
                    # remote-in: tile slave 'remote_in_{j}_{r}' → xbar[j].in_(n_cores+r) (from the group).
                    self.bind(self, f'remote_in_{j}_{r}', self._xbars[j], f'in_{n_cores + r}')

    # ---------- port factories ----------

    def i_INPUT(self, port: int) -> SlaveItf:
        """TCDM request port ``port`` (0 ≤ port < num_cores * tcdm_ports_per_core)."""
        if port < 0 or port >= self._config.num_tcdm_ports:
            raise RuntimeError(
                f'InsituCacheTile port {port} out of range [0, '
                f'{self._config.num_tcdm_ports})')
        return SlaveItf(self, f'in_{port}', signature='io')

    def num_flush_ports(self) -> int:
        """Number of flush ports = number of controllers (one per controller)."""
        return self._config.num_controllers

    def i_FLUSH(self, ctrl: int) -> SlaveItf:
        """Flush/invalidate trigger for controller ``ctrl`` (a write invalidates its lines)."""
        return SlaveItf(self, f'flush_{ctrl}', signature='io')

    def i_CONFIG_CORE(self, ctrl: int) -> SlaveItf:
        """E3 runtime partition config for controller ``ctrl``."""
        return SlaveItf(self, f'config_core_{ctrl}', signature='io')

    def i_CONFIG_XBAR(self, port_class: int) -> SlaveItf:
        """E3 runtime partition config for port-class crossbar ``port_class``."""
        return SlaveItf(self, f'config_xbar_{port_class}', signature='io')

    def i_REMOTE_IN(self, port_class: int, slot: int = 0) -> SlaveItf:
        """Remote-in slave for port-class ``port_class``, remote slot ``slot`` (other tiles → this tile)."""
        return SlaveItf(self, f'remote_in_{port_class}_{slot}', signature='io')

    def o_REMOTE_OUT(self, port_class: int, slot: int, itf: SlaveItf):
        """Bind this tile's remote-out master for port-class ``port_class``, remote slot ``slot``."""
        self.itf_bind(f'remote_out_{port_class}_{slot}', itf, signature='io')

    def o_L2(self, itf: SlaveItf):
        """Bind the tile's L2 output to ``itf``.

        Fan-in of every controller's refill + evict + each coalescer's write-through
        flush to a single external slave. The composite framework routes any of the
        inner masters (bound to `self.l2` in ``__init__``) to ``itf``.
        """
        self.itf_bind('l2', itf, signature='io')
