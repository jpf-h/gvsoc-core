#
# Copyright (C) 2026 ETH Zurich and University of Bologna
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#

"""Configuration classes for the InSitu Cache performance model.

These classes are the single source of truth for every parameter the model exposes.
Defaults mirror the **latest** ``cachepool_cache_ctrl`` RTL configuration documented in
``prompt/insitu_cache_architecture_v2.md``. The pre-2026-05-22 defaults (from
``prompt/insitu_cache_architecture.md``) corresponded to a different RTL revision; see
``prompt/insitu_cache_architecture_v2.md`` §0 for the diff.

The three ``Config`` subclasses below map 1:1 onto the three GVSoC ``Component`` subclasses
that make up the model: ``InsituCacheController``, ``InsituCacheCoalescer``, and
``InsituCacheInterco``. Each Component's constructor takes its matching Config and the
Config's fields become the component's published properties (read from C++ via
``get_js_config()``).

``InsituCacheTileConfig`` is a plain Python class (not a ``Config``) that bundles the three
together for the tile wrapper.

**Phase-A status (2026-05-22):** new config fields added for `WriteThroughMode`,
`EnableMultiReadPend`, `EnableSPM` / `BankDepthForSPM`, `EnableFlush`. The actual C++
behaviour for those fields is partly stubbed — see the controller `.cpp` for what's wired up
today vs. what's documented for a future Phase-B refactor.

**2026-06-01 update:** the shipping `cachepool_512` config is now the **production**
cache (**folded + hash-way + forwarding-buffer ON**), so :func:`make_cachepool_512_config`
defaults to that (`use_hash_way_select=True`, folded penalties, new
`use_forwarding_buffer=True`). The unfolded + LRU + no-fwd-buffer config — this model's
prior 2026-05-22 default — is now :func:`make_cachepool_512_conventional_config`. See
`insitu_cache_architecture_v2.md` §0.1.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from config_tree import Config, cfg_field


class InsituCacheControllerConfig(Config):
    """Per-cache-controller configuration."""

    # -------- Geometry --------
    cache_line_bytes: int = cfg_field(default=64, dump=True, desc=(
        "Cache line size in bytes (512b = 64B in the canonical RTL)"
    ))
    num_ways: int = cfg_field(default=4, dump=True, desc=(
        "Set associativity per controller"
    ))
    num_sets: int = cfg_field(default=128, dump=True, desc=(
        "Sets per controller (CacheBankDepth in the RTL)"
    ))
    bank_factor: int = cfg_field(default=2, dump=True, desc=(
        "RTL L1BankFactor = NumPseudoDualBanks (hardcoded 2): the pseudo-dual-port split. Used by "
        "the structural cache core's bank model (insitu_cache_bank_array) to detect same-cycle "
        "read-vs-write conflicts on the same (way, bank-select). Power-of-two. Ignored by the "
        "cycle-approximate controller (which uses set_busy)."
    ))
    tcdm_word_bytes: int = cfg_field(default=4, dump=True, desc=(
        "Upstream request granularity in bytes (NarrowDataWidth/8 = 4)"
    ))
    refill_beat_bytes: int = cfg_field(default=16, dump=True, desc=(
        "Width of each refill beat in bytes (128b = 16B). The cache issues one refill "
        "request per miss with size=cache_line_bytes and uses set_duration() to model "
        "the multi-beat occupancy on the refill port."
    ))

    # -------- Replacement policy --------
    use_hash_way_select: bool = cfg_field(default=True, dump=True, desc=(
        "When true, the victim way on a miss is derived from a deterministic hash of "
        "(tag, set). When false, full LRU victim search is used. Hits always use "
        "tag-match. The shipping `cachepool_512` config defaults to hash-way (=1); the "
        "RTL forces hash-way whenever the cache is folded OR the forwarding buffer is on "
        "(see `use_forwarding_buffer`). LRU is legal only for the unfolded conventional "
        "cache with the buffer off."
    ))

    # -------- New write & MSHR mode knobs (latest RTL) --------
    write_through_mode: bool = cfg_field(default=False, dump=True, desc=(
        "When true, writes go straight to L2 via the coalescer (write-through). When "
        "false (default, matches the latest RTL `WriteThroughMode=0`), the cache is in "
        "pure write-back: write hits mark the line dirty and don't push to L2 until "
        "eviction. Phase-A note: the controller still issues a coalescer event on "
        "write hits regardless of this flag — Phase B will gate it."
    ))
    inline_sync_miss: bool = cfg_field(default=False, dump=True, desc=(
        "CLOSED-LOOP protocol. When a refill resolves SYNCHRONOUSLY (downstream memory returns "
        "IO_REQ_OK in the same call), complete the miss INLINE and return IO_REQ_OK — exactly as a "
        "hit does (data filled, miss latency stamped) — instead of parking on the MSHR and calling "
        "resp(). A real core LSU treats OK as done (writes fire-and-forget, reads complete with "
        "latency) and faults on a re-entrant resp() during its request, so the park+resp path "
        "deadlocks/aborts it. Default False keeps the park+MSHR-drain path the open-loop calib "
        "driver relies on for its same-line MSHR-merge throughput modelling (coal_cold). "
        "Single-outstanding-refill model."
    ))
    functional_writethrough: bool = cfg_field(default=False, dump=True, desc=(
        "FUNCTIONAL coherence only (not a timing knob). When true, every write also pushes "
        "its real bytes straight to the backing memory (bypassing the coalescer, which carries "
        "no data), so a backdoor reader — e.g. the ISS/HTIF syscall path that reads target "
        "memory through a separate port — sees the writes. Required for CLOSED-LOOP runs "
        "(Spatz cluster) to compute correctly and terminate; without it write-back dirty data "
        "is invisible to HTIF and the program hangs in a bogus syscall. Default False so the "
        "open-loop calib/microbench memory-traffic and throughput numbers are untouched. NB: "
        "the data path is orthogonal to the latency model; this only adds bytes to memory."
    ))
    enable_multi_read_pend: bool = cfg_field(default=False, dump=True, desc=(
        "When true, multiple read requests to the same in-flight (READ_PEND) cache line "
        "queue on a per-line linked list rather than stalling with MSHR_FULL. Matches "
        "the RTL `ENABLE_MULTI_READ_PEND` compile-time option. Reduces MSHR-full stalls "
        "on read-heavy multi-port workloads."
    ))
    use_forwarding_buffer: bool = cfg_field(default=True, dump=True, desc=(
        "Model the RTL `UseForwardingBuffer` (1-entry write-back SRAM row cache between "
        "the access controller and the data/meta banks; `sram_forwarding_buffer.sv`). "
        "Production `cachepool_512` default = on. RTL constraint: the buffer requires "
        "`use_hash_way_select=True`. Perf effect (suppressing the SRAM-read latency for "
        "repeated same-row accesses, same-cycle RAW resolution) is currently folded into "
        "`hit_latency_cycles` — this flag is informational/fidelity until an explicit "
        "same-row forward model lands (Phase B)."
    ))

    # -------- Hyper-SPM partitioning --------
    enable_spm: bool = cfg_field(default=False, dump=True, desc=(
        "Enable Hyper-SPM partitioning. When true, `bank_depth_for_spm` rows of bank "
        "depth are reserved for software-managed SPM. The cache's effective capacity "
        "shrinks accordingly. See architecture v2 doc §5."
    ))
    bank_depth_for_spm: int = cfg_field(default=0, dump=True, desc=(
        "Number of bank depth rows reserved for SPM (only meaningful if `enable_spm`). "
        "Each row × NumPseudoDualBanks × SetAssociativity × line_bytes is taken out of "
        "the cache's effective capacity."
    ))

    # -------- Flush / Invalidate --------
    enable_flush: bool = cfg_field(default=False, dump=True, desc=(
        "Reserved. When true, the controller exposes a `cache_sync` interface mirroring "
        "the RTL's 2-bit `cache_sync_insn_i` (flush+inval / flush-only / inval-only / "
        "init). Phase-A: not yet implemented in the C++ model — set true to reserve the "
        "Python-side API surface; flush has no runtime effect yet."
    ))

    # -------- Timing knobs --------
    hit_latency_cycles: int = cfg_field(default=4, dump=True, desc=(
        "Cycles from request acceptance at the controller to response emit on a read hit "
        "(the ISOLATED / pipeline-fill latency)"
    ))
    # Structural-core (sync-slave) overrides. The structural InsituCacheCore reads hit_latency_cycles /
    # miss_penalty_cycles by default, but its analytic sync-slave path has a DIFFERENT latency
    # decomposition than the calibrated async controller (no interco(1)/drain stages), so it needs its
    # own values. Calibrated 2026-07-25 on the insitu_cache_calib TB (structural tile + inline sync,
    # STRUCT_BANKS=1) against the RTL standalone reference: isolated warm read-hit = 10 cyc →
    # structural_hit_latency_cycles=10 (measured == knob); cold read-miss = MemLatency+17 →
    # structural_miss_penalty_cycles=12 (ML + refill_bank_write(2) + 12 + ~3 serve/response = ML+17).
    # -1 = fall back to hit_latency_cycles / miss_penalty_cycles (unchanged behaviour).
    structural_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Sync-slave structural core read-hit latency override (-1 = use hit_latency_cycles)."
    ))
    structural_miss_penalty_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Sync-slave structural core miss-penalty override (-1 = use miss_penalty_cycles)."
    ))
    structural_install_tail_cycles: int = cfg_field(default=0, dump=True, desc=(
        "Sync-slave structural core: cycles the controller's refill pipeline stays busy AFTER the miss "
        "response is emitted (the RTL install pipeline tail). Added to the refill-occupancy gate "
        "(sync_refill_busy_until_) only — NOT to the reported per-access latency — so the isolated cold-"
        "miss latency is unchanged but cold-miss throughput drops to the RTL rate (~1/(ML+17)). Set to 3 "
        "to match the RTL (its install pipeline holds the controller ~3 cycles past the response)."
    ))
    structural_write_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Sync-slave structural core store-ack latency (D2: the winfo-FIFO ack, paid by write hits AND "
        "write misses — the WRITE_PEND merge is invisible to the LSU). -1 = use the read-hit value."
    ))
    streaming_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Steady-state per-access read-hit latency once the hit pipeline is full. The RTL hit "
        "path has three decoupling registers (coalescer req-spill, resp-spill, "
        "rsp_spliter/output-FIFO) that an isolated access fills in series (→ hit_latency_cycles) "
        "but a back-to-back stream keeps continuously occupied (→ this value). Modelled as a "
        "per-controller warmth gradient: base = streaming + min(hit_latency-streaming, "
        "cycles_since_last_read_hit), so an isolated hit pays the full fill latency, a streaming "
        "hit pays this, and a gapped hit interpolates (RTL gap-sweep 7/8/9/10). READ hits only "
        "(writes/forwarded reads keep their own latency). -1 = OFF (every hit pays "
        "hit_latency_cycles, unchanged — the Spatz/default path)."
    ))
    bank_accept_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Per-set bank ACCEPT interval (cycles between accepting two successive accesses to the "
        "same set). The data bank is PIPELINED: it accepts a new access every this-many cycles, "
        "each responding `latency` cycles later — so back-to-back accesses to a hot/heavily-reused "
        "set PIPELINE rather than serialize at the full hit latency. Default 1 = one access per "
        "set per cycle (BankFactor=1). Real kernels reuse few lines intensely across many ports; "
        "advancing the per-set busy stamp by the full latency (the pre-2026-06 behaviour) "
        "over-serialized those and inflated per-access latency ~6x on real traces (see "
        "prompt/insitu_cache_realkernel_alignment_2026-06-12.md)."
    ))
    scalar_bypass_port: int = cfg_field(default=-1, dump=True, desc=(
        "Input port index of the Snitch scalar bypass (RTL: the scalar goes through a 2:1 xbar, "
        "not the coalescer). A read from this port takes scalar_hit_latency_cycles and bypasses "
        "the per-set bank contention (its own port). -1 = no bypass (all ports treated as VLSU). "
        "Requires the interco to tag the request port (forward_initiator). Calib DUT = 4."
    ))
    scalar_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Read-hit latency on the scalar bypass port (RTL ≈ 3 cy — shorter pipeline than the VLSU "
        "coalescer path). <0 = use hit_latency_cycles. Only meaningful with scalar_bypass_port>=0."
    ))
    write_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Cycles from acceptance to response on a WRITE hit. The RTL acks a write earlier "
        "than a read returns (write-info FIFO push: `wresp_valid = ~winfo_fifo_empty`), so "
        "the warm-write latency (RTL 8) is below the read-hit latency (RTL 10). Default -1 "
        "means 'same as hit_latency_cycles' (no change for existing configs)."
    ))
    write_commit_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Min cycles between accepting two write hits (a controller-wide write-commit "
        "backpressure). The RTL write path serializes through the write-info FIFO + per-way "
        "commit, so sustained write throughput is ~½ the read throughput (RTL 0.49 vs 0.86 "
        "acc/cyc). Default 1 = no extra serialization (writes pipeline like reads)."
    ))
    fwd_hit_latency_cycles: int = cfg_field(default=-1, dump=True, desc=(
        "Read-hit latency when the line is resident in the 1-entry forwarding buffer (the "
        "immediately-previous access touched the same line). The RTL forwards from the "
        "buffer combinationally, skipping the SRAM read → faster than a normal hit (RTL "
        "read-after-write same word = 7 vs normal hit 10). Only used when "
        "`use_forwarding_buffer=True`. Default -1 = 'same as hit_latency_cycles'."
    ))
    refill_bank_write_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Extra cycles after a refill response arrives before pending MSHR requests serve. "
        "Default 1 reflects the unfolded `DataPartSplit=1` RTL default; set to 2 if "
        "modelling a folded `PartSplit>1` configuration."
    ))
    miss_penalty_cycles: int = cfg_field(default=0, dump=True, desc=(
        "Fixed cache-pipeline overhead added to a refill's completion, on top of the "
        "memory latency and refill_bank_write_cycles (miss detect + downstream issue + "
        "refill ingest/install + response). Calibrated against the RTL standalone "
        "testbench, where a cold read-miss = MemLatency + 17 cyc (see "
        "prompt/insitu_cache_calib_report.md). Default 0 (no change for existing configs)."
    ))
    folded_evict_penalty_cycles: int = cfg_field(default=0, dump=True, desc=(
        "Extra cycles on a dirty eviction to model the folded-SRAM full-line read. "
        "Default 0 (unfolded). Set to 3 to mimic the old `PartSplit=4` mode."
    ))
    mshr_drain_cycles_per_subarray: int = cfg_field(default=1, dump=True, desc=(
        "Cycles added per pending MSHR subarray when draining after a refill"
    ))

    # -------- Occupancy model (deferred-completion path; default OFF = inline) --------
    # The default model resolves a miss INLINE (synchronous): issue_refill -> the memory's
    # OK response -> refill_resp_handler in the same cycle, so FIFO/resource counters never
    # accumulate depth and per-access latency is flat. That suffices for latency + memory-
    # traffic calibration and is what the Spatz integration uses. Setting defer_refills=True
    # switches to an occupancy model where refills complete via scheduled events, so cache
    # resources (refill pool, evic FIFO, MSHR) bind, outstanding accumulates, and per-access
    # latency inflates under load — needed to match the RTL wide-refill (BurstLength=1) miss
    # *throughput* (see prompt/insitu_cache_calib_report.md §9.1 + REPORT_BL1.md).
    defer_refills: bool = cfg_field(default=False, dump=True, desc=(
        "Enable the deferred-completion occupancy path. Default False = inline resolution "
        "(unchanged behaviour; Spatz/BL4 take this path). True = refill completions are "
        "serialized through the install pipeline (refill_drain_cycles) so per-access latency "
        "inflates under load and miss throughput is install-rate-bound (calib wide config)."
    ))
    refill_drain_cycles: int = cfg_field(default=0, dump=True, desc=(
        "Min cycles between successive refill/writeback *completions* draining into the "
        "cache (only when defer_refills). Models the near-serial single-line install "
        "pipeline (~1 completion per N cycles) that inflates per-access latency under load "
        "and sets wide-mode miss throughput. 0 = no completion serialization. The driver's "
        "outstanding budget + this rate together reproduce the RTL wide-refill numbers."
    ))

    # -------- FIFO depths (§3.3) --------
    resp_fifo_depth: int = cfg_field(default=4, dump=True, desc=(
        "Hit response FIFO depth"
    ))
    retr_fifo_depth: int = cfg_field(default=16, dump=True, desc=(
        "MSHR retrieval FIFO depth (16 in newer RTL builds)"
    ))
    miss_fifo_depth: int = cfg_field(default=4, dump=True, desc=(
        "Miss request FIFO depth"
    ))
    evic_fifo_depth: int = cfg_field(default=4, dump=True, desc=(
        "Eviction (writeback) FIFO depth"
    ))
    wt_fifo_depth: int = cfg_field(default=4, dump=True, desc=(
        "Write-through FIFO depth"
    ))


class InsituCacheCoalescerConfig(Config):
    """Configuration for the write-through coalescer (RTL §10)."""

    cache_line_bytes: int = cfg_field(default=64, dump=True, desc=(
        "Cache line size in bytes — must match the controller"
    ))
    watchdog_cycles: int = cfg_field(default=4, dump=True, desc=(
        "Cycles with no new coalescable write before forcing a flush"
    ))


class InsituCacheIntercoConfig(Config):
    """Configuration for the upstream TCDM → cache-controller interconnect (RTL §5)."""

    num_inputs: int = cfg_field(default=20, dump=True, desc=(
        "Number of upstream TCDM request ports (num_cores * tcdm_ports_per_core)"
    ))
    num_outputs: int = cfg_field(default=4, dump=True, desc=(
        "Number of downstream cache controllers"
    ))
    dynamic_offset: int = cfg_field(default=2, dump=True, desc=(
        "Bit offset at which log2(num_outputs) bits select the controller. "
        "Default 2 => 4-byte granularity interleaving (bits [3:2] for num_outputs=4)."
    ))
    interco_latency_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Fixed forward latency through the interco (1 cycle per §5)"
    ))
    enable_input_coalesce: bool = cfg_field(default=False, dump=True, desc=(
        "Model the input par-coalescer: same-cycle, same-line READ-HIT requests across "
        "ports to the same output collapse into ONE cache lookup. The first such read "
        "forwards normally; followers in the same cycle to the same line inherit its "
        "latency and do NOT re-consume the per-output accept slot — so N same-line reads "
        "cost ~one bank access (RTL coal_warm = ~4x the single-port hit rate). Misses are "
        "forwarded through (the controller's MSHR merge handles them → coal_cold mem_rd "
        "unchanged). Default False = no coalescing (Spatz/default path unchanged)."
    ))
    cache_line_bytes: int = cfg_field(default=64, dump=True, desc=(
        "Cache line size in bytes — used only by enable_input_coalesce to group same-line "
        "requests. Must match the controller's cache_line_bytes."
    ))
    coalesce_max_latency: int = cfg_field(default=-1, dump=True, desc=(
        "Only a forwarded read whose resulting latency is <= this many cycles seeds the "
        "coalescing window (i.e. only TRUE warm hits coalesce). A cold line that was just "
        "refilled inline reports a large (refill-sized) latency and so does NOT coalesce — "
        "its same-cycle followers fall through to the controller's MSHR merge, keeping the "
        "cold-miss-stream throughput unchanged. -1 = no limit (merge any OK read)."
    ))
    forward_initiator: bool = cfg_field(default=False, dump=True, desc=(
        "When true, tag each forwarded request's `initiator` field with its input-port index so "
        "the downstream controller can identify the port (used for the scalar bypass, "
        "controller.scalar_bypass_port). Default False = leave initiator untouched (Spatz path "
        "unchanged; the user request is resp'd at the controller and never reaches memory, so the "
        "tag does not interfere with the memory models' LR/SC initiator use)."
    ))
    per_cycle_output_arb: bool = cfg_field(default=False, dump=True, desc=(
        "Output accept arbitration mode. Default False = ACCUMULATE: a monotonic per-output "
        "busy-until cyclestamp models sustained 1/cyc backpressure across cycles — correct for "
        "CLOSED-LOOP use (Spatz, microbench), where the core actually stalls on the returned "
        "latency. True = PER-CYCLE: reset the accept counter each cycle and serialize only "
        "genuinely same-cycle requests (output_accept_width per cycle) — correct for OPEN-LOOP "
        "trace replay (calib), where the trace's t_issue already encodes the RTL's cross-cycle "
        "backpressure, so accumulating would double-count it (~+33 cy hit inflation on fft)."
    ))
    output_accept_width: int = cfg_field(default=1, dump=True, desc=(
        "Per-cycle-arbitration mode only: how many accepts one output absorbs per cycle before "
        "same-cycle followers serialize (the k-th same-cycle accept waits k//width). Models the "
        "RTL coalescer + wide-datapath accept bandwidth. 1 = strict one-per-cycle (different-line "
        "same-cycle reads each cost +1). Ignored when per_cycle_output_arb is False."
    ))
    defer_to_coalescer: bool = cfg_field(default=False, dump=True, desc=(
        "Phase-1 structural mode. When True the interco becomes a pure ADDRESS ROUTER: it skips "
        "the inline read-merge AND the output-accept arbitration AND the interco_latency add, and "
        "just routes each request to its owning output and forwards it — the downstream "
        "InsituCacheParCoalescer does the merge + arbitration + latency instead. Set by the tile "
        "when use_structural_coalescer=True. Default False = the interco does everything inline "
        "(today's calibrated path), byte-identical."
    ))


class InsituCacheParCoalescerConfig(Config):
    """Configuration for the per-controller parallel coalescer (RTL ``par_coalescer``).

    Phase-1 increment 1: the structural front-end that does the same-cycle/same-line read
    merge + output-accept arbitration extracted from the interco. Mirrors the interco's
    merge/arb knobs so the structural path reproduces the calibrated numbers.
    """

    interco_latency_cycles: int = cfg_field(default=1, dump=True, desc=(
        "Fixed forward latency added by the coalescer (the interco no longer adds it in "
        "structural mode)."
    ))
    merge_same_line_reads: bool = cfg_field(default=True, dump=True, desc=(
        "Same-cycle, same-line READ followers inherit the cycle's first warm-hit latency and "
        "return OK without re-forwarding or consuming an accept slot (RTL coal_warm)."
    ))
    cache_line_bytes: int = cfg_field(default=64, dump=True, desc=(
        "Cache line size in bytes — for same-line grouping. Must match the controller."
    ))
    coalesce_max_latency: int = cfg_field(default=-1, dump=True, desc=(
        "Only a forwarded read with latency <= this seeds the merge window (TRUE warm hits "
        "only). -1 = no limit."
    ))
    output_accept_width: int = cfg_field(default=1, dump=True, desc=(
        "Per-cycle-arbitration accept bandwidth (see InsituCacheIntercoConfig.output_accept_width)."
    ))
    per_cycle_output_arb: bool = cfg_field(default=False, dump=True, desc=(
        "Output arbitration mode (see InsituCacheIntercoConfig.per_cycle_output_arb)."
    ))


@dataclass
class InsituCacheTileConfig:
    """Top-level tile configuration. Plain Python container — not a ``Config``.

    The tile Python component (``InsituCacheTile``) uses this to decide how many
    controllers/coalescers/intercos to instantiate, and passes the ``Config`` sub-objects
    down to each instantiated component.
    """

    num_controllers: int = 4
    num_cores: int = 4
    tcdm_ports_per_core: int = 5
    # Phase-1 structural mode: when True the tile inserts one InsituCacheParCoalescer between
    # each interco output and its controller (merge + arb done there; interco becomes a pure
    # router). Default False = monolithic interco does merge+arb inline (today, byte-identical).
    use_structural_coalescer: bool = False
    # Phase-2 inc1 — per-core controller cardinality. When True the *integration site*
    # (snitch_cluster) sets num_controllers = nb_core (RTL NumL1CacheCtrl = NumCores → one cache
    # per core), instead of the factory's fixed 4. Default False = num_controllers unchanged
    # (today's topology, byte-identical). NB: requires power-of-two nb_core (interco routes by
    # `(addr>>dynamic_offset)&(num_outputs-1)`); per-controller capacity is not yet RTL-scaled
    # (that is Phase-2 inc3) so total tile capacity changes with the controller count until then.
    controllers_track_cores: bool = False
    # Structural-rewrite switch (Step 4): when True the tile instantiates the RTL-faithful
    # per-cycle InsituCacheCore instead of the cycle-approximate InsituCacheController. Default
    # False = the calibrated controller (today). First runnable on the OPEN-LOOP calib tile only
    # (async per-cycle FSM); the cluster keeps the controller until the synchronous-slave mode lands.
    use_structural_core: bool = False
    # Structural TILE composition (2026-06-21): when True the tile is built RTL-faithfully —
    # NrTCDMPortsPerCore per-port-class InsituCacheXbar crossbars (shared-L1 any-core→any-bank routing)
    # + per-core multi-lane InsituCacheCore cells — instead of the flat single hashed interco. Default
    # False = the flat tile (byte-identical fallback). Implies use_structural_core for the cells.
    structural_tile: bool = False
    # A2: insert the structural per-cell par_coalescer (4 VLSU lanes merge → 1 wide beat; scalar lane
    # bypasses to the core's 2nd input), matching cachepool_cache_ctrl. Default False = A1 (the core's
    # multi-lane port takes all 5 lanes directly, no merge). Only meaningful with structural_tile.
    cell_coalescer: bool = False
    # A3: insert the spatz_cache_amo / LR-SC shim on the scalar lane (j=n_ppc-1) of each cell, between
    # the lane crossbar and the core (cachepool_tile.sv:658). READ/WRITE pass through; LR/SC/AMO handled.
    # Default False. Only meaningful with structural_tile.
    amo_lane: bool = False
    num_remote_port_core: int = 0   # remote-out/-in slots per port-class (0 = single-tile, no remote)
    num_tiles: int = 1              # NumTiles (for route.hpp partition-mode selection)
    tile_id: int = 0                # this tile's id (stamped for remote routing)
    addr_width: int = 32            # TCDM address width (route.hpp field positions)
    # Crossbar / remote-xbar (cross-tile hop) request-path latency. The RTL `tcdm_cache_interco` has one
    # request-side `spill_register` per port, and the group's AXI xbar uses `CUT_ALL_PORTS` (a pipeline cut
    # on every port) — so each xbar hop costs ~1 cycle on the request path (the response path is a
    # fall-through register = 0). Calibrated 2026-07-26 to 1. The remote xbar is the same module, so the
    # cross-tile hop is also 1. 0 = no xbar/hop latency (the pre-calibration default).
    xbar_latency_cycles: int = 1
    hop_latency_cycles: int = 1
    # E1 MSB address rotation (tcdm_cache_interco.sv:387-404): rotate the N routing bits above
    # dyn_offset (BankSel+TileID) to the address MSB before a request reaches its local bank, so
    # those bits cannot alias into the bank's set index. OFF collapsed effective per-bank capacity
    # by 2^N (16-core 4x4: 256 sets -> 16 sets = 64 KiB -> 4 KiB per bank). Requires dyn_offset ==
    # log2(cache_line_bytes) (rotation must not move line-offset bits). The banks unrotate every
    # L2-side egress (refill/evict/functional-WT) via route.hpp::unrotate_addr. False = pre-E1.
    enable_rotation: bool = True
    # Placed regions "base:size:tile_shift:bank_shift[,...]" — address windows whose
    # TileID/BankSel routing fields sit at configured bit positions instead of the flat slice
    # (route.hpp PlacedRegion; ManyRVData rlc_am doc/RLC_HW.md §1). Broadcast to every tile xbar
    # AND the remote xbars — a region defines what an address means, so every router must agree.
    # Empty = flat routing everywhere.
    regions: str = ''
    controller: InsituCacheControllerConfig = field(
        default_factory=InsituCacheControllerConfig)
    coalescer: InsituCacheCoalescerConfig = field(
        default_factory=InsituCacheCoalescerConfig)
    interco: InsituCacheIntercoConfig = field(
        default_factory=InsituCacheIntercoConfig)

    @property
    def num_tcdm_ports(self) -> int:
        """Total TCDM ports entering the tile (interco inputs)."""
        return self.num_cores * self.tcdm_ports_per_core


def make_cachepool_512_config() -> InsituCacheTileConfig:
    """Canonical ``cachepool_512`` configuration — the **production** CachePool cache.

    As of 2026-06-01 the shipping `cachepool_512` config is the production cache:
    **folded + hash-way + forwarding-buffer ON** (`l1d_use_folded=1`,
    `l1d_use_hash_way=1`, `l1d_use_fwd_buf=1`; see `insitu_cache_architecture_v2.md`
    §0.1). This factory tracks that default. (The earlier 2026-05-22 default of
    unfolded + LRU is now the explicit opt-in *conventional* cache — see
    :func:`make_cachepool_512_conventional_config`.)

    Geometry: 4-way, 64 B line, write-back, no SPM, no flush.

    Note on topology: this factory still produces a `num_controllers=4` tile,
    matching the prior OLD architecture. The latest RTL has a single wide cache +
    an N→1 coalescer (see v2 doc §11 Phase B). For Phase-A we keep the
    multi-controller GVSoC topology and tune the per-controller knobs to the
    production defaults. Full topology migration is tracked under Phase B.
    """
    controller = InsituCacheControllerConfig(
        cache_line_bytes=64,
        num_ways=4,
        num_sets=128,
        tcdm_word_bytes=4,
        refill_beat_bytes=16,
        # Production cachepool_512: hash-way select (folded + fwd-buffer require it).
        use_hash_way_select=True,
        use_forwarding_buffer=True,
        # Pure write-back (WriteThroughMode=0).
        write_through_mode=False,
        # NB: inline_sync_miss / functional_writethrough are DRIVER/integration flags, not cache
        # geometry — they are left at their field defaults (False) here so every open-loop config
        # derived from this factory (calib, conventional, legacy) inherits the calibrated
        # park+MSHR+defer_refills timing path. The closed-loop Spatz cluster turns them on at the
        # instantiation site (snitch_cluster.py) because the real core LSU requires a synchronous
        # slave + functional coherence. Setting them here would silently bypass the occupancy model
        # (reserve_install_pipe) in every derived open-loop DUT — see insitu_cache_calib_report §10.
        # Multi-read MSHR is opt-in; the legacy GVSoC behaviour matches `False`.
        enable_multi_read_pend=False,
        # SPM and flush off by default — see v2 doc §5/§6.
        enable_spm=False,
        bank_depth_for_spm=0,
        enable_flush=False,
        # Calibrated against the RTL standalone testbench (prompt/insitu_cache_calib_report.md),
        # which elaborates the production (folded+hash+fwd) DUT:
        #   warm read-hit (isolated) = 10 cyc → interco(1) + hit_latency(9).
        #   cold read-miss (isolated) = MemLatency + 17 cyc → folded refill_bank_write(2)
        #   + miss_penalty(7) + interco + drain. (Unfolded would be bank_write=1 +
        #   miss_penalty=8 — same total; see make_cachepool_512_conventional_config.)
        hit_latency_cycles=9,
        miss_penalty_cycles=7,
        # Structural sync-slave core (used by the cachepool target): its analytic path has no
        # interco(1)/drain, so it needs its own knobs. Calibrated 2026-07-25 on the calib TB vs the
        # same RTL reference: isolated warm read-hit = 10 → 10 (measured == knob); cold read-miss =
        # MemLatency+17 → ML + refill_bank_write(2) + 12 + ~3 serve/response overhead = ML+17.
        structural_hit_latency_cycles=10,
        structural_miss_penalty_cycles=12,
        structural_install_tail_cycles=3,   # install-pipeline tail after the miss response (RTL occupancy)
        structural_write_hit_latency_cycles=8,   # D2 store ack (RTL warm-write latency = 8, no interco)
        # Write path (calibrated vs RTL warm_write): write hit acks 2 cyc faster than a
        # read returns (8 vs 10 incl. interco), and writes serialize at ~½ read throughput.
        write_hit_latency_cycles=7,   # interco(1) + 7 = 8 (RTL warm write)
        write_commit_cycles=2,        # ~0.49 acc/cyc sustained (RTL ~0.49)
        # Forwarding buffer: a read on the just-touched line forwards combinationally.
        fwd_hit_latency_cycles=6,     # interco(1) + 6 = 7 (RTL read-after-write same word)
        # DataPartSplit=4 (folded) ⇒ +1 refill bank-write cycle, folded-evict penalty.
        refill_bank_write_cycles=2,
        folded_evict_penalty_cycles=3,
        mshr_drain_cycles_per_subarray=1,
        resp_fifo_depth=4,
        retr_fifo_depth=16,
        miss_fifo_depth=4,
        evic_fifo_depth=4,
        wt_fifo_depth=4,
    )
    coalescer = InsituCacheCoalescerConfig(cache_line_bytes=64, watchdog_cycles=4)
    interco = InsituCacheIntercoConfig(
        num_inputs=4 * 5, num_outputs=4, dynamic_offset=2, interco_latency_cycles=1,
    )
    return InsituCacheTileConfig(
        num_controllers=4, num_cores=4, tcdm_ports_per_core=5,
        controller=controller, coalescer=coalescer, interco=interco,
    )


def make_cachepool_fpu_512_config() -> InsituCacheTileConfig:
    """Matches RTL ``config/cachepool_fpu_512.mk`` (commit f5c3ef4) — the 4-tile / 16-core CachePool
    GROUP. Per-tile config for :class:`InsituCacheGroup` (the group clones it per tile with tile_id).

    Topology (cachepool_pkg.sv): ``num_tiles=4``, ``num_cores_per_tile=4``, ``NumL1CacheCtrl=NumCores=16``
    → 4 controllers/tile, ``NrTCDMPortsPerCore=5`` (4 Spatz VLSU + 1 Snitch/FPU scalar),
    ``num_remote_ports_per_tile=2`` → ``NumRemotePortCore=2``.
    Per-controller cache: 4-way × 256-set × 64 B = **64 KiB** (256 KiB/tile ÷ 4), ``L1BankFactor=2``
    (hardcoded in the pkg; the .mk ``l1d_bank_factor=1`` is overridden), folded + hash-way + fwd-buffer.
    ``L1CoalFactor=2``. L2: 4 channels, ``l2_interleave=16`` (used at the cluster L2 step, not group routing).
    Line-granular bank routing (``dynamic_offset = log2(line)``) so an access never spans the granule.
    """
    cfg = make_cachepool_512_config()
    # Per-controller geometry: 64 KiB (= 256 KiB/tile ÷ 4 controllers).
    cfg.controller.num_sets = 256
    cfg.controller.bank_factor = 2
    # Group topology (per-tile values; InsituCacheGroup builds num_tiles of these).
    cfg.structural_tile = True
    cfg.num_tiles = 4
    cfg.num_cores = 4                 # cores per tile
    cfg.num_controllers = 4           # controllers per tile (NumL1CtrlTile = NumL1CacheCtrl/NumTiles)
    cfg.tcdm_ports_per_core = 5
    cfg.num_remote_port_core = 2      # num_remote_ports_per_tile=2
    cfg.addr_width = 32
    # Line-granular routing (the structural xbar routes a whole access to one bank; no cross-granule split).
    cfg.interco.dynamic_offset = 6    # log2(64)
    cfg.interco.num_inputs = cfg.num_cores * cfg.tcdm_ports_per_core
    cfg.interco.num_outputs = cfg.num_controllers
    return cfg


def make_cachepool_512_conventional_config() -> InsituCacheTileConfig:
    """The opt-in **conventional** (unfolded + LRU + no forwarding-buffer) cache.

    Matches the RTL `l1d_use_folded=0 / l1d_use_hash_way=0 / l1d_use_fwd_buf=0`
    selection (the plain set-associative cache). This was the default this model
    tracked on 2026-05-22; it is now an alternative. Same calibrated latency
    target (cold miss = MemLatency + 17, warm hit = 10) reached with the unfolded
    knob values (`refill_bank_write_cycles=1`, `miss_penalty_cycles=8`,
    `folded_evict_penalty_cycles=0`).
    """
    cfg = make_cachepool_512_config()
    cfg.controller.use_hash_way_select = False
    cfg.controller.use_forwarding_buffer = False
    cfg.controller.refill_bank_write_cycles = 1
    cfg.controller.folded_evict_penalty_cycles = 0
    cfg.controller.miss_penalty_cycles = 8
    return cfg


def make_cachepool_512_calib_config() -> InsituCacheTileConfig:
    """Single-controller geometry matching the RTL calibration DUT (one
    ``cachepool_cache_ctrl`` with 5 core ports), for use with the
    ``insitu_cache_calib`` testbench.

    The RTL standalone calibration testbench (``ManyRVData_rebase/reports/
    cache_calib/``) instruments exactly **one** controller serving 5 core ports
    (4 Spatz-VLSU + 1 Snitch scalar) and a single line-refill port. To diff
    GVSoC against it 1:1 we collapse the tile to a single controller and route
    all 5 TCDM ports to it:

    - 1 controller, 5 input ports (interco ``num_outputs=1``).
    - 4-way × 256-set × 64 B line = **64 KiB / controller**, i.e. 1024 lines —
      the RTL ``NumCacheEntry=1024`` per controller.
    - Inherits the **production** config (folded + hash-way + fwd-buffer,
      write-back), matching the RTL calib DUT elaboration banner
      (``PartSplit=4, Folded=1, Hash=1``). For the conventional cache instead,
      start from :func:`make_cachepool_512_conventional_config`.

    Note: the GVSoC tile has no input-side `par_coalescer` and no scalar
    bypass-xbar (Phase-B items), so port 4 is treated like the VLSU ports and the
    coalescer adds no input-path latency. Hit-pipeline depth is therefore tuned
    via the controller / interco latency knobs to match the RTL 10/7-cyc numbers.
    """
    cfg = make_cachepool_512_config()
    cfg.num_controllers = 1
    cfg.num_cores = 1
    cfg.tcdm_ports_per_core = 5            # 4 VLSU + 1 scalar → 5 input ports
    cfg.controller.num_sets = 256          # 1024 entries / 4 ways = 64 KiB / ctrl
    cfg.interco.num_inputs = 5
    cfg.interco.num_outputs = 1
    cfg.interco.dynamic_offset = 2
    # Input par-coalescer: the 4 VLSU ports reading the same line in the same cycle merge
    # into one cache lookup (RTL coal_warm). Hit-path only; misses still go through the
    # MSHR merge. Calib-only — the spatz tile's interco keeps this off.
    cfg.interco.enable_input_coalesce = True
    cfg.interco.cache_line_bytes = cfg.controller.cache_line_bytes
    # Only true warm hits coalesce: interco(1) + hit_latency(9) ≈ 10. A cold line refilled
    # inline reports a refill-sized latency (>> this) so its same-cycle followers do NOT
    # merge — they fall through to the controller's MSHR merge (coal_cold throughput intact).
    cfg.interco.coalesce_max_latency = cfg.controller.hit_latency_cycles + 7
    # Streaming read-hit pipelining: an isolated hit pays interco(1)+9 = 10 (RTL), a
    # back-to-back stream settles to interco(1)+6 = 7 (RTL). The coalescer inherits the
    # first reader's latency, so coal_warm follows to 7/7/7 with no further change.
    cfg.controller.streaming_hit_latency_cycles = cfg.controller.hit_latency_cycles - 3
    # Scalar bypass: port 4 (Snitch scalar) goes through the RTL 2:1 xbar, not the coalescer —
    # a read hit returns in ~3 cy and does not contend for the per-set bank. The interco tags
    # each request's port so the controller can recognise port 4.
    cfg.controller.scalar_bypass_port = 4
    cfg.controller.scalar_hit_latency_cycles = 3
    cfg.interco.forward_initiator = True
    # NB: per_cycle_output_arb is intentionally left at its default (False = accumulate) here.
    # The output-arbitration mode tracks the *injection semantics* of the trace, not the DUT:
    #   • synthetic phase traces inject at max rate (delay=0) and RELY on accumulate-mode
    #     backpressure to produce the saturated-throughput metrics (coal_warm, cold_stream…);
    #   • real-kernel traces carry pre-throttled t_issue, for which accumulate double-counts
    #     the RTL backpressure (~+33 cy hit inflation) and per-cycle arbitration is correct.
    # The real-kernel replay opts in via INSITU_CALIB_PER_CYCLE_ARB (see insitu_cache_calib).
    return cfg


def make_cachepool_512_legacy_config() -> InsituCacheTileConfig:
    """Pre-2026-05-22 (folded, hash-way, write-back+WT-coalesced) configuration.

    Retained for regression: this matches the OLD ``cachepool_512`` RTL on the
    ``cachepool_dev_refactoring`` branch as documented in
    `insitu_cache_architecture.md` (pre-v2). Use this for tests that pin the old
    timing baseline.
    """
    cfg = make_cachepool_512_config()
    cfg.controller.use_hash_way_select = True
    cfg.controller.use_forwarding_buffer = False  # predates the fwd-buffer param
    cfg.controller.write_through_mode = False  # was "hybrid" — see v1 doc §15
    cfg.controller.refill_bank_write_cycles = 2
    cfg.controller.folded_evict_penalty_cycles = 3
    return cfg
