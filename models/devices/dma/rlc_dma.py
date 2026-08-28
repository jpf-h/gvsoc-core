# SPDX-FileCopyrightText: 2026 ETH Zurich and University of Bologna
# SPDX-License-Identifier: Apache-2.0

import gvsoc.systree


class RlcDma(gvsoc.systree.Component):
    """The RLC DMA engine (ManyRVData rlc_am doc/RLC_HW.md §3).

    Two registers per submission ring (ring r: submit W @ r*8, retired count
    R @ r*8+4), a descriptor-read master that must reach the shared L1 (the
    chains are write-back cached), and a payload-read master that reads DRAM
    directly (payload is write-through). See rlc_dma.cpp.
    """

    def __init__(self, parent: gvsoc.systree.Component, name: str,
                 nb_rings: int = 16, line_bytes: int = 64):
        super().__init__(parent, name)
        self.add_sources(['devices/dma/rlc_dma.cpp'])
        self.add_properties({'nb_rings': nb_rings, 'line_bytes': line_bytes})

    def i_INPUT(self) -> gvsoc.systree.SlaveItf:
        return gvsoc.systree.SlaveItf(self, 'input', signature='io')

    def o_DESC(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('desc', itf, signature='io')

    def o_MEM(self, itf: gvsoc.systree.SlaveItf):
        self.itf_bind('mem', itf, signature='io')
