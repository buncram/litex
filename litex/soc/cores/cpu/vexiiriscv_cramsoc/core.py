#
# This file is part of LiteX.
#
# Copyright (c) 2022 Ilia Sergachev <ilia@sergachev.ch>
# Copyright (c) 2022 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

import os

from migen import *

from litex.soc.cores.cpu import CPU, CPU_GCC_TRIPLE_RISCV32
from litex.soc.interconnect.csr import *
from litex.soc.interconnect import axi
from litex.soc.interconnect import wishbone
from litex.soc.integration.soc import SoCRegion, SoCIORegion
from litex.soc.integration.soc import SoCBusHandler

from soc_oss.axi_axil_adapter import AXI2AXILiteAdapter
from soc_oss.axi_crossbar import AXICrossbar
from soc_oss.axil_crossbar import AXILiteCrossbar
from soc_oss.axi_common import *
from litex.soc.integration.doc import AutoDoc,ModuleDoc
from soc_oss.axi_adapter import AXIAdapter
from soc_oss.axil_adapter import AXILiteAdapter
from soc_oss.axil_ahb_adapter import AXILite2AHBAdapter

class Open(Signal): pass

# VexRiscv Timer -----------------------------------------------------------------------------------

class VexRiscvTimer(Module, AutoCSR):
    def __init__(self):
        self._latch    = CSR()
        self._time     = CSRStatus(64)
        self._time_cmp = CSRStorage(64, reset=2**64-1)
        self.interrupt = Signal()

        # # #

        time = Signal(64)
        self.sync += time.eq(time + 1)
        self.sync += If(self._latch.re, self._time.status.eq(time))

        time_cmp = Signal(64, reset=2**64-1)
        self.sync += If(self._latch.re, time_cmp.eq(self._time_cmp.storage))

        self.comb += self.interrupt.eq(time >= time_cmp)


# VexRiscv on AXI ----------------------------------------------------------------------------------------

class VexiiRiscvCramsoc(CPU, AutoDoc):
    category             = "softcore"
    family               = "riscv"
    name                 = "vexiiriscv_cramsoc"
    human_name           = "Vexiiriscv AXI for simulation model"
    data_width           = 32
    endianness           = "little"
    gcc_triple           = CPU_GCC_TRIPLE_RISCV32
    linker_output_format = "elf32-littleriscv"
    nop                  = "nop"
    variants             = ["standard"]
    io_regions           = {
        # Origin, Length.
        0x4000_0000 : 0x2000_0000,
        0xE000_0000 : 0x1000_0000,
    }

    # Memory Mapping.
    @property
    def mem_map(self):
        return {
            "periph"   : 0x4000_0000, # for Daric peripherals + Litex peripherals (need to divide up the space more finely)
            "memory"   : 0x6000_0000, # 0x2000_0000
            "norflash" : 0x7000_0000, # this is included inside the "memory" range; we can address up to 0x8000_0000
            "csr"      : 0xE000_0000, # put RISCV-specific CSRs in the upper region of Daric I/O
        }

    # GCC Flags.
    @property
    def gcc_flags(self):
        flags = "-march=rv32i2p0_mac   -mabi=ilp32"
        flags += " -D__vexriscv__"
        return flags

    def __init__(self, platform, variant="standard-debug", with_timer=False, link_docs=True):
        self.platform         = platform
        self.variant          = variant
        self.human_name       = "VexiiRiscv cramsoc"
        self.external_variant = None
        self.interrupt        = Signal(32)

        self.intro = ModuleDoc("""

This is a RISCV32-IMAC-Zkn(e/d) (32-bit, RV32 instruction set with integer, multiply, atomic, compressed,
and AES cryptography extensions) CPU. It is based on the `VexiiRiscv` core.
""")

        # Dummies - not used in sim model
        self.trimming_reset   = Signal(32)
        self.trimming_reset_ena = Signal()
        self.satp_mode = Signal()
        self.satp_asid = Signal(9)
        self.satp_ppn = Signal(22)
        self.wfi_active = Signal()
        self.privilege = Signal(2)
        self.cmbist = Signal()
        self.cmatpg = Signal()
        self.vexsramtrm = Signal(3)

        # SoC-specific signals -------------------------------------------------------
        # Create AXI-Full Interfaces, attached to the CPU
        IBUS_ID = Signal(2, reset=0)
        DBUS_ID = Signal(reset=1)
        self.ibus_axi   =  ibus = axi.AXIInterface(data_width=64, address_width=32, id_width = 3, bursting=True)
        self.dbus_axi   =  dbus = axi.AXIInterface(data_width=64, address_width=32, id_width = 3, bursting=True)
        self.pbus_axi   = axi.AXIInterface(data_width=32, address_width=32, id_width = 4, bursting=True)

        # peri convert to axil
        peri_axil = axi.AXILiteInterface(name="bus_axil", bursting = False)
        self.submodules += AXI2AXILiteAdapter(platform, self.pbus_axi, peri_axil)

        # axil crossbar to corecsr
        peripherals = axi.AXILiteInterface(data_width=32, address_width=32)
        corecsr = axi.AXILiteInterface(data_width=32, address_width=32)
        p_xbar = AXILiteCrossbar(platform=platform)
        self.submodules += p_xbar
        p_xbar.add_slave(name = "cpu", s_axil=peri_axil)
        p_xbar.add_master(name = "corecsr", m_axil=corecsr, origin=self.mem_map["csr"], size=self.io_regions[self.mem_map["csr"]])
        p_xbar.add_master(name = "peripherals", m_axil=peripherals, origin=self.mem_map["periph"], size=self.io_regions[self.mem_map["periph"]])

        # Expose AXI-Lite Interfaces.
        self.periph_buses     = [corecsr] # Peripheral buses (Connected to main SoC's bus). Leave blank because we don't want any bus to me inferred for the core generation.
        self.memory_buses     = [
            ['ibus', ibus],
            ['dbus', dbus],
            ['pbus', peripherals]
        ]

        self.reset = Signal()
        self.reset_address = 0x6000_0000 # hard coded from vexiiriscv spinal code
        self.reset_comb = Signal()
        self.comb += [
            self.reset_comb.eq(self.reset | ResetSignal("sys"))
        ]
        self.hartid = Signal(32, reset=0)
        self.rdtime = Signal(64, reset=0)

        fetch_ar_id = Signal()
        fetch_r_id = Signal()
        lsu_ar_id = Signal(2)
        lsu_r_id = Signal(2)
        lsu_aw_id = Signal(2)
        lsu_b_id = Signal(2)

        self.comb += [
            ibus.ar.id.eq(Cat(fetch_ar_id, IBUS_ID)),
            fetch_r_id.eq(ibus.r.id[0]),
            self.dbus_axi.aw.id.eq(Cat(lsu_aw_id, DBUS_ID)),
            lsu_b_id.eq(self.dbus_axi.b.id[:2]),
            self.dbus_axi.ar.id.eq(Cat(lsu_ar_id, DBUS_ID)),
            lsu_r_id.eq(self.dbus_axi.r.id[:2]),
        ]

        self.m_ext = Signal()
        self.s_ext = Signal()
        # self.submodules.legacy_int = VexLegacyInt(m_ext, s_ext)
        # self.comb += [
        #    self.legacy_int.interrupts.eq(self.interrupt)
        # ]

        # CPU Instance.
        self.cpu_params = dict(
            i_clk                    = ClockSignal("sys"),
            i_reset                  = self.reset_comb,
            #i_PrivilegedPlugin_api_harts_0_hartId             = self.hartid,
            i_PrivilegedPlugin_logic_rdtime = self.rdtime,
            i_PrivilegedPlugin_logic_harts_0_int_m_timer = 0,
            i_PrivilegedPlugin_logic_harts_0_int_m_software = 0,
            i_PrivilegedPlugin_logic_harts_0_int_m_external = self.m_ext,
            i_PrivilegedPlugin_logic_harts_0_int_s_external = self.s_ext,

            o_FetchL1Axi4Plugin_logic_axi_ar_valid            = ibus.ar.valid,
            i_FetchL1Axi4Plugin_logic_axi_ar_ready            = ibus.ar.ready,
            o_FetchL1Axi4Plugin_logic_axi_ar_payload_addr     = ibus.ar.addr,
            o_FetchL1Axi4Plugin_logic_axi_ar_payload_burst    = ibus.ar.burst,
            o_FetchL1Axi4Plugin_logic_axi_ar_payload_cache    = ibus.ar.cache,
            o_FetchL1Axi4Plugin_logic_axi_ar_payload_len      = ibus.ar.len,
            o_FetchL1Axi4Plugin_logic_axi_ar_payload_prot     = ibus.ar.prot,
            o_FetchL1Axi4Plugin_logic_axi_ar_payload_size     = ibus.ar.size,
            o_FetchL1Axi4Plugin_logic_axi_ar_payload_id       = fetch_ar_id,

            #o_iBusAxi_ar_payload_region = ibus.ar.region,
            #o_iBusAxi_ar_payload_qos    = ibus.ar.qos,
            #o_iBusAxi_ar_payload_lock   = ibus.ar.lock,

            i_FetchL1Axi4Plugin_logic_axi_r_valid        = ibus.r.valid,
            o_FetchL1Axi4Plugin_logic_axi_r_ready        = ibus.r.ready,
            i_FetchL1Axi4Plugin_logic_axi_r_payload_last = ibus.r.last,
            i_FetchL1Axi4Plugin_logic_axi_r_payload_resp = ibus.r.resp,
            i_FetchL1Axi4Plugin_logic_axi_r_payload_data = ibus.r.data,
            i_FetchL1Axi4Plugin_logic_axi_r_payload_id   = fetch_r_id, # not on M3

            # Data Bus (AXI).
            o_LsuL1Axi4Plugin_logic_axi_aw_valid           = self.dbus_axi.aw.valid,
            i_LsuL1Axi4Plugin_logic_axi_aw_ready           = self.dbus_axi.aw.ready,
            o_LsuL1Axi4Plugin_logic_axi_aw_payload_addr    = self.dbus_axi.aw.addr,
            o_LsuL1Axi4Plugin_logic_axi_aw_payload_burst   = self.dbus_axi.aw.burst,
            o_LsuL1Axi4Plugin_logic_axi_aw_payload_cache   = self.dbus_axi.aw.cache,
            o_LsuL1Axi4Plugin_logic_axi_aw_payload_len     = self.dbus_axi.aw.len,
            o_LsuL1Axi4Plugin_logic_axi_aw_payload_id      = lsu_aw_id,
            o_LsuL1Axi4Plugin_logic_axi_aw_payload_size    = self.dbus_axi.aw.size,
            o_LsuL1Axi4Plugin_logic_axi_aw_payload_prot    = self.dbus_axi.aw.prot,

            # o_dBusAxi_aw_payload_region = self.dbus_axi.aw.region, # not on M3
            # o_dBusAxi_aw_payload_qos    = self.dbus_axi.aw.qos, # not on M3
            # o_dBusAxi_aw_payload_lock   = self.dbus_axi.aw.lock,

            o_LsuL1Axi4Plugin_logic_axi_w_valid        = self.dbus_axi.w.valid,
            i_LsuL1Axi4Plugin_logic_axi_w_ready        = self.dbus_axi.w.ready,
            o_LsuL1Axi4Plugin_logic_axi_w_payload_last = self.dbus_axi.w.last,
            o_LsuL1Axi4Plugin_logic_axi_w_payload_strb = self.dbus_axi.w.strb,
            o_LsuL1Axi4Plugin_logic_axi_w_payload_data = self.dbus_axi.w.data,

            i_LsuL1Axi4Plugin_logic_axi_b_valid        = self.dbus_axi.b.valid,
            o_LsuL1Axi4Plugin_logic_axi_b_ready        = self.dbus_axi.b.ready,
            i_LsuL1Axi4Plugin_logic_axi_b_payload_id   = lsu_b_id,
            i_LsuL1Axi4Plugin_logic_axi_b_payload_resp = self.dbus_axi.b.resp,

            o_LsuL1Axi4Plugin_logic_axi_ar_valid          = self.dbus_axi.ar.valid,
            i_LsuL1Axi4Plugin_logic_axi_ar_ready          = self.dbus_axi.ar.ready,
            o_LsuL1Axi4Plugin_logic_axi_ar_payload_addr   = self.dbus_axi.ar.addr,
            o_LsuL1Axi4Plugin_logic_axi_ar_payload_burst  = self.dbus_axi.ar.burst,
            o_LsuL1Axi4Plugin_logic_axi_ar_payload_cache  = self.dbus_axi.ar.cache,
            o_LsuL1Axi4Plugin_logic_axi_ar_payload_len    = self.dbus_axi.ar.len,
            o_LsuL1Axi4Plugin_logic_axi_ar_payload_prot   = self.dbus_axi.ar.prot,
            o_LsuL1Axi4Plugin_logic_axi_ar_payload_size   = self.dbus_axi.ar.size,
            o_LsuL1Axi4Plugin_logic_axi_ar_payload_id     = lsu_ar_id, # not on M3

            # o_dBusAxi_ar_payload_lock   = self.dbus_axi.ar.lock,
            # o_dBusAxi_ar_payload_region = self.dbus_axi.ar.region, # not on M3
            # o_dBusAxi_ar_payload_qos    = self.dbus_axi.ar.qos, # not oon M3

            i_LsuL1Axi4Plugin_logic_axi_r_valid         = self.dbus_axi.r.valid,
            o_LsuL1Axi4Plugin_logic_axi_r_ready         = self.dbus_axi.r.ready,
            i_LsuL1Axi4Plugin_logic_axi_r_payload_last  = self.dbus_axi.r.last,
            i_LsuL1Axi4Plugin_logic_axi_r_payload_resp  = self.dbus_axi.r.resp,
            i_LsuL1Axi4Plugin_logic_axi_r_payload_data  = self.dbus_axi.r.data,
            i_LsuL1Axi4Plugin_logic_axi_r_payload_id    = lsu_r_id, # not on M3

            o_LsuCachelessAxi4Plugin_logic_axi_aw_valid = self.pbus_axi.aw.valid,
            i_LsuCachelessAxi4Plugin_logic_axi_aw_ready = self.pbus_axi.aw.ready,
            o_LsuCachelessAxi4Plugin_logic_axi_aw_payload_addr = self.pbus_axi.aw.addr,
            o_LsuCachelessAxi4Plugin_logic_axi_aw_payload_size = self.pbus_axi.aw.size,
            o_LsuCachelessAxi4Plugin_logic_axi_aw_payload_cache = self.pbus_axi.aw.cache,
            o_LsuCachelessAxi4Plugin_logic_axi_aw_payload_prot = self.pbus_axi.aw.prot,
            o_LsuCachelessAxi4Plugin_logic_axi_w_valid = self.pbus_axi.w.valid,
            i_LsuCachelessAxi4Plugin_logic_axi_w_ready = self.pbus_axi.w.ready,
            o_LsuCachelessAxi4Plugin_logic_axi_w_payload_data = self.pbus_axi.w.data,
            o_LsuCachelessAxi4Plugin_logic_axi_w_payload_strb = self.pbus_axi.w.strb,
            o_LsuCachelessAxi4Plugin_logic_axi_w_payload_last = self.pbus_axi.w.last,
            i_LsuCachelessAxi4Plugin_logic_axi_b_valid = self.pbus_axi.b.valid,
            o_LsuCachelessAxi4Plugin_logic_axi_b_ready = self.pbus_axi.b.ready,
            i_LsuCachelessAxi4Plugin_logic_axi_b_payload_resp = self.pbus_axi.b.resp,
            o_LsuCachelessAxi4Plugin_logic_axi_ar_valid = self.pbus_axi.ar.valid,
            i_LsuCachelessAxi4Plugin_logic_axi_ar_ready = self.pbus_axi.ar.ready,
            o_LsuCachelessAxi4Plugin_logic_axi_ar_payload_addr = self.pbus_axi.ar.addr,
            o_LsuCachelessAxi4Plugin_logic_axi_ar_payload_size = self.pbus_axi.ar.size,
            o_LsuCachelessAxi4Plugin_logic_axi_ar_payload_cache = self.pbus_axi.ar.cache,
            o_LsuCachelessAxi4Plugin_logic_axi_ar_payload_prot = self.pbus_axi.ar.prot,
            i_LsuCachelessAxi4Plugin_logic_axi_r_valid = self.pbus_axi.r.valid,
            o_LsuCachelessAxi4Plugin_logic_axi_r_ready = self.pbus_axi.r.ready,
            i_LsuCachelessAxi4Plugin_logic_axi_r_payload_data = self.pbus_axi.r.data,
            i_LsuCachelessAxi4Plugin_logic_axi_r_payload_resp = self.pbus_axi.r.resp,
            i_LsuCachelessAxi4Plugin_logic_axi_r_payload_last = self.pbus_axi.r.last,
        )
        platform.add_source_dir("VexiiRiscv/VexiiRiscv-cramsoc.sv")

        # Add Timer (Optional).
        if with_timer:
            self.add_timer()

        # Add Debug (Optional).
        if "debug" in variant:
            self.add_debug()

    def add_jtag(self, pads):
        trst = Signal()
        self.comb += [
            trst.eq(~pads.trst_n)
        ]
        self.cpu_params.update(
            i_EmbeddedRiscvJtag_logic_jtag_tdi      = pads.tdi,
            o_EmbeddedRiscvJtag_logic_jtag_tdo      = pads.tdo,
            i_EmbeddedRiscvJtag_logic_jtag_tms      = pads.tms,
            i_EmbeddedRiscvJtag_logic_jtag_tck      = pads.tck,
            i_EmbeddedRiscvJtag_logic_ndmreset      = trst,
        )

    def set_reset_address(self, reset_address):
        print("**TODO RESET ADDRESS SETTING TODO**")
        if False:
            self.reset_address = reset_address
            reset_mux = Signal(32, reset=reset_address)
            self.comb += [
                If(self.trimming_reset_ena,
                    reset_mux.eq(self.trimming_reset)
                ).Else(
                    reset_mux.eq(Signal(32, reset=reset_address))
                )
            ]
            self.cpu_params.update(i_externalResetVector=reset_mux)

    def add_timer(self):
        self.submodules.timer = VexRiscvTimer()
        self.cpu_params.update(i_PrivilegedPlugin_logic_harts_0_int_m_timer=self.timer.interrupt)

    def add_debug(self):
        self.o_resetOut   = Signal()
        reset_debug_logic = Signal()
        debug_reset       = Signal()
        self.sync += reset_debug_logic.eq(self.o_resetOut)
        self.sync += debug_reset.eq(reset_debug_logic | ResetSignal())

        self.cpu_params.update(
            i_reset            = ResetSignal() | debug_reset,
            # o_debug_resetOut   = self.o_resetOut
        )

    @staticmethod
    def add_sources(platform, variant="standard"):
        platform.add_source("VexiiRiscv/VexiiRiscv-cramsoc.sv")
        platform.add_source("VexiiRiscv/early0_AesZknPlugin_logic_onData_rom_storage.v")

    def add_soc_components(self, soc):
        # Pass I/D Caches info to software.
        base_variant = str(self.variant.split('+')[0])
        # DCACHE is present on all variants except minimal and lite.
        if not base_variant in ["minimal", "lite"]:
            soc.add_config("CPU_HAS_DCACHE")
        # ICACHE is present on all variants except minimal.
        if not base_variant in ["minimal"]:
            soc.add_config("CPU_HAS_ICACHE")

    def use_external_variant(self, variant_filename):
        self.external_variant = True
        self.platform.add_source(variant_filename)

    def do_finalize(self):
        assert hasattr(self, "reset_address")
        if not self.external_variant:
            self.add_sources(self.platform, self.variant)
        self.specials += Instance("VexiiRiscv", **self.cpu_params)
        if hasattr(self, "cfu_params"):
            self.specials += Instance("Cfu", **self.cfu_params)
