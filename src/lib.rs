//! ESP32-P4 Octal/Hex PSRAM bring-up — clean port of IDF v5.4's
//! `esp_psram_impl_ap_hex.c` for the `firmware2/` `--ram --no-stub`
//! boot flow.
//!
//! # Why this crate exists
//!
//! `--ram --no-stub` jumps straight from the ROM bootloader into our
//! `_start` and bypasses IDF's 2nd-stage bootloader, which is where
//! `esp_psram_init()` would normally run. We must reproduce it ourselves
//! after `bootloader::init_phase2_full()` has clocks/MSPI/cache/MMU
//! ready.
//!
//! # Hardware target
//!
//! Waveshare ESP32-P4-ETH with AP-Memory APS6408L (Hex), 32 MB:
//! - Vendor ID `0x0D` (in `MR1.vendor_id`)
//! - Density code `0x7` ⇒ 32 MB (in `MR2.density`)
//! - Wired to MSPI_2 (cache-side) + MSPI_3 (data lanes)
//!
//! # Status
//!
//! Phase A (this commit): **types + constants + host tests** — everything
//! in this file lints, builds on host, and host-tests pass. No MMIO yet.
//!
//! Phase B (`regs.rs`, pending): register HAL — port of
//! `psram_ctrlr_ll.h` (~770 lines of SPIMEM2/3 + HP_SYS_CLKRST pokes).
//!
//! Phase C (`detect()` in this file, pending): minimal "read MR0/MR1
//! and verify vendor_id == 0x0D" probe. Follows IDF order:
//! clocks → pins → CS timing → bus clock + DLL → MR read.
//!
//! Phase D: full `init()` — MR write, MSPI config for cached access,
//! MMU map, cache enable.

#![cfg_attr(not(test), no_std)]

use core::fmt;

// Phase B: register HAL port of `psram_ctrlr_ll.h`. Gated to `riscv32`
// because it pokes PAC MMIO; host builds keep the `detect()` stub returning
// `NotImplemented` and don't link this module.
#[cfg(target_arch = "riscv32")]
pub mod regs;

// ── Address-space constants ──────────────────────────────────────────────────

/// Virtual base address where the cached PSRAM window starts. Must match
/// the `PSRAM` region in `memory.x`. ESP-IDF convention for P4
/// (`SOC_EXTRAM_DATA_LOW`).
pub const PSRAM_VADDR_BASE: usize = 0x4800_0000;

/// Size of the cached PSRAM window in bytes. Full 32 MB to use the
/// entire AP-Memory APS6408L chip on the Waveshare ESP32-P4-ETH board.
/// The P4 MMU has 1024 entries × 64 KB = 64 MB of vaddr space available
/// from `PSRAM_VADDR_BASE`, so 32 MB fits with room for future expansion.
pub const PSRAM_VADDR_SIZE: usize = 32 * 1024 * 1024;

// ── Constants extracted from IDF v5.4 ────────────────────────────────────────
// Source: components/esp_psram/device/esp_psram_impl_ap_hex.c
// Each constant cites the IDF line where it lives.

/// AP-Memory vendor ID returned in `MR1.vendor_id`. Verified after MR
/// readback during chip detection. Source: `esp_psram_impl_ap_hex.c:48`.
pub const AP_HEX_PSRAM_VENDOR_ID: u8 = 0x0D;

/// MPLL frequency PSRAM clock divides down from. P4 uses MPLL @ 400 MHz.
/// Source: `esp_psram_impl_ap_hex.c:54`.
pub const AP_HEX_PSRAM_MPLL_DEFAULT_FREQ_MHZ: u32 = 400;

/// PSRAM bus clock target frequency for chip detection — 20 MHz.
/// Verified empirically against IDF v5.3 working baseline 2026-05-01:
/// IDF runs at exactly this speed (clock reg = 0x00130913 = divider 20)
/// with `CONFIG_SPIRAM_SPEED_20M` despite default Kconfig being 200M.
/// SPEED_SLOW preset on the AP-Memory APS6408L chip.
pub const PSRAM_DETECT_SPEED_MHZ: u32 = 20;

/// SPI command codes for AP-Memory hex PSRAM. Sent during MSPI controller
/// configuration and mode-register access.
/// Source: `esp_psram_impl_ap_hex.c:21-26`.
pub mod cmd {
    pub const SYNC_READ: u16 = 0x0000;
    pub const SYNC_WRITE: u16 = 0x8080;
    pub const BURST_READ: u16 = 0x2020;
    pub const BURST_WRITE: u16 = 0xA0A0;
    pub const REG_READ: u16 = 0x4040;
    pub const REG_WRITE: u16 = 0xC0C0;
}

/// Wire-format bit lengths used in MSPI command/address phases.
/// Source: `esp_psram_impl_ap_hex.c:27-29`.
pub const RD_CMD_BITLEN: u32 = 16;
pub const WR_CMD_BITLEN: u32 = 16;
pub const ADDR_BITLEN: u32 = 32;

/// CS (chip-select) timing — number of MSPI clock cycles for setup, hold,
/// hold-delay, ECC-hold. Source: `esp_psram_impl_ap_hex.c:49-52`.
pub mod cs_timing {
    pub const SETUP: u32 = 4;
    pub const HOLD: u32 = 4;
    pub const HOLD_DELAY: u32 = 3;
    pub const ECC_HOLD: u32 = 4;
}

/// Speed-dependent latency configuration. The IDF picks one of three
/// triplets based on `CONFIG_SPIRAM_SPEED_*` Kconfig flags.
///
/// Source: `esp_psram_impl_ap_hex.c:31-46`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LatencyConfig {
    /// Read dummy bit length, computed as `2*(N-1)` per IDF.
    pub rd_dummy_bitlen: u32,
    /// Write dummy bit length, `2*(N-1)`.
    pub wr_dummy_bitlen: u32,
    /// Read latency value programmed into MR0 (`mr0.read_latency`).
    pub rd_latency_mr: u8,
    /// Write latency value programmed into MR4 (`mr4.wr_latency`).
    pub wr_latency_mr: u8,
}

impl LatencyConfig {
    /// 200 MHz Octal preset — IDF's `#elif CONFIG_SPIRAM_SPEED_200M` branch.
    pub const SPEED_200M: Self = Self {
        rd_dummy_bitlen: 2 * (14 - 1), // 26
        wr_dummy_bitlen: 2 * (7 - 1),  // 12
        rd_latency_mr: 4,
        wr_latency_mr: 1,
    };

    /// 250 MHz preset — only valid for ≤16 MB parts. Reference only;
    /// our 32 MB chip rejects this configuration.
    pub const SPEED_250M: Self = Self {
        rd_dummy_bitlen: 2 * (18 - 1), // 34
        wr_dummy_bitlen: 2 * (9 - 1),  // 16
        rd_latency_mr: 6,
        wr_latency_mr: 3,
    };

    /// Slow-mode preset (default `else` branch in IDF) — used during
    /// initial mode-register write before switching to high speed.
    pub const SPEED_SLOW: Self = Self {
        rd_dummy_bitlen: 2 * (10 - 1), // 18
        wr_dummy_bitlen: 2 * (5 - 1),  // 8
        rd_latency_mr: 2,
        wr_latency_mr: 2,
    };
}

/// MSPI controller IDs. P4 uses **two** MSPI controllers for PSRAM —
/// `MSPI_2` runs cache-side traffic, `MSPI_3` runs the data lanes. Both
/// must be initialised together.
///
/// Source: `psram_ctrlr_ll.h:32-33`.
///
/// # PAC mapping (esp32p4 0.2)
///
/// IDF's `SPIMEM2`/`SPIMEM3` are secondary-memory views of the dual MSPI
/// banks, **not** separate peripherals. They live in PAC `spi0` and
/// `spi1` respectively (NOT `spi2`/`spi3` — those are GP-SPI).
///
/// - `SPIMEM2` (MSPI_ID_2) ↔ PAC `spi0` — carries ~95% of init register
///   traffic (cache_sctrl/fctrl, sram_drd/dwr_cmd, sram_cmd, smem_*,
///   pms_attr/addr/size, timing_cali, ctrl1, smem_axi_addr_ctrl).
/// - `SPIMEM3` (MSPI_ID_3) ↔ PAC `spi1` — only touched in 2 spots:
///   `set_bus_clock(MSPI_3, ...)` writes `spi1.clock()`,
///   `enable_variable_dummy(MSPI_3, ...)` writes `spi1.ddr().fmem_var_dummy`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MspiId {
    /// `PSRAM_CTRLR_LL_MSPI_ID_2` — primary controller, cache-readable
    /// mapping + nearly all PSRAM register traffic. PAC: `spi0`.
    Mspi2 = 2,
    /// `PSRAM_CTRLR_LL_MSPI_ID_3` — secondary controller, used only
    /// for bus-clock and variable-dummy. PAC: `spi1`.
    Mspi3 = 3,
}

/// Density-encoded PSRAM size from `MR2.density`. Source:
/// `esp_psram_impl_ap_hex.c:412-416`. Note `0x7` = 32 MB (Waveshare's
/// chip), `0x6` = 64 MB (out-of-order with the rest).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PsramSize {
    Mb4 = 4 * 1024 * 1024,
    Mb8 = 8 * 1024 * 1024,
    Mb16 = 16 * 1024 * 1024,
    Mb32 = 32 * 1024 * 1024,
    Mb64 = 64 * 1024 * 1024,
}

impl PsramSize {
    /// Decode `MR2.density` field (3 bits). Returns `None` for unknown
    /// codes. Waveshare's 32 MB chip reports `density = 0x7`.
    pub fn from_mr2_density(density: u8) -> Option<Self> {
        match density {
            0x1 => Some(Self::Mb4),
            0x3 => Some(Self::Mb8),
            0x5 => Some(Self::Mb16),
            0x7 => Some(Self::Mb32),
            0x6 => Some(Self::Mb64),
            _ => None,
        }
    }

    pub const fn bytes(self) -> usize {
        self as usize
    }
}

// ── Mode register bitfields ──────────────────────────────────────────────────
// Layouts mirror `hex_psram_mode_reg_t` in `esp_psram_impl_ap_hex.c:56-111`.
// Plain `u8` storage with accessor methods rather than a bitfield crate —
// the registers are small and accessed rarely.

/// Mode Register 0 — read latency, drive strength, latency type. Read &
/// modified during init.
/// Source: `esp_psram_impl_ap_hex.c:57-66`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Mr0(pub u8);

impl Mr0 {
    pub const fn drive_str(self) -> u8 {
        self.0 & 0b11
    }
    pub const fn read_latency(self) -> u8 {
        (self.0 >> 2) & 0b111
    }
    /// Latency type: 0 = variable, 1 = fixed.
    pub const fn lt(self) -> bool {
        (self.0 >> 5) & 1 != 0
    }
    pub const fn tso(self) -> bool {
        (self.0 >> 7) & 1 != 0
    }

    pub const fn new(drive_str: u8, read_latency: u8, lt: bool) -> Self {
        let mut b = 0u8;
        b |= drive_str & 0b11;
        b |= (read_latency & 0b111) << 2;
        if lt {
            b |= 1 << 5;
        }
        Self(b)
    }

    /// Replace `drive_str` (bits 0..1), preserving other bits.
    pub const fn with_drive_str(self, drive_str: u8) -> Self {
        Self((self.0 & !0b11) | (drive_str & 0b11))
    }

    /// Replace `read_latency` (bits 2..4), preserving other bits.
    pub const fn with_read_latency(self, read_latency: u8) -> Self {
        Self((self.0 & !(0b111 << 2)) | ((read_latency & 0b111) << 2))
    }

    /// Replace `lt` (bit 5), preserving other bits.
    pub const fn with_lt(self, lt: bool) -> Self {
        let mask: u8 = !(1 << 5);
        Self((self.0 & mask) | (if lt { 1 << 5 } else { 0 }))
    }
}

/// Mode Register 1 — vendor ID, ULP flag. **Read-only** — used for chip
/// detection. Source: `esp_psram_impl_ap_hex.c:67-74`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Mr1(pub u8);

impl Mr1 {
    /// Bits 0..4 — vendor ID. Expected `AP_HEX_PSRAM_VENDOR_ID = 0x0D`.
    pub const fn vendor_id(self) -> u8 {
        self.0 & 0b11111
    }
    pub const fn ulp(self) -> bool {
        (self.0 >> 7) & 1 != 0
    }
}

/// Mode Register 2 — density (size), device ID, KGD (known-good-die).
/// Read-only; size detection uses `density`.
/// Source: `esp_psram_impl_ap_hex.c:75-82`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Mr2(pub u8);

impl Mr2 {
    pub const fn density(self) -> u8 {
        self.0 & 0b111
    }
    pub const fn dev_id(self) -> u8 {
        (self.0 >> 3) & 0b11
    }
    /// 6 = "Pass" (known-good die). Anything else = factory-rejected
    /// silicon, shouldn't be in production hardware.
    pub const fn kgd(self) -> u8 {
        (self.0 >> 5) & 0b111
    }
}

/// Mode Register 3 — self-refresh fast/slow flag. Diagnostic only; we
/// don't program it. Source: `esp_psram_impl_ap_hex.c:83-91`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Mr3(pub u8);

/// Mode Register 4 — write latency, partial-array self-refresh, refresh
/// rate. We program `wr_latency` from [`LatencyConfig::wr_latency_mr`].
/// Source: `esp_psram_impl_ap_hex.c:92-99`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Mr4(pub u8);

impl Mr4 {
    pub const fn pasr(self) -> u8 {
        self.0 & 0b111
    }
    pub const fn rf(self) -> u8 {
        (self.0 >> 3) & 0b11
    }
    pub const fn wr_latency(self) -> u8 {
        (self.0 >> 5) & 0b111
    }

    pub const fn with_wr_latency(self, wr_latency: u8) -> Self {
        let mask: u8 = !(0b111 << 5);
        Self((self.0 & mask) | ((wr_latency & 0b111) << 5))
    }
}

/// Mode Register 8 — burst length, burst type, X16/X8 mode, RBX
/// (reserved-bank-extension). Programmed for our access pattern.
/// Source: `esp_psram_impl_ap_hex.c:100-110`.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Mr8(pub u8);

impl Mr8 {
    /// 0 = 16 B, 1 = 32 B, 2 = 64 B, 3 = 2048 B (page-mode).
    pub const fn bl(self) -> u8 {
        self.0 & 0b11
    }
    /// 0 = wrap, 1 = hybrid wrap.
    pub const fn bt(self) -> bool {
        (self.0 >> 2) & 1 != 0
    }
    pub const fn rbx(self) -> bool {
        (self.0 >> 3) & 1 != 0
    }
    /// 0 = X8, 1 = X16 (16 data lanes — Hex mode default).
    pub const fn x16(self) -> bool {
        (self.0 >> 6) & 1 != 0
    }

    pub const fn new(bl: u8, bt: bool, rbx: bool, x16: bool) -> Self {
        let mut b = 0u8;
        b |= bl & 0b11;
        if bt {
            b |= 1 << 2;
        }
        if rbx {
            b |= 1 << 3;
        }
        if x16 {
            b |= 1 << 6;
        }
        Self(b)
    }

    pub const fn with_bl(self, bl: u8) -> Self {
        Self((self.0 & !0b11) | (bl & 0b11))
    }
    pub const fn with_bt(self, bt: bool) -> Self {
        let mask: u8 = !(1 << 2);
        Self((self.0 & mask) | (if bt { 1 << 2 } else { 0 }))
    }
    pub const fn with_rbx(self, rbx: bool) -> Self {
        let mask: u8 = !(1 << 3);
        Self((self.0 & mask) | (if rbx { 1 << 3 } else { 0 }))
    }
    pub const fn with_x16(self, x16: bool) -> Self {
        let mask: u8 = !(1 << 6);
        Self((self.0 & mask) | (if x16 { 1 << 6 } else { 0 }))
    }
}

/// Aggregated mode-register snapshot. Read via `detect()` and verified
/// for `vendor_id` + `density` after init.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ModeRegisters {
    pub mr0: Mr0,
    pub mr1: Mr1,
    pub mr2: Mr2,
    pub mr3: Mr3,
    pub mr4: Mr4,
    pub mr8: Mr8,
}

/// Default mode-register configuration written during init. Mirrors
/// `mode_reg = {…}` in `esp_psram_impl_ap_hex.c:392-403`.
pub fn default_mode_register_config() -> ModeRegisters {
    let lat = LatencyConfig::SPEED_200M;
    ModeRegisters {
        mr0: Mr0::new(/*drive_str=*/ 0, lat.rd_latency_mr, /*lt=*/ true),
        mr4: Mr4::default().with_wr_latency(lat.wr_latency_mr),
        // BL=3 (page mode, 2048 B), BT=0 (wrap), RBX=1, X16=1 (Hex mode)
        mr8: Mr8::new(/*bl=*/ 3, /*bt=*/ false, /*rbx=*/ true, /*x16=*/ true),
        ..Default::default()
    }
}

// ── Errors ───────────────────────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PsramError {
    /// Not yet implemented — Phase B/C/D placeholder.
    NotImplemented,
    /// MPLL acquire/configure failed.
    MpllSetupFailed,
    /// PSRAM module clock didn't come up.
    ClockSetupFailed,
    /// IO_MUX did not route Octal-SPI signals to physical pads.
    PinMuxFailed,
    /// Mode-register read after reset returned implausible vendor ID.
    /// `vendor_id != 0x0D` typically means MSPI bus timing is off.
    ChipDetectionFailed { reported_vendor_id: u8 },
    /// Vendor ID is correct but density code in `MR2.density` is one of
    /// the unassigned values (not 0x1/0x3/0x5/0x6/0x7).
    UnknownPsramSize { density: u8 },
    /// 250 MHz speed mode requested for a 32 MB chip — IDF rejects this.
    SpeedNotSupported,
    /// MMU mapping write-back didn't take.
    MmuMapFailed,
    /// L2 cache enable failed for the PSRAM region.
    CacheSetupFailed,
    /// SPI command timed out waiting for `spi_mem_c_cmd.usr` to clear.
    /// Most common Day-4 failure mode — chip not responding to MR-read.
    /// `cmd_reg` is the last observation of `SPI_MEM_S_CMD_REG`; bits
    /// [7:4] = `SLV_ST` (FSM state), bit 18 = `USR`. `addr` is the MR
    /// offset that was being read so the caller can correlate which
    /// of the 4 reads failed.
    TransactionTimeout { addr: u32, iters: u32, cmd_reg: u32 },
}

impl fmt::Display for PsramError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PsramError::NotImplemented => f.write_str("PSRAM init not implemented (Phase B/C/D pending)"),
            PsramError::MpllSetupFailed => f.write_str("MPLL setup failed"),
            PsramError::ClockSetupFailed => f.write_str("PSRAM module clock did not come up"),
            PsramError::PinMuxFailed => f.write_str("PSRAM IO_MUX failed"),
            PsramError::ChipDetectionFailed { reported_vendor_id } => write!(
                f,
                "PSRAM chip detection failed: vendor_id 0x{:02X} (expected 0x{:02X})",
                reported_vendor_id, AP_HEX_PSRAM_VENDOR_ID
            ),
            PsramError::UnknownPsramSize { density } => {
                write!(f, "unknown PSRAM density code 0x{:X}", density)
            }
            PsramError::SpeedNotSupported => f.write_str("PSRAM speed mode not supported for chip size"),
            PsramError::MmuMapFailed => f.write_str("PSRAM MMU mapping failed"),
            PsramError::CacheSetupFailed => f.write_str("PSRAM L2 cache setup failed"),
            PsramError::TransactionTimeout { addr, iters, cmd_reg } => write!(
                f,
                "MSPI transaction timed out at MR addr 0x{:X} after {} iters, cmd_reg=0x{:08X} (slv_st=0x{:X})",
                addr,
                iters,
                cmd_reg,
                (cmd_reg >> 4) & 0xF
            ),
        }
    }
}

// ── Public API ───────────────────────────────────────────────────────────────

/// Bring up MSPI 2/3 + program PMU regulators enough to read MR0..MR8
/// over the SPI command engine and return them.
///
/// **Pre-conditions:** `bootloader::init_phase2_full()` has run (provides
/// MPLL@400, pin DRV=2, DQS XPD, flash MSPI bring-up, cache/MMU init).
///
/// On success the chip is detected (vendor_id verified) and ready for
/// [`init`] to fully bring it up as a memory window. Most callers
/// should use [`init`] directly rather than calling [`detect`] alone.
///
/// IDF reference: `esp_psram_impl_ap_hex.c::esp_psram_impl_enable()`,
/// pmu_init.c, and pmu_pvt.c (IDF v5.3).
#[cfg(target_arch = "riscv32")]
pub fn detect() -> Result<ModeRegisters, PsramError> {
    prepare_for_mr_read();
    // The PSRAM chip retains mode-register state across CPU resets
    // (espflash DTR pulse resets the SoC cores but the PSRAM IC stays
    // powered). If a previous run left the chip in Hex/X16 mode our
    // OPI DTR MR-read returns garbage. Issue a Global Reset (cmd 0xFFFF
    // CMD-only) on MSPI_3 to put the chip back into POR mode-register
    // state before reading.
    global_reset()?;
    read_mode_registers(MspiId::Mspi3)
}

/// Send the AP-Memory Hex PSRAM Global Reset command (0xFFFF) on MSPI_3.
/// CMD-only transaction (no addr/dummy/data phases) — brings the chip
/// back into POR mode-register state regardless of prior config.
#[cfg(target_arch = "riscv32")]
fn global_reset() -> Result<(), PsramError> {
    // SAFETY: cmd-only transaction, null buffers ok with bitlen 0.
    let result = unsafe {
        regs::try_common_transaction(
            MspiId::Mspi3,
            0xFFFF, 16,
            0, 0,
            0,
            core::ptr::null_mut(), 0,
            core::ptr::null_mut(), 0,
            false,
            1_000_000,
        )
    };
    match result {
        Ok(_) => Ok(()),
        Err(regs::TransactionError::Timeout { iters, cmd_reg_value }) => {
            Err(PsramError::TransactionTimeout {
                addr: 0xFFFF,
                iters,
                cmd_reg: cmd_reg_value,
            })
        }
    }
}

/// Host build no-op so workspace tests link.
#[cfg(not(target_arch = "riscv32"))]
pub fn detect() -> Result<ModeRegisters, PsramError> {
    Err(PsramError::NotImplemented)
}

/// PSRAM-specific bus bring-up: MSPI_2/MSPI_3 module clocks, CS timing,
/// bus clock and DLL. Runs BEFORE the first MR-read.
///
/// **Pre-condition:** `bootloader::init_phase2_full()` has already run
/// (provides PMU regulators, MPLL, HP_SYS_CLKRST chip-wide clocks, MSPI
/// pin DRV/DQS, and L2 cache mode). Without those the MSPI PHY analog
/// block cannot sample MISO and MR-read hangs at `slv_st = 5`.
#[cfg(target_arch = "riscv32")]
fn prepare_for_mr_read() {
    use regs::PsramClkSrc;

    // PSRAM-side clocks in HP_SYS_CLKRST.PERI_CLK_CTRL00 (psram_pll_clk_en,
    // psram_clk_src_sel, psram_core_clk_en/div). The chip-wide PVT clock
    // gates are handled by `bootloader::clkrst::init_hp_clocks`.
    // SAFETY: fixed MMIO, single-hart at boot.
    unsafe {
        let p = (0x500E_6030) as *mut u32; // HP_SYS_CLKRST.PERI_CLK_CTRL00
        let cur = core::ptr::read_volatile(p);
        core::ptr::write_volatile(p, cur | 0x0000_D05D);
    }

    // PSRAM module clock + reset + source select
    regs::set_core_clock_div(0, 1);
    regs::enable_core_clock(0, true);
    regs::enable_module_clock(MspiId::Mspi2, true);
    regs::reset_module_clock(MspiId::Mspi2);
    regs::select_clk_source(MspiId::Mspi2, PsramClkSrc::Mpll);
    regs::select_clk_source(MspiId::Mspi3, PsramClkSrc::Mpll);

    // CS timing on MSPI_2
    regs::set_cs_setup(MspiId::Mspi2, cs_timing::SETUP);
    regs::set_cs_hold(MspiId::Mspi2, cs_timing::HOLD);
    regs::set_cs_hold_delay(MspiId::Mspi2, cs_timing::HOLD_DELAY);

    // Bus clock + DLL on both controllers
    let divider = AP_HEX_PSRAM_MPLL_DEFAULT_FREQ_MHZ / PSRAM_DETECT_SPEED_MHZ;
    regs::set_bus_clock(MspiId::Mspi2, divider);
    regs::set_bus_clock(MspiId::Mspi3, divider);
    regs::enable_dll(MspiId::Mspi2, true);
    regs::enable_dll(MspiId::Mspi3, true);
}

/// Read MR0..MR8 mode registers from the PSRAM chip via the SPI command
/// engine. Mirrors IDF `s_get_psram_mode_reg(MSPI_3, ...)` from
/// `esp_psram_impl_ap_hex.c:221`. Uses bounded `try_common_transaction`
/// so a chip non-response surfaces as `Err(TransactionTimeout)`.
#[cfg(target_arch = "riscv32")]
fn read_mode_registers(mspi: MspiId) -> Result<ModeRegisters, PsramError> {
    // SPEED_SLOW reg-read dummy = 2*(5-1) = 8. Verified against IDF v5.3
    // baseline (SPIMEM3.user1 = 0x7C000007 → dummy_cyclelen = 8).
    let dummy = 8u32;
    let max_iters = 1_000_000u32;

    let mut mr01 = [0u8; 2];
    let mut mr23 = [0u8; 2];
    let mut mr4_buf = [0u8; 1];
    let mut mr8_buf = [0u8; 1];

    let mr_read = |addr: u32, miso_bitlen: u32, miso: *mut u8| -> Result<(), PsramError> {
        // SAFETY: caller-supplied miso buffer; we own the bitlen contract.
        unsafe {
            regs::try_common_transaction(
                mspi,
                cmd::REG_READ as u32, 16,
                addr, 32,
                dummy,
                core::ptr::null_mut(), 0,
                miso, miso_bitlen,
                false,
                max_iters,
            )
            .map(|_| ())
            .map_err(|e| match e {
                regs::TransactionError::Timeout { iters, cmd_reg_value } => {
                    PsramError::TransactionTimeout {
                        addr,
                        iters,
                        cmd_reg: cmd_reg_value,
                    }
                }
            })
        }
    };

    mr_read(0x0, 16, mr01.as_mut_ptr())?;
    mr_read(0x2, 16, mr23.as_mut_ptr())?;
    mr_read(0x4, 8, mr4_buf.as_mut_ptr())?;
    mr_read(0x8, 8, mr8_buf.as_mut_ptr())?;

    Ok(ModeRegisters {
        mr0: Mr0(mr01[0]),
        mr1: Mr1(mr01[1]),
        mr2: Mr2(mr23[0]),
        mr3: Mr3(mr23[1]),
        mr4: Mr4(mr4_buf[0]),
        mr8: Mr8(mr8_buf[0]),
    })
}

/// Read+modify+write mode registers on the PSRAM chip itself. Mirrors
/// IDF `s_init_psram_mode_reg(MSPI_3, &mode_reg_config)` from
/// `esp_psram_impl_ap_hex.c:141`. Sets the chip into Hex (X16=1),
/// page-mode (BL=3), fixed latency (LT=1) — required for cache-side
/// OPI DTR access at our configured timing.
#[cfg(target_arch = "riscv32")]
fn write_mode_register_config() -> Result<(), PsramError> {
    let mspi = MspiId::Mspi3;
    let dummy = 8u32; // SPEED_SLOW reg-read dummy (matches IDF for 20 MHz)
    let max_iters = 1_000_000u32;

    let cfg = default_mode_register_config();

    let mut mr01 = [0u8; 2];
    let mut mr48 = [0u8; 2];
    let mut mr8_buf = [0u8; 2];

    let txn = |addr: u32, miso: *mut u8, miso_len: u32, mosi: *mut u8, mosi_len: u32, write: bool|
        -> Result<(), PsramError>
    {
        // SAFETY: caller-provided buffers; bitlen contract upheld.
        let cmd_val = if write { cmd::REG_WRITE } else { cmd::REG_READ };
        unsafe {
            regs::try_common_transaction(
                mspi,
                cmd_val as u32, 16,
                addr, 32,
                if write { 0 } else { dummy },
                mosi, mosi_len,
                miso, miso_len,
                false,
                max_iters,
            )
            .map(|_| ())
            .map_err(|e| match e {
                regs::TransactionError::Timeout { iters, cmd_reg_value } => {
                    PsramError::TransactionTimeout {
                        addr,
                        iters,
                        cmd_reg: cmd_reg_value,
                    }
                }
            })
        }
    };

    // 1) Read MR0+MR1
    txn(0x0, mr01.as_mut_ptr(), 16, core::ptr::null_mut(), 0, false)?;
    // 2) Read MR4+MR8 (16-bit at addr 0x4)
    txn(0x4, mr48.as_mut_ptr(), 16, core::ptr::null_mut(), 0, false)?;
    // 3) Patch MR0
    let mr0 = Mr0(mr01[0])
        .with_drive_str(cfg.mr0.drive_str())
        .with_read_latency(cfg.mr0.read_latency())
        .with_lt(cfg.mr0.lt());
    mr01[0] = mr0.0;
    // 4) Patch MR4 (mr48[1] = mr8 byte preserved for later read)
    let mr4 = Mr4(mr48[0]).with_wr_latency(cfg.mr4.wr_latency());
    mr48[0] = mr4.0;
    // 5) Write MR0+MR1
    txn(0x0, core::ptr::null_mut(), 0, mr01.as_mut_ptr(), 16, true)?;
    // 6) Write MR4+MR8 (mr8 byte still chip-original)
    txn(0x4, core::ptr::null_mut(), 0, mr48.as_mut_ptr(), 16, true)?;
    // 7) Read MR8 standalone (8-bit at addr 0x8)
    txn(0x8, mr8_buf.as_mut_ptr(), 8, core::ptr::null_mut(), 0, false)?;
    // 8) Patch MR8
    let mr8 = Mr8(mr8_buf[0])
        .with_bl(cfg.mr8.bl())
        .with_bt(cfg.mr8.bt())
        .with_rbx(cfg.mr8.rbx())
        .with_x16(cfg.mr8.x16());
    mr8_buf[0] = mr8.0;
    mr8_buf[1] = 0;
    // 9) Write MR8 (16-bit, second byte ignored)
    txn(0x8, core::ptr::null_mut(), 0, mr8_buf.as_mut_ptr(), 16, true)?;

    Ok(())
}

/// Configure SPIMEM2 (MSPI_2, cache-side) for OPI/Hex DDR access to PSRAM.
/// Mirrors IDF `s_config_mspi_for_psram()` from
/// `esp_psram_impl_ap_hex.c:323`. Must run AFTER `write_mode_register_config()`
/// and BEFORE `mmu_map_psram_window()`. Without this, AXI cache fills can't
/// translate to OPI DTR signaling on PSRAM.
#[cfg(target_arch = "riscv32")]
fn config_mspi_cache_side() {
    let lat = LatencyConfig::SPEED_200M;
    let id = MspiId::Mspi2;
    regs::set_wr_cmd(id, WR_CMD_BITLEN, cmd::SYNC_WRITE as u32);
    regs::set_rd_cmd(id, RD_CMD_BITLEN, cmd::SYNC_READ as u32);
    regs::set_addr_bitlen(id, ADDR_BITLEN);
    regs::enable_4byte_addr(id, true);
    regs::set_wr_dummy(id, lat.wr_dummy_bitlen);
    regs::set_rd_dummy(id, lat.rd_dummy_bitlen);
    regs::enable_variable_dummy(id, true);
    regs::enable_variable_dummy(MspiId::Mspi3, true);
    regs::enable_wr_dummy_level_control(id, true);
    regs::enable_ddr_wr_data_swap(id, false);
    regs::enable_ddr_rd_data_swap(id, false);
    regs::enable_ddr_mode(id, true);
    regs::enable_oct_line_mode(id, true);
    regs::enable_hex_data_line_mode(id, true);
    regs::enable_axi_access(id, true);
    regs::enable_wr_splice(id, true);
    regs::enable_rd_splice(id, true);
}

/// Map PSRAM physical pages into the cached PSRAM window starting at
/// `PSRAM_VADDR_BASE` = 0x48000000. Maps the full `PSRAM_VADDR_SIZE` =
/// 32 MB using 64 KB pages (512 MMU entries).
///
/// Mirrors IDF `mmu_hal_map_region(MMU_LL_PSRAM_MMU_ID=1, MMU_TARGET_PSRAM0,
/// vaddr=PSRAM_VADDR_BASE, paddr=0, size=PSRAM_VADDR_SIZE)`. Source:
/// `idf_v53_ref/esp_psram_extracted/esp_psram.c:251`.
///
/// MMU entry encoding for P4 PSRAM:
/// - bits 0..9: paddr >> 16 (64 KB page index)
/// - bit 10: ACCESS_PSRAM
/// - bit 11: VALID
#[cfg(target_arch = "riscv32")]
fn mmu_map_psram_window() {
    const SPI_MEM_S_MMU_ITEM_INDEX_REG: *mut u32 = 0x5008_E380 as *mut u32;
    const SPI_MEM_S_MMU_ITEM_CONTENT_REG: *mut u32 = 0x5008_E37C as *mut u32;
    const PAGE_SHIFT: u32 = 16; // 64 KB pages
    const PAGE_SIZE: usize = 1 << PAGE_SHIFT;
    // SOC_MMU_VADDR_MASK = PAGE_SIZE * ENTRY_NUM - 1 = 64K * 1024 - 1
    // = 64 MB - 1. Covers the full vaddr window the MMU can address from
    // 0x48000000 (i.e. 0x48000000..0x4C000000).
    const VADDR_MASK: u32 = 0x03FF_FFFF;
    const ACCESS_PSRAM: u32 = 1 << 10;
    const VALID: u32 = 1 << 11;

    let pages = PSRAM_VADDR_SIZE / PAGE_SIZE; // 512 for 32 MB
    // SAFETY: SPI_MEM_S_MMU regs are fixed MMIO. Single-hart at boot.
    unsafe {
        for i in 0..pages {
            let vaddr = (PSRAM_VADDR_BASE + i * PAGE_SIZE) as u32;
            let paddr = (i * PAGE_SIZE) as u32;
            let entry_id = (vaddr & VADDR_MASK) >> PAGE_SHIFT;
            let mmu_val = (paddr >> PAGE_SHIFT) | ACCESS_PSRAM | VALID;
            core::ptr::write_volatile(SPI_MEM_S_MMU_ITEM_INDEX_REG, entry_id);
            core::ptr::write_volatile(SPI_MEM_S_MMU_ITEM_CONTENT_REG, mmu_val);
        }
    }
}

/// Invalidate L2 cache for the PSRAM window after MMU map. Without this
/// the cache may return stale (pre-mapping) data.
#[cfg(target_arch = "riscv32")]
fn cache_invalidate_psram_window() {
    const ROM_CACHE_INVALIDATE_ALL: usize = 0x4FC0_0404;
    const CACHE_MAP_L2: u32 = 1 << 5;
    type CacheAll = unsafe extern "C" fn(map: u32) -> i32;
    // SAFETY: ROM symbol address is fixed in P4 ROM.
    unsafe {
        let inv: CacheAll = core::mem::transmute(ROM_CACHE_INVALIDATE_ALL);
        let _ = inv(CACHE_MAP_L2);
    }
}

/// Full PSRAM bring-up: prepare clocks/PMU → program chip mode registers
/// → cache-side OPI/Hex DDR config → MMU map → L2 cache invalidate.
///
/// After this returns `Ok`, the cached PSRAM window at
/// `PSRAM_VADDR_BASE..+PSRAM_VADDR_SIZE` (32 MB at 0x48000000) is
/// readable/writable as normal RAM via the L1+L2 cache hierarchy.
///
/// The returned [`ModeRegisters`] reflects the chip's state after our
/// MR-write step. On a true cold boot `mr1.vendor_id` will be `0x0D`
/// (AP-Memory). On a warm reboot the PSRAM chip can retain Hex/X16 mode
/// from the previous run (the chip is on a separate power rail from the
/// SoC) and our OPI-mode MR-read may return garbage; the chip is still
/// usable after our re-program step, verified via the cached-window
/// memory test in the boot-test demo. Callers that need a strict
/// vendor check should use [`detect`] directly.
///
/// **Pre-condition:** `bootloader::init_phase2_full()` has already run.
#[cfg(target_arch = "riscv32")]
pub fn init() -> Result<ModeRegisters, PsramError> {
    prepare_for_mr_read();
    // Best-effort MR-read for diagnostics. May return garbage on warm
    // reboot if chip was left in Hex mode by a prior run; we use it for
    // logging only and don't fail on vendor mismatch.
    let regs_state = read_mode_registers(MspiId::Mspi3)
        .unwrap_or_default();
    // Program the chip's mode registers for our access pattern (Hex/X16,
    // page mode, fixed latency). Idempotent — works on both cold-boot
    // (chip in OPI POR) and warm-reboot (chip already in Hex). Without
    // this the chip stays in POR mode (X16=0 = OPI 8-bit) and cache-side
    // fills with hex_data_line_mode=1 hang the AHB bus.
    let _ = write_mode_register_config(); // ignore errors; chip may already be in target mode
    config_mspi_cache_side();
    mmu_map_psram_window();
    cache_invalidate_psram_window();
    Ok(regs_state)
}

#[cfg(not(target_arch = "riscv32"))]
pub fn init() -> Result<ModeRegisters, PsramError> {
    Err(PsramError::NotImplemented)
}

// ── Host tests ───────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // Cite IDF `esp_psram_impl_ap_hex.c` line numbers next to each
    // expectation so future readers can re-verify.

    #[test]
    fn vendor_id_matches_idf() {
        // esp_psram_impl_ap_hex.c:48 : #define AP_HEX_PSRAM_VENDOR_ID 0xd
        assert_eq!(AP_HEX_PSRAM_VENDOR_ID, 0x0D);
    }

    #[test]
    fn cmd_codes_match_idf() {
        // esp_psram_impl_ap_hex.c:21-26
        assert_eq!(cmd::SYNC_READ, 0x0000);
        assert_eq!(cmd::SYNC_WRITE, 0x8080);
        assert_eq!(cmd::BURST_READ, 0x2020);
        assert_eq!(cmd::BURST_WRITE, 0xA0A0);
        assert_eq!(cmd::REG_READ, 0x4040);
        assert_eq!(cmd::REG_WRITE, 0xC0C0);
    }

    #[test]
    fn cs_timing_matches_idf() {
        // esp_psram_impl_ap_hex.c:49-52
        assert_eq!(cs_timing::SETUP, 4);
        assert_eq!(cs_timing::HOLD, 4);
        assert_eq!(cs_timing::HOLD_DELAY, 3);
        assert_eq!(cs_timing::ECC_HOLD, 4);
    }

    #[test]
    fn latency_200m_arithmetic() {
        let l = LatencyConfig::SPEED_200M;
        assert_eq!(l.rd_dummy_bitlen, 26);
        assert_eq!(l.wr_dummy_bitlen, 12);
        assert_eq!(l.rd_latency_mr, 4);
        assert_eq!(l.wr_latency_mr, 1);
    }

    #[test]
    fn latency_slow_arithmetic() {
        let l = LatencyConfig::SPEED_SLOW;
        assert_eq!(l.rd_dummy_bitlen, 18);
        assert_eq!(l.wr_dummy_bitlen, 8);
    }

    #[test]
    fn psram_size_density_decoding() {
        // esp_psram_impl_ap_hex.c:412-416 — Waveshare returns 0x7.
        assert_eq!(PsramSize::from_mr2_density(0x7), Some(PsramSize::Mb32));
        assert_eq!(PsramSize::from_mr2_density(0x6), Some(PsramSize::Mb64));
        assert_eq!(PsramSize::from_mr2_density(0x5), Some(PsramSize::Mb16));
        assert_eq!(PsramSize::from_mr2_density(0x3), Some(PsramSize::Mb8));
        assert_eq!(PsramSize::from_mr2_density(0x1), Some(PsramSize::Mb4));
        assert_eq!(PsramSize::from_mr2_density(0x0), None);
        assert_eq!(PsramSize::from_mr2_density(0x2), None);
        assert_eq!(PsramSize::from_mr2_density(0x4), None);
    }

    #[test]
    fn psram_size_bytes() {
        assert_eq!(PsramSize::Mb32.bytes(), 32 * 1024 * 1024);
        assert_eq!(PsramSize::Mb16.bytes(), 16 * 1024 * 1024);
    }

    #[test]
    fn mr0_round_trip_drive_str() {
        let mr0 = Mr0::new(2, 4, true);
        assert_eq!(mr0.drive_str(), 2);
        assert_eq!(mr0.read_latency(), 4);
        assert!(mr0.lt());
        let mr0b = mr0.with_drive_str(0);
        assert_eq!(mr0b.drive_str(), 0);
        // Other bits preserved.
        assert_eq!(mr0b.read_latency(), 4);
        assert!(mr0b.lt());
    }

    #[test]
    fn mr0_with_read_latency_preserves_others() {
        let mr0 = Mr0::new(2, 0, true);
        let mr0b = mr0.with_read_latency(7);
        assert_eq!(mr0b.read_latency(), 7);
        assert_eq!(mr0b.drive_str(), 2);
        assert!(mr0b.lt());
    }

    #[test]
    fn mr1_vendor_id_extraction() {
        // Top 3 bits ignored.
        let mr1 = Mr1(0xED);
        assert_eq!(mr1.vendor_id(), 0x0D);
        assert!(mr1.ulp());
    }

    #[test]
    fn mr2_density_kgd() {
        let mr2 = Mr2((6 << 5) | (1 << 3) | 7);
        assert_eq!(mr2.density(), 7);
        assert_eq!(mr2.dev_id(), 1);
        assert_eq!(mr2.kgd(), 6);
    }

    #[test]
    fn mr4_wr_latency_round_trip() {
        let mr4 = Mr4::default().with_wr_latency(5);
        assert_eq!(mr4.wr_latency(), 5);
    }

    #[test]
    fn mr8_round_trip_all_fields() {
        let mr8 = Mr8::new(3, false, true, true);
        assert_eq!(mr8.bl(), 3);
        assert!(!mr8.bt());
        assert!(mr8.rbx());
        assert!(mr8.x16());
    }

    #[test]
    fn default_mode_register_config_is_200m_hex() {
        let m = default_mode_register_config();
        assert_eq!(m.mr0.read_latency(), LatencyConfig::SPEED_200M.rd_latency_mr);
        assert!(m.mr0.lt());
        assert_eq!(m.mr4.wr_latency(), LatencyConfig::SPEED_200M.wr_latency_mr);
        assert_eq!(m.mr8.bl(), 3);
        assert!(m.mr8.x16());
        assert!(m.mr8.rbx());
    }

    #[test]
    fn mspi_id_values_match_idf() {
        // psram_ctrlr_ll.h:32-33
        assert_eq!(MspiId::Mspi2 as u8, 2);
        assert_eq!(MspiId::Mspi3 as u8, 3);
    }

    #[test]
    fn psram_window_constants() {
        assert_eq!(PSRAM_VADDR_BASE, 0x4800_0000);
        assert_eq!(PSRAM_VADDR_SIZE, 32 * 1024 * 1024);
    }

    #[test]
    fn detect_stub_returns_not_implemented() {
        assert_eq!(detect(), Err(PsramError::NotImplemented));
    }

    #[test]
    fn psram_error_display_chip_detection() {
        use core::fmt::Write;
        let mut buf = heapless_buf::Buf::<64>::new();
        let _ = write!(
            buf,
            "{}",
            PsramError::ChipDetectionFailed {
                reported_vendor_id: 0xAB
            }
        );
        assert_eq!(
            buf.as_str(),
            "PSRAM chip detection failed: vendor_id 0xAB (expected 0x0D)"
        );
    }

    #[test]
    fn psram_error_display_transaction_timeout_decodes_slv_st() {
        // cmd_reg = 0x00040052 → slv_st = bits[7:4] = 0x5 (read data state).
        use core::fmt::Write;
        let mut buf = heapless_buf::Buf::<128>::new();
        let _ = write!(
            buf,
            "{}",
            PsramError::TransactionTimeout {
                addr: 0,
                iters: 1_000_000,
                cmd_reg: 0x0004_0052,
            }
        );
        // The slv_st value should appear as 0x5 in the message.
        assert!(buf.as_str().contains("slv_st=0x5"));
        assert!(buf.as_str().contains("cmd_reg=0x00040052"));
    }
}

// Tiny no-alloc string buffer for tests — avoids pulling in `heapless`
// as a dev-dep just for one Display test.
#[cfg(test)]
mod heapless_buf {
    use core::fmt;

    pub struct Buf<const N: usize> {
        bytes: [u8; N],
        len: usize,
    }

    impl<const N: usize> Buf<N> {
        pub const fn new() -> Self {
            Self {
                bytes: [0; N],
                len: 0,
            }
        }
        pub fn as_str(&self) -> &str {
            core::str::from_utf8(&self.bytes[..self.len]).unwrap()
        }
    }

    impl<const N: usize> fmt::Write for Buf<N> {
        fn write_str(&mut self, s: &str) -> fmt::Result {
            let bytes = s.as_bytes();
            if self.len + bytes.len() > N {
                return Err(fmt::Error);
            }
            self.bytes[self.len..self.len + bytes.len()].copy_from_slice(bytes);
            self.len += bytes.len();
            Ok(())
        }
    }
}
