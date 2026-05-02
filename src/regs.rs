//! Register-HAL port of `hal/esp32p4/include/hal/psram_ctrlr_ll.h` (IDF v5.4).
//!
//! Each public function mirrors a `psram_ctrlr_ll_*` inline 1:1, taking the
//! same arguments in the same order. Bodies poke either:
//!
//! - the PSRAM-side `SPI_MEM_S` / `SPI1_MEM_S` register banks (IDF's
//!   `SPIMEM2` / `SPIMEM3`), accessed through esp32p4 PAC register-block
//!   types pointed at the PSRAM_MSPI{0,1} bases — see [`spimem2`] /
//!   [`spimem3`];
//! - `HP_SYS_CLKRST` for clock & reset control;
//! - the ROM SPI command driver (`esp_rom_spi_cmd_config/start/set_op_mode`)
//!   for mode-register transactions.
//!
//! Ported clean — no `iter1-9` debug instrumentation. The `bootloader` crate
//! supplies REGI2C/MPLL/pin_mux/flash bring-up.

#![allow(unsafe_code)]

use super::MspiId;
use core::ptr;

// ── Constants ────────────────────────────────────────────────────────────────

/// `DR_REG_PSRAM_MSPI0_BASE` from `soc/reg_base.h`. IDF `SPIMEM2`. Follows
/// the same register layout as PAC `spi0::RegisterBlock`.
pub const SPIMEM2_BASE: usize = 0x5008_E000;

/// `DR_REG_PSRAM_MSPI1_BASE` from `soc/reg_base.h`. IDF `SPIMEM3`. Follows
/// the same register layout as PAC `spi1::RegisterBlock`.
pub const SPIMEM3_BASE: usize = 0x5008_F000;

/// `psram_ctrlr_ll.h:35-37` — number of permission-mask regions and the
/// per-region attribute bits.
pub const PMS_REGION_NUMS: usize = 4;
pub const PMS_ATTR_WRITABLE: u32 = 1 << 0;
pub const PMS_ATTR_READABLE: u32 = 1 << 1;

/// `psram_ctrlr_ll.h:39` — max payload bytes per common-transaction.
pub const FIFO_MAX_BYTES: usize = 64;

/// Bitwise-OR of `SPI_MEM_S_*_AFIFO_REMPTY` + `SPI_MEM_S_ALL_FIFO_EMPTY`
/// status bits in `SMEM_AXI_ADDR_CTRL`. When the register == this value,
/// all pending PSRAM transactions have drained.
/// See `psram_ctrlr_ll.h:759-764` (`ALL_TRANSACTION_DONE` macro).
pub const ALL_TRANSACTION_DONE: u32 = 0xFC00_0000;

// ── ROM symbol addresses (esp32p4.rom.ld v5.4) ───────────────────────────────

/// `void esp_rom_spi_cmd_config(int spi_num, esp_rom_spi_cmd_t *pcmd);`
const ROM_SPI_CMD_CONFIG: usize = 0x4FC0_0108;
/// `void esp_rom_spi_cmd_start(int, uint8_t*, uint16_t, uint8_t, bool);`
const ROM_SPI_CMD_START: usize = 0x4FC0_010C;
/// `void esp_rom_spi_set_op_mode(int spi_num, esp_rom_spiflash_read_mode_t);`
const ROM_SPI_SET_OP_MODE: usize = 0x4FC0_0110;

type RomSpiCmdConfig = unsafe extern "C" fn(spi_num: i32, pcmd: *mut EspRomSpiCmd);
type RomSpiCmdStart = unsafe extern "C" fn(
    spi_num: i32,
    rx_buf: *mut u8,
    rx_len: u16,
    cs_en_mask: u8,
    is_write_erase: bool,
);
type RomSpiSetOpMode = unsafe extern "C" fn(spi_num: i32, mode: u32);

// ── Types mirrored from IDF / ROM ────────────────────────────────────────────

/// `esp_rom_spi_cmd_t` from `rom/opi_flash.h`. Layout must match the IDF C
/// struct exactly — the ROM driver reads it via the pointer.
#[repr(C)]
pub struct EspRomSpiCmd {
    pub cmd: u16,
    pub cmd_bit_len: u16,
    pub addr: *mut u32,
    pub addr_bit_len: u32,
    pub tx_data: *mut u32,
    pub tx_data_bit_len: u32,
    pub rx_data: *mut u32,
    pub rx_data_bit_len: u32,
    pub dummy_bit_len: u32,
}

/// `esp_rom_spiflash_read_mode_t` from `esp_rom_spiflash.h`. Only the
/// values we use are listed; raw discriminants match IDF order.
#[repr(u32)]
pub enum SpiFlashReadMode {
    OpiDtrMode = 7,
}

/// `soc_periph_psram_clk_src_t` from `clk_tree_defs.h`. Decoded in
/// `psram_ctrlr_ll_select_clk_source` (psram_ctrlr_ll.h:329-352).
#[repr(u32)]
#[derive(Copy, Clone)]
pub enum PsramClkSrc {
    Xtal = 0,
    Mpll = 1,
    Spll = 2,
    Cpll = 3,
}

// ── Peripheral handles ───────────────────────────────────────────────────────

/// PAC register-block reference at the PSRAM_MSPI0 base. IDF's `SPIMEM2`.
#[inline(always)]
fn spimem2() -> &'static esp32p4::spi0::RegisterBlock {
    // SAFETY: SPIMEM2_BASE is a fixed MMIO region; the PAC `spi0` layout
    // describes the bank used by both PSRAM_MSPI0 and FLASH_SPI0.
    unsafe { &*(SPIMEM2_BASE as *const _) }
}

/// PAC register-block reference at the PSRAM_MSPI1 base. IDF's `SPIMEM3`.
#[inline(always)]
fn spimem3() -> &'static esp32p4::spi1::RegisterBlock {
    // SAFETY: same as spimem2.
    unsafe { &*(SPIMEM3_BASE as *const _) }
}

/// PAC handle for `HP_SYS_CLKRST` clock/reset registers.
#[inline(always)]
fn clkrst() -> &'static esp32p4::hp_sys_clkrst::RegisterBlock {
    // SAFETY: HP_SYS_CLKRST::PTR is the PAC-provided MMIO base.
    unsafe { &*esp32p4::HP_SYS_CLKRST::PTR }
}

// ── psram_ctrlr_ll.h ports ───────────────────────────────────────────────────
//
// Most inlines ignore `mspi_id` (they always poke SPIMEM2 fields). We keep
// the parameter in our signatures so call sites read identically to IDF.

/// `psram_ctrlr_ll_set_wr_cmd` (psram_ctrlr_ll.h:50)
#[inline(always)]
pub fn set_wr_cmd(_mspi: MspiId, cmd_bitlen: u32, cmd_val: u32) {
    let s = spimem2();
    s.cache_sctrl().modify(|_, w| w.cache_sram_usr_wcmd().set_bit());
    s.sram_dwr_cmd().modify(|_, w| {
        // SAFETY: bitlen 4-bit, value 16-bit; both fit after IDF's `n-1`.
        unsafe {
            w.cache_sram_usr_wr_cmd_bitlen().bits((cmd_bitlen - 1) as u8);
            w.cache_sram_usr_wr_cmd_value().bits(cmd_val as u16)
        }
    });
}

/// `psram_ctrlr_ll_set_rd_cmd` (psram_ctrlr_ll.h:67)
#[inline(always)]
pub fn set_rd_cmd(_mspi: MspiId, cmd_bitlen: u32, cmd_val: u32) {
    let s = spimem2();
    s.cache_sctrl().modify(|_, w| w.cache_sram_usr_rcmd().set_bit());
    s.sram_drd_cmd().modify(|_, w| {
        // SAFETY: same bit widths as set_wr_cmd.
        unsafe {
            w.cache_sram_usr_rd_cmd_bitlen().bits((cmd_bitlen - 1) as u8);
            w.cache_sram_usr_rd_cmd_value().bits(cmd_val as u16)
        }
    });
}

/// `psram_ctrlr_ll_set_addr_bitlen` (psram_ctrlr_ll.h:83)
#[inline(always)]
pub fn set_addr_bitlen(_mspi: MspiId, addr_bitlen: u32) {
    spimem2().cache_sctrl().modify(|_, w| {
        // SAFETY: 6-bit field; IDF callers pass ≤32.
        unsafe { w.sram_addr_bitlen().bits((addr_bitlen - 1) as u8) }
    });
}

/// `psram_ctrlr_ll_enable_4byte_addr` (psram_ctrlr_ll.h:97)
#[inline(always)]
pub fn enable_4byte_addr(_mspi: MspiId, en: bool) {
    spimem2()
        .cache_sctrl()
        .modify(|_, w| w.cache_usr_saddr_4byte().bit(en));
}

/// `psram_ctrlr_ll_set_wr_dummy` (psram_ctrlr_ll.h:110)
#[inline(always)]
pub fn set_wr_dummy(_mspi: MspiId, dummy_n: u32) {
    spimem2().cache_sctrl().modify(|_, w| {
        w.usr_wr_sram_dummy().set_bit();
        // SAFETY: 6-bit field.
        unsafe { w.sram_wdummy_cyclelen().bits((dummy_n - 1) as u8) }
    });
}

/// `psram_ctrlr_ll_set_rd_dummy` (psram_ctrlr_ll.h:125)
#[inline(always)]
pub fn set_rd_dummy(_mspi: MspiId, dummy_n: u32) {
    spimem2().cache_sctrl().modify(|_, w| {
        w.usr_rd_sram_dummy().set_bit();
        // SAFETY: 6-bit field.
        unsafe { w.sram_rdummy_cyclelen().bits((dummy_n - 1) as u8) }
    });
}

/// `psram_ctrlr_ll_enable_variable_dummy` (psram_ctrlr_ll.h:140)
///
/// Branches on `mspi_id` — MSPI_2 writes `SPIMEM2.smem_ddr.smem_var_dummy`,
/// MSPI_3 writes `SPIMEM3.ddr.fmem_var_dummy`.
#[inline(always)]
pub fn enable_variable_dummy(mspi: MspiId, en: bool) {
    match mspi {
        MspiId::Mspi2 => {
            spimem2()
                .spi_smem_ddr()
                .modify(|_, w| w.spi_smem_var_dummy().bit(en));
        }
        MspiId::Mspi3 => {
            spimem3()
                .ddr()
                .modify(|_, w| w.spi_fmem_var_dummy().bit(en));
        }
    }
}

/// `psram_ctrlr_ll_enable_wr_dummy_level_control` (psram_ctrlr_ll.h:156)
#[inline(always)]
pub fn enable_wr_dummy_level_control(_mspi: MspiId, en: bool) {
    spimem2().sram_cmd().modify(|_, w| w.sdummy_wout().bit(en));
}

/// `psram_ctrlr_ll_enable_rd_dummy_level_control` (psram_ctrlr_ll.h:169)
#[inline(always)]
pub fn enable_rd_dummy_level_control(_mspi: MspiId, en: bool) {
    spimem2().sram_cmd().modify(|_, w| w.sdummy_rin().bit(en));
}

/// `psram_ctrlr_ll_enable_ddr_mode` (psram_ctrlr_ll.h:182)
#[inline(always)]
pub fn enable_ddr_mode(_mspi: MspiId, en: bool) {
    // IDF field is `smem_ddr_en`; PAC drops the prefix → `en()`.
    spimem2().spi_smem_ddr().modify(|_, w| w.en().bit(en));
}

/// `psram_ctrlr_ll_enable_ddr_wr_data_swap` (psram_ctrlr_ll.h:195)
#[inline(always)]
pub fn enable_ddr_wr_data_swap(_mspi: MspiId, en: bool) {
    spimem2().spi_smem_ddr().modify(|_, w| w.wdat_swp().bit(en));
}

/// `psram_ctrlr_ll_enable_ddr_rd_data_swap` (psram_ctrlr_ll.h:208)
#[inline(always)]
pub fn enable_ddr_rd_data_swap(_mspi: MspiId, en: bool) {
    spimem2().spi_smem_ddr().modify(|_, w| w.rdat_swp().bit(en));
}

/// `psram_ctrlr_ll_enable_oct_line_mode` (psram_ctrlr_ll.h:221)
///
/// Sets one bit on `cache_sctrl` (`sram_oct`) and four on `sram_cmd`.
#[inline(always)]
pub fn enable_oct_line_mode(_mspi: MspiId, en: bool) {
    let s = spimem2();
    s.cache_sctrl().modify(|_, w| w.sram_oct().bit(en));
    s.sram_cmd().modify(|_, w| {
        w.scmd_oct().bit(en);
        w.saddr_oct().bit(en);
        w.sdout_oct().bit(en);
        w.sdin_oct().bit(en)
    });
}

/// `psram_ctrlr_ll_enable_hex_data_line_mode` (psram_ctrlr_ll.h:238)
#[inline(always)]
pub fn enable_hex_data_line_mode(_mspi: MspiId, en: bool) {
    spimem2().sram_cmd().modify(|_, w| {
        w.sdin_hex().bit(en);
        w.sdout_hex().bit(en)
    });
}

/// `psram_ctrlr_ll_enable_axi_access` (psram_ctrlr_ll.h:252)
#[inline(always)]
pub fn enable_axi_access(_mspi: MspiId, en: bool) {
    spimem2().cache_fctrl().modify(|_, w| {
        w.axi_req_en().bit(en);
        // IDF: `close_axi_inf_en = !en`.
        w.spi_close_axi_inf_en().bit(!en)
    });
}

/// `psram_ctrlr_ll_enable_wr_splice` (psram_ctrlr_ll.h:266)
#[inline(always)]
pub fn enable_wr_splice(_mspi: MspiId, en: bool) {
    spimem2().ctrl1().modify(|_, w| w.aw_splice_en().bit(en));
}

/// `psram_ctrlr_ll_enable_rd_splice` (psram_ctrlr_ll.h:279)
#[inline(always)]
pub fn enable_rd_splice(_mspi: MspiId, en: bool) {
    spimem2().ctrl1().modify(|_, w| w.ar_splice_en().bit(en));
}

// ── Clocks / resets (HP_SYS_CLKRST) ──────────────────────────────────────────

/// `_psram_ctrlr_ll_enable_module_clock` (psram_ctrlr_ll.h:292)
#[inline(always)]
pub fn enable_module_clock(_mspi: MspiId, en: bool) {
    let r = clkrst();
    r.soc_clk_ctrl0().modify(|_, w| w.psram_sys_clk_en().bit(en));
    r.peri_clk_ctrl00().modify(|_, w| w.psram_pll_clk_en().bit(en));
}

/// `psram_ctrlr_ll_reset_module_clock` (psram_ctrlr_ll.h:309). Pulses
/// AXI and APB resets — high then low.
#[inline(always)]
pub fn reset_module_clock(_mspi: MspiId) {
    let r = clkrst();
    r.hp_rst_en0()
        .modify(|_, w| w.rst_en_dual_mspi_axi().set_bit());
    r.hp_rst_en0()
        .modify(|_, w| w.rst_en_dual_mspi_axi().clear_bit());
    r.hp_rst_en0()
        .modify(|_, w| w.rst_en_dual_mspi_apb().set_bit());
    r.hp_rst_en0()
        .modify(|_, w| w.rst_en_dual_mspi_apb().clear_bit());
}

/// `_psram_ctrlr_ll_select_clk_source` (psram_ctrlr_ll.h:329)
#[inline(always)]
pub fn select_clk_source(_mspi: MspiId, clk_src: PsramClkSrc) {
    clkrst().peri_clk_ctrl00().modify(|_, w| {
        // SAFETY: 2-bit field; PsramClkSrc discriminants ∈ {0,1,2,3}.
        unsafe { w.psram_clk_src_sel().bits(clk_src as u8) }
    });
}

/// `_psram_ctrlr_ll_set_core_clock_div` (psram_ctrlr_ll.h:365)
#[inline(always)]
pub fn set_core_clock_div(_spi_num: u8, freqdiv: u32) {
    clkrst().peri_clk_ctrl00().modify(|_, w| {
        // SAFETY: 8-bit field.
        unsafe { w.psram_core_clk_div_num().bits((freqdiv - 1) as u8) }
    });
}

/// `_psram_ctrlr_ll_enable_core_clock` (psram_ctrlr_ll.h:380)
#[inline(always)]
pub fn enable_core_clock(_spi_num: u8, en: bool) {
    clkrst()
        .peri_clk_ctrl00()
        .modify(|_, w| w.psram_core_clk_en().bit(en));
}

// ── Bus clock (raw register write — IDF uses WRITE_PERI_REG) ─────────────────

/// `psram_ctrlr_ll_set_bus_clock` (psram_ctrlr_ll.h:396).
///
/// IDF writes the entire `SPI_MEM_S_SRAM_CLK_REG` / `SPI1_MEM_S_CLOCK_REG`
/// in one shot — it doesn't read-modify-write because all bits are part of
/// the clock divider triplet. `freqdiv == 1` writes the `CLK_EQU_SYSCLK`
/// bit. Otherwise the value packs `(freqdiv - 1)` into N (bits 16..23),
/// `H` (bits 8..15), `L` (bits 0..7).
#[inline(always)]
pub fn set_bus_clock(mspi: MspiId, freqdiv: u32) {
    let reg = match mspi {
        // SPI_MEM_S_SRAM_CLK_REG = PSRAM_MSPI0_BASE + 0x50.
        MspiId::Mspi2 => (SPIMEM2_BASE + 0x50) as *mut u32,
        // SPI1_MEM_S_CLOCK_REG = PSRAM_MSPI1_BASE + 0x14.
        MspiId::Mspi3 => (SPIMEM3_BASE + 0x14) as *mut u32,
    };
    let val = if freqdiv == 1 {
        1u32 << 31 // SCLK_EQU_SYSCLK
    } else {
        let n = freqdiv - 1;
        let h = freqdiv / 2 - 1;
        let l = freqdiv - 1;
        (n << 16) | (h << 8) | l
    };
    // SAFETY: target is fixed MMIO; single 32-bit aligned store matches IDF.
    unsafe { ptr::write_volatile(reg, val) }
}

/// `psram_ctrlr_ll_enable_dll` (psram_ctrlr_ll.h:422).
///
/// Both paths write into PSRAM_MSPI0 (= SPIMEM2): MSPI_2 →
/// `smem_timing_cali.smem_dll_timing_cali`, MSPI_3 →
/// `mem_timing_cali.mem_dll_timing_cali`.
#[inline(always)]
pub fn enable_dll(mspi: MspiId, en: bool) {
    let s = spimem2();
    match mspi {
        MspiId::Mspi2 => {
            s.spi_smem_timing_cali()
                .modify(|_, w| w.spi_smem_dll_timing_cali().bit(en));
        }
        MspiId::Mspi3 => {
            s.timing_cali().modify(|_, w| w.dll_timing_cali().bit(en));
        }
    }
}

// ── CS timing (psram_ctrlr_ll.h:440-489) ─────────────────────────────────────

/// `psram_ctrlr_ll_set_cs_setup`
#[inline(always)]
pub fn set_cs_setup(_mspi: MspiId, setup_n: u32) {
    spimem2().spi_smem_ac().modify(|_, w| {
        w.spi_smem_cs_setup().set_bit();
        // SAFETY: 5-bit field.
        unsafe { w.spi_smem_cs_setup_time().bits((setup_n - 1) as u8) }
    });
}

/// `psram_ctrlr_ll_set_cs_hold`
#[inline(always)]
pub fn set_cs_hold(_mspi: MspiId, hold_n: u32) {
    spimem2().spi_smem_ac().modify(|_, w| {
        w.spi_smem_cs_hold().set_bit();
        // SAFETY: 5-bit field.
        unsafe { w.spi_smem_cs_hold_time().bits((hold_n - 1) as u8) }
    });
}

/// `psram_ctrlr_ll_set_cs_hold_delay`
#[inline(always)]
pub fn set_cs_hold_delay(_mspi: MspiId, hold_delay_n: u32) {
    spimem2().spi_smem_ac().modify(|_, w| {
        // SAFETY: 6-bit field.
        unsafe { w.spi_smem_cs_hold_delay().bits((hold_delay_n - 1) as u8) }
    });
}

/// `psram_ctrlr_ll_set_cs_hold_ecc`
#[inline(always)]
pub fn set_cs_hold_ecc(_mspi: MspiId, hold_n: u32) {
    spimem2().spi_smem_ac().modify(|_, w| {
        // SAFETY: 3-bit field.
        unsafe { w.spi_smem_ecc_cs_hold_time().bits((hold_n - 1) as u8) }
    });
}

// ── ECC / split-trans / page-size (psram_ctrlr_ll.h:498-603) ─────────────────

/// `psram_ctrlr_ll_enable_split_trans`
#[inline(always)]
pub fn enable_split_trans(_mspi: MspiId, en: bool) {
    spimem2()
        .spi_smem_ac()
        .modify(|_, w| w.spi_smem_split_trans_en().bit(en));
}

/// `psram_ctrlr_ll_enable_ecc_addr_conversion`
#[inline(always)]
pub fn enable_ecc_addr_conversion(_mspi: MspiId, en: bool) {
    spimem2()
        .spi_smem_ecc_ctrl()
        .modify(|_, w| w.spi_smem_ecc_addr_en().bit(en));
}

/// `psram_ctrlr_ll_set_page_size`
#[inline(always)]
pub fn set_page_size(_mspi: MspiId, page_size: u32) {
    let bits = match page_size {
        256 => 0u8,
        512 => 1,
        1024 => 2,
        2048 => 3,
        _ => return, // IDF asserts; we no-op until callers are typed.
    };
    spimem2().spi_smem_ecc_ctrl().modify(|_, w| {
        // SAFETY: 2-bit field; `bits` ∈ 0..=3 by construction.
        unsafe { w.spi_smem_page_size().bits(bits) }
    });
}

/// `psram_ctrlr_ll_get_page_size`
#[inline(always)]
pub fn get_page_size(_mspi: MspiId) -> u32 {
    match spimem2()
        .spi_smem_ecc_ctrl()
        .read()
        .spi_smem_page_size()
        .bits()
    {
        0 => 256,
        1 => 512,
        2 => 1024,
        3 => 2048,
        _ => 0,
    }
}

// ── PMS regions (psram_ctrlr_ll.h:613-698) ───────────────────────────────────

/// `psram_ctrlr_ll_enable_pms_region_ecc`
#[inline(always)]
pub fn enable_pms_region_ecc(_mspi: MspiId, region_id: usize, en: bool) {
    debug_assert!(region_id < PMS_REGION_NUMS);
    spimem2()
        .spi_smem_pms_attr(region_id)
        .modify(|_, w| w.spi_smem_pms_ecc().bit(en));
}

/// `psram_ctrlr_ll_set_pms_region_attr`
#[inline(always)]
pub fn set_pms_region_attr(_mspi: MspiId, region_id: usize, attr_mask: u32) {
    debug_assert!(region_id < PMS_REGION_NUMS);
    spimem2().spi_smem_pms_attr(region_id).modify(|_, w| {
        w.spi_smem_pms_wr_attr()
            .bit(attr_mask & PMS_ATTR_WRITABLE != 0);
        w.spi_smem_pms_rd_attr()
            .bit(attr_mask & PMS_ATTR_READABLE != 0)
    });
}

/// `psram_ctrlr_ll_set_pms_region_start_addr`
#[inline(always)]
pub fn set_pms_region_start_addr(_mspi: MspiId, region_id: usize, addr: u32) {
    debug_assert!(region_id < PMS_REGION_NUMS);
    spimem2().spi_smem_pms_addr(region_id).modify(|_, w| {
        // SAFETY: 27-bit field; addresses are page-aligned.
        unsafe { w.s().bits(addr) }
    });
}

/// `psram_ctrlr_ll_set_pms_region_size`
#[inline(always)]
pub fn set_pms_region_size(_mspi: MspiId, region_id: usize, size: u32) {
    debug_assert!(region_id < PMS_REGION_NUMS);
    spimem2().spi_smem_pms_size(region_id).modify(|_, w| {
        // SAFETY: 15-bit field.
        unsafe { w.spi_smem_pms_size().bits(size as u16) }
    });
}

/// `psram_ctrlr_ll_get_pms_region_start_addr`
#[inline(always)]
pub fn get_pms_region_start_addr(_mspi: MspiId, region_id: usize) -> u32 {
    debug_assert!(region_id < PMS_REGION_NUMS);
    spimem2().spi_smem_pms_addr(region_id).read().s().bits()
}

/// `psram_ctrlr_ll_get_pms_region_size`
#[inline(always)]
pub fn get_pms_region_size(_mspi: MspiId, region_id: usize) -> u32 {
    debug_assert!(region_id < PMS_REGION_NUMS);
    spimem2()
        .spi_smem_pms_size(region_id)
        .read()
        .spi_smem_pms_size()
        .bits() as u32
}

// ── Common transaction (ROM) ─────────────────────────────────────────────────

/// `psram_ctrlr_ll_common_transaction` (psram_ctrlr_ll.h:737) →
/// `psram_ctrlr_ll_common_transaction_base` (psram_ctrlr_ll.h:706).
///
/// Drives the ROM SPI command engine. `mode` is hardcoded to `OPI_DTR`
/// and `cs_mask = 1<<1` (CS1) per IDF.
///
/// **Known issue (memory note `project_smartbox_bootloader.md`):** in
/// the previous `firmware/` port this hung when called with `mspi =
/// MspiId::Mspi3` for MR-read. ROM's `cmd_start` polls `SPI_USR` forever
/// and never returns. Day-4 debug iteration is pending. For Phase C
/// `detect()` we may want to use `MspiId::Mspi2` first and verify.
///
/// # Safety
///
/// `mosi_data` and `miso_data` must point to valid buffers of at least
/// `mosi_bitlen / 8` and `miso_bitlen / 8` bytes respectively. Caller must
/// guarantee the buffers remain valid for the duration of the ROM call.
#[allow(clippy::too_many_arguments)]
pub unsafe fn common_transaction(
    mspi: MspiId,
    cmd: u32,
    cmd_bitlen: u32,
    addr: u32,
    addr_bitlen: u32,
    dummy_bits: u32,
    mosi_data: *mut u8,
    mosi_bitlen: u32,
    miso_data: *mut u8,
    miso_bitlen: u32,
    is_write_erase: bool,
) {
    let spi_num = mspi as i32;

    // 1. Set op mode = OPI_DTR (mode=7).
    let set_mode: RomSpiSetOpMode = core::mem::transmute(ROM_SPI_SET_OP_MODE);
    set_mode(spi_num, SpiFlashReadMode::OpiDtrMode as u32);

    // 2. Build the cmd descriptor on the stack and configure the engine.
    let mut addr_local: u32 = addr;
    let mut cmd_struct = EspRomSpiCmd {
        cmd: cmd as u16,
        cmd_bit_len: cmd_bitlen as u16,
        addr: &mut addr_local as *mut u32,
        addr_bit_len: addr_bitlen,
        tx_data: mosi_data as *mut u32,
        tx_data_bit_len: mosi_bitlen,
        rx_data: miso_data as *mut u32,
        rx_data_bit_len: miso_bitlen,
        dummy_bit_len: dummy_bits,
    };
    let cmd_config: RomSpiCmdConfig = core::mem::transmute(ROM_SPI_CMD_CONFIG);
    cmd_config(spi_num, &mut cmd_struct as *mut EspRomSpiCmd);

    // 3. Kick off — `cs_mask = 1 << 1 = 2` (CS1) per IDF.
    let cmd_start: RomSpiCmdStart = core::mem::transmute(ROM_SPI_CMD_START);
    cmd_start(
        spi_num,
        miso_data,
        (miso_bitlen / 8) as u16,
        1 << 1,
        is_write_erase,
    );
}

/// Bounded variant of [`common_transaction`] for hardware bring-up. Same
/// ROM `set_op_mode` + `cmd_config` setup, but replaces ROM `cmd_start`
/// (which polls forever) with manual USR-kick + bounded poll. On timeout
/// the engine is left in whatever state it reached so the caller can dump
/// SPIMEM registers.
///
/// USR is bit 18 of `SPI_MEM_S_CMD_REG` at offset 0x00. MISO data lives
/// in W0..W15 starting at base + 0x58 (16 × u32 = 64 bytes max).
///
/// # Safety
///
/// Same contract as [`common_transaction`].
#[allow(clippy::too_many_arguments)]
pub unsafe fn try_common_transaction(
    mspi: MspiId,
    cmd: u32,
    cmd_bitlen: u32,
    addr: u32,
    addr_bitlen: u32,
    dummy_bits: u32,
    mosi_data: *mut u8,
    mosi_bitlen: u32,
    miso_data: *mut u8,
    miso_bitlen: u32,
    is_write_erase: bool,
    max_iters: u32,
) -> Result<u32, TransactionError> {
    let _ = is_write_erase;
    let spi_num = mspi as i32;
    let base = match mspi {
        MspiId::Mspi2 => SPIMEM2_BASE,
        MspiId::Mspi3 => SPIMEM3_BASE,
    };

    // 1. Set OPI_DTR mode (ROM).
    let set_mode: RomSpiSetOpMode = core::mem::transmute(ROM_SPI_SET_OP_MODE);
    set_mode(spi_num, SpiFlashReadMode::OpiDtrMode as u32);

    // 2. Configure cmd descriptor (ROM — only writes registers, doesn't kick).
    let mut addr_local: u32 = addr;
    let mut cmd_struct = EspRomSpiCmd {
        cmd: cmd as u16,
        cmd_bit_len: cmd_bitlen as u16,
        addr: &mut addr_local as *mut u32,
        addr_bit_len: addr_bitlen,
        tx_data: mosi_data as *mut u32,
        tx_data_bit_len: mosi_bitlen,
        rx_data: miso_data as *mut u32,
        rx_data_bit_len: miso_bitlen,
        dummy_bit_len: dummy_bits,
    };
    let cmd_config: RomSpiCmdConfig = core::mem::transmute(ROM_SPI_CMD_CONFIG);
    cmd_config(spi_num, &mut cmd_struct as *mut EspRomSpiCmd);

    // 3. Set MISC.cs_en_mask for CS1 (PSRAM lives on CS1, flash on CS0).
    //    Bit 0 = CS0_DIS, Bit 1 = CS1_DIS (1 = disabled, 0 = active).
    //    Target: CS0_DIS=1, CS1_DIS=0.
    let misc_reg = (base + 0x34) as *mut u32;
    let cur_misc = ptr::read_volatile(misc_reg);
    let new_misc = (cur_misc | 0x1) & !0x2;
    if new_misc != cur_misc {
        ptr::write_volatile(misc_reg, new_misc);
    }

    // 4. Manually kick USR.
    let cmd_reg = base as *mut u32;
    ptr::write_volatile(cmd_reg, 1u32 << 18);

    // 4. Poll bounded.
    let mut iters: u32 = 0;
    loop {
        let v = ptr::read_volatile(cmd_reg as *const u32);
        if v & (1u32 << 18) == 0 {
            break;
        }
        iters = iters.wrapping_add(1);
        if iters >= max_iters {
            return Err(TransactionError::Timeout {
                iters,
                cmd_reg_value: v,
            });
        }
        core::hint::spin_loop();
    }

    // 5. Extract MISO bytes from W0..W{N}. W0 lives at base + 0x58.
    if !miso_data.is_null() && miso_bitlen > 0 {
        let n_bytes = ((miso_bitlen + 7) / 8) as usize;
        let n_words = (n_bytes + 3) / 4;
        let w_base = (base + 0x58) as *const u32;
        for i in 0..n_words {
            let word = ptr::read_volatile(w_base.add(i));
            for b in 0..4 {
                let off = i * 4 + b;
                if off >= n_bytes {
                    break;
                }
                ptr::write(miso_data.add(off), ((word >> (b * 8)) & 0xFF) as u8);
            }
        }
    }
    Ok(iters)
}

/// Result of [`try_common_transaction`] when USR doesn't clear in time.
#[derive(Debug, Copy, Clone)]
pub enum TransactionError {
    Timeout { iters: u32, cmd_reg_value: u32 },
}

/// `psram_ctrlr_ll_wait_all_transaction_done` (psram_ctrlr_ll.h:757).
///
/// Polls `SPI_MEM_S_SMEM_AXI_ADDR_CTRL_REG` (PSRAM_MSPI0 + 0x178) until
/// all six FIFO/AFIFO empty bits (26..31) are set. Read-only register.
#[inline(always)]
pub fn wait_all_transaction_done() {
    let reg = (SPIMEM2_BASE + 0x178) as *const u32;
    // SAFETY: read-only MMIO; volatile prevents the compiler from hoisting.
    while unsafe { ptr::read_volatile(reg) } != ALL_TRANSACTION_DONE {
        core::hint::spin_loop();
    }
}
