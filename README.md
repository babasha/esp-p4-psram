# esp-p4-psram

Octal/Hex PSRAM bring-up for the ESP32-P4 — pure Rust, `no_std`. Drop-in
replacement for the `CONFIG_SPIRAM_BOOT_INIT` portion of the IDF v5.3
bootloader.

> **Status (2026-05-02):** hardware-validated end-to-end on Waveshare
> ESP32-P4-ETH (AP-Memory APS6408L, 32 MB Hex, vendor=0x0D). Wide-span smoke
> test passes across the full 32 MB chip via the cache-side AXI window.

## Why this exists

ESP32-P4 PSRAM bring-up is one of the most fragile parts of chip init —
analog regulator tuning, DLL lock timing, separate MSPI_2 (cache-side) and
MSPI_3 (config-side) controllers, mode-register writes that depend on
exact controller register state, all conspire to make this a many-day
debug if you don't have a working baseline.

This crate is the working baseline, hand-translated from IDF v5.4's
`esp_psram_impl_ap_hex.c` and `psram_ctrlr_ll.h`. The
[two-day debug log][debug-log] for getting MR-reads to come back is in the
project history if you're interested in why the EXT_LDO and MR-write
sequence look the way they do.

[debug-log]: https://github.com/babasha/esp-p4-psram/wiki

## Quick start

```toml
[dependencies]
bootloader = { package = "esp-p4-bootloader", version = "0.1" }
psram      = { package = "esp-p4-psram",      version = "0.1" }
```

```rust
#![no_std]
#![no_main]

#[entry]
fn main() -> ! {
    // PSRAM init must run AFTER the chip-wide bring-up (cache + MMU + PMU
    // + MSPI pins must already be ready). See `esp-p4-bootloader`.
    bootloader::init_phase2_full();

    match psram::init() {
        Ok(regs) => {
            // PSRAM is mapped at 0x4800_0000 .. 0x4A00_0000 (32 MB virtual
            // window via cache + MMU). Reads/writes go through L1+L2 cache.
            let p = 0x4800_0000 as *mut u32;
            unsafe { p.write_volatile(0xDEAD_BEEF) };
            assert_eq!(unsafe { p.read_volatile() }, 0xDEAD_BEEF);
            // ...
        }
        Err(e) => {
            // Chip continues to run from HP SRAM if PSRAM is missing /
            // misconfigured. PSRAM-resident code/data won't be available.
        }
    }

    loop {}
}
```

## What it does, in order

1. **Module clocks**: `set_core_clock_div / enable_core_clock / enable_module_clock /
   reset_module_clock / select_clk_source` on MSPI_2 + MSPI_3.
2. **CS timing** on MSPI_2: setup=4, hold=4, hold_delay=3.
3. **Bus clock** 20 MHz on both MSPI_2 + MSPI_3 + DLL enable.
4. **MSPI_2 cache-side config**: WR/RD cmd, addr_bitlen=32, dummy=12/26,
   DDR mode, OCT/Hex line, AXI access, SPLICE.
5. **MR-read on MSPI_3** (best-effort vendor verify, tolerant of warm-reboot
   PSRAM Hex-mode retention).
6. **MR-write on MSPI_3** to program the chip into Hex mode (X16 = 1, BL = 3,
   LT = 1) — required because chip POR mode regs have X16=0 (8-bit OPI) but
   the cache-side controller is configured for 16-bit Hex.
7. **MMU map** — 512 entries × 64 KB = 32 MB at virtual 0x4800_0000.
8. **L2 invalidate** for the PSRAM window.

## Hardware caveats — read before debugging

- **`wdt_flashboot_mod_en`** must be cleared before this runs — see
  `esp-p4-bootloader::wdt::disable_all`.
- **`PMU_EXT_LDO_P1_0P1A_ANA = 0x57000000`** must be programmed BEFORE this
  runs — without IDF's tuned LDO voltage the MSPI PHY can't sample MISO and
  every MR-read times out forever. `bootloader::pmu::init_active_state`
  handles it.
- **PSRAM smoke tests must span > L1 D-cache.** A 4-word write at
  `0x48000000+0..12` falsely passes even with PSRAM offline because all
  4 words sit in one L1 cache line and `Cache_Invalidate_All(L2)` doesn't
  flush L1. Always test with offsets ≥ 256 KB apart.
- **Unmapped P4 MSPI slave returns `0x00000000`**, not `0xFFFFFFFF`. PSRAM-
  offline reads return zeros, not all-ones.
- **PSRAM Hex-mode retention across CPU reset.** PSRAM chip is on a separate
  power rail from the SoC; warm reboot of the SoC leaves the PSRAM in Hex
  mode while our cmd_config defaults match POR (8-bit OPI). `psram::init()`
  is tolerant — best-effort MR-read for diagnostics, always re-runs MR-write
  (idempotent in correct state).

## Validation

```
psram::init() OK — vendor=0x0D density=0x7 (32 MB) kgd=0x6
smoke: writing 5 patterns across 32 MB
  OK @ 0x48000000 = 0xDEADBEEF
  OK @ 0x48040000 = 0xCAFEBABE  (256 KB)
  OK @ 0x48100000 = 0x12345678  (1 MB)
  OK @ 0x49000000 = 0xA5A55A5A  (16 MB)
  OK @ 0x49F00000 = 0xF00DCAFE  (31 MB)
smoke: PASS
```

## License

Dual-licensed under [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE).
