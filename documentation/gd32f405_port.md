# VESC on GigaDevice GD32F405RKT6 (3 MB flash)

Experimental port for boards where the STM32F405RGT6 is replaced by the pin
compatible GD32F405RKT6 to get more flash. **Nothing here has been verified on
hardware yet.**

## Building

```
make fw_60_gd32          # VESC 6 (hw_60) pinout on a GD32F405RKT6
```

Any board name that ends in `_gd32` is built for the GD32: the top level
`Makefile` passes `MCU_GD32F405=1` to `make/fw.mk`, which selects
`ld_gd32f405_3m.ld` and defines `GD32F405_COMPAT`. All GD32 specific code is
guarded by `GD32F405_COMPAT`, so the STM32 targets build exactly as before.
To add another board: copy `hwconf/trampa/vesc6/hw_60_gd32.h`, include the
other board's header and change `HW_NAME`.

## Flash map

GD32F405xK: only the first 512 KB is the zero-wait *code area*; the rest is
the *data area* (executable, but slow). Bank0 (1 MB) has the STM32F405 sector
layout, bank1 starts at 0x08100000.

| Address    | Sectors | Size  | Use                                  | Speed |
|------------|---------|-------|--------------------------------------|-------|
| 0x08000000 | 0       | 16K   | vectors + `.text2`                   | fast  |
| 0x08004000 | 1-2     | 32K   | EEPROM emulation (config)            | fast  |
| 0x0800C000 | 3-7     | 464K  | app: real-time code (`flash2`)       | fast  |
| 0x08080000 | 8-11    | 512K  | app: non real-time code/data (`.text_slow`) | slow |
| 0x080FFFF0 | 11      | 8B    | CRC flag + CRC (end of app image)    | slow  |
| 0x08100000 | 12-15   | 64K   | bootloader (GD32 build)              | slow  |
| 0x08110000 | 16      | 64K   | reserved                             | slow  |
| 0x08120000 | 17      | 128K  | VESC_IF / package NVM                | slow  |
| 0x08140000 | 18      | 128K  | QML UI                               | slow  |
| 0x08160000 | 19      | 128K  | LispBM code                          | slow  |
| 0x08180000 | 20      | 128K  | LispBM const heap / image            | slow  |
| 0x081A0000 | 21-23   | 384K  | free                                 | slow  |
| 0x08200000 | 24-27   | 1M    | new firmware staging area            | slow  |

The application image is up to 1 MB (`.bin` is always 1048568 bytes because
the CRC info sits at 0x080FFFF0). Compare STM32F405: 512 KB image, of which
464 KB for code.

GD32 erase sector numbers (FMC_CTL SN) are not linear: s12-s23 use SN 16-27
(same as the StdPeriph `FLASH_Sector_12..23`), s24-s27 use SN 12-15
(`GD32_FLASH_Sector_24..27` in `flash_helper.c`).

## What goes where

`ld_gd32f405_3m.ld` puts the following into the slow area: terminal,
confgenerator, BMS, QML UI, Blackmagic SWD programmer, UAVCAN/libcanard, the
LispBM VESC glue + extension libraries + reader/printer/image code, NRF/LoRa
drivers and newlib printf/scanf/strtod/dtoa. Everything else (ChibiOS, HAL,
`motor/`, `encoder/`, `hwconf/`, IRQ handlers, `util/`, `comm/`,
`applications/`, `imu/`, LispBM evaluator core, libgcc, libc mem/str
functions) stays in the fast area. More can be moved with
`__attribute__((section(".text_slow")))` / `".rodata_slow"` or by object name
in the linker script.

Executing the hot code from RAM is not done: the GD32 code area is already
zero-wait (it is served from internal SRAM after the boot copy, which is also
why the GD32 takes ~140 ms from reset to the first instruction).

`documentation/gd32_isr_check.py <elf> <toolchain-prefix>` walks all direct
calls reachable from the interrupt vectors and lists any function outside the
fast area (function-pointer calls are not followed).

## Bootloader / firmware update

The stock bldc-bootloader (0x080E0000, staging 0x08080000, 384 KB, app sectors
0-7) does **not** work with this layout. A matching bootloader is required:

* lives at 0x08100000 (s12), VTOR = 0x08100000
* reads the new image from 0x08200000 (s24), max 1 MB - 6 (raw) or 393208
  bytes (heatshrink compressed, VESC Tool limit)
* erases/writes app sectors 0-11
* switches from the PLL to IRC16M safely on entry

It is a small patch to `vedderb/bldc-bootloader`
(`make GD32=1 build_args='-DHW_VER=60'`). Flash it once with SWD.

VESC Tool notes:

* Firmware upload: the GD32 `.bin` is >700000 bytes, so VESC Tool sends it
  uncompressed (1 MB). It fits the 1 MB staging area (1048568 + 6 bytes).
* Bootloader upload from VESC Tool is refused (`COMM_ERASE_BOOTLOADER` returns
  an error) because VESC Tool hardcodes the bootloader offset (staging + 384 KB)
  for the STM32 layout.
* `HW_NAME` is `60_GD32`, so VESC Tool does not offer stock STM32 builds.
  A stock STM32 build must never be installed on a GD32 running this layout,
  because it jumps to the bootloader at 0x080E0000.

## Other GD32 workarounds (all guarded by GD32F405_COMPAT)

* USB OTG FS (`ChibiOS_3.0.5/.../OTGv1/usb_lld.c`): FIFOs are sized before
  they are flushed in `usb_lld_reset()`, and `otg_core_reset()` waits
  `GD32_OTG_CORE_RESET_DELAY_CYCLES` (default 25M cycles, ~150 ms) after the
  PHY power up (ChibiOS forum topic 6008). The delay runs once in
  `usbStart()` with the kernel locked, before the IWDG is started.
* Clocks (`ChibiOS_3.0.5/.../STM32F4xx/hal_lld.c`, `flash_helper.c`): HCLK is
  divided by 4 and a delay is inserted before switching SYSCLK from the PLL to
  HSI (GD32 migration note 5.2).
* HSE startup: ChibiOS waits for HSERDY without a timeout, flash wait states
  are left at the STM32 value (5 WS @ 168 MHz), which is safe on the GD32.
