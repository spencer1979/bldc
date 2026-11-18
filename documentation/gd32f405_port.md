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

Executing the hot code from RAM is not done: the GD32 user manual describes
the first part of the flash as executing "without waiting time" (UM Rev3.0
2.2). How this is implemented internally (often described as a copy to
SRAM at startup, which would also explain the 130-150 ms startup time in DS
Table 4-5) is not documented by GigaDevice.

Functions in an object that is otherwise placed in the slow area but that can
run in interrupt context are marked with `GD32_FAST_FUNC(name)`
(`conf_general.h`, empty on STM32), which puts them into `.text_fast.*` and
the linker script keeps that in the fast area. Currently: `read_hall_lbm`
(assigned to `utils_read_hall_fun`, called from the FOC ISR) and
`icuwidthcb`/`icuperiodcb` (LispBM ICU callbacks, TIM ISR). `lispif_c_lib.o`
(the VESC_IF wrappers for native libraries, which native libs call from their
PWM/encoder callbacks) is kept in the fast area as a whole.

`documentation/gd32_isr_check.py <elf> <toolchain-prefix> [roots] [-ignore]`
walks the call graph from all interrupt vectors, also following function
addresses loaded from literal pools of reachable code and function pointers
stored in `.data`, and lists any reachable function or literal data reference
in the slow area. Current result:

```
gd32_isr_check.py build/60_gd32/60_gd32.elf <prefix> read_hall_lbm icuwidthcb \
    icuperiodcb -target_destroy_callback -target_printf
roots: 65, reachable functions: 198, slow functions: 0
literal refs to slow-area data from reachable code: 0
```

(`target_*` are Blackmagic callbacks in `.data` that run in a thread.) The
call graph of code inside native libraries is not visible to this check: their
code runs from the LispBM flash area (bank1, slow) anyway, so ISR callbacks
registered by native libraries run slower than on the STM32.

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
* HSE startup: ChibiOS waits for HSERDY without a timeout (same as STM32).
* Flash (`hal_lld.c` in both repos): only WSCNT is written to FMC_WS (=
  FLASH_ACR); PRFTEN/ICEN/DCEN are reserved bits on the GD32.
* ADC (`motor/gd32_adc.c`): ADC clock HCLK/5 = 33.6 MHz instead of 42 MHz
  (max 40 MHz) and ADC calibration after power on.
* Motor type: only FOC (the GD32 ADC has no injected group, which BLDC/DC
  mode needs). `mc_interface.c` forces `MOTOR_TYPE_FOC`, `hw_60_core.c` skips
  the injected channel setup.
* `utils_is_func_valid()` accepts the full 3 MB flash (native libraries are
  loaded from bank1).
* UAVCAN firmware update (`libcanard/canard_driver.c`): staging area
  0x08200000 / 1 MB.

## Hardware audit (GD32F405 vs. STM32F405, hw_60 pinout)

References: GD32F405xx datasheet Rev2.8 (DS), GD32F4xx user manual Rev3.0
and Rev2.8 (UM), GD32F4xx/STM32F4xx migration guide (MIG), ChibiOS forum
topics 5940 and 6008. Status: OK = same behaviour, Handled = different and
handled in code, Fixed = problem found in this audit and fixed, HW = needs
confirmation on hardware.

| Item | Status | Notes / reference |
|---|---|---|
| Pin AF: TIM1 CH1-3 PA8-10 AF1, CH1N-3N PB13-15 AF1 | OK | DS Tables 2-7..2-10 |
| Pin AF: USART3 PB10/11 AF7, I2C2 PB10/11 AF4, CAN1 PB8/9 AF9, OTG FS PA11/12 AF10, TIM3 enc PC6-8 AF2, TIM4 CH1 PB6 AF2, SPI1 PA4-7 AF5, UART4/5 PC11/PC12 AF8 | OK | DS Tables 2-7..2-10 |
| ADC inputs PA0-3, PA5/6, PC0-5 = IN0-3, IN5/6, IN10-15 | OK | DS Table 2-6 |
| LQFP64 pins 31/47 (VCAP on ST) | OK | NC on GD32, any connection allowed (MIG 2.1) |
| Supply 3.3 V | OK | VDD/VDDA 2.6-3.6 V (DS Table 4-2) |
| PLL 8 MHz HSE, M8 N336 P2 Q7 -> 168 MHz, USB 48 MHz | OK | within PLL limits (UM 4.3), fHCLK max 168 MHz (DS) |
| APB1 /4, APB2 /2, timer clocks | OK | TIMERSEL=0 gives the same x2 timer clocks (UM RCU_CFG1) |
| Voltage regulator / over-drive | OK | LDOVS high mode, high-drive not needed at 168 MHz |
| PLL -> HSI switch (bootloader jump, reset) | Handled | AHB /4 + delay (MIG 5.2) |
| FLASH_ACR PRFTEN/ICEN/DCEN | Fixed | reserved bits in FMC_WS (UM 2.4.1), only WSCNT written |
| Flash sectors, SN encoding s12-s27 | OK | UM 2.4.5 FMC_CTL SN |
| EEPROM emulation / write-then-read in code area | HW | read-after-write coherency of the zero wait area not documented |
| TIM1 PWM, dead time, break (CCHP = BDTR) | OK | UM TIMERx_CCHP |
| TIM2 slave reset from TIM1 TRGO (ITI0), TIM8 TRGO = ITI1 | OK | UM TIMER1 trigger selection table |
| ADC regular trigger T2_CC2 / T8_CC1 encodings | OK | UM Table 14-4 |
| ADC clock 42 MHz | Fixed | max 40 MHz (DS Table 4-29, UM 14.4.2), now HCLK/5 = 33.6 MHz; one triple sequence 4.0 us instead of 3.2 us |
| ADC calibration | Fixed | UM 14.4.1, done in `gd32_adc_calibrate_all()` |
| ADC injected group (BLDC/DC mode) | Fixed | not present on GD32F4 (UM 14.7: ADC_CTL1 bits 23:12 reserved, no ISQ/IDATA), FOC only |
| ADC triple regular simultaneous + DMA mode 1, sampling time encoding, Vrefint ch17, TSVREN | OK | UM 14.5, 14.7.14 |
| ADC SWRCST edge (MIG 5.4), 8-bit alignment | OK | not used |
| DMA mappings (ADC1 DMA2 S4 C0, SPI1, SPI3, I2C2) | OK | UM Tables 10-2/10-3 |
| USART + DMA ordering (MIG 5.5) | OK | serial drivers are interrupt driven, UART DMA not used |
| SPI CR1 reconfiguration (MIG 5.3) | OK | ChibiOS clears CR1 first |
| I2C2 with DMA (ChibiOS forum 5940) | HW | only used by app_nunchuk/app_dpv, IMU uses bit-bang I2C |
| CAN1 bit timing register | OK | UM 26.4.8 |
| USB OTG FS FIFO flush order + PHY delay | Handled | ChibiOS forum 6008 |
| IRQ vector numbering | OK | UM Table 6-2 |
| IWDG | HW | IRC32K only typical 32 kHz (DS Table 4-20), no min/max; VESC reload 140 = ~17.6 ms nominal |
| Startup time 130-150 ms | HW | DS Table 4-5 (power up); whether it also applies to software/IWDG reset is not documented |
| PVD level PLS LEV6 | HW | 2.8 V on ST, 3.0 V on GD32 (UM PMU_CTL LVDT); only used for erase voltage range |
| PC13-15 output speed | HW | 2 MHz max (UM GPIO), only relevant for MK5/MK6 variants |
| Unique ID 0x1FFF7A10, flash size 0x1FFF7A22 | OK | UM 1.6.1/1.6.2 |
| SysTick (periodic, no tickless) | OK | CH_CFG_ST_TIMEDELTA = 0 |
| ISR code placement in slow flash | Fixed | `GD32_FAST_FUNC`, `lispif_c_lib.o` fast, check script result 0 |
| `utils_is_func_valid` 1 MB limit | Fixed | native libs in bank1 failed "Invalid stop function" |
| UAVCAN firmware update staging | Fixed | was STM32 sector 8 / 384 KB |
