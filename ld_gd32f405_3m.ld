/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio.

    This file is part of ChibiOS.

    ChibiOS is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    (at your option) any later version.

    ChibiOS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/*
 * GigaDevice GD32F405RKT6 (3072 KB flash) memory setup for VESC.
 *
 * Selected with a board name ending in "_gd32" (e.g. make fw_60_gd32), which
 * sets MCU_GD32F405=1 in make/fw.mk. The STM32 targets keep using
 * ld_eeprom_emu.ld and are not affected by this file.
 *
 * GD32F405xK flash facts (GD32F4xx user manual / datasheet):
 *  - Only the first 512 KB (0x08000000 - 0x0807FFFF) is the zero-wait
 *    "code area". Everything above is the "data area": executable, but every
 *    fetch that misses has a long wait.
 *  - Bank0 = first 1 MB, same sector layout as STM32F405 (sectors 0..11).
 *  - Bank1 = 0x08100000 - 0x082FFFFF: 4x16K (s12-15), 64K (s16),
 *    7x128K (s17-23), 4x256K (s24-27).
 *
 * Full VESC GD32 flash map (see flash_helper.c and
 * documentation/gd32f405_port.md):
 *
 *  0x08000000 s0       16K  vectors + .text2              FAST
 *  0x08004000 s1-s2    32K  EEPROM emulation (config)     FAST
 *  0x0800C000 s3-s7   464K  application, real-time code   FAST  (flash2)
 *  0x08080000 s8-s11  512K  application, non-RT code/data SLOW  (flash_slow)
 *  0x080FFFF0          8B   CRC flag + CRC                SLOW  (crcinfo)
 *  ---- bank1 (not part of the app image, managed by flash_helper.c) ----
 *  0x08100000 s12-s15  64K  bootloader (GD32 build of bldc-bootloader)
 *  0x08110000 s16      64K  reserved
 *  0x08120000 s17     128K  VESC_IF / package NVM
 *  0x08140000 s18     128K  QML UI
 *  0x08160000 s19     128K  LispBM code
 *  0x08180000 s20     128K  LispBM const heap / image
 *  0x081A0000 s21-s23 384K  free
 *  0x08200000 s24-s27   1M  new-app staging area (firmware update)
 *
 * Note: the CRC info (8 bytes) sits at the very end of
 * the 1 MB application region, so the app image (.bin) is up to 1 MB - 8.
 *
 * Note: Use of ram1 and ram2 is mutually exclusive with use of ram0.
 */
MEMORY
{
    flash      : org = 0x08000000, len = 16k
    flash2     : org = 0x0800C000, len = 512k - 48k            /* FAST: s3..s7 */
    flash_slow : org = 0x08080000, len = 512k - 16             /* SLOW: s8..s11 - CRC_INFO */
    crcinfo    : org = 0x080FFFF0, len = 8                     /* CRC info */
    ram0  : org = 0x20000000, len = 128k    /* SRAM1 + SRAM2 */
    ram1  : org = 0x20000000, len = 112k    /* SRAM1 */
    ram2  : org = 0x2001C000, len = 16k     /* SRAM2 */
    ram3  : org = 0x00000000, len = 0
    ram4  : org = 0x10000000, len = 62k     /* CCM SRAM */
    libif : org = 0x1000F800, len = 2k
    ram5  : org = 0x40024000, len = 4k      /* BCKP SRAM */
    ram6  : org = 0x00000000, len = 0
    ram7  : org = 0x00000000, len = 0
}

/* RAM region to be used for Main stack. This stack accommodates the processing
   of all exceptions and interrupts*/
REGION_ALIAS("MAIN_STACK_RAM", ram0);

/* RAM region to be used for the process stack. This is the stack used by
   the main() function.*/
REGION_ALIAS("PROCESS_STACK_RAM", ram0);

/* RAM region to be used for data segment.*/
REGION_ALIAS("DATA_RAM", ram0);

/* RAM region to be used for BSS segment.*/
REGION_ALIAS("BSS_RAM", ram0);

__ram0_start__          = ORIGIN(ram0);
__ram0_size__           = LENGTH(ram0);
__ram0_end__            = __ram0_start__ + __ram0_size__;
__ram1_start__          = ORIGIN(ram1);
__ram1_size__           = LENGTH(ram1);
__ram1_end__            = __ram1_start__ + __ram1_size__;
__ram2_start__          = ORIGIN(ram2);
__ram2_size__           = LENGTH(ram2);
__ram2_end__            = __ram2_start__ + __ram2_size__;
__ram3_start__          = ORIGIN(ram3);
__ram3_size__           = LENGTH(ram3);
__ram3_end__            = __ram3_start__ + __ram3_size__;
__ram4_start__          = ORIGIN(ram4);
__ram4_size__           = LENGTH(ram4);
__ram4_end__            = __ram4_start__ + __ram4_size__;
__ram5_start__          = ORIGIN(ram5);
__ram5_size__           = LENGTH(ram5);
__ram5_end__            = __ram5_start__ + __ram5_size__;
__ram6_start__          = ORIGIN(ram6);
__ram6_size__           = LENGTH(ram6);
__ram6_end__            = __ram6_start__ + __ram6_size__;
__ram7_start__          = ORIGIN(ram7);
__ram7_size__           = LENGTH(ram7);
__ram7_end__            = __ram7_start__ + __ram7_size__;

ENTRY(Reset_Handler)

SECTIONS
{
    . = 0;
    _text = .;

    startup : ALIGN(16) SUBALIGN(16)
    {
        KEEP(*(.vectors))
    } > flash

    constructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__init_array_start = .);
        KEEP(*(SORT(.init_array.*)))
        KEEP(*(.init_array))
        PROVIDE(__init_array_end = .);
    } > flash2

    destructors : ALIGN(4) SUBALIGN(4)
    {
        PROVIDE(__fini_array_start = .);
        KEEP(*(.fini_array))
        KEEP(*(SORT(.fini_array.*)))
        PROVIDE(__fini_array_end = .);
    } > flash2

    /*
     * Code and read-only data that is NOT on the real-time path goes to the
     * slow GD32 data area (s8..s11). This statement must come before .text so
     * that these input sections are claimed here first. Everything not listed
     * (ChibiOS kernel/HAL/port, motor/ (mcpwm_foc, foc_math, mc_interface,
     * virtual motor...), encoder/, hwconf/, irq_handlers, timeout, util/,
     * comm/, applications/, imu/, the LispBM evaluator core, libgcc and the
     * libc string/memory functions) stays in the fast 512 KB code area.
     *
     * Add more with __attribute__((section(".text_slow"))) /
     * __attribute__((section(".rodata_slow"))) or by object name below.
     */
    .text_slow : ALIGN(16)
    {
        __text_slow_start__ = .;
        *(.text_slow .text_slow.*)
        *(.rodata_slow .rodata_slow.*)

        /* Terminal, configuration (de)serialization, BMS */
        */obj/terminal.o(.text .text.* .rodata .rodata.*)
        */obj/confgenerator.o(.text .text.* .rodata .rodata.*)
        */obj/bms.o(.text .text.* .rodata .rodata.*)

        /* QML UI blobs */
        */obj/qmlui.o(.text .text.* .rodata .rodata.*)

        /* Blackmagic SWD programmer */
        */obj/bm_if.o(.text .text.* .rodata .rodata.*)
        */obj/swdptap.o(.text .text.* .rodata .rodata.*)
        */obj/swdptap_generic.o(.text .text.* .rodata .rodata.*)
        */obj/timing.o(.text .text.* .rodata .rodata.*)
        */obj/platform.o(.text .text.* .rodata .rodata.*)
        */obj/exception.o(.text .text.* .rodata .rodata.*)
        */obj/target.o(.text .text.* .rodata .rodata.*)
        */obj/adiv5_swdp.o(.text .text.* .rodata .rodata.*)
        */obj/adiv5.o(.text .text.* .rodata .rodata.*)
        */obj/cortexm.o(.text .text.* .rodata .rodata.*)
        */obj/nrf51.o(.text .text.* .rodata .rodata.*)
        */obj/stm32f1.o(.text .text.* .rodata .rodata.*)
        */obj/stm32f4.o(.text .text.* .rodata .rodata.*)
        */obj/stm32h7.o(.text .text.* .rodata .rodata.*)
        */obj/stm32l0.o(.text .text.* .rodata .rodata.*)
        */obj/stm32l4.o(.text .text.* .rodata .rodata.*)

        /* UAVCAN (libcanard) */
        */obj/canard.o(.text .text.* .rodata .rodata.*)
        */obj/canard_driver.o(.text .text.* .rodata .rodata.*)
        */obj/esc_*.o(.text .text.* .rodata .rodata.*)
        */obj/protocol_*.o(.text .text.* .rodata .rodata.*)
        */obj/param_*.o(.text .text.* .rodata .rodata.*)
        */obj/file_*.o(.text .text.* .rodata .rodata.*)
        */obj/vesc_RTData.o(.text .text.* .rodata .rodata.*)

        /* LispBM VESC glue, extension libraries, reader/printer, image */
        */obj/lispif.o(.text .text.* .rodata .rodata.*)
        */obj/lispif_vesc_extensions.o(.text .text.* .rodata .rodata.*)
        */obj/lispif_c_lib.o(.text .text.* .rodata .rodata.*)
        */obj/lbm_vesc_utils.o(.text .text.* .rodata .rodata.*)
        */obj/array_extensions.o(.text .text.* .rodata .rodata.*)
        */obj/math_extensions.o(.text .text.* .rodata .rodata.*)
        */obj/string_extensions.o(.text .text.* .rodata .rodata.*)
        */obj/mutex_extensions.o(.text .text.* .rodata .rodata.*)
        */obj/lbm_dyn_lib.o(.text .text.* .rodata .rodata.*)
        */obj/tokpar.o(.text .text.* .rodata .rodata.*)
        */obj/print.o(.text .text.* .rodata .rodata.*)
        */obj/lbm_image.o(.text .text.* .rodata .rodata.*)
        */obj/lbm_flat_value.o(.text .text.* .rodata .rodata.*)
        */obj/lbm_prof.o(.text .text.* .rodata .rodata.*)

        /* NRF / LoRa remote drivers */
        */obj/nrf_driver.o(.text .text.* .rodata .rodata.*)
        */obj/rfhelp.o(.text .text.* .rodata .rodata.*)
        */obj/rf.o(.text .text.* .rodata .rodata.*)
        */obj/SX1278.o(.text .text.* .rodata .rodata.*)
        */obj/SX1278_hw.o(.text .text.* .rodata .rodata.*)

        /* newlib formatted I/O and float<->string conversion */
        *libg_nano.a:libc_a-nano-*printf*.o(.text .text.* .rodata .rodata.*)
        *libg_nano.a:libc_a-nano-*scanf*.o(.text .text.* .rodata .rodata.*)
        *libg_nano.a:libc_a-*printf.o(.text .text.* .rodata .rodata.*)
        *libg_nano.a:libc_a-sscanf.o(.text .text.* .rodata .rodata.*)
        *libg_nano.a:libc_a-strtod.o(.text .text.* .rodata .rodata.*)
        *libg_nano.a:libc_a-dtoa.o(.text .text.* .rodata .rodata.*)
        *libg_nano.a:libc_a-mprec.o(.text .text.* .rodata .rodata.*)
        *libg_nano.a:libc_a-gdtoa-*.o(.text .text.* .rodata .rodata.*)
        . = ALIGN(16);
        __text_slow_end__ = .;
    } > flash_slow

    .text : ALIGN(16)
    {
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
        *(.glue_7t)
        *(.glue_7)
        *(.gcc*)
    } > flash2

    .text2 : ALIGN(16)
    {
        *(.text2)
        *(.text2.*)
    } > flash

    .ARM.extab :
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > flash2

    .ARM.exidx : {
        PROVIDE(__exidx_start = .);
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        PROVIDE(__exidx_end = .);
     } > flash2

    .eh_frame_hdr :
    {
        *(.eh_frame_hdr)
    } > flash2

    .eh_frame : ONLY_IF_RO
    {
        *(.eh_frame)
    } > flash2
    
    .textalign : ONLY_IF_RO
    {
        . = ALIGN(8);
    } > flash2

    . = ALIGN(4);
    _etext = .;
    _textdata = _etext;
    
    _crcinfo_start_address = 0x080FFFF0;
    
    .crcinfo _crcinfo_start_address :
	{
		KEEP(*(.crcinfo))
	}  > crcinfo

    .mstack :
    {
        . = ALIGN(8);
        __main_stack_base__ = .;
        . += __main_stack_size__;
        . = ALIGN(8);
        __main_stack_end__ = .;
    } > MAIN_STACK_RAM

    .pstack :
    {
        __process_stack_base__ = .;
        __main_thread_stack_base__ = .;
        . += __process_stack_size__;
        . = ALIGN(8);
        __process_stack_end__ = .;
        __main_thread_stack_end__ = .;
    } > PROCESS_STACK_RAM

    .data : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_data = .);
        *(.data)
        *(.data.*)
        *(.ramtext)
        . = ALIGN(4);
        PROVIDE(_edata = .);
    } > DATA_RAM AT > flash2

    .bss : ALIGN(4)
    {
        . = ALIGN(4);
        PROVIDE(_bss_start = .);
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
        PROVIDE(_bss_end = .);
        PROVIDE(end = .);
    } > BSS_RAM

    /* Outside the ranges initialized at boot (data copy and bss zeroing), so
       contents survive a reset. Used for crash diagnostics (crash_info). */
    .noinit (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.noinit)
        *(.noinit.*)
        . = ALIGN(4);
    } > BSS_RAM

    .ram0 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram0)
        *(.ram0.*)
        . = ALIGN(4);
        __ram0_free__ = .;
    } > ram0

    .ram1 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram1)
        *(.ram1.*)
        . = ALIGN(4);
        __ram1_free__ = .;
    } > ram1

    .ram2 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram2)
        *(.ram2.*)
        . = ALIGN(4);
        __ram2_free__ = .;
    } > ram2

    .ram3 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram3)
        *(.ram3.*)
        . = ALIGN(4);
        __ram3_free__ = .;
    } > ram3

    .ram4 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram4)
        *(.ram4.*)
        . = ALIGN(4);
        __ram4_free__ = .;
    } > ram4
    
    .libif (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.libif)
    } > libif

    .ram5 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram5)
        *(.ram5.*)
        . = ALIGN(4);
        __ram5_free__ = .;
    } > ram5

    .ram6 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram6)
        *(.ram6.*)
        . = ALIGN(4);
        __ram6_free__ = .;
    } > ram6

    .ram7 (NOLOAD) : ALIGN(4)
    {
        . = ALIGN(4);
        *(.ram7)
        *(.ram7.*)
        . = ALIGN(4);
        __ram7_free__ = .;
    } > ram7
}

/* Heap default boundaries, it is defaulted to be the non-used part
   of ram0 region.*/
__heap_base__   = __ram0_free__;
__heap_end__    = __ram0_end__;
