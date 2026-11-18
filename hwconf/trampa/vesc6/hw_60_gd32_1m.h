/*
	Based on hw_60.h, Copyright 2016 - 2020 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * VESC 6 (hw_60, MK1) with the STM32F405RGT6 replaced by a pin compatible
 * GigaDevice GD32F405RGT6 (1 MB flash: 512 KB zero wait state code area +
 * 512 KB data area, GD32F405 datasheet Rev2.8 Table 2-1). Pinout identical
 * to hw_60.
 *
 * Build: make fw_60_gd32_1m
 * The "_gd32_1m" suffix keeps the STM32 flash layout and linker script
 * (ld_eeprom_emu.ld, application in s0-s7 = the code area, stock bootloader
 * location 0x080E0000) and only defines GD32F405_COMPAT (GD32 workarounds).
 *
 * HW_NAME is changed so that VESC Tool does not offer/auto-install a stock
 * STM32 build without the GD32 workarounds, nor a 3 MB GD32 build
 * ("60_GD32", which needs a different bootloader).
 */

#ifndef HW_60_GD32_1M_H_
#define HW_60_GD32_1M_H_

#if !defined(GD32F405_COMPAT) || defined(GD32F405_FLASH_3M) || defined(GD32F405_NO_HSE)
#error "hw_60_gd32_1m.h must be built with GD32F405_COMPAT only (use: make fw_60_gd32_1m)"
#endif

#define HW60_IS_MK1

#include "hw_60_core.h"

#undef HW_NAME
#define HW_NAME					"60_GD32_1M"

#endif /* HW_60_GD32_1M_H_ */
