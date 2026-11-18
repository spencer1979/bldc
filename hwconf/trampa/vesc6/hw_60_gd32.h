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
 * GigaDevice GD32F405RKT6 (3 MB flash). Pinout identical to hw_60.
 *
 * Build: make fw_60_gd32
 * The "_gd32" suffix makes the build system select ld_gd32f405_3m.ld and
 * define GD32F405_COMPAT and GD32F405_FLASH_3M. To make a GD32 variant of another F405 board, copy
 * this file, include that board's header and change HW_NAME.
 *
 * HW_NAME is changed on purpose so VESC Tool does not offer/auto-install a
 * stock STM32 build (with the STM32 bootloader address 0x080E0000, which is
 * application code in the GD32 layout).
 */

#ifndef HW_60_GD32_H_
#define HW_60_GD32_H_

#if !defined(GD32F405_COMPAT) || !defined(GD32F405_FLASH_3M) || defined(GD32F405_NO_HSE)
#error "hw_60_gd32.h must be built with GD32F405_COMPAT and GD32F405_FLASH_3M (use: make fw_60_gd32)"
#endif

#define HW60_IS_MK1

#include "hw_60_core.h"

#undef HW_NAME
#define HW_NAME					"60_GD32"

#endif /* HW_60_GD32_H_ */
