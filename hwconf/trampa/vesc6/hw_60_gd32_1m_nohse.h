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
 * Crystal-less variant of hw_60_gd32_1m.h (same chip, same flash layout): for
 * boards without HSE/LSE crystal. Build: make fw_60_gd32_1m_nohse
 *
 * The "_nohse" suffix defines GD32F405_NO_HSE: the PLL runs from IRC16M
 * (16 MHz / 16 * 336 / 2 = 168 MHz, APB1 42 MHz, APB2 84 MHz as before, see
 * hwconf/mcuconf.h) and USBFS is clocked from IRC48M, which the CTC trims to
 * the USB SOF from the host (ChibiOS STM32F4xx hal_lld.c). All clocks are
 * only as accurate as IRC16M (GD32F405 datasheet Table 4-18), see
 * documentation/gd32f405_port.md.
 *
 * HW_NAME differs from the crystal build so VESC Tool keeps them apart.
 */

#ifndef HW_60_GD32_1M_NOHSE_H_
#define HW_60_GD32_1M_NOHSE_H_

#if !defined(GD32F405_COMPAT) || defined(GD32F405_FLASH_3M) || !defined(GD32F405_NO_HSE)
#error "hw_60_gd32_1m_nohse.h must be built with: make fw_60_gd32_1m_nohse"
#endif

#define HW60_IS_MK1

#include "hw_60_core.h"

#undef HW_NAME
#define HW_NAME					"60_GD32_1M_NOHSE"

#endif /* HW_60_GD32_1M_NOHSE_H_ */
