/*
	Copyright 2016 - 2022 Benjamin Vedder	benjamin@vedder.se

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

#pragma GCC optimize ("Os")

#include "flash_helper.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_sys.h"
#include "mc_interface.h"
#include "timeout.h"
#include "hw.h"
#include "crc.h"
#include "buffer.h"
#include <string.h>

#ifdef USE_LISPBM
#include "lispif.h"
#endif

/*
 * Defines
 */
#ifndef GD32F405_COMPAT
#define FLASH_SECTORS							12
#define BOOTLOADER_BASE							11
#define APP_BASE								0
#define NEW_APP_BASE							8
#define NEW_APP_SECTORS							3
#define APP_MAX_SIZE							(1024 * 128 * 4 - 8) // Note that the bootloader needs 8 extra bytes
#define QMLUI_BASE								9
#define LISP_BASE								10
#define LISP_CONST_BASE							8
#define QMLUI_MAX_SIZE							(1024 * 128 - 8)
#define LISP_MAX_SIZE							(1024 * 128 - 8)
#define NVM_BASE								11
#else
/*
 * GigaDevice GD32F405xK, 3 MB flash (see ld_gd32f405_3m.ld and
 * documentation/gd32f405_port.md).
 *
 * Bank0 (sectors 0-11, 1 MB) has the same layout as the STM32F405 and holds
 * the whole application image: s0 vectors, s1-s2 EEPROM emulation, s3-s7 fast
 * code (the GD32 zero-wait code area ends at 0x08080000) and s8-s11 slow code.
 * Bank1 (sectors 12-27, 2 MB) holds everything that is not part of the image.
 *
 * Indices below are indices into flash_addr[] / flash_sector[], which are the
 * same as the GD32 logical sector numbers.
 *
 * MATCHING BOOTLOADER REQUIRED: the stock bldc-bootloader (at 0x080E0000,
 * staging at 0x08080000, 384 KB) is NOT compatible with this layout. A GD32
 * build of the bootloader must live at 0x08100000 (s12), read the new image
 * from 0x08200000 (s24, max 1 MB) and erase/write app sectors 0-11.
 */
#define FLASH_SECTORS							28
#define BOOTLOADER_BASE							12 // s12-s15, 64 KB
#define APP_BASE								0
#define NEW_APP_BASE							24 // s24-s27, 4 x 256 KB
#define NEW_APP_SECTORS							4
#define APP_MAX_SIZE							(1024 * 1024 - 8) // Note that the bootloader needs 8 extra bytes
#define NVM_BASE								17
#define QMLUI_BASE								18
#define LISP_BASE								19
#define LISP_CONST_BASE							20
#define QMLUI_MAX_SIZE							(1024 * 128 - 8)
#define LISP_MAX_SIZE							(1024 * 128 - 8)
#endif

// Base address of the Flash sectors
#define ADDR_FLASH_SECTOR_0    					((uint32_t)0x08000000) // Base @ of Sector 0, 16 Kbytes
#define ADDR_FLASH_SECTOR_1    					((uint32_t)0x08004000) // Base @ of Sector 1, 16 Kbytes
#define ADDR_FLASH_SECTOR_2    					((uint32_t)0x08008000) // Base @ of Sector 2, 16 Kbytes
#define ADDR_FLASH_SECTOR_3						((uint32_t)0x0800C000) // Base @ of Sector 3, 16 Kbytes
#define ADDR_FLASH_SECTOR_4    					((uint32_t)0x08010000) // Base @ of Sector 4, 64 Kbytes
#define ADDR_FLASH_SECTOR_5    					((uint32_t)0x08020000) // Base @ of Sector 5, 128 Kbytes
#define ADDR_FLASH_SECTOR_6     				((uint32_t)0x08040000) // Base @ of Sector 6, 128 Kbytes
#define ADDR_FLASH_SECTOR_7     				((uint32_t)0x08060000) // Base @ of Sector 7, 128 Kbytes
#define ADDR_FLASH_SECTOR_8     				((uint32_t)0x08080000) // Base @ of Sector 8, 128 Kbytes
#define ADDR_FLASH_SECTOR_9 				    ((uint32_t)0x080A0000) // Base @ of Sector 9, 128 Kbytes
#define ADDR_FLASH_SECTOR_10				    ((uint32_t)0x080C0000) // Base @ of Sector 10, 128 Kbytes
#define ADDR_FLASH_SECTOR_11				    ((uint32_t)0x080E0000) // Base @ of Sector 11, 128 Kbytes

#ifdef GD32F405_COMPAT
// GD32F405xK bank1
#define ADDR_FLASH_SECTOR_12				    ((uint32_t)0x08100000) // 16 Kbytes
#define ADDR_FLASH_SECTOR_13				    ((uint32_t)0x08104000) // 16 Kbytes
#define ADDR_FLASH_SECTOR_14				    ((uint32_t)0x08108000) // 16 Kbytes
#define ADDR_FLASH_SECTOR_15				    ((uint32_t)0x0810C000) // 16 Kbytes
#define ADDR_FLASH_SECTOR_16				    ((uint32_t)0x08110000) // 64 Kbytes
#define ADDR_FLASH_SECTOR_17				    ((uint32_t)0x08120000) // 128 Kbytes
#define ADDR_FLASH_SECTOR_18				    ((uint32_t)0x08140000) // 128 Kbytes
#define ADDR_FLASH_SECTOR_19				    ((uint32_t)0x08160000) // 128 Kbytes
#define ADDR_FLASH_SECTOR_20				    ((uint32_t)0x08180000) // 128 Kbytes
#define ADDR_FLASH_SECTOR_21				    ((uint32_t)0x081A0000) // 128 Kbytes
#define ADDR_FLASH_SECTOR_22				    ((uint32_t)0x081C0000) // 128 Kbytes
#define ADDR_FLASH_SECTOR_23				    ((uint32_t)0x081E0000) // 128 Kbytes
#define ADDR_FLASH_SECTOR_24				    ((uint32_t)0x08200000) // 256 Kbytes
#define ADDR_FLASH_SECTOR_25				    ((uint32_t)0x08240000) // 256 Kbytes
#define ADDR_FLASH_SECTOR_26				    ((uint32_t)0x08280000) // 256 Kbytes
#define ADDR_FLASH_SECTOR_27				    ((uint32_t)0x082C0000) // 256 Kbytes

/*
 * Values for the SN field (bits 7:3) of FMC_CTL (= FLASH->CR). The GD32 SN
 * encoding is not linear: sectors 12-23 use SN 16-27, which is the same as
 * the STM32F42x FLASH_Sector_12..23 definitions in the StdPeriph library, but
 * sectors 24-27 use SN 12-15, which have no StdPeriph definition.
 * FLASH_EraseSector() clears bits 7:3 with SECTOR_MASK (0xFFFFFF07), so these
 * values can be passed to it directly.
 */
#define GD32_FLASH_Sector_24					((uint16_t)(12 << 3))
#define GD32_FLASH_Sector_25					((uint16_t)(13 << 3))
#define GD32_FLASH_Sector_26					((uint16_t)(14 << 3))
#define GD32_FLASH_Sector_27					((uint16_t)(15 << 3))

#define BOOTLOADER_ADDRESS						ADDR_FLASH_SECTOR_12
#else
#define BOOTLOADER_ADDRESS						ADDR_FLASH_SECTOR_11
#endif

#define VECTOR_TABLE_ADDRESS					((uint32_t*)ADDR_FLASH_SECTOR_0)
#define VECTOR_TABLE_SIZE						((uint32_t)(ADDR_FLASH_SECTOR_1 - ADDR_FLASH_SECTOR_0))
#define EEPROM_EMULATION_SIZE					((uint32_t)(ADDR_FLASH_SECTOR_4 - ADDR_FLASH_SECTOR_2))

#define APP_START_ADDRESS						((uint32_t*)(ADDR_FLASH_SECTOR_3))
#define APP_SIZE								((uint32_t)(APP_MAX_SIZE - VECTOR_TABLE_SIZE - EEPROM_EMULATION_SIZE))

#define	APP_CRC_WAS_CALCULATED_FLAG				((uint32_t)0x00000000)
#define	APP_CRC_WAS_CALCULATED_FLAG_ADDRESS		((uint32_t*)(ADDR_FLASH_SECTOR_0 + APP_MAX_SIZE - 8))
#define APP_CRC_ADDRESS							((uint32_t*)(ADDR_FLASH_SECTOR_0 + APP_MAX_SIZE - 4))

#define ERASE_VOLTAGE_RANGE						(uint8_t)((PWR->CSR & PWR_CSR_PVDO) ? VoltageRange_2 : VoltageRange_3)

typedef struct {
	uint32_t crc_flag;
	uint32_t crc;
} crc_info_t;

// Make sure the app image has the CRC bits set to '1' to later write the flag and CRC.
const crc_info_t __attribute__((section (".crcinfo"))) crc_info = {0xFFFFFFFF, 0xFFFFFFFF};

// Private functions
static uint16_t erase_sector(uint32_t sector);
static uint16_t write_data(uint32_t base, uint8_t *data, uint32_t len);
static void qmlui_check(int ind);

// Private variables
typedef struct {
	bool check_done;
	bool ok;
} _code_checks;

static _code_checks code_checks[3] = {0};
static int code_sectors[3] = {QMLUI_BASE, LISP_BASE, LISP_CONST_BASE};

// Private constants
static const uint32_t flash_addr[FLASH_SECTORS] = {
		ADDR_FLASH_SECTOR_0,
		ADDR_FLASH_SECTOR_1,
		ADDR_FLASH_SECTOR_2,
		ADDR_FLASH_SECTOR_3,
		ADDR_FLASH_SECTOR_4,
		ADDR_FLASH_SECTOR_5,
		ADDR_FLASH_SECTOR_6,
		ADDR_FLASH_SECTOR_7,
		ADDR_FLASH_SECTOR_8,
		ADDR_FLASH_SECTOR_9,
		ADDR_FLASH_SECTOR_10,
		ADDR_FLASH_SECTOR_11,
#ifdef GD32F405_COMPAT
		ADDR_FLASH_SECTOR_12,
		ADDR_FLASH_SECTOR_13,
		ADDR_FLASH_SECTOR_14,
		ADDR_FLASH_SECTOR_15,
		ADDR_FLASH_SECTOR_16,
		ADDR_FLASH_SECTOR_17,
		ADDR_FLASH_SECTOR_18,
		ADDR_FLASH_SECTOR_19,
		ADDR_FLASH_SECTOR_20,
		ADDR_FLASH_SECTOR_21,
		ADDR_FLASH_SECTOR_22,
		ADDR_FLASH_SECTOR_23,
		ADDR_FLASH_SECTOR_24,
		ADDR_FLASH_SECTOR_25,
		ADDR_FLASH_SECTOR_26,
		ADDR_FLASH_SECTOR_27,
#endif
};
static const uint16_t flash_sector[FLASH_SECTORS] = {
		FLASH_Sector_0,
		FLASH_Sector_1,
		FLASH_Sector_2,
		FLASH_Sector_3,
		FLASH_Sector_4,
		FLASH_Sector_5,
		FLASH_Sector_6,
		FLASH_Sector_7,
		FLASH_Sector_8,
		FLASH_Sector_9,
		FLASH_Sector_10,
		FLASH_Sector_11,
#ifdef GD32F405_COMPAT
		FLASH_Sector_12, // SN 16
		FLASH_Sector_13,
		FLASH_Sector_14,
		FLASH_Sector_15,
		FLASH_Sector_16,
		FLASH_Sector_17,
		FLASH_Sector_18,
		FLASH_Sector_19,
		FLASH_Sector_20,
		FLASH_Sector_21,
		FLASH_Sector_22,
		FLASH_Sector_23, // SN 27
		GD32_FLASH_Sector_24, // SN 12
		GD32_FLASH_Sector_25,
		GD32_FLASH_Sector_26,
		GD32_FLASH_Sector_27, // SN 15
#endif
};

uint16_t flash_helper_erase_new_app(uint32_t new_app_size) {
#ifdef USE_LISPBM
	lispif_stop();
#endif

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	new_app_size += flash_addr[NEW_APP_BASE];

	mc_interface_ignore_input_both(5000);
	mc_interface_release_motor_override_both();

	if (!mc_interface_wait_for_motor_release_both(3.0)) {
		return 100;
	}

	utils_sys_lock_cnt();
	timeout_configure_IWDT_slowest();

	for (int i = 0;i < NEW_APP_SECTORS;i++) {
		if (new_app_size > flash_addr[NEW_APP_BASE + i]) {
			uint16_t res = FLASH_EraseSector(flash_sector[NEW_APP_BASE + i], ERASE_VOLTAGE_RANGE);
			if (res != FLASH_COMPLETE) {
				FLASH_Lock();
				timeout_configure_IWDT();
				mc_interface_ignore_input_both(5000);
				utils_sys_unlock_cnt();
				return res;
			}
		} else {
			break;
		}
	}

	FLASH_Lock();
	timeout_configure_IWDT();
	mc_interface_ignore_input_both(100);
	utils_sys_unlock_cnt();

	return FLASH_COMPLETE;
}

uint16_t flash_helper_erase_bootloader(void) {
#ifdef GD32F405_COMPAT
	/*
	 * VESC Tool writes a bootloader to NEW_APP_BASE + 384 KB (hardcoded for
	 * HW_TYPE_VESC), which matches the STM32 layout only. In the GD32 layout
	 * that would erase the bootloader at 0x08100000 and write the new one into
	 * the staging area, leaving the device without a bootloader. Refuse, so
	 * that the upload fails cleanly. Flash the GD32 bootloader with SWD.
	 */
	return FLASH_ERROR_OPERATION;
#else
	return erase_sector(flash_sector[BOOTLOADER_BASE]);
#endif
}

uint16_t flash_helper_write_new_app_data(uint32_t offset, uint8_t *data, uint32_t len) {
	return write_data(flash_addr[NEW_APP_BASE] + offset, data, len);
}

uint16_t flash_helper_erase_code(int ind) {
#ifdef USE_LISPBM
	if (ind == CODE_IND_LISP || ind == CODE_IND_LISP_CONST) {
		lispif_stop();
	}
#endif

	uint8_t *ptr = flash_helper_code_data_raw(ind);

	bool has_data = false;
	for (int i = 0;i < (1024 * 128); i++) {
		if (*ptr != 0xFF) {
			has_data = true;
			break;
		}
	}

	if (!has_data) {
		return FLASH_COMPLETE;
	}

	code_checks[ind].check_done = false;
	code_checks[ind].ok = false;
	return erase_sector(flash_sector[code_sectors[ind]]);
}

uint16_t flash_helper_write_code(int ind, uint32_t offset, uint8_t *data, uint32_t len) {
	code_checks[ind].check_done = false;
	code_checks[ind].ok = false;
	return write_data(flash_addr[code_sectors[ind]] + offset, data, len);
}

uint8_t* flash_helper_code_data(int ind) {
	qmlui_check(ind);

	if (code_checks[ind].check_done && code_checks[ind].ok) {
		return (uint8_t*)(flash_addr[code_sectors[ind]]) + 8;
	} else {
		return 0;
	}
}

uint8_t* flash_helper_code_data_raw(int ind) {
	return (uint8_t*)flash_addr[code_sectors[ind]];
}

uint32_t flash_helper_code_size(int ind) {
	qmlui_check(ind);

	if (code_checks[ind].check_done && code_checks[ind].ok) {
		uint8_t *base = (uint8_t*)(flash_addr[code_sectors[ind]]);
		int32_t index = 0;
		return buffer_get_uint32(base, &index);
	} else {
		return 0;
	}
}

uint16_t flash_helper_code_flags(int ind) {
	qmlui_check(ind);

	if (code_checks[ind].check_done && code_checks[ind].ok) {
		uint8_t *base = (uint8_t*)(flash_addr[code_sectors[ind]]);
		int32_t index = 6;
		return buffer_get_uint16(base, &index);
	} else {
		return 0;
	}
}

/**
 * Stop the system and jump to the bootloader.
 */
void flash_helper_jump_to_bootloader(void) {
	typedef void (*pFunction)(void);

	mc_interface_release_motor_override();
	usbDisconnectBus(&USBD1);
	usbStop(&USBD1);

	sdStop(&HW_UART_DEV);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT);

	// Disable watchdog
	timeout_configure_IWDT_slowest();

	chSysDisable();

#ifdef GD32F405_COMPAT
	// GD32F4xx: switching from the PLL back to IRC16M at full speed can make
	// the clocks drop out (GigaDevice migration note, section 5.2). The
	// bootloader re-initializes the clocks, so leave the PLL safely here:
	// divide HCLK by 4, wait, then select HSI.
	RCC->CFGR |= RCC_CFGR_HPRE_DIV4;
	for (volatile uint32_t i = 0xFFFF;i > 0;i--) {
		__NOP();
	}
	RCC->CFGR &= ~RCC_CFGR_SW; // SW = HSI
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) {}
#endif

	pFunction jump_to_bootloader;

	// Variable that will be loaded with the start address of the application
	volatile uint32_t* jump_address;
	const volatile uint32_t* bootloader_address = (volatile uint32_t*)BOOTLOADER_ADDRESS;

	// Get jump address from application vector table
	jump_address = (volatile uint32_t*) bootloader_address[1];

	// Load this address into function pointer
	jump_to_bootloader = (pFunction) jump_address;

	// Clear pending interrupts
	SCB->ICSR = SCB_ICSR_PENDSVCLR_Msk;

	// Disable all interrupts
	for(int i = 0;i < 8;i++) {
		NVIC->ICER[i] = NVIC->IABR[i];
	}

	// Set stack pointer
	__set_MSP((uint32_t) (bootloader_address[0]));

	// Jump to the bootloader
	jump_to_bootloader();
}

uint8_t* flash_helper_get_sector_address(uint32_t fsector) {
	uint8_t *res = 0;

	for (int i = 0;i < FLASH_SECTORS;i++) {
		if (flash_sector[i] == fsector) {
			res = (uint8_t *)flash_addr[i];
			break;
		}
	}

	return res;
}

/**
  * @brief  Compute the CRC of the application code to verify its integrity
  * @retval FAULT_CODE_NONE or FAULT_CODE_FLASH_CORRUPTION
  */
uint32_t flash_helper_verify_flash_memory(void) {
	uint32_t crc;
	// Look for a flag indicating that the CRC was previously computed.
	// If it is blank (0xFFFFFFFF), calculate and store the CRC.
	if(APP_CRC_WAS_CALCULATED_FLAG_ADDRESS[0] == APP_CRC_WAS_CALCULATED_FLAG) {
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
		crc32_reset();

		// compute vector table (sector 0)
		crc32(VECTOR_TABLE_ADDRESS, (VECTOR_TABLE_SIZE) / 4);

		// skip emulated EEPROM (sector 1 and 2)

		// compute application code
		crc = crc32(APP_START_ADDRESS, (APP_SIZE) / 4);

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, DISABLE);

		// A CRC over the full image should return zero.
		return (crc == 0) ? FAULT_CODE_NONE : FAULT_CODE_FLASH_CORRUPTION;
	} else {
		FLASH_Unlock();
		FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
				FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

		// Write the flag to indicate CRC has been computed.
		uint16_t res = FLASH_ProgramWord((uint32_t)APP_CRC_WAS_CALCULATED_FLAG_ADDRESS, APP_CRC_WAS_CALCULATED_FLAG);
		if (res != FLASH_COMPLETE) {
			FLASH_Lock();
			return FAULT_CODE_FLASH_CORRUPTION;
		}

		// Compute flash crc including the new flag
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, ENABLE);
		crc32_reset();

		// compute vector table (sector 0)
		crc32(VECTOR_TABLE_ADDRESS, (VECTOR_TABLE_SIZE) / 4);

		// skip emulated EEPROM (sector 1 and 2)

		// compute application code
		crc = crc32(APP_START_ADDRESS, (APP_SIZE - 4) / 4);

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CRC, DISABLE);

		//Store CRC
		res = FLASH_ProgramWord((uint32_t)APP_CRC_ADDRESS, crc);
		if (res != FLASH_COMPLETE) {
			FLASH_Lock();
			return FAULT_CODE_FLASH_CORRUPTION;
		}
		FLASH_Lock();

		// reboot
		NVIC_SystemReset();
		return FAULT_CODE_NONE;
	}
}

uint32_t flash_helper_verify_flash_memory_chunk(void) {
	static uint32_t index = 0;
	uint32_t chunk_size = 1024;
	uint32_t res = FAULT_CODE_NONE;
	uint32_t crc = 0;
	uint32_t tot_bytes = VECTOR_TABLE_SIZE + APP_SIZE;

	// Make sure RCC_AHB1Periph_CRC is enabled
	if (index == 0) {
		crc32_reset();
	}

	if ((index + chunk_size) >= tot_bytes) {
		chunk_size = tot_bytes - index;
	}

	if (index < VECTOR_TABLE_SIZE) {
		crc32(VECTOR_TABLE_ADDRESS + index / 4, chunk_size / 4);
	} else {
		crc = crc32(APP_START_ADDRESS + (index - VECTOR_TABLE_SIZE) / 4, chunk_size / 4);
	}

	index += chunk_size;
	if (index >= tot_bytes) {
		index = 0;
		if (crc != 0) {
			res = FAULT_CODE_FLASH_CORRUPTION;
		}
	}

	return res;
}

uint32_t flash_helper_app_crc(void) {
	return *APP_CRC_ADDRESS;
}

static uint16_t erase_sector(uint32_t sector) {
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	mc_interface_ignore_input_both(5000);
	mc_interface_release_motor_override_both();

	if (!mc_interface_wait_for_motor_release_both(3.0)) {
		return 100;
	}

	utils_sys_lock_cnt();
	timeout_configure_IWDT_slowest();

	uint16_t res = FLASH_EraseSector(sector, ERASE_VOLTAGE_RANGE);

	FLASH_Lock();
	timeout_configure_IWDT();
	mc_interface_ignore_input_both(100);
	utils_sys_unlock_cnt();
	return res;
}

static uint16_t write_data(uint32_t base, uint8_t *data, uint32_t len) {
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR |
			FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	mc_interface_ignore_input_both(5000);
	mc_interface_release_motor_override_both();

	if (!mc_interface_wait_for_motor_release_both(3.0)) {
		return 100;
	}

	utils_sys_lock_cnt();
	timeout_configure_IWDT_slowest();

	for (uint32_t i = 0;i < len;i++) {
		uint16_t res = FLASH_ProgramByte(base + i, data[i]);
		if (res != FLASH_COMPLETE) {
			FLASH_Lock();
			timeout_configure_IWDT();
			mc_interface_ignore_input_both(5000);
			utils_sys_unlock_cnt();
			return res;
		}
	}

	FLASH_Lock();
	timeout_configure_IWDT();
	mc_interface_ignore_input_both(100);
	utils_sys_unlock_cnt();

	return FLASH_COMPLETE;
}

static void qmlui_check(int ind) {
	if (code_checks[ind].check_done) {
		return;
	}

	uint8_t *base = (uint8_t*)(flash_addr[code_sectors[ind]]);
	int32_t index = 0;
	uint32_t qmlui_len = buffer_get_uint32(base, &index);
	uint16_t qmlui_crc = buffer_get_uint16(base, &index);

	if (qmlui_len <= QMLUI_MAX_SIZE) {
		uint16_t crc_calc = crc16(base + index, qmlui_len + 2); // CRC includes the 2 byte flags
		code_checks[ind].ok = crc_calc == qmlui_crc;
	} else {
		code_checks[ind].ok = false;
	}

	code_checks[ind].check_done = true;
}

#define VESC_IF_NVM_REGION_SIZE	(1024 * 128)
#define VESC_IF_NVM_ADDRESS		(flash_addr[NVM_BASE])

/**
  * @brief  Reads len bytes to v from nvm at address
  * @param	v: array of bytes to which the result will be written
  * @param	len: number of bytes to read
  * @param	address: address of the first byte
  * @retval Boolean indicating success or failure
  */
bool flash_helper_read_nvm(uint8_t *v, unsigned int len, unsigned int address) {
	if ((address + len) > VESC_IF_NVM_REGION_SIZE) {
		return false;
	}

	memcpy(v, (uint8_t*)(VESC_IF_NVM_ADDRESS + address), len);

	return true;
}

/**
  * @brief  Writes len bytes from v to nvm at address
  * @param	v: array of bytes to write
  * @param	len: number of bytes to write
  * @param	address: address of the first byte
  * @retval Boolean indicating success or failure
  */
bool flash_helper_write_nvm(uint8_t *v, unsigned int len, unsigned int address) {
	if ((address + len) > VESC_IF_NVM_REGION_SIZE) {
		return false;
	}

	uint16_t res = write_data(VESC_IF_NVM_ADDRESS + address, v, len);

	return (res == FLASH_COMPLETE);
}

/**
  * @brief  Erase region of NVM used by packages.
  * @retval Boolean indicating success or failure
  */
bool flash_helper_wipe_nvm(void) {
	return (erase_sector(flash_sector[NVM_BASE]) == FLASH_COMPLETE);
}
