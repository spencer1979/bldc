/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

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

#ifndef LSM6DSO_WRAPPER_H_
#define LSM6DSO_WRAPPER_H_

#include "ch.h"
#include "hal.h"
#include "i2c_bb.h"
#include "spi_bb.h"
#include "datatypes.h"

// I2C Addresses
#define LSM6DSO_I2C_ADDR_A 0x6A  // SDO pin = 0 (GND)
#define LSM6DSO_I2C_ADDR_B 0x6B  // SDO pin = 1 (VDDIO)

// Register Addresses
#define LSM6DSO_WHO_AM_I            0x0F
#define LSM6DSO_WHO_AM_I_VALUE      0x6C

#define LSM6DSO_CTRL1_XL            0x10  // Accel config
#define LSM6DSO_CTRL2_G             0x11  // Gyro config
#define LSM6DSO_CTRL3_C             0x12  // Control register 3
#define LSM6DSO_CTRL4_C             0x13  // Control register 4
#define LSM6DSO_CTRL6_C             0x15  // Control register 6 (Gyro filter)
#define LSM6DSO_CTRL8_XL            0x17  // Accel control 8 (LPF2)

#define LSM6DSO_OUTX_L_G            0x22  // Gyro X/Y/Z then accel X/Y/Z output

// CTRL1_XL - Accelerometer Control (bits [3:2])
// 00=±2g, 01=±4g, 10=±8g, 11=±16g
#define LSM6DSO_ACCEL_FS_2G         0x00
#define LSM6DSO_ACCEL_FS_4G         0x04
#define LSM6DSO_ACCEL_FS_8G         0x08
#define LSM6DSO_ACCEL_FS_16G        0x0C

// CTRL2_G - Gyroscope Control (bits [3:2])
// 00=±125dps, 01=±250dps, 10=±500dps, 11=±2000dps
#define LSM6DSO_GYRO_FS_125DPS      0x00
#define LSM6DSO_GYRO_FS_250DPS      0x04
#define LSM6DSO_GYRO_FS_500DPS      0x08
#define LSM6DSO_GYRO_FS_2000DPS     0x0C

// CTRL3_C - Control Register 3
#define LSM6DSO_BDU_BLOCK_UPDATE    0x40
#define LSM6DSO_IF_INC_ENABLED      0x04

// Interface types
#define LSM6DSO_I2C_INTF 0
#define LSM6DSO_SPI_INTF 1

// State structure
typedef struct {
	i2c_bb_state i2c_bb;
	spi_bb_state spi_bb;
	
	struct {
		uint8_t interface;  // I2C or SPI
		uint8_t addr;       // I2C address
		void (*delay_ms)(uint32_t ms);
	} sensor;
	
	int rate_hz;
	IMU_FILTER filter;
	void (*read_callback)(float *accel, float *gyro, float *mag);
	volatile bool should_stop;
	volatile bool is_running;
} LSM6DSO_STATE;

// Public API
void lsm6dso_wrapper_init(LSM6DSO_STATE *s, stkalign_t *work_area, size_t work_area_size);
void lsm6dso_wrapper_set_read_callback(LSM6DSO_STATE *s, void(*func)(float *accel, float *gyro, float *mag));
void lsm6dso_wrapper_stop(LSM6DSO_STATE *s);

#endif /* LSM6DSO_WRAPPER_H_ */
