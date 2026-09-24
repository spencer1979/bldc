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

#ifndef LSM6DSO_H_
#define LSM6DSO_H_

#include "ch.h"
#include "hal.h"
#include "i2c_bb.h"
#include "spi_bb.h"

void lsm6dso_set_rate_hz(int hz);
void lsm6dso_set_filter(IMU_FILTER f);
bool lsm6dso_init(i2c_bb_state *i2c_state, spi_bb_state *spi_state, SPIDriver *spi_hw,
		stkalign_t *work_area, size_t work_area_size);
void lsm6dso_set_read_callback(void(*func)(float *accel, float *gyro, float *mag));
void lsm6dso_stop(void);

// I2C Addresses
#define LSM6DSO_ADDR_A  				0X6A
#define LSM6DSO_ADDR_B  				0X6B

// Register Map
#define LSM6DSO_WHO_AM_I_REG  			0X0F
#define LSM6DSO_CTRL1_XL  				0X10
#define LSM6DSO_CTRL2_G  				0X11
#define LSM6DSO_CTRL3_C  				0X12
#define LSM6DSO_CTRL4_C  				0X13
#define LSM6DSO_CTRL6_C  				0X15
#define LSM6DSO_CTRL7_G  				0X16
#define LSM6DSO_CTRL8_XL  				0X17
#define LSM6DSO_CTRL9_XL  				0X18
#define LSM6DSO_OUTX_L_G  				0X22
#define LSM6DSO_OUTX_H_G  				0X23
#define LSM6DSO_OUTY_L_G  				0X24
#define LSM6DSO_OUTY_H_G  				0X25
#define LSM6DSO_OUTZ_L_G  				0X26
#define LSM6DSO_OUTZ_H_G  				0X27
#define LSM6DSO_OUTX_L_A  				0X28
#define LSM6DSO_OUTX_H_A  				0X29
#define LSM6DSO_OUTY_L_A  				0X2A
#define LSM6DSO_OUTY_H_A  				0X2B
#define LSM6DSO_OUTZ_L_A  				0X2C
#define LSM6DSO_OUTZ_H_A  				0X2D

// CTRL1_XL ODR
#define LSM6DSO_ODR_XL_OFF     			(0x00 << 4)
#define LSM6DSO_ODR_XL_12_5Hz  			(0x01 << 4)
#define LSM6DSO_ODR_XL_26Hz    			(0x02 << 4)
#define LSM6DSO_ODR_XL_52Hz    			(0x03 << 4)
#define LSM6DSO_ODR_XL_104Hz   			(0x04 << 4)
#define LSM6DSO_ODR_XL_208Hz   			(0x05 << 4)
#define LSM6DSO_ODR_XL_417Hz   			(0x06 << 4)
#define LSM6DSO_ODR_XL_833Hz   			(0x07 << 4)
#define LSM6DSO_ODR_XL_1667Hz  			(0x08 << 4)
#define LSM6DSO_ODR_XL_3333Hz  			(0x09 << 4)
#define LSM6DSO_ODR_XL_6667Hz  			(0x0A << 4)

// CTRL1_XL FS
#define LSM6DSO_FS_XL_2g       			(0x00 << 2)
#define LSM6DSO_FS_XL_4g       			(0x02 << 2)
#define LSM6DSO_FS_XL_8g       			(0x03 << 2)
#define LSM6DSO_FS_XL_16g      			(0x01 << 2)

// CTRL1_XL LPF2
#define LSM6DSO_LPF2_XL_EN     			(0x01 << 1)

// CTRL2_G ODR
#define LSM6DSO_ODR_G_OFF      			(0x00 << 4)
#define LSM6DSO_ODR_G_12_5Hz   			(0x01 << 4)
#define LSM6DSO_ODR_G_26Hz     			(0x02 << 4)
#define LSM6DSO_ODR_G_52Hz     			(0x03 << 4)
#define LSM6DSO_ODR_G_104Hz    			(0x04 << 4)
#define LSM6DSO_ODR_G_208Hz    			(0x05 << 4)
#define LSM6DSO_ODR_G_417Hz    			(0x06 << 4)
#define LSM6DSO_ODR_G_833Hz    			(0x07 << 4)
#define LSM6DSO_ODR_G_1667Hz   			(0x08 << 4)
#define LSM6DSO_ODR_G_3333Hz   			(0x09 << 4)
#define LSM6DSO_ODR_G_6667Hz   			(0x0A << 4)

// CTRL2_G FS
#define LSM6DSO_FS_G_250dps    			(0x00 << 0)
#define LSM6DSO_FS_G_500dps    			(0x01 << 0)
#define LSM6DSO_FS_G_1000dps   			(0x02 << 0)
#define LSM6DSO_FS_G_2000dps   			(0x03 << 0)

// CTRL3_C bits
#define LSM6DSO_SW_RESET       			(0x01 << 0)
#define LSM6DSO_IF_INC         			(0x01 << 2)
#define LSM6DSO_BDU            			(0x01 << 6)

#endif /* LSM6DSO_H_ */
