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

#ifndef LSM6DSV32X_VESC_H_
#define LSM6DSV32X_VESC_H_

#include "ch.h"
#include "hal.h"
#include "i2c_bb.h"
#include "spi_bb.h"
#include "datatypes.h"

void lsm6dsv32x_vesc_set_rate_hz(int hz);
void lsm6dsv32x_vesc_set_filter(IMU_FILTER f);
bool lsm6dsv32x_vesc_init(i2c_bb_state *i2c_state, spi_bb_state *spi_state, SPIDriver *spi_hw,
		stkalign_t *work_area, size_t work_area_size);
void lsm6dsv32x_vesc_set_read_callback(void(*func)(float *accel, float *gyro, float *mag));
void lsm6dsv32x_vesc_stop(void);

/* I2C 7-bit addresses */
#define LSM6DSV32X_VESC_ADDR_A  0x6A
#define LSM6DSV32X_VESC_ADDR_B  0x6B

/* WHO_AM_I */
#define LSM6DSV32X_VESC_WHO_AM_I_REG    0x0F
#define LSM6DSV32X_VESC_WHO_AM_I_VALUE  0x70

/* Control registers */
#define LSM6DSV32X_VESC_CTRL1   0x10  /* XL ODR + op_mode */
#define LSM6DSV32X_VESC_CTRL2   0x11  /* GY ODR + op_mode */
#define LSM6DSV32X_VESC_CTRL3   0x12  /* if_inc, bdu */
#define LSM6DSV32X_VESC_CTRL4   0x13  /* DRDY_MASK, DRDY_PULSED */
#define LSM6DSV32X_VESC_CTRL6   0x15  /* GY full-scale + LPF1 BW */
#define LSM6DSV32X_VESC_CTRL7   0x16  /* LPF1_G_EN */
#define LSM6DSV32X_VESC_CTRL8   0x17  /* XL full-scale + LPF2/HPF BW */
#define LSM6DSV32X_VESC_CTRL9   0x18  /* LPF2_XL_EN */

/* Status */
#define LSM6DSV32X_VESC_STATUS  0x1E

/* Output data registers (gyro 0x22..0x27, accel 0x28..0x2D) */
#define LSM6DSV32X_VESC_OUTX_L_G  0x22

/* ODR values written to CTRL1[3:0] / CTRL2[3:0] */
#define LSM6DSV32X_VESC_ODR_OFF    0x00
#define LSM6DSV32X_VESC_ODR_15Hz   0x03
#define LSM6DSV32X_VESC_ODR_30Hz   0x04
#define LSM6DSV32X_VESC_ODR_60Hz   0x05
#define LSM6DSV32X_VESC_ODR_120Hz  0x06
#define LSM6DSV32X_VESC_ODR_240Hz  0x07
#define LSM6DSV32X_VESC_ODR_480Hz  0x08
#define LSM6DSV32X_VESC_ODR_960Hz  0x09
#define LSM6DSV32X_VESC_ODR_1920Hz 0x0A
#define LSM6DSV32X_VESC_ODR_3840Hz 0x0B
#define LSM6DSV32X_VESC_ODR_7680Hz 0x0C

/* CTRL6 gyro full-scale (bits[3:0]) */
#define LSM6DSV32X_VESC_FS_G_4000DPS 0x0C

/* CTRL8 accel full-scale (bits[1:0]) */
#define LSM6DSV32X_VESC_FS_XL_4G    0x00
#define LSM6DSV32X_VESC_FS_XL_8G    0x01
#define LSM6DSV32X_VESC_FS_XL_16G   0x02
#define LSM6DSV32X_VESC_FS_XL_32G   0x03
/* CTRL8 bit 2: datasheet reserved bit that must always be written as 1 */
#define LSM6DSV32X_VESC_CTRL8_MUST_SET 0x04

/* CTRL3 bits */
#define LSM6DSV32X_VESC_IF_INC    0x04
#define LSM6DSV32X_VESC_BDU       0x40

#endif /* LSM6DSV32X_VESC_H_ */
