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

#ifndef HW_SPESC_MK5_LSM6DSO_H_
#define HW_SPESC_MK5_LSM6DSO_H_

#define HW_SPESC_MK5
#define HW_SPESC_MK5_LSM6DSO
#define HW_NAME "SPEsc_MK5_LSM6DSO"

// LSM6DSO I2C pins (same as other SPEsc variants)
#define LSM6DSO_SDA_GPIO GPIOB
#define LSM6DSO_SDA_PIN 2
#define LSM6DSO_SCL_GPIO GPIOA
#define LSM6DSO_SCL_PIN 15

// IMU Rotation: 90 degree rotation same as other SPEsc variants
#define IMU_ROT_90

#include "hw_spesc_core.h"

#endif /* HW_SPESC_MK5_LSM6DSO_H_ */
