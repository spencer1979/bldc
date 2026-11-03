/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

	SPEsc MK5 — LSM6DSV32X on I2C (same bus as MK5 BMI160 / MK4 LSM6DSV32X).

	Build: make fw_spesc_mk5_lsm6dsv32x_i2c

	SDA  PB2
	SCL  PA15
 */

#ifndef HW_SPESC_MK5_LSM6DSV32X_I2C_H_
#define HW_SPESC_MK5_LSM6DSV32X_I2C_H_

#define HW_NAME						"SPESC MK5 LSM6DSV32X I2C"

#define HW_SPESC_MK5
#define HW_SPESC_MK5_LSM6DSV32X

#define IMU_ROT_90

#define LSM6DSV32X_SDA_GPIO GPIOB
#define LSM6DSV32X_SDA_PIN 2
#define LSM6DSV32X_SCL_GPIO GPIOA
#define LSM6DSV32X_SCL_PIN 15

#include "hw_spesc_core.h"

#endif /* HW_SPESC_MK5_LSM6DSV32X_I2C_H_ */
