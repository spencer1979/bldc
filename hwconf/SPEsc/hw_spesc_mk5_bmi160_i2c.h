/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

	SPEsc MK5 — BMI160 on I2C.

	Build: make fw_spesc_mk5_bmi160_i2c

	SDA  PB2
	SCL  PA15
 */

#ifndef HW_SPESC_MK5_BMI160_I2C_H_
#define HW_SPESC_MK5_BMI160_I2C_H_

#define HW_NAME						"SPESC MK5 BMI160 I2C"

#define HW_SPESC_MK5
#define HW_SPESC_MK5_BMI160

#define IMU_DEV					IMU_DEV_BMI160
#define IMU_COM					IMU_COM_I2C_BB
#define IMU_I2C_SDA_GPIO		GPIOB
#define IMU_I2C_SDA_PIN			2
#define IMU_I2C_SCL_GPIO		GPIOA
#define IMU_I2C_SCL_PIN			15

#include "hw_spesc_core.h"

#endif /* HW_SPESC_MK5_BMI160_I2C_H_ */
