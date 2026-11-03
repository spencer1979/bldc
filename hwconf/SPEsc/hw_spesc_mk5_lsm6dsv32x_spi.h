/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

	SPEsc MK5 — LSM6DSV32X on hardware SPI1 (same pin map as Thor301).

	Build: make fw_spesc_mk5_lsm6dsv32x_spi

	IMU_SPI_MOSI  PB5
	IMU_SPI_MISO  PB4
	IMU_SPI_SCK   PB3
	IMU_SPI_NSS   PA15
	INT1          PB12

	Hardware SPI1 on PB3/4/5 (AF5), NSS on PA15 (GPIO). Mode 3.
	MK5 has no SPI encoder — SPID1 uses the Port B pin set only.
 */

#ifndef HW_SPESC_MK5_LSM6DSV32X_SPI_H_
#define HW_SPESC_MK5_LSM6DSV32X_SPI_H_

#define HW_NAME						"SPESC MK5 LSM6DSV32X SPI"

#define HW_SPESC_MK5
#define HW_SPESC_MK5_LSM6DSV32X

#define IMU_ROT_90

// Hardware SPI for IMU (Thor301 pin map)
#define IMU_DEV					IMU_DEV_LSM6DSV32X
#define IMU_COM					IMU_COM_SPI_HW
#define IMU_SPI_DEV				SPID1
#define IMU_SPI_AF				GPIO_AF_SPI1
#define IMU_SPI_NSS_GPIO		GPIOA
#define IMU_SPI_NSS_PIN			15
#define IMU_SPI_SCK_GPIO		GPIOB
#define IMU_SPI_SCK_PIN			3
#define IMU_SPI_MOSI_GPIO		GPIOB
#define IMU_SPI_MOSI_PIN		5
#define IMU_SPI_MISO_GPIO		GPIOB
#define IMU_SPI_MISO_PIN		4

#define IMU_DRDY_GPIO			GPIOB
#define IMU_DRDY_PIN			12

#define HW_LIM_IMU_SAMPLE_RATE_HZ	10000

#include "hw_spesc_core.h"

#endif /* HW_SPESC_MK5_LSM6DSV32X_SPI_H_ */
