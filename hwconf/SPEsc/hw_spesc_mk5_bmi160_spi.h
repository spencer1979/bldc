/*
	Copyright 2026 Benjamin Vedder	benjamin@vedder.se

	SPEsc MK5 — BMI160 on the LSM6DSV32X SPI footprint (pin compatible).

	Build: make fw_spesc_mk5_bmi160_spi

	IMU_SPI_MOSI  PB5
	IMU_SPI_MISO  PB4
	IMU_SPI_SCK   PB3
	IMU_SPI_NSS   PA15
	INT1          PB12 (input only; driver polls over SPI)

	Hardware SPI1 on PB3/4/5 (AF5), NSS on PA15 (GPIO). BMI160 SPI mode 0.
	Do not define HW_SPESC_MK5_BMI160 here — that macro enables the I2C pins.
 */

#ifndef HW_SPESC_MK5_BMI160_SPI_H_
#define HW_SPESC_MK5_BMI160_SPI_H_

#define HW_NAME						"SPESC MK5 BMI160 SPI"

#define HW_SPESC_MK5
#define HW_SPESC_MK5_BMI160_SPI

#define BMI160_SPI_PORT_NSS		GPIOA
#define BMI160_SPI_PIN_NSS		15
#define BMI160_SPI_PORT_SCK		GPIOB
#define BMI160_SPI_PIN_SCK		3
#define BMI160_SPI_PORT_MOSI	GPIOB
#define BMI160_SPI_PIN_MOSI		5
#define BMI160_SPI_PORT_MISO	GPIOB
#define BMI160_SPI_PIN_MISO		4

#define BMI160_HWSPI_DEV		SPID1
#define BMI160_HWSPI_AF			GPIO_AF_SPI1

#include "hw_spesc_core.h"

#endif /* HW_SPESC_MK5_BMI160_SPI_H_ */
