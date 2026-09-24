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

#include "lsm6dso.h"
#include "terminal.h"
#include "commands.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

static thread_t *lsm6dso_thread_ref = NULL;
static i2c_bb_state *m_i2c_bb;
static spi_bb_state *m_spi_bb;
static SPIDriver *m_hwspi_dev;
static volatile uint16_t lsm6dso_addr;
static int rate_hz = 1000;
static IMU_FILTER filter;

static void terminal_read_reg(int argc, const char **argv);
static bool read_reg(uint8_t reg, uint8_t *res);
static bool write_reg(uint8_t reg, uint8_t value);
static bool read_gyro_accel(int16_t *gyro, int16_t *accel);
static THD_FUNCTION(lsm6dso_thread, arg);

// Function pointers
static void(*read_callback)(float *accel, float *gyro, float *mag) = 0;

void lsm6dso_set_rate_hz(int hz) {
	rate_hz = hz;
}

void lsm6dso_set_filter(IMU_FILTER f) {
	filter = f;
}

bool lsm6dso_init(i2c_bb_state *i2c_state, spi_bb_state *spi_state, SPIDriver *spi_hw,
		stkalign_t *work_area, size_t work_area_size) {

	read_callback = 0;

	m_i2c_bb = i2c_state;
	m_spi_bb = spi_state;
	m_hwspi_dev = spi_hw;

	uint8_t rxb[1];

	lsm6dso_addr = LSM6DSO_ADDR_A;
	bool res = read_reg(LSM6DSO_WHO_AM_I_REG, rxb);
	if (!res || rxb[0] != 0x6C) {
		commands_printf("LSM6DSO Address A failed, trying B (rx: %d)", rxb[0]);
		lsm6dso_addr = LSM6DSO_ADDR_B;
		res = read_reg(LSM6DSO_WHO_AM_I_REG, rxb);
		if (!res || rxb[0] != 0x6C) {
			commands_printf("LSM6DSO Address B failed (rx: %d)", rxb[0]);
			return false;
		}
	}

	// Software reset
	res = write_reg(LSM6DSO_CTRL3_C, LSM6DSO_SW_RESET);
	if (!res) {
		commands_printf("LSM6DSO Reset FAILED");
		return false;
	}
	chThdSleepMilliseconds(10);

	// Configure accelerometer: ±16g, ODR based on rate_hz
	uint8_t regv = LSM6DSO_FS_XL_16g;
	
	// Disable LPF2 by default, will configure in CTRL8_XL instead
	// uint8_t regv = LSM6DSO_FS_XL_16g | LSM6DSO_LPF2_XL_EN;
	
	if (rate_hz <= 13) {
		regv |= LSM6DSO_ODR_XL_12_5Hz;
	} else if (rate_hz <= 26) {
		regv |= LSM6DSO_ODR_XL_26Hz;
	} else if (rate_hz <= 52) {
		regv |= LSM6DSO_ODR_XL_52Hz;
	} else if (rate_hz <= 104) {
		regv |= LSM6DSO_ODR_XL_104Hz;
	} else if (rate_hz <= 208) {
		regv |= LSM6DSO_ODR_XL_208Hz;
	} else if (rate_hz <= 417) {
		regv |= LSM6DSO_ODR_XL_417Hz;
	} else if (rate_hz <= 833) {
		regv |= LSM6DSO_ODR_XL_833Hz;
	} else if (rate_hz <= 1667) {
		regv |= LSM6DSO_ODR_XL_1667Hz;
	} else {
		regv |= LSM6DSO_ODR_XL_3333Hz;
	}

	res = write_reg(LSM6DSO_CTRL1_XL, regv);
	if (!res) {
		commands_printf("LSM6DSO Accel Config FAILED");
		return false;
	}

	// Configure accelerometer filtering in CTRL8_XL based on filter level
	uint8_t ctrl8_xl = 0;
	if (filter >= IMU_FILTER_MEDIUM) {
		// Enable LPF2 with adaptive bandwidth
		ctrl8_xl = 0x01;  // LPF2_XL_EN
		if (filter == IMU_FILTER_HIGH) {
			// Stronger filtering: ODR/20
			ctrl8_xl |= (0x02 << 6);  // LPF2_XL_BW[1:0] = 10
		} else {
			// Medium filtering: ODR/10
			ctrl8_xl |= (0x01 << 6);  // LPF2_XL_BW[1:0] = 01
		}
	}
	write_reg(LSM6DSO_CTRL8_XL, ctrl8_xl);

	// Configure gyroscope filtering in CTRL7_G based on filter level
	uint8_t ctrl7_g = 0;
	if (filter >= IMU_FILTER_MEDIUM) {
		// Enable gyroscope LPF1
		if (filter == IMU_FILTER_HIGH) {
			// Strongest filter: G_LPF1_BW = 111 (narrowest)
			ctrl7_g = (0x07 << 5);
		} else {
			// Medium filter: G_LPF1_BW = 100
			ctrl7_g = (0x04 << 5);
		}
	}
	write_reg(LSM6DSO_CTRL7_G, ctrl7_g);

	// Configure gyroscope: ±2000dps, same ODR as accel
	regv = LSM6DSO_FS_G_2000dps;
	
	if (rate_hz <= 13) {
		regv |= LSM6DSO_ODR_G_12_5Hz;
	} else if (rate_hz <= 26) {
		regv |= LSM6DSO_ODR_G_26Hz;
	} else if (rate_hz <= 52) {
		regv |= LSM6DSO_ODR_G_52Hz;
	} else if (rate_hz <= 104) {
		regv |= LSM6DSO_ODR_G_104Hz;
	} else if (rate_hz <= 208) {
		regv |= LSM6DSO_ODR_G_208Hz;
	} else if (rate_hz <= 417) {
		regv |= LSM6DSO_ODR_G_417Hz;
	} else if (rate_hz <= 833) {
		regv |= LSM6DSO_ODR_G_833Hz;
	} else if (rate_hz <= 1667) {
		regv |= LSM6DSO_ODR_G_1667Hz;
	} else {
		regv |= LSM6DSO_ODR_G_3333Hz;
	}

	res = write_reg(LSM6DSO_CTRL2_G, regv);
	if (!res) {
		commands_printf("LSM6DSO Gyro Config FAILED");
		return false;
	}

	// Configure block update and register auto-increment
	regv = LSM6DSO_BDU | LSM6DSO_IF_INC;
	res = write_reg(LSM6DSO_CTRL3_C, regv);
	if (!res) {
		commands_printf("LSM6DSO BDU Config FAILED");
		return false;
	}

	terminal_register_command_callback(
			"lsm_read_reg",
			"Read register of the LSM6DSO",
			"[reg]",
			terminal_read_reg);

	lsm6dso_thread_ref = chThdCreateStatic(work_area, work_area_size, NORMALPRIO, lsm6dso_thread, NULL);

	return true;
}

void lsm6dso_stop(void) {
	if (lsm6dso_thread_ref != NULL) {
		chThdTerminate(lsm6dso_thread_ref);
		chThdWait(lsm6dso_thread_ref);
	}
	lsm6dso_thread_ref = NULL;
	terminal_unregister_callback(terminal_read_reg);
}

void lsm6dso_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	read_callback = func;
}

static bool read_reg(uint8_t reg, uint8_t *res) {
	bool ok = false;

	if (m_i2c_bb) {
		uint8_t txb[1];
		uint8_t rxb[1];
		txb[0] = reg;
		ok = i2c_bb_tx_rx(m_i2c_bb, lsm6dso_addr, txb, 1, rxb, 1);

		if (ok) {
			*res = rxb[0];
		}
	} else if (m_spi_bb) {
		chMtxLock(&(m_spi_bb->mutex));
		spi_bb_begin(m_spi_bb);
		spi_bb_exchange_8_mode_3(m_spi_bb, reg | 0x80);
		spi_bb_delay();
		*res = spi_bb_exchange_8_mode_3(m_spi_bb, 0);
		spi_bb_end(m_spi_bb);
		chMtxUnlock(&(m_spi_bb->mutex));
		ok = true;
	} else if (m_hwspi_dev) {
		uint8_t txb[2] = {reg | 0x80, 0};
		uint8_t rxb[2];
		spiAcquireBus(m_hwspi_dev);
		spiSelect(m_hwspi_dev);
		spiExchange(m_hwspi_dev, 2, txb, rxb);
		spiUnselect(m_hwspi_dev);
		spiReleaseBus(m_hwspi_dev);
		*res = rxb[1];
		ok = true;
	}

	return ok;
}

static bool write_reg(uint8_t reg, uint8_t value) {
	bool ok = false;

	if (m_i2c_bb) {
		uint8_t txb[2];
		uint8_t rxb[1];
		txb[0] = reg;
		txb[1] = value;
		ok = i2c_bb_tx_rx(m_i2c_bb, lsm6dso_addr, txb, 2, rxb, 1);
	} else if (m_spi_bb) {
		chMtxLock(&(m_spi_bb->mutex));
		spi_bb_begin(m_spi_bb);
		spi_bb_exchange_8_mode_3(m_spi_bb, reg);
		spi_bb_delay();
		spi_bb_exchange_8_mode_3(m_spi_bb, value);
		spi_bb_end(m_spi_bb);
		chMtxUnlock(&(m_spi_bb->mutex));
		ok = true;
	} else if (m_hwspi_dev) {
		uint8_t txb[2] = {reg, value};
		spiAcquireBus(m_hwspi_dev);
		spiSelect(m_hwspi_dev);
		spiSend(m_hwspi_dev, 2, txb);
		spiUnselect(m_hwspi_dev);
		spiReleaseBus(m_hwspi_dev);
		ok = true;
	}

	return ok;
}

static bool read_gyro_accel(int16_t *gyro, int16_t *accel) {
	uint8_t rxb[12];
	bool ok = false;

	if (m_i2c_bb) {
		uint8_t txb[1];
		txb[0] = LSM6DSO_OUTX_L_G;
		ok = i2c_bb_tx_rx(m_i2c_bb, lsm6dso_addr, txb, 1, rxb, 12);
	} else if (m_spi_bb) {
		chMtxLock(&(m_spi_bb->mutex));
		spi_bb_begin(m_spi_bb);
		spi_bb_exchange_8_mode_3(m_spi_bb, LSM6DSO_OUTX_L_G | 0x80);
		spi_bb_delay();
		for (int i = 0; i < 12; i++) {
			rxb[i] = spi_bb_exchange_8_mode_3(m_spi_bb, 0);
		}
		spi_bb_end(m_spi_bb);
		chMtxUnlock(&(m_spi_bb->mutex));
		ok = true;
	} else if (m_hwspi_dev) {
		uint8_t txb[13];
		uint8_t rxb_full[13];
		memset(txb, 0, 13);
		txb[0] = LSM6DSO_OUTX_L_G | 0x80;
		spiAcquireBus(m_hwspi_dev);
		spiSelect(m_hwspi_dev);
		spiExchange(m_hwspi_dev, 13, txb, rxb_full);
		spiUnselect(m_hwspi_dev);
		spiReleaseBus(m_hwspi_dev);
		memcpy(rxb, rxb_full + 1, 12);
		ok = true;
	}

	if (ok) {
		gyro[0] = (int16_t)((uint16_t)rxb[1] << 8) | rxb[0];
		gyro[1] = (int16_t)((uint16_t)rxb[3] << 8) | rxb[2];
		gyro[2] = (int16_t)((uint16_t)rxb[5] << 8) | rxb[4];

		accel[0] = (int16_t)((uint16_t)rxb[7] << 8) | rxb[6];
		accel[1] = (int16_t)((uint16_t)rxb[9] << 8) | rxb[8];
		accel[2] = (int16_t)((uint16_t)rxb[11] << 8) | rxb[10];
	}

	return ok;
}

static void terminal_read_reg(int argc, const char **argv) {
	if (argc == 2) {
		int reg = -1;
		sscanf(argv[1], "%d", &reg);

		if (reg >= 0) {
			uint8_t res;
			if (read_reg(reg, &res)) {
				commands_printf("Reg 0x%02x: %d (0x%02x)\n", reg, res, res);
			} else {
				commands_printf("Read failed");
			}
		} else {
			commands_printf("Invalid register");
		}
	} else {
		commands_printf("Usage: lsm_read_reg [reg]");
	}
}

static THD_FUNCTION(lsm6dso_thread, arg) {
	(void)arg;
	chRegSetThreadName("LSM6DSO");

	const systime_t interval = US2ST(1000000 / rate_hz) - 1;

	while (!chThdShouldTerminateX()) {
		int16_t gyro_raw[3];
		int16_t accel_raw[3];

		if (read_gyro_accel(gyro_raw, accel_raw)) {
			float tmp_accel[3], tmp_gyro[3], tmp_mag[3];

			// LSM6DSO sensitivity: ±16g -> 0.488 mg/LSB, ±2000dps -> 70 mdps/LSB
			tmp_accel[0] = (float)accel_raw[0] * 0.488f / 1000.0f;
			tmp_accel[1] = (float)accel_raw[1] * 0.488f / 1000.0f;
			tmp_accel[2] = (float)accel_raw[2] * 0.488f / 1000.0f;

			tmp_gyro[0] = (float)gyro_raw[0] * 70.0f / 1000.0f;
			tmp_gyro[1] = (float)gyro_raw[1] * 70.0f / 1000.0f;
			tmp_gyro[2] = (float)gyro_raw[2] * 70.0f / 1000.0f;

			memset(tmp_mag, 0, sizeof(tmp_mag));

			if (read_callback) {
				read_callback(tmp_accel, tmp_gyro, tmp_mag);
			}
		}

		chThdSleep(interval);
	}
}
