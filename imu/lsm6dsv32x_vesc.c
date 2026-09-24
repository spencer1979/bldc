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

#include "lsm6dsv32x_vesc.h"
#include "commands.h"

#include <string.h>

static thread_t *lsm6dsv32x_thread_ref = NULL;
static i2c_bb_state *m_i2c_bb;
static spi_bb_state *m_spi_bb;
static SPIDriver *m_hwspi_dev;
static volatile uint16_t lsm6dsv32x_addr;
static int rate_hz = 1000;
static IMU_FILTER filter = IMU_FILTER_LOW;

static bool read_reg(uint8_t reg, uint8_t *res);
static bool write_reg(uint8_t reg, uint8_t value);
static bool read_gyro_accel(int16_t *gyro, int16_t *accel);
static THD_FUNCTION(lsm6dsv32x_thread, arg);

static void(*read_callback)(float *accel, float *gyro, float *mag) = 0;

void lsm6dsv32x_vesc_set_rate_hz(int hz) {
	rate_hz = hz;
}

void lsm6dsv32x_vesc_set_filter(IMU_FILTER f) {
	filter = f;
}

bool lsm6dsv32x_vesc_init(i2c_bb_state *i2c_state, spi_bb_state *spi_state, SPIDriver *spi_hw,
		stkalign_t *work_area, size_t work_area_size) {

	read_callback = 0;
	m_i2c_bb = i2c_state;
	m_spi_bb = spi_state;
	m_hwspi_dev = spi_hw;

	uint8_t id = 0;
	lsm6dsv32x_addr = LSM6DSV32X_VESC_ADDR_A;
	bool ok = read_reg(LSM6DSV32X_VESC_WHO_AM_I_REG, &id);
	if (!ok || id != LSM6DSV32X_VESC_WHO_AM_I_VALUE) {
		commands_printf("LSM6DSV32X address A failed, trying B (rx: %d)", id);
		lsm6dsv32x_addr = LSM6DSV32X_VESC_ADDR_B;
		ok = read_reg(LSM6DSV32X_VESC_WHO_AM_I_REG, &id);
		if (!ok || id != LSM6DSV32X_VESC_WHO_AM_I_VALUE) {
			commands_printf("LSM6DSV32X address B failed (rx: %d)", id);
			return false;
		}
	}

	/* Poll mode: always use max ODR so reads always get the freshest sample */
	static const struct { uint16_t hz; uint8_t code; } odr_ladder[] = {
		{8, 0x2}, {15, 0x3}, {30, 0x4}, {60, 0x5}, {120, 0x6}, {240, 0x7},
		{480, 0x8}, {960, 0x9}, {1920, 0xA}, {3840, 0xB}, {7680, 0xC},
	};
	uint16_t odr_hz = odr_ladder[10].hz;
	uint8_t odr = odr_ladder[10].code;

	/* Accel LPF2: choose widest cutoff <= sample_rate/div */
	uint8_t div = (filter == IMU_FILTER_HIGH) ? 8 : (filter == IMU_FILTER_MEDIUM) ? 4 : 2;
	uint16_t cutoff = (uint16_t)rate_hz / div;
	bool lpf2_en = odr_hz / 2 > cutoff;
	uint8_t lpf2_bw = 0;
	if (lpf2_en) {
		static const uint16_t lpf2_n[8] = {4, 10, 20, 45, 100, 200, 400, 800};
		lpf2_bw = 7;
		for (uint8_t code = 0; code < 8; code++) {
			if (odr_hz / lpf2_n[code] <= cutoff) {
				lpf2_bw = code;
				break;
			}
		}
	}

	bool lpf1_en = filter != IMU_FILTER_LOW;
	uint8_t lpf1_bw = (filter == IMU_FILTER_HIGH) ? 0x2 : 0x0;

	ok = write_reg(LSM6DSV32X_VESC_CTRL3, LSM6DSV32X_VESC_BDU | LSM6DSV32X_VESC_IF_INC);
	ok = ok && write_reg(LSM6DSV32X_VESC_CTRL8, (uint8_t)((lpf2_bw << 5) | LSM6DSV32X_VESC_CTRL8_MUST_SET | LSM6DSV32X_VESC_FS_XL_32G));
	ok = ok && write_reg(LSM6DSV32X_VESC_CTRL9, lpf2_en ? 0x08 : 0);
	ok = ok && write_reg(LSM6DSV32X_VESC_CTRL6, (uint8_t)((lpf1_bw << 4) | LSM6DSV32X_VESC_FS_G_4000DPS));
	ok = ok && write_reg(LSM6DSV32X_VESC_CTRL7, lpf1_en ? 0x01 : 0);
	ok = ok && write_reg(LSM6DSV32X_VESC_CTRL4, 0);
	ok = ok && write_reg(LSM6DSV32X_VESC_CTRL1, odr);
	ok = ok && write_reg(LSM6DSV32X_VESC_CTRL2, odr);

	if (!ok) {
		commands_printf("LSM6DSV32X config failed");
		return false;
	}

	lsm6dsv32x_thread_ref = chThdCreateStatic(work_area, work_area_size,
			NORMALPRIO, lsm6dsv32x_thread, NULL);

	return true;
}

void lsm6dsv32x_vesc_stop(void) {
	if (lsm6dsv32x_thread_ref != NULL) {
		chThdTerminate(lsm6dsv32x_thread_ref);
		chThdWait(lsm6dsv32x_thread_ref);
	}
	lsm6dsv32x_thread_ref = NULL;
}

void lsm6dsv32x_vesc_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	read_callback = func;
}

static bool read_reg(uint8_t reg, uint8_t *res) {
	bool ok = false;

	if (m_i2c_bb) {
		uint8_t txb[1] = {reg};
		uint8_t rxb[1];
		ok = i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 1, rxb, 1);
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
		uint8_t txb[2] = {(uint8_t)(reg | 0x80), 0};
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
		uint8_t txb[2] = {reg, value};
		ok = i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 2, NULL, 0);
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
		uint8_t txb[1] = {LSM6DSV32X_VESC_OUTX_L_G};
		ok = i2c_bb_tx_rx(m_i2c_bb, lsm6dsv32x_addr, txb, 1, rxb, 12);
	} else if (m_spi_bb) {
		chMtxLock(&(m_spi_bb->mutex));
		spi_bb_begin(m_spi_bb);
		spi_bb_exchange_8_mode_3(m_spi_bb, LSM6DSV32X_VESC_OUTX_L_G | 0x80);
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
		memset(txb, 0, sizeof(txb));
		txb[0] = LSM6DSV32X_VESC_OUTX_L_G | 0x80;
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

static THD_FUNCTION(lsm6dsv32x_thread, arg) {
	(void)arg;
	chRegSetThreadName("LSM6DSV32X");

	const systime_t interval = US2ST(1000000 / rate_hz) - 1;

	while (!chThdShouldTerminateX()) {
		int16_t gyro_raw[3];
		int16_t accel_raw[3];

		if (read_gyro_accel(gyro_raw, accel_raw)) {
			float tmp_accel[3], tmp_gyro[3], tmp_mag[3];

			/* LSM6DSV32X: ±32g -> 0.976 mg/LSB, ±4000dps -> 140 mdps/LSB */
			tmp_accel[0] = (float)accel_raw[0] * 0.976f / 1000.0f;
			tmp_accel[1] = (float)accel_raw[1] * 0.976f / 1000.0f;
			tmp_accel[2] = (float)accel_raw[2] * 0.976f / 1000.0f;

			tmp_gyro[0] = (float)gyro_raw[0] * 140.0f / 1000.0f;
			tmp_gyro[1] = (float)gyro_raw[1] * 140.0f / 1000.0f;
			tmp_gyro[2] = (float)gyro_raw[2] * 140.0f / 1000.0f;

			memset(tmp_mag, 0, sizeof(tmp_mag));

			if (read_callback) {
				read_callback(tmp_accel, tmp_gyro, tmp_mag);
			}
		}

		chThdSleep(interval);
	}
}
