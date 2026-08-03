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

#include "lsm6dso_wrapper.h"
#include "utils_math.h"

#include <stdio.h>
#include <string.h>

// Threads
static THD_FUNCTION(lsm6dso_thread, arg);

// Private functions
static bool reset_init_lsm6dso(LSM6DSO_STATE *s);
static void user_delay_ms(uint32_t ms);
static bool read_reg(LSM6DSO_STATE *s, uint8_t reg, uint8_t *res);
static bool write_reg(LSM6DSO_STATE *s, uint8_t reg, uint8_t value);
static bool read_gyro_accel(LSM6DSO_STATE *s, uint8_t *res);

static uint8_t get_lsm6dso_acc_odr(int rate_hz);
static uint8_t get_lsm6dso_gyro_odr(int rate_hz);

void lsm6dso_wrapper_init(LSM6DSO_STATE *s, stkalign_t *work_area, size_t work_area_size) {
	s->read_callback = 0;
	s->rate_hz = MAX(s->rate_hz, 1);

	if (s->sensor.interface == LSM6DSO_SPI_INTF) {
		s->rate_hz = MIN(s->rate_hz, 5000);
	} else {
		s->rate_hz = MIN(s->rate_hz, 1000);
	}

	if (reset_init_lsm6dso(s)) {
		s->should_stop = false;
		chThdCreateStatic(work_area, work_area_size, NORMALPRIO, lsm6dso_thread, s);
	}
}

void lsm6dso_wrapper_set_read_callback(LSM6DSO_STATE *s, void(*func)(float *accel, float *gyro, float *mag)) {
	s->read_callback = func;
}

void lsm6dso_wrapper_stop(LSM6DSO_STATE *s) {
	s->should_stop = true;
	while(s->is_running) {
		chThdSleep(1);
	}
}

static bool reset_init_lsm6dso(LSM6DSO_STATE *s) {
	s->sensor.delay_ms = user_delay_ms;

	// Try address A first
	s->sensor.addr = LSM6DSO_I2C_ADDR_A;
	uint8_t whoami = 0;
	bool ok = read_reg(s, LSM6DSO_WHO_AM_I, &whoami);

	if (!ok || whoami != LSM6DSO_WHO_AM_I_VALUE) {
		// Try address B
		s->sensor.addr = LSM6DSO_I2C_ADDR_B;
		ok = read_reg(s, LSM6DSO_WHO_AM_I, &whoami);
		if (!ok || whoami != LSM6DSO_WHO_AM_I_VALUE) {
			return false;
		}
	}

	// Software reset
	ok = write_reg(s, LSM6DSO_CTRL3_C, 0x01);
	if (!ok) return false;

	chThdSleep(MS2ST(100));

	// Block data update + register auto-increment
	ok = write_reg(s, LSM6DSO_CTRL3_C, LSM6DSO_BDU_BLOCK_UPDATE | LSM6DSO_IF_INC_ENABLED);
	if (!ok) return false;

	// Configure accelerometer: ±16g, ODR based on rate_hz
	uint8_t acc_odr = get_lsm6dso_acc_odr(s->rate_hz);
	uint8_t ctrl1_xl = (acc_odr << 4) | LSM6DSO_ACCEL_FS_16G;
	
	if (s->filter == IMU_FILTER_MEDIUM || s->filter == IMU_FILTER_HIGH) {
		ctrl1_xl |= (1 << 1); // Enable LPF2
	}
	
	ok = write_reg(s, LSM6DSO_CTRL1_XL, ctrl1_xl);
	if (!ok) return false;

	// Configure gyroscope: ±2000 dps, ODR
	uint8_t gyro_odr = get_lsm6dso_gyro_odr(s->rate_hz);
	uint8_t ctrl2_g = (gyro_odr << 4) | LSM6DSO_GYRO_FS_2000DPS;
	ok = write_reg(s, LSM6DSO_CTRL2_G, ctrl2_g);
	if (!ok) return false;

	// Accel LPF2 bandwidth configuration
	if (s->filter == IMU_FILTER_MEDIUM) {
		write_reg(s, LSM6DSO_CTRL8_XL, 0x40); // ODR/4
	} else if (s->filter == IMU_FILTER_HIGH) {
		write_reg(s, LSM6DSO_CTRL8_XL, 0x80); // ODR/8
	} else {
		write_reg(s, LSM6DSO_CTRL8_XL, 0x00); // ODR/2 (default)
	}

	// Gyro LPF1 configuration
	uint8_t ctrl6_c = 0;
	if (s->filter == IMU_FILTER_MEDIUM) {
		ctrl6_c = 0x01;
	} else if (s->filter == IMU_FILTER_HIGH) {
		ctrl6_c = 0x02;
	}
	write_reg(s, LSM6DSO_CTRL6_C, ctrl6_c);

	// Enable gyro LPF1 in CTRL4_C if filter is enabled
	uint8_t ctrl4_c = 0;
	if (s->filter != IMU_FILTER_LOW) {
		ctrl4_c = (1 << 1); // LPF1_SEL_G
	}
	write_reg(s, LSM6DSO_CTRL4_C, ctrl4_c);

	return true;
}

static bool read_reg(LSM6DSO_STATE *s, uint8_t reg, uint8_t *res) {
	if (s->sensor.interface == LSM6DSO_I2C_INTF) {
		return i2c_bb_tx_rx(&s->i2c_bb, s->sensor.addr, &reg, 1, res, 1);
	} else if (s->sensor.interface == LSM6DSO_SPI_INTF) {
		// SPI mode: read bit is MSB set
		uint8_t txb = reg | 0x80;
		spi_bb_exchange_8(&s->spi_bb, txb);
		*res = spi_bb_exchange_8(&s->spi_bb, 0);
		return true;
	}
	return false;
}

static bool write_reg(LSM6DSO_STATE *s, uint8_t reg, uint8_t value) {
	if (s->sensor.interface == LSM6DSO_I2C_INTF) {
		uint8_t txb[2] = {reg, value};
		uint8_t rxb[1];
		return i2c_bb_tx_rx(&s->i2c_bb, s->sensor.addr, txb, 2, rxb, 1);
	} else if (s->sensor.interface == LSM6DSO_SPI_INTF) {
		uint8_t txb = reg & 0x7F;
		spi_bb_exchange_8(&s->spi_bb, txb);
		spi_bb_exchange_8(&s->spi_bb, value);
		return true;
	}
	return false;
}

static bool read_gyro_accel(LSM6DSO_STATE *s, uint8_t *res) {
	if (s->sensor.interface == LSM6DSO_I2C_INTF) {
		uint8_t reg = LSM6DSO_OUTX_L_G;
		return i2c_bb_tx_rx(&s->i2c_bb, s->sensor.addr, &reg, 1, res, 12);
	} else if (s->sensor.interface == LSM6DSO_SPI_INTF) {
		uint8_t txb = LSM6DSO_OUTX_L_G | 0x80;
		spi_bb_exchange_8(&s->spi_bb, txb);
		for (int i = 0; i < 12; i++) {
			res[i] = spi_bb_exchange_8(&s->spi_bb, 0);
		}
		return true;
	}
	return false;
}

static void user_delay_ms(uint32_t ms) {
	chThdSleep(MS2ST(ms));
}

static uint8_t get_lsm6dso_acc_odr(int rate_hz) {
	if (rate_hz <= 13) return 0x01;      // 12.5 Hz
	else if (rate_hz <= 26) return 0x02; // 26 Hz
	else if (rate_hz <= 52) return 0x03; // 52 Hz
	else if (rate_hz <= 104) return 0x04; // 104 Hz
	else if (rate_hz <= 208) return 0x05; // 208 Hz
	else if (rate_hz <= 416) return 0x06; // 416 Hz
	else if (rate_hz <= 833) return 0x07; // 833 Hz
	else if (rate_hz <= 1666) return 0x08; // 1.66 kHz
	else if (rate_hz <= 3332) return 0x09; // 3.33 kHz
	return 0x0A; // 6.66 kHz
}

static uint8_t get_lsm6dso_gyro_odr(int rate_hz) {
	if (rate_hz <= 13) return 0x01;      // 12.5 Hz
	else if (rate_hz <= 26) return 0x02; // 26 Hz
	else if (rate_hz <= 52) return 0x03; // 52 Hz
	else if (rate_hz <= 104) return 0x04; // 104 Hz
	else if (rate_hz <= 208) return 0x05; // 208 Hz
	else if (rate_hz <= 416) return 0x06; // 416 Hz
	else if (rate_hz <= 833) return 0x07; // 833 Hz
	else if (rate_hz <= 1666) return 0x08; // 1.66 kHz
	else if (rate_hz <= 3332) return 0x09; // 3.33 kHz
	return 0x0A; // 6.66 kHz
}

static THD_FUNCTION(lsm6dso_thread, arg) {
	LSM6DSO_STATE *s = (LSM6DSO_STATE*)arg;

	chRegSetThreadName("LSM6DSO Sampling");

	s->is_running = true;

	systime_t iteration_timer = chVTGetSystemTimeX();
	const systime_t desired_interval = US2ST(1000000 / s->rate_hz);

	for(;;) {
		uint8_t rxb[12];
		bool res = read_gyro_accel(s, rxb);

		if (res) {
			// Parse gyro (first 6 bytes) and accel (last 6 bytes)
			float tmp_accel[3], tmp_gyro[3], tmp_mag[3];

			// Gyroscope: ±2000 dps = 70 mdps/LSB
			tmp_gyro[0] = (float)((int16_t)((uint16_t)rxb[1] << 8) + rxb[0]) * (2000.0f / 32768.0f);
			tmp_gyro[1] = (float)((int16_t)((uint16_t)rxb[3] << 8) + rxb[2]) * (2000.0f / 32768.0f);
			tmp_gyro[2] = (float)((int16_t)((uint16_t)rxb[5] << 8) + rxb[4]) * (2000.0f / 32768.0f);

			// Accelerometer: ±16 g = 0.488 mg/LSB
			tmp_accel[0] = (float)((int16_t)((uint16_t)rxb[7] << 8) + rxb[6]) * (16.0f / 32768.0f);
			tmp_accel[1] = (float)((int16_t)((uint16_t)rxb[9] << 8) + rxb[8]) * (16.0f / 32768.0f);
			tmp_accel[2] = (float)((int16_t)((uint16_t)rxb[11] << 8) + rxb[10]) * (16.0f / 32768.0f);

			// No magnetometer (6-axis IMU)
			memset(tmp_mag, 0, sizeof(tmp_mag));

			if (s->read_callback) {
				s->read_callback(tmp_accel, tmp_gyro, tmp_mag);
			}
		}

		if (s->should_stop) {
			s->is_running = false;
			return;
		}

		// Delay between loops
		iteration_timer += desired_interval;
		systime_t current_time = chVTGetSystemTimeX();
		systime_t remaining_sleep_time = iteration_timer - current_time;
		if (remaining_sleep_time > 0 && remaining_sleep_time < desired_interval) {
			chThdSleep(remaining_sleep_time);
		} else {
			iteration_timer = current_time;
			chThdSleep(desired_interval);
		}
	}
}
