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

#include "bmi270_wrapper.h"
#include "utils_math.h"

#include <string.h>

// Threads
static THD_FUNCTION(bmi270_thread, arg);

// Private functions
static bool reset_init_bmi270(BMI270_STATE *s);
static void user_delay_us(uint32_t period, void *intf_ptr);
static uint8_t get_bmi270_acc_odr(int rate_hz);
static uint8_t get_bmi270_gyro_odr(int rate_hz);

void bmi270_wrapper_init(BMI270_STATE *s, stkalign_t *work_area, size_t work_area_size) {
	s->read_callback = 0;
	s->rate_hz = MAX(s->rate_hz, 1);

	if (s->sensor.intf == BMI2_SPI_INTF) {
		s->rate_hz = MIN(s->rate_hz, 5000);
	} else {
		s->rate_hz = MIN(s->rate_hz, 1000);
	}

	if (reset_init_bmi270(s)) {
		s->should_stop = false;
		chThdCreateStatic(work_area, work_area_size, NORMALPRIO, bmi270_thread, s);
	}
}

void bmi270_wrapper_set_read_callback(BMI270_STATE *s, void(*func)(float *accel, float *gyro, float *mag)) {
	s->read_callback = func;
}

void bmi270_wrapper_stop(BMI270_STATE *s) {
	s->should_stop = true;
	while(s->is_running) {
		chThdSleep(1);
	}
}

static bool reset_init_bmi270(BMI270_STATE *s) {
	int8_t rslt;

	s->sensor.delay_us = user_delay_us;
	s->sensor.read_write_len = 32;

	rslt = bmi270_init(&(s->sensor));
	if (rslt != BMI2_OK) {
		return false;
	}

	uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
	rslt = bmi270_sensor_enable(sens_list, 2, &(s->sensor));
	if (rslt != BMI2_OK) {
		return false;
	}

	struct bmi2_sens_config cfg[2];
	cfg[0].type = BMI2_ACCEL;
	cfg[1].type = BMI2_GYRO;

	rslt = bmi2_get_sensor_config(cfg, 2, &(s->sensor));
	if (rslt != BMI2_OK) {
		return false;
	}

	cfg[0].cfg.acc.range = BMI2_ACC_RANGE_16G;
	cfg[1].cfg.gyr.range = BMI2_GYR_RANGE_2000;

	cfg[0].cfg.acc.odr = get_bmi270_acc_odr(s->rate_hz);
	cfg[1].cfg.gyr.odr = get_bmi270_gyro_odr(s->rate_hz);

	if (s->filter == IMU_FILTER_LOW) {
		cfg[0].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
		cfg[1].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
	} else if (s->filter == IMU_FILTER_MEDIUM) {
		cfg[0].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;
		cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;
		cfg[0].cfg.acc.odr = MIN(cfg[0].cfg.acc.odr + 1, BMI2_ACC_ODR_1600HZ);
		cfg[1].cfg.gyr.odr = MIN(cfg[1].cfg.gyr.odr + 1, BMI2_GYR_ODR_3200HZ);
	} else if (s->filter == IMU_FILTER_HIGH) {
		cfg[0].cfg.acc.bwp = BMI2_ACC_OSR4_AVG1;
		cfg[1].cfg.gyr.bwp = BMI2_GYR_OSR4_MODE;
		cfg[0].cfg.acc.odr = MIN(cfg[0].cfg.acc.odr + 2, BMI2_ACC_ODR_1600HZ);
		cfg[1].cfg.gyr.odr = MIN(cfg[1].cfg.gyr.odr + 2, BMI2_GYR_ODR_3200HZ);
	}

	cfg[0].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
	cfg[1].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
	cfg[1].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

	rslt = bmi2_set_sensor_config(cfg, 2, &(s->sensor));

	return rslt == BMI2_OK;
}

static void user_delay_us(uint32_t period, void *intf_ptr) {
	(void)intf_ptr;

	systime_t sleep = US2ST(period);
	if (sleep == 0) {
		sleep = 1;
	}

	chThdSleep(sleep);
}

static uint8_t get_bmi270_acc_odr(int rate_hz) {
	if (rate_hz <= 25) {
		return BMI2_ACC_ODR_25HZ;
	} else if (rate_hz <= 50) {
		return BMI2_ACC_ODR_50HZ;
	} else if (rate_hz <= 100) {
		return BMI2_ACC_ODR_100HZ;
	} else if (rate_hz <= 200) {
		return BMI2_ACC_ODR_200HZ;
	} else if (rate_hz <= 400) {
		return BMI2_ACC_ODR_400HZ;
	} else if (rate_hz <= 800) {
		return BMI2_ACC_ODR_800HZ;
	}

	return BMI2_ACC_ODR_1600HZ;
}

static uint8_t get_bmi270_gyro_odr(int rate_hz) {
	if (rate_hz <= 25) {
		return BMI2_GYR_ODR_25HZ;
	} else if (rate_hz <= 50) {
		return BMI2_GYR_ODR_50HZ;
	} else if (rate_hz <= 100) {
		return BMI2_GYR_ODR_100HZ;
	} else if (rate_hz <= 200) {
		return BMI2_GYR_ODR_200HZ;
	} else if (rate_hz <= 400) {
		return BMI2_GYR_ODR_400HZ;
	} else if (rate_hz <= 800) {
		return BMI2_GYR_ODR_800HZ;
	}

	return BMI2_GYR_ODR_1600HZ;
}

static THD_FUNCTION(bmi270_thread, arg) {
	BMI270_STATE *s = (BMI270_STATE*)arg;

	chRegSetThreadName("BMI270 Sampling");

	s->is_running = true;

	systime_t iteration_timer = chVTGetSystemTimeX();
	const systime_t desired_interval = US2ST(1000000 / s->rate_hz);

	for(;;) {
		struct bmi2_sens_data sensor_data;
		memset(&sensor_data, 0, sizeof(sensor_data));

		int8_t rslt = bmi2_get_sensor_data(&sensor_data, &(s->sensor));

		if (rslt == BMI2_OK &&
				(sensor_data.status & BMI2_DRDY_ACC) &&
				(sensor_data.status & BMI2_DRDY_GYR)) {
			float tmp_accel[3], tmp_gyro[3], tmp_mag[3];

			tmp_accel[0] = (float)sensor_data.acc.x * 16.0f / 32768.0f;
			tmp_accel[1] = (float)sensor_data.acc.y * 16.0f / 32768.0f;
			tmp_accel[2] = (float)sensor_data.acc.z * 16.0f / 32768.0f;

			tmp_gyro[0] = (float)sensor_data.gyr.x * 2000.0f / 32768.0f;
			tmp_gyro[1] = (float)sensor_data.gyr.y * 2000.0f / 32768.0f;
			tmp_gyro[2] = (float)sensor_data.gyr.z * 2000.0f / 32768.0f;

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
		systime_t remainin_sleep_time = iteration_timer - current_time;
		if (remainin_sleep_time > 0 && remainin_sleep_time < desired_interval) {
			// Sleep the remaining time.
			chThdSleep(remainin_sleep_time);
		}
		else {
			// Read was too slow or CPU was too buzy, reset the schedule.
			iteration_timer = current_time;
			chThdSleep(desired_interval);
		}
	}
}