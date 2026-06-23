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

#ifndef IMU_BMI270_WRAPPER_H_
#define IMU_BMI270_WRAPPER_H_

#include "ch.h"
#include "hal.h"

#include <stdint.h>
#include <stdbool.h>

#include "i2c_bb.h"
#include "bmi270.h"

typedef struct {
	void(*read_callback)(float *accel, float *gyro, float *mag);
	struct bmi2_dev sensor;
	volatile bool is_running;
	volatile bool should_stop;
	int rate_hz;
	IMU_FILTER filter;
} BMI270_STATE;

void bmi270_wrapper_init(BMI270_STATE *s, stkalign_t *work_area, size_t work_area_size);
void bmi270_wrapper_set_read_callback(BMI270_STATE *s, void(*func)(float *accel, float *gyro, float *mag));
void bmi270_wrapper_stop(BMI270_STATE *s);

#endif /* IMU_BMI270_WRAPPER_H_ */