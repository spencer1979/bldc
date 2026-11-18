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

/*
 * GD32F405 ADC differences, used by mcpwm_foc_init() (GD32F405_COMPAT only).
 * References: GD32F4xx User Manual Rev3.0 chapter 14, GD32F405 datasheet
 * Rev2.8 table 4-29.
 *
 * - The ADC clock must not exceed 40 MHz. The STM32 setting (PCLK2 / 2 =
 *   42 MHz) is out of spec, so ADCCK[2:0] in ADC_SYNCCTL (= ADC->CCR bits
 *   18:16, bit 18 does not exist on the STM32) is set to HCLK / 5 = 33.6 MHz.
 *   One 5-channel triple sequence (15 + 12 cycles each) then takes 4.0 us
 *   instead of 3.2 us. Override with GD32_ADC_CLOCK_SEL (0-7, see
 *   ADC_SYNCCTL ADCCK in the user manual) if needed.
 * - The ADC has a calibration (CLB/RSTCLB in ADC_CTL1 = ADC->CR2 bits 2/3)
 *   that "should be performed before starting A/D conversion" (14.4.1).
 *   The STM32 code never does this.
 *
 * These functions are only built for GD32F405_COMPAT. They live in their
 * own file so that mcpwm_foc.c keeps its line numbers (the FOC profiling
 * macros record __LINE__) and the STM32 binaries stay identical.
 */

#include "conf_general.h"

#ifdef GD32F405_COMPAT

#include "ch.h"
#include "stm32f4xx_conf.h"

#ifndef GD32_ADC_CLOCK_SEL
#define GD32_ADC_CLOCK_SEL				4 // 3'b100: HCLK / 5
#endif
#define GD32_ADC_SYNCCTL_ADCCK_MASK		(7U << 16)
#define GD32_ADC_CTL1_CLB				(1U << 2)
#define GD32_ADC_CTL1_RSTCLB			(1U << 3)

void gd32_adc_set_clock(void) {
	ADC->CCR = (ADC->CCR & ~GD32_ADC_SYNCCTL_ADCCK_MASK) |
			((uint32_t)GD32_ADC_CLOCK_SEL << 16);
}

static void calibrate(ADC_TypeDef *adc) {
	adc->CR2 |= GD32_ADC_CTL1_RSTCLB;
	for (volatile int i = 0;i < 100000 && (adc->CR2 & GD32_ADC_CTL1_RSTCLB);i++) {}

	adc->CR2 |= GD32_ADC_CTL1_CLB;
	// tCAL = 131 ADC clocks (about 4 us). The timeout only protects against
	// a part that does not implement CLB.
	for (volatile int i = 0;i < 100000 && (adc->CR2 & GD32_ADC_CTL1_CLB);i++) {}
}

/*
 * Call after ADC_Cmd(ADCx, ENABLE) and before the timers start triggering
 * conversions.
 */
void gd32_adc_calibrate_all(void) {
	// ADCON is set. Wait >= 14 ADC clocks (and tSU = 1 us) before calibrating.
	chSysPolledDelayX(SYSTEM_CORE_CLOCK / 100000); // 10 us
	calibrate(ADC1);
	calibrate(ADC2);
	calibrate(ADC3);
}

#endif
