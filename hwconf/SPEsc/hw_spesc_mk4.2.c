/*
	Copyright 2012-2016 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
	*/

#include "hw.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "terminal.h"
#include "commands.h"
#include "mc_interface.h"
#include "ledpwm.h"
// Variables
static volatile bool i2c_running = false;
// fan control thread
// 可以調成 64，但這會讓 fan_control_thread 的 stack 只有 64 bytes，對於大多數簡單 thread 可能夠用，但如果 thread 內部有呼叫較多函式或用到較多區域變數，可能會有 stack overflow 風險。
// 建議測試後觀察是否有異常（如 thread 異常終止），若有問題再調回 128。

static THD_WORKING_AREA(fan_control_thread_wa, 96); // 96 bytes stack size, adjust as needed
static THD_FUNCTION(fan_control_thread, arg);

// I2C configuration
static const I2CConfig i2cfg = {
	OPMODE_I2C,
	100000,
	STD_DUTY_CYCLE};

void hw_init_gpio(void)
{
	// GPIO clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// LEDs
	palSetPadMode(GPIOB, 0,
				  PAL_MODE_OUTPUT_PUSHPULL |
					  PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(GPIOB, 1,
				  PAL_MODE_OUTPUT_PUSHPULL |
					  PAL_STM32_OSPEED_HIGHEST);

	// External Buzzer (using servo pin!)

	palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN,
				  PAL_MODE_OUTPUT_PUSHPULL |
					  PAL_STM32_OSPEED_HIGHEST);
	
	palSetPadMode(LIGHT_BACK_GPIO , LIGHT_BACK_PIN,
				PAL_MODE_OUTPUT_PUSHPULL |
				PAL_STM32_OSPEED_HIGHEST); // Rear light
	palSetPadMode(LIGHT_FRONT_GPIO, LIGHT_FRONT_PIN,
				PAL_MODE_OUTPUT_PUSHPULL |
				PAL_STM32_OSPEED_HIGHEST); // front light
	palSetPadMode(FAN_GPIO, FAN_PIN,
				PAL_MODE_OUTPUT_PUSHPULL |
				PAL_STM32_OSPEED_HIGHEST); // fan

	palSetPadMode(EXTERNAL_DCDC_GPIO, EXTERNAL_DCDC_PIN,
				PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST );// external dc-dc board power on/off control 
	// External board control default off 
	EXT_DCDC_OFF();
	//test buzzer 
	EXT_BUZZER_ON();
	chThdSleepMilliseconds(100);
	EXT_BUZZER_OFF();
	chThdSleepMilliseconds(100);
	EXT_BUZZER_ON();
	chThdSleepMilliseconds(100);
	EXT_BUZZER_OFF();
	//Test Fan 
	FAN_ON();
	LIGHT_FRONT_ON();
	LIGHT_BACK_ON();
	chThdSleepMilliseconds(200);
	LIGHT_BACK_OFF();
	LIGHT_FRONT_OFF();
	chThdSleepMilliseconds(200);
	LIGHT_FRONT_ON();
	LIGHT_BACK_ON();
	chThdSleepMilliseconds(200);
	LIGHT_BACK_OFF();
	LIGHT_FRONT_OFF();
	// FAN control default off 
	FAN_OFF();
	// External dcdc control default on 
	EXT_DCDC_ON();
	// GPIOA Configuration: Channel 1 to 3 as alternate function push-pull
	palSetPadMode(GPIOA, 8, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 9, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

	palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
	palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) | PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

	// Hall sensors
	palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
	palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

	// ADC Pins
	palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

	palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
	palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);

	
}

void hw_setup_adc_channels(void)
{
	// ADC1 regular channels
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, ADC_SampleTime_15Cycles);

	// ADC2 regular channels
	ADC_RegularChannelConfig(ADC2, ADC_Channel_1, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_6, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_0, 5, ADC_SampleTime_15Cycles);

	// ADC3 regular channels
	ADC_RegularChannelConfig(ADC3, ADC_Channel_2, 1, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_3, 3, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, ADC_SampleTime_15Cycles);
	ADC_RegularChannelConfig(ADC3, ADC_Channel_1, 5, ADC_SampleTime_15Cycles);

	// Injected channels
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, ADC_SampleTime_15Cycles);
	ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, ADC_SampleTime_15Cycles);
	// fan control thread
	chThdCreateStatic(fan_control_thread_wa, sizeof(fan_control_thread_wa), LOWPRIO, fan_control_thread, NULL);
}

void hw_start_i2c(void)
{
	i2cAcquireBus(&HW_I2C_DEV);

	if (!i2c_running)
	{
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
					  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
						  PAL_STM32_OTYPE_OPENDRAIN |
						  PAL_STM32_OSPEED_MID1 |
						  PAL_STM32_PUDR_PULLUP);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
					  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
						  PAL_STM32_OTYPE_OPENDRAIN |
						  PAL_STM32_OSPEED_MID1 |
						  PAL_STM32_PUDR_PULLUP);

		i2cStart(&HW_I2C_DEV, &i2cfg);
		i2c_running = true;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

void hw_stop_i2c(void)
{
	i2cAcquireBus(&HW_I2C_DEV);

	if (i2c_running)
	{
		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

		i2cStop(&HW_I2C_DEV);
		i2c_running = false;
	}

	i2cReleaseBus(&HW_I2C_DEV);
}

/**
 * Try to restore the i2c bus
 */
void hw_try_restore_i2c(void)
{
	if (i2c_running)
	{
		i2cAcquireBus(&HW_I2C_DEV);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
					  PAL_STM32_OTYPE_OPENDRAIN |
						  PAL_STM32_OSPEED_MID1 |
						  PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
					  PAL_STM32_OTYPE_OPENDRAIN |
						  PAL_STM32_OSPEED_MID1 |
						  PAL_STM32_PUDR_PULLUP);

		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		chThdSleep(1);

		for (int i = 0; i < 16; i++)
		{
			palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
			palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
			chThdSleep(1);
		}

		// Generate start then stop condition
		palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
		chThdSleep(1);
		palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
		chThdSleep(1);
		palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

		palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
					  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
						  PAL_STM32_OTYPE_OPENDRAIN |
						  PAL_STM32_OSPEED_MID1 |
						  PAL_STM32_PUDR_PULLUP);

		palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
					  PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
						  PAL_STM32_OTYPE_OPENDRAIN |
						  PAL_STM32_OSPEED_MID1 |
						  PAL_STM32_PUDR_PULLUP);

		HW_I2C_DEV.state = I2C_STOP;
		i2cStart(&HW_I2C_DEV, &i2cfg);

		i2cReleaseBus(&HW_I2C_DEV);
	}
}

#ifdef HW_HAS_4_WIRE_FAN
// 1. 初始化 PWM 腳位（建議在 hw_init_gpio() 裡呼叫一次）
static void fan_pwm_init(void) {
    // 開啟 TIM2 時鐘
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    // 設定 PB3 為 TIM2_CH2 alternate function
    palSetPadMode(GPIOB, 3, PAL_MODE_ALTERNATE(GPIO_AF_TIM2) | PAL_STM32_OSPEED_HIGHEST);

    // 設定 TIM2 為 PWM 輸出
    TIM_TimeBaseInitTypeDef tim_init;
    TIM_OCInitTypeDef oc_init;

    tim_init.TIM_Period = 999;      // PWM 週期 (1kHz)
    tim_init.TIM_Prescaler = 83;    // 84MHz/84=1MHz, 1MHz/1000=1kHz
    tim_init.TIM_ClockDivision = 0;
    tim_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &tim_init);

    oc_init.TIM_OCMode = TIM_OCMode_PWM1;
    oc_init.TIM_OutputState = TIM_OutputState_Enable;
    oc_init.TIM_Pulse = 0; // 預設 0% 占空比
    oc_init.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM2, &oc_init);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);
}

void set_fan_pwm(float duty) {
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    TIM2->CCR2 = (uint16_t)(duty * 999.0f); // 999 = Period
}

#endif

static THD_FUNCTION(fan_control_thread, arg)
{
	(void)arg;

	chRegSetThreadName("fan_control_thread");

#ifdef HW_HAS_4_WIRE_FAN
	// PWM initialization for fan control
	fan_pwm_init();
#endif

	float temp_t;
	for (;;)
	{
		temp_t = mc_interface_temp_fet_filtered();
#ifdef HW_HAS_4_WIRE_FAN
		// 依溫度線性調整風扇轉速
		const float t_start = mc_interface_get_configuration()->bms.t_limit_start;
		const float t_end = mc_interface_get_configuration()->bms.t_limit_end;
		float duty = 0.0f;
		if (temp_t > t_start) {
			duty = (temp_t - t_start) / (t_end - t_start);
			if (duty > 1.0f) duty = 1.0f;
			if (duty < 0.2f) duty = 0.2f; // 最低轉速
		}
		set_fan_pwm(duty);
		if (duty > 0.0f) {
			FAN_ON();
		} else {
			FAN_OFF();
		}
#else
		if (temp_t > mc_interface_get_configuration()->bms.t_limit_start)
		{
			FAN_ON();
		}
		else
		{
			FAN_OFF();
		}
#endif
		chThdSleepMilliseconds(2000);
	}
}
