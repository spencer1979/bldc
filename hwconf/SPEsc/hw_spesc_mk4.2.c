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
#include "mcpwm_foc.h"
// Variables
static volatile bool i2c_running = false;
// fan control thread
static THD_WORKING_AREA(fan_control_thread_wa, 128);
static THD_FUNCTION(fan_control_thread, arg);

// startup tone thread
static THD_WORKING_AREA(startup_tone_thread_wa, 96);
static THD_FUNCTION(startup_tone_thread, arg);
static void play_tone_decay(int channel, float freq, float time_s, float vol_start);

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

	// startup tone thread
	chThdCreateStatic(startup_tone_thread_wa, sizeof(startup_tone_thread_wa), LOWPRIO, startup_tone_thread, NULL);
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
static THD_FUNCTION(fan_control_thread, arg)
{
	(void)arg;
	chRegSetThreadName("fan_control_thread");
	float temp_t;
	for (;;)
	{
		temp_t = mc_interface_temp_fet_filtered();
		if ( temp_t > mc_interface_get_configuration()->bms.t_limit_start )
		{
			FAN_ON();
		}
		else
		{
			FAN_OFF();
		}

		chThdSleepMilliseconds(2000);
	}
}

static void play_tone_decay(int channel, float freq, float time_s, float vol_start)
{
	// 分為 30 個步驟做指數衰減
	const int steps = 30;
	float vol = vol_start;
	int ms_per_step = (int)(time_s * 1000.0f / (float)steps);
	if (ms_per_step < 1) {
		ms_per_step = 1;
	}
	for (int i = 0; i < steps; i++) {
		vol *= 0.93f;
		mcpwm_foc_play_tone(channel, freq, vol);
		chThdSleepMilliseconds(ms_per_step);
	}
}

static THD_FUNCTION(startup_tone_thread, arg)
{
	(void)arg;
	chRegSetThreadName("startup_tone_thread");
	
	// 等待系統初始化
	chThdSleepMilliseconds(3000);

	// 定義音符（時長，頻率Hz）- 簡短 2 秒啟動音效
	typedef struct {
		float dur;
		float freq;
	} note_t;

	// DJI Mavic Air 2 真實啟動音效 - 從 MIDI 檔案分析
	static const note_t melody[] = {
		// 多重音符同時播放，取主要音符
		{0.36f, 1046.502f}, // C6 (0.000s, 0.360s) - 主導音
		{0.256f, 523.251f}, // C5 (0.012s, 0.256s) - 同時播放
		{0.07f, 415.305f},  // G#5 (0.233s, 0.070s) - 短音
		{0.244f, 1479.978f}, // F#7 (0.244s, 0.244s) - 高音
		{0.279f, 587.330f}, // D5 (0.244s, 0.279s)
		{0.407f, 783.991f}, // G5 (0.430s, 0.407s) - 較強音 (力度81)
	};

	const int notes = (int)(sizeof(melody) / sizeof(melody[0]));

	// 播放旋律 - 增大音量
	for (int i = 0; i < notes; i++) {
		float dur = melody[i].dur;
		float freq = melody[i].freq;
		// 若 freq 小於 1Hz 視為休止符
		if (freq < 1.0f) {
			chThdSleepMilliseconds((int)(dur * 1000.0f));
		} else {
			play_tone_decay(0, freq, dur, 3.5f); // 頻道0，起始音量 3.5 (更大聲)
		}
		// 音符間短暫間隔 - DJI 原版幾乎無間隔
		chThdSleepMilliseconds(10);
	}

	// 停止所有音效
	mcpwm_foc_stop_audio(1);
	
	// 結束執行緒
	chThdExit(MSG_OK);
}
