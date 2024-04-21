/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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
#ifndef HW_SPESC_H

#define HW_SPESC_H
// SPEsc Hardware Vesion 
#define HW_VERSION_MAJOR  3
#define HW_VERSION_MINOR  3 // V3.4 is same V3.3

#define str(x) #x
#define strname(name) str(name)

//SPESC Hardware define



/** BMI160 and LSM6DS3 are pin to pin , only the angle difference is 90 degrees. */
//#define HW_USE_LSM6DS3_IMU

/**If you uncomment HW_MOS_PACKAGE_TO263 will undefine HIGH POWER */
//#define HW_MOS_PACKAGE_TO263 

#ifdef HW_USE_LSM6DS3_IMU
    #define HW_IMU "LSM" //LSM6DS3
#else
    #define HW_IMU "BMI" //BMI160
#endif


#ifdef HW_MOS_PACKAGE_TO263
    #undef  HIGH_POWER
    #define HW_MOS_PACKAGE "TO263"
#else
    #define HW_MOS_PACKAGE "TOLL"
#endif

/**If you use high current, the current sensing resistor must be 3mohm ,And the packaging needs to be TOLL package.*/
#define HIGH_POWER 
// Define the hardware name
#ifdef HIGH_POWER
#define HW_NAME "SPEsc.3mR V" strname( HW_VERSION_MAJOR  )  "." strname(HW_VERSION_MINOR) "_" HW_IMU "_" HW_MOS_PACKAGE
#else
#define HW_NAME "SPEsc.5mR V" strname( HW_VERSION_MAJOR  )  "." strname(HW_VERSION_MINOR) "_" HW_IMU "_" HW_MOS_PACKAGE
#endif

#define INVERTED_SHUNT_POLARITY
#define HW_HAS_3_SHUNTS
#define HW_USE_INTERNAL_RC //We don't have an external crystal oscillator

#define HW_DEAD_TIME_NSEC 1000.0


// SPEsc Hardware pin configuration
#define LIGHT_FRONT_GPIO GPIOC
#define LIGHT_FRONT_PIN 5

#if (HW_VERSION_MAJOR >= 3 )
#define LIGHT_BACK_GPIO GPIOB
#define LIGHT_BACK_PIN 5
#else
#define LIGHT_BACK_GPIO GPIOB
#define LIGHT_BACK_PIN 7
#endif
#define FAN_GPIO GPIOC
#define FAN_PIN 12
#define EXTERNAL_DCDC_GPIO GPIOD
#define EXTERNAL_DCDC_PIN 2
// Macros
#define LED_GREEN_ON() palSetPad(GPIOB, 0)
#define LED_GREEN_OFF() palClearPad(GPIOB, 0)
#define LED_RED_ON() palSetPad(GPIOB, 1)
#define LED_RED_OFF() palClearPad(GPIOB, 1)

#define HAS_EXT_BUZZER 1

#if (HW_VERSION_MAJOR == 3 )

     #if (HW_VERSION_MINOR == 2 )
        #define HW_HAS_NO_CAN
        #define EXT_BUZZER_GPIO GPIOB
        #define EXT_BUZZER_PIN 9
        #define EXT_BUZZER_ON() palSetPad(EXT_BUZZER_GPIO, EXT_BUZZER_PIN) // uses can_tx
        #define EXT_BUZZER_OFF() palClearPad(EXT_BUZZER_GPIO, EXT_BUZZER_PIN)
    #elif (HW_VERSION_MINOR == 3 ) //Hardware version 3.3 or above uses TJA1015 CAN bus
        #define EXT_BUZZER_GPIO GPIOC
        #define EXT_BUZZER_PIN 13
        #define EXT_BUZZER_ON() palSetPad(EXT_BUZZER_GPIO, EXT_BUZZER_PIN) // uses can_tx
        #define EXT_BUZZER_OFF() palClearPad(EXT_BUZZER_GPIO, EXT_BUZZER_PIN)
    #else
        #define EXT_BUZZER_GPIO GPIOB
        #define EXT_BUZZER_PIN 6
        #define EXT_BUZZER_ON() palSetPad(EXT_BUZZER_GPIO, EXT_BUZZER_PIN)// uses servo pin for buzzer
        #define EXT_BUZZER_OFF() palClearPad(EXT_BUZZER_GPIO, EXT_BUZZER_PIN)
    #endif 

#else
    #define EXT_BUZZER_GPIO GPIOB
    #define EXT_BUZZER_PIN 6
    #define EXT_BUZZER_ON() palSetPad(EXT_BUZZER_GPIO, EXT_BUZZER_PIN)// uses servo pin for buzzer
    #define EXT_BUZZER_OFF() palClearPad(EXT_BUZZER_GPIO, EXT_BUZZER_PIN)
#endif

// Enable J17 J16 COB LED
#define LIGHT_FRONT_ON() palSetPad(LIGHT_FRONT_GPIO, LIGHT_FRONT_PIN)
#define LIGHT_FRONT_OFF() palClearPad(LIGHT_FRONT_GPIO, LIGHT_FRONT_PIN)
#define LIGHT_BACK_ON() palSetPad(LIGHT_BACK_GPIO, LIGHT_BACK_PIN) // V2 is PB7 , V3 is PC5
#define LIGHT_BACK_OFF() palClearPad(LIGHT_BACK_GPIO, LIGHT_BACK_PIN)
// 5V FAN control
#define FAN_ON() palSetPad(FAN_GPIO, FAN_PIN)
#define FAN_OFF() palClearPad(FAN_GPIO, FAN_PIN)
// External dcdc control
#define EXT_DCDC_ON() palSetPad(EXTERNAL_DCDC_GPIO, EXTERNAL_DCDC_PIN)
#define EXT_DCDC_OFF() palClearPad(EXTERNAL_DCDC_GPIO, EXTERNAL_DCDC_PIN)


/*
 * ADC Vector
 *
 * 0:	IN0		SENS1
 * 1:	IN1		SENS2
 * 2:	IN2		SENS3
 * 3:	IN10	CURR1
 * 4:	IN11	CURR2
 * 5:	IN12	CURR3
 * 6:	IN5		ADC_EXT1
 * 7:	IN6		ADC_EXT2
 * 8:	IN3		TEMP_PCB
 * 9:	IN14	TEMP_MOTOR
 * 10:	IN15	ADC_EXT3, Shutdown
 * 11:	IN13	AN_IN
 * 12:	Vrefint
 * 13:	IN0		SENS1
 * 14:	IN1		SENS2
 */

#define HW_ADC_CHANNELS 15
#define HW_ADC_INJ_CHANNELS 3
#define HW_ADC_NBR_CONV 5

// ADC Indexes
#define ADC_IND_SENS1 0
#define ADC_IND_SENS2 1
#define ADC_IND_SENS3 2
#define ADC_IND_CURR1 3
#define ADC_IND_CURR2 4
#define ADC_IND_CURR3 5
#define ADC_IND_VIN_SENS 11
#define ADC_IND_EXT 6
#define ADC_IND_EXT2 7
#define ADC_IND_TEMP_MOS 8
#define ADC_IND_TEMP_MOTOR 9
#define ADC_IND_VREFINT 12

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG 3.3
#endif
#ifndef VIN_R1
    #if (  HW_VERSION_MAJOR <=2)
        #define VIN_R1 56250.0
    #else
        #define VIN_R1 68000.0 // V3.1 Changed from 56000 to 68000 after 2022-08-02
    #endif
#endif

#ifndef VIN_R2
#define VIN_R2 2200.0
#endif

#ifndef CURRENT_AMP_GAIN
#define CURRENT_AMP_GAIN 20.0
#endif

#ifndef CURRENT_SHUNT_RES
    #ifdef HIGH_POWER
        #define CURRENT_SHUNT_RES 0.0003// Max current +/-(1.6V/0.3m ohm) /20= 275A Max  , we use 260A 
    #else
        #define CURRENT_SHUNT_RES 0.0005 // Max current +/-165A max
    #endif
#endif
// Input voltage
#define GET_INPUT_VOLTAGE() ((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
#define NTC_RES(adc_val) ((4095.0 * 10500.0) / adc_val - 10500.0)
#define NTC_TEMP(adc_ind) (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)

#define NTC_RES_MOTOR(adc_val) (10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta) (1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

// Voltage on ADC channel
#define ADC_VOLTS(ch) ((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE 1
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE 1
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE 1
#endif


// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO GPIOA
#define HW_ADC_EXT_PIN 5
#define HW_ADC_EXT2_GPIO GPIOA
#define HW_ADC_EXT2_PIN 6

// UART Peripheral
#define HW_UART_DEV SD3
#define HW_UART_GPIO_AF GPIO_AF_USART3
#define HW_UART_TX_PORT GPIOB
#define HW_UART_TX_PIN 10
#define HW_UART_RX_PORT GPIOB
#define HW_UART_RX_PIN 11

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER TIM4
#define HW_ICU_TIM_CLK_EN() RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV ICUD4
#define HW_ICU_CHANNEL ICU_CHANNEL_1
#define HW_ICU_GPIO_AF GPIO_AF_TIM4
#define HW_ICU_GPIO GPIOB
#define HW_ICU_PIN 6

// I2C Peripheral
#define HW_I2C_DEV I2CD2
#define HW_I2C_GPIO_AF GPIO_AF_I2C2
#define HW_I2C_SCL_PORT GPIOB
#define HW_I2C_SCL_PIN 10
#define HW_I2C_SDA_PORT GPIOB
#define HW_I2C_SDA_PIN 11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1 GPIOC
#define HW_HALL_ENC_PIN1 6
#define HW_HALL_ENC_GPIO2 GPIOC
#define HW_HALL_ENC_PIN2 7
#define HW_HALL_ENC_GPIO3 GPIOC
#define HW_HALL_ENC_PIN3 8
#define HW_ENC_TIM TIM3
#define HW_ENC_TIM_AF GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN() RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC EXTI_PinSource8
#define HW_ENC_EXTI_CH EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC TIM3_IRQHandler

// // NRF pins
// #define NRF_PORT_CSN			GPIOB
// #define NRF_PIN_CSN				12
// #define NRF_PORT_SCK			GPIOB
// #define NRF_PIN_SCK				4
// #define NRF_PORT_MOSI			GPIOB
// #define NRF_PIN_MOSI			3
// #define NRF_PORT_MISO			GPIOD
// #define NRF_PIN_MISO			2

// SPI pins
#define HW_SPI_DEV SPID1
#define HW_SPI_GPIO_AF GPIO_AF_SPI1
#define HW_SPI_PORT_NSS GPIOA
#define HW_SPI_PIN_NSS 4
#define HW_SPI_PORT_SCK GPIOA
#define HW_SPI_PIN_SCK 5
#define HW_SPI_PORT_MOSI GPIOA
#define HW_SPI_PIN_MOSI 7
#define HW_SPI_PORT_MISO GPIOA
#define HW_SPI_PIN_MISO 6


#ifdef HW_USE_LSM6DS3_IMU
    #define LSM6DS3_SDA_GPIO GPIOB
    #define LSM6DS3_SDA_PIN 2
    #define LSM6DS3_SCL_GPIO GPIOA
    #define LSM6DS3_SCL_PIN 15
    #define IMU_ROT_90// Need to rotate 270 degress 
#else // BMI160
    #define BMI160_SDA_GPIO GPIOB
    #define BMI160_SDA_PIN 2
    #define BMI160_SCL_GPIO GPIOA
    #define BMI160_SCL_PIN 15
    
#endif


// Permanent UART Peripheral (for NRF51)
#define HW_UART_P_BAUD 115200
#define HW_UART_P_DEV SD4
#define HW_UART_P_GPIO_AF GPIO_AF_UART4
#define HW_UART_P_TX_PORT GPIOC
#define HW_UART_P_TX_PIN 10
#define HW_UART_P_RX_PORT GPIOC
#define HW_UART_P_RX_PIN 11

// NRF SWD
#define NRF5x_SWDIO_GPIO GPIOB
#define NRF5x_SWDIO_PIN 4
#define NRF5x_SWCLK_GPIO GPIOB
#define NRF5x_SWCLK_PIN 3

// Measurement macros
#define ADC_V_L1 ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2 ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3 ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO (ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1() palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2() palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3() palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Default setting overrides
#ifndef MCCONF_DEFAULT_MOTOR_TYPE
#define MCCONF_DEFAULT_MOTOR_TYPE MOTOR_TYPE_FOC
#endif

#ifndef MCCONF_FOC_SAMPLE_V0_V7
#define MCCONF_FOC_SAMPLE_V0_V7 false // Run control loop in both v0 and v7 (requires phase shunts)
#endif

#ifndef MCCONF_L_IN_CURRENT_MAX
#define MCCONF_L_IN_CURRENT_MAX 60.0 // Input current limit in Amperes (Upper)
#endif
#ifndef MCCONF_L_IN_CURRENT_MIN
#define MCCONF_L_IN_CURRENT_MIN -40.0 // Input current limit in Amperes (Lower)
#endif
#ifndef MCCONF_L_MIN_VOLTAGE
#define MCCONF_L_MIN_VOLTAGE 24.0 // Minimum input voltage
#endif
#ifndef MCCONF_L_MAX_VOLTAGE
#define MCCONF_L_MAX_VOLTAGE 95 // Maximum input voltage
#endif
#ifndef MCCONF_FOC_F_ZV
#define MCCONF_FOC_F_ZV 20000.0
#endif
#ifndef MCCONF_FOC_DT_US
#define MCCONF_FOC_DT_US 0.23 // Microseconds for dead time compensation
#endif

#ifdef HIGH_POWER
// Setting limits

    #ifndef MCCONF_L_MAX_ABS_CURRENT
    #define MCCONF_L_MAX_ABS_CURRENT 260.0 // The maximum absolute current above which a fault is generated
    #endif
    #ifndef MCCONF_L_CURRENT_MAX
    #define MCCONF_L_CURRENT_MAX 210 // Current limit in Amperes (Upper)
    #endif
    #ifndef MCCONF_L_CURRENT_MIN
    #define MCCONF_L_CURRENT_MIN -210.0 // Current limit in Amperes (Lower)
    #endif
    #define HW_LIM_CURRENT -265.0, 260.0
    #define HW_LIM_CURRENT_ABS 0.0, 260.0
#else 
    #ifndef HW_MOS_PACKAGE_TO263
        #ifndef MCCONF_L_MAX_ABS_CURRENT
        #define MCCONF_L_MAX_ABS_CURRENT 160.0 // The maximum absolute current above which a fault is generated
        #endif
        #ifndef MCCONF_L_CURRENT_MAX
        #define MCCONF_L_CURRENT_MAX 110 // Current limit in Amperes (Upper)
        #endif
        #ifndef MCCONF_L_CURRENT_MIN
        #define MCCONF_L_CURRENT_MIN -110.0 // Current limit in Amperes (Lower)
        #endif
        #define HW_LIM_CURRENT -160.0, 160.0
        #define HW_LIM_CURRENT_ABS 0.0, 160.0
    #else
        #ifndef MCCONF_L_MAX_ABS_CURRENT
        #define MCCONF_L_MAX_ABS_CURRENT 120.0 // The maximum absolute current above which a fault is generated
        #endif
        #ifndef MCCONF_L_CURRENT_MAX
        #define MCCONF_L_CURRENT_MAX 90 // Current limit in Amperes (Upper)
        #endif
        #ifndef MCCONF_L_CURRENT_MIN
        #define MCCONF_L_CURRENT_MIN -90.0 // Current limit in Amperes (Lower)
        #endif
        #define HW_LIM_CURRENT -100.0, 100.0
        #define HW_LIM_CURRENT_ABS 0.0, 120.0
    #endif
#endif 

#define HW_LIM_CURRENT_IN -40.0, 60.0
#define HW_LIM_VIN 24.0, 100 // use 100V cap for V3 , set 4.2*20S
#define HW_LIM_ERPM -300e3, 300e3
#define HW_LIM_DUTY_MIN 0.0, 0.1
#define HW_LIM_DUTY_MAX 0.0, 0.93
#define HW_LIM_TEMP_FET -40.0, 120.0

#endif /* HW_SPESC*/
