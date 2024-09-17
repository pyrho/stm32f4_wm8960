/*
 * WM8960 Audio Codec driver header file
 *
 * Date: 2024-09-03
 * Author: @pyrho
 */

#ifndef __WM8960_H
#define __WM8960_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// P.63 of datasheet
// 7bit addr + R/W bit, MSB first
// The 7bit address is `0b0011010` the 8th bit is the R/W bit which needs to
// be set to `0` to be able to write.
// So this translates to 0x34 as stated in the datasheet
// Sparkfun uses `0x1A`, because they're using Arduino's "Wire" which I guess
// does the left shift on its own.
#define WM8960_I2C_ADDR 0x34

// WM8960 register addresses (copied from
// https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library/blob/main/src/SparkFun_WM8960_Arduino_Library.h)
// Also available P.67 "Register map"
#define WM8960_R00_LEFT_INPUT_VOLUME 0x00
#define WM8960_R01_RIGHT_INPUT_VOLUME 0x01
#define WM8960_R02_LOUT1_VOLUME 0x02
#define WM8960_R03_ROUT1_VOLUME 0x03
#define WM8960_R04_CLOCKING_1 0x04
#define WM8960_R05_ADC_DAC_CTRL_1 0x05
#define WM8960_R06_ADC_DAC_CTRL_2 0x06
#define WM8960_R07_AUDIO_INTERFACE_1 0x07
#define WM8960_R08_CLOCKING_2 0x08
#define WM8960_R09_AUDIO_INTERFACE_2 0x09
#define WM8960_R10_LEFT_DAC_VOLUME 0x0A
#define WM8960_R11_RIGHT_DAC_VOLUME 0x0B
#define WM8960_R15_RESET 0x0F
#define WM8960_R16_3D_CONTROL 0x10
#define WM8960_R17_ALC1 0x11
#define WM8960_R18_ALC2 0x12
#define WM8960_R19_ALC3 0x13
#define WM8960_R20_NOISE_GATE 0x14
#define WM8960_R21_LEFT_ADC_VOLUME 0x15
#define WM8960_R22_RIGHT_ADC_VOLUME 0x16
#define WM8960_R23_ADDITIONAL_CONTROL_1 0x17
#define WM8960_R24_ADDITIONAL_CONTROL_2 0x18
#define WM8960_R25_PWR_MGMT_1 0x19
#define WM8960_R26_PWR_MGMT_2 0x1A
#define WM8960_R27_ADDITIONAL_CONTROL_3 0x1B
#define WM8960_R28_ANTI_POP_1 0x1C
#define WM8960_R29_ANTI_POP_2 0x1D
#define WM8960_R32_ADCL_SIGNAL_PATH 0x20
#define WM8960_R33_ADCR_SIGNAL_PATH 0x21
#define WM8960_R34_LEFT_OUT_MIX_1 0x22
#define WM8960_R37_RIGHT_OUT_MIX_2 0x25
#define WM8960_R38_MONO_OUT_MIX_1 0x26
#define WM8960_R39_MONO_OUT_MIX_2 0x27
#define WM8960_R40_LOUT2_VOLUME 0x28
#define WM8960_R41_ROUT2_VOLUME 0x29
#define WM8960_R42_MONO_OUT_VOLUME 0x2A
#define WM8960_R43_INPUT_BOOST_MIXER_1 0x2B
#define WM8960_R44_INPUT_BOOST_MIXER_2 0x2C
#define WM8960_R45_BYPASS_1 0x2D
#define WM8960_R46_BYPASS_2 0x2E
#define WM8960_R47_PWR_MGMT_3 0x2F
#define WM8960_R48_ADDITIONAL_CONTROL_4 0x30
#define WM8960_R49_CLASS_D_CONTROL_1 0x31
#define WM8960_R51_CLASS_D_CONTROL_3 0x33
#define WM8960_R52_PLL_N 0x34
#define WM8960_R53_PLL_K_1 0x35
#define WM8960_R54_PLL_K_2 0x36
#define WM8960_R55_PLL_K_3 0x37

// SYSCLK divide
#define WM8960_SYSCLK_DIV_BY_1 0
#define WM8960_SYSCLK_DIV_BY_2 2
#define WM8960_CLKSEL_MCLK 0
#define WM8960_CLKSEL_PLL 1
#define WM8960_PLL_MODE_INTEGER 0
#define WM8960_PLL_MODE_FRACTIONAL 1
#define WM8960_PLLPRESCALE_DIV_1 0
#define WM8960_PLLPRESCALE_DIV_2 1

// VMIDSEL settings
#define WM8960_SETTING_VMIDSEL_DISABLED 0b00

/** For "playback/record" (see P.64) */
#define WM8960_SETTING_VMIDSEL_2X50KOHM 0b01

/** For "Low power standby" (see P.64) */
#define WM8960_SETTING_VMIDSEL_2X250KOHM 0b10

/** For "Fast startup" */
#define WM8960_SETTING_VMIDSEL_2X5KOHM 0b11

// Mic (aka PGA) BOOST gain options
#define WM8960_SETTING_MIC_BOOST_GAIN_0DB 0b00
#define WM8960_SETTING_MIC_BOOST_GAIN_13DB 0b01
#define WM8960_SETTING_MIC_BOOST_GAIN_20DB 0b10
#define WM8960_SETTING_MIC_BOOST_GAIN_29DB 0b11

#define WM8960_SETTING_LINBOOST_GAIN_MUTE 0b000
#define WM8960_SETTING_LINBOOST_GAIN_0DB 0b101
#define WM8960_SETTING_LINBOOST_GAIN_6DB 0b111


// Gain mins, maxes, offsets and step-sizes for all the amps within the codec.
#define WM8960_PGA_GAIN_MIN -17.25
#define WM8960_PGA_GAIN_MAX 30.00
#define WM8960_PGA_GAIN_OFFSET 17.25
#define WM8960_PGA_GAIN_STEPSIZE 0.75
#define WM8960_HP_GAIN_MIN -73.00
#define WM8960_HP_GAIN_MAX 6.00
#define WM8960_HP_GAIN_OFFSET 121.00
#define WM8960_HP_GAIN_STEPSIZE 1.00
#define WM8960_SPEAKER_GAIN_MIN -73.00
#define WM8960_SPEAKER_GAIN_MAX 6.00
#define WM8960_SPEAKER_GAIN_OFFSET 121.00
#define WM8960_SPEAKER_GAIN_STEPSIZE 1.00
#define WM8960_ADC_GAIN_MIN -97.00
#define WM8960_ADC_GAIN_MAX 30.00
#define WM8960_ADC_GAIN_OFFSET 97.50
#define WM8960_ADC_GAIN_STEPSIZE 0.50
#define WM8960_DAC_GAIN_MIN -97.00
#define WM8960_DAC_GAIN_MAX 30.00
#define WM8960_DAC_GAIN_OFFSET 97.50
#define WM8960_DAC_GAIN_STEPSIZE 0.50

// Output Mixer gain options
// These are used to control the gain (aka volume) at the following settings:
// LI2LOVOL
// LB2LOVOL
// RI2LOVOL
// RB2LOVOL
// These are useful as analog bypass signal path options.
#define WM8960_OUTPUT_MIXER_GAIN_0DB 0
#define WM8960_OUTPUT_MIXER_GAIN_NEG_3DB 1
#define WM8960_OUTPUT_MIXER_GAIN_NEG_6DB 2
#define WM8960_OUTPUT_MIXER_GAIN_NEG_9DB 3
#define WM8960_OUTPUT_MIXER_GAIN_NEG_12DB 4
#define WM8960_OUTPUT_MIXER_GAIN_NEG_15DB 5
#define WM8960_OUTPUT_MIXER_GAIN_NEG_18DB 6
#define WM8960_OUTPUT_MIXER_GAIN_NEG_21DB 7


typedef struct WM8960_t {
  I2C_HandleTypeDef *i2cHandle;
  uint16_t registerState[56];
} WM8960_t;

HAL_StatusTypeDef WM8960_init(I2C_HandleTypeDef *i_hi2c, WM8960_t** o_dev);
bool WM8960_isReady(I2C_HandleTypeDef *hi2c);

#endif //!__WM8960_H
