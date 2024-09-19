/*
 * wm8960.c
 *
 * Date: 2024-09-03
 * Author: @pyrho
 */

#include "wm8960.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// Copied from
// https://github.com/sparkfun/SparkFun_WM8960_Arduino_Library/blob/main/src/SparkFun_WM8960_Arduino_Library.h
// {{{
// The WM8960 does not support I2C reads This means we must keep a local
// copy of all the register values We will instantiate with default values As we
// write to the device, we will also make sure To update our local copy as well,
// stored here in this array. Each register is 9-bits, so we will store them as
// a uint16_t They are in order from R0-R55, and we even keep blank spots for
// the "reserved" registers. This way we can use the register address macro
// defines above to easily access each local copy of each register.
// Example: _registerLocalCopy[WM8960_REG_LEFT_INPUT_VOLUME]
const uint16_t REGISTER_DEFAULTS[56] = {
    0x0097, // R0 (0x00)
    0x0097, // R1 (0x01)
    0x0000, // R2 (0x02)
    0x0000, // R3 (0x03)
    0x0000, // R4 (0x04)
    0x0008, // F5 (0x05)
    0x0000, // R6 (0x06)
    0x000A, // R7 (0x07)
    0x01C0, // R8 (0x08)
    0x0000, // R9 (0x09)
    0x00FF, // R10 (0x0a)
    0x00FF, // R11 (0x0b)
    0x0000, // R12 (0x0C) RESERVED
    0x0000, // R13 (0x0D) RESERVED
    0x0000, // R14 (0x0E) RESERVED
    0x0000, // R15 (0x0F) RESERVED
    0x0000, // R16 (0x10)
    0x007B, // R17 (0x11)
    0x0100, // R18 (0x12)
    0x0032, // R19 (0x13)
    0x0000, // R20 (0x14)
    0x00C3, // R21 (0x15)
    0x00C3, // R22 (0x16)
    0x01C0, // R23 (0x17)
    0x0000, // R24 (0x18)
    0x0000, // R25 (0x19)
    0x0000, // R26 (0x1A)
    0x0000, // R27 (0x1B)
    0x0000, // R28 (0x1C)
    0x0000, // R29 (0x1D)
    0x0000, // R30 (0x1E) RESERVED
    0x0000, // R31 (0x1F) RESERVED
    0x0100, // R32 (0x20)
    0x0100, // R33 (0x21)
    0x0050, // R34 (0x22)
    0x0000, // R35 (0x23) RESERVED
    0x0000, // R36 (0x24) RESERVED
    0x0050, // R37 (0x25)
    0x0000, // R38 (0x26)
    0x0000, // R39 (0x27)
    0x0000, // R40 (0x28)
    0x0000, // R41 (0x29)
    0x0040, // R42 (0x2A)
    0x0000, // R43 (0x2B)
    0x0000, // R44 (0x2C)
    0x0050, // R45 (0x2D)
    0x0050, // R46 (0x2E)
    0x0000, // R47 (0x2F)
    0x0002, // R48 (0x30)
    0x0037, // R49 (0x31)
    0x0000, // R50 (0x32) RESERVED
    0x0080, // R51 (0x33)
    0x0008, // R52 (0x34)
    0x0031, // R53 (0x35)
    0x0026, // R54 (0x36)
    0x00e9, // R55 (0x37)
};
// }}}

#define ENABLE 1
#define DISABLE 0

#define BIT_VREF 6

/** This could be any bit */
#define BIT_RESET 1
#define BIT_VMIDSEL_1 8
#define BIT_VMIDSEL_0 7
#define BIT_RMIC 5
#define BIT_LMIC 4
#define BIT_AINL 5
#define BIT_AINR 4
#define BIT_LMN1 8
#define BIT_RMN1 8
#define BIT_LINMUTE 7
#define BIT_IPVU 8
#define BIT_RINMUTE 7
#define BIT_LMP2 6
#define BIT_RMP2 6
#define BIT_RMIC2B 3
#define BIT_LMIC2B 3
#define BIT_RMIC2B 3
#define BIT_L2MO 7
#define BIT_R2MO 7
#define BIT_OUT3 1
#define BIT_LOMIX 3
#define BIT_ROMIX 4
#define BIT_WL_1 3
#define BIT_WL_0 2
#define BIT_FORMAT_1 1
#define BIT_FORMAT_0 0
#define BIT_ADCL 3
#define BIT_ADCR 2
#define BIT_LOOPBACK 0
#define BIT_ROUT1 5
#define BIT_LOUT1 6
#define BIT_DACL 8
#define BIT_DACR 7
#define BIT_DACVU 8
#define BIT_DACMU 3
#define BIT_ALRCGPIO 6

#define WORD_LENGTH_16 0
#define WORD_LENGTH_20 1
#define WORD_LENGTH_24 2
#define WORD_LENGTH_32 3
#define FORMAT_I2S 2

// Private API {{{
//

// convertDBtoSetting
// This function will take in a dB value (as a float), and return the
// corresponding volume setting necessary.
// For example, Headphone volume control goes from 47-120.
// While PGA gain control is from 0-63.
// The offset values allow for proper conversion.
//
// dB - float value of dB
//
// offset - the differnce from lowest dB value to lowest setting value
//
// stepSize - the dB step for each setting (aka the "resolution" of the setting)
// This is 0.75dB for the PGAs, 0.5 for ADC/DAC, and 1dB for most other amps.
//
// minDB - float of minimum dB setting allowed, note this is not mute on the
// amp. "True mute" is always one stepSize lower.
//
// maxDB - float of maximum dB setting allowed. If you send anything higher, it
// will be limited to this max value.
uint8_t convertDBtoSetting(float dB, float offset, float stepSize, float minDB,
                           float maxDB) {
  // Limit incoming dB values to acceptable range. Note, the minimum limit we
  // want to limit this too is actually one step lower than the minDB, because
  // that is still an acceptable dB level (it is actually "true mute").
  // Note, the PGA amp does not have a "true mute" setting available, so we
  // must check for its unique minDB of -17.25.

  // Limit max. This is the same for all amps.
  if (dB > maxDB)
    dB = maxDB;

  // PGA amp doesn't have mute setting, so minDB should be limited to minDB
  // Let's check for the PGAs unique minDB (-17.25) to know we are currently
  // converting a PGA setting.
  if (minDB == WM8960_PGA_GAIN_MIN) {
    if (dB < minDB)
      dB = minDB;
  } else // Not PGA. All other amps have a mute setting below minDb
  {
    if (dB < (minDB - stepSize))
      dB = (minDB - stepSize);
  }

  // Adjust for offset
  // Offset is the number that gets us from the minimum dB option of an amp
  // up to the minimum setting value in the register.
  dB = dB + offset;

  // Find out how many steps we are above the minimum (at this point, our
  // minimum is "0". Note, because dB comes in as a float, the result of this
  // division (volume) can be a partial number. We will round that next.
  float volume = dB / stepSize;

  volume = round(volume); // round to the nearest setting value.

  // Serial debug (optional)
  // Serial.print("\t");
  // Serial.print((uint8_t)volume);

  return (uint8_t)volume; // cast from float to unsigned 8-bit integer.
}

HAL_StatusTypeDef _writeRegister(WM8960_t *dev, uint8_t address, uint16_t data,
                                 uint16_t dataSize) {

  // Shift the register address left 1 bit to leave room to the 9th
  // bit of the data
  uint16_t address_byte = address << 1;

  // Add the MSbit of the data to write to the register's address
  address_byte |= (data >> 8);

  uint8_t data_without_9th_bit = (0x00FF & data);

  return HAL_I2C_Mem_Write(dev->i2cHandle, WM8960_I2C_ADDR, address_byte,
                           I2C_MEMADD_SIZE_8BIT, &data_without_9th_bit,
                           dataSize, HAL_MAX_DELAY);
}

HAL_StatusTypeDef _writeBit(WM8960_t *dev, uint8_t registerAddress,
                            uint8_t bitNumber, uint8_t bitValue) {
  uint16_t currentRegisterValue = dev->registerState[registerAddress];

  if (bitValue == 1) {
    currentRegisterValue |= (1 << bitNumber); // Set only the bit we want
  } else {
    currentRegisterValue &= ~(1 << bitNumber); // Clear only the bit we want
  }

  HAL_StatusTypeDef ret =
      _writeRegister(dev, registerAddress, currentRegisterValue, 1);
  if (ret == HAL_OK) {
    dev->registerState[registerAddress] = currentRegisterValue;
  }
  return ret;
}

HAL_StatusTypeDef _writeMultiBits(WM8960_t *dev, uint8_t registerAddress,
                                  uint8_t settingMsbNum, uint8_t settingLsbNum,
                                  uint8_t setting) {
  uint8_t numOfBits = (settingMsbNum - settingLsbNum) + 1;

  // Get the local copy of the register
  uint16_t regvalue = dev->registerState[registerAddress];

  for (int i = 0; i < numOfBits; i++) {
    regvalue &= ~(1 << (settingLsbNum + i)); // Clear bits we care about
  }

  // Shift and set the bits from in incoming desired setting value
  regvalue |= (setting << settingLsbNum);

  // Write modified value to device
  // If successful, update local copy
  HAL_StatusTypeDef ret = _writeRegister(dev, registerAddress, regvalue, 1);
  if (ret == HAL_OK) {
    dev->registerState[registerAddress] = regvalue;
  }
  return ret;
}

HAL_StatusTypeDef pgaLeftIPVUSet(WM8960_t *dev) {
  return _writeBit(dev, WM8960_R00_LEFT_INPUT_VOLUME, 8, 1);
}

// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
HAL_StatusTypeDef _setLINVOL(WM8960_t *dev, uint8_t volume) {
  if (volume > 63)
    volume = 63; // Limit incoming values max
  HAL_StatusTypeDef result1 =
      _writeMultiBits(dev, WM8960_R00_LEFT_INPUT_VOLUME, 5, 0, volume);
  HAL_StatusTypeDef result2 = pgaLeftIPVUSet(dev);
  return (result1 + result2);
}

HAL_StatusTypeDef _setLINVOLDB(WM8960_t *dev, float dB) {
  // Create an unsigned integer volume setting variable we can send to
  // setLINVOL()
  uint8_t volume =
      convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE,
                         WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX);

  return _setLINVOL(dev, volume);
}

// Causes left and right input PGA volumes to be updated (LINVOL and RINVOL)
HAL_StatusTypeDef pgaRightIPVUSet(WM8960_t *dev) {
  return _writeBit(dev, WM8960_R01_RIGHT_INPUT_VOLUME, 8, 1);
}

// 0-63, (0 = -17.25dB) <<-- 0.75dB steps -->> (63 = +30dB)
HAL_StatusTypeDef _setRINVOL(WM8960_t *dev, uint8_t volume) {
  if (volume > 63)
    volume = 63; // Limit incoming values max
  HAL_StatusTypeDef result1 =
      _writeMultiBits(dev, WM8960_R01_RIGHT_INPUT_VOLUME, 5, 0, volume);
  HAL_StatusTypeDef result2 = pgaRightIPVUSet(dev);
  return result1 + result2;
}

// setRINVOLDB
// Sets the volume of the PGA input buffer amp to a specified dB value
// passed in as a float argument.
// Valid dB settings are -17.25 up to +30.00
// -17.25 = -17.25dB (MIN)
// ... 0.75dB steps ...
// 30.00 = +30.00dB  (MAX)
bool _setRINVOLDB(WM8960_t *dev, float dB) {
  // Create an unsigned integer volume setting variable we can send to
  // setRINVOL()
  uint8_t volume =
      convertDBtoSetting(dB, WM8960_PGA_GAIN_OFFSET, WM8960_PGA_GAIN_STEPSIZE,
                         WM8960_PGA_GAIN_MIN, WM8960_PGA_GAIN_MAX);

  return _setRINVOL(dev, volume);
}

HAL_StatusTypeDef _SYSCLKDIV(WM8960_t *dev) {
  // P.57 - MCLK is 24mhz, dividing by 2 gives 12mhz ~
  // BCLK output from STM32 is 1.4mhz
  // Like in that table P.57
  return _writeMultiBits(dev, WM8960_R04_CLOCKING_1, 2, 1,
                         WM8960_SYSCLK_DIV_BY_2);
}

HAL_StatusTypeDef _enablePLL(WM8960_t *dev) {
  return _writeBit(dev, WM8960_R26_PWR_MGMT_2, 0, 1);
}

HAL_StatusTypeDef _setHeadphoneVolume(WM8960_t *dev, uint8_t volume) {
  // Updates both left and right channels
  // Handles the OUT1VU (volume update) bit control, so that it happens at the
  // same time on both channels. Note, we must also make sure that the outputs
  // are enabled in the WM8960_REG_PWR_MGMT_2 [6:5]
  // Grab local copy of register
  // Modify the bits we need to
  // Write register in device, including the volume update bit write
  // If successful, save locally.

  // Limit inputs
  if (volume > 127)
    volume = 127;

  // LEFT
  HAL_StatusTypeDef result1 =
      _writeMultiBits(dev, WM8960_R02_LOUT1_VOLUME, 6, 0, volume);
  // RIGHT
  HAL_StatusTypeDef result2 =
      _writeMultiBits(dev, WM8960_R03_ROUT1_VOLUME, 6, 0, volume);
  // UPDATES

  // Updated left channel
  HAL_StatusTypeDef result3 = _writeBit(dev, WM8960_R02_LOUT1_VOLUME, 8, 1);

  // Updated right channel
  HAL_StatusTypeDef result4 = _writeBit(dev, WM8960_R03_ROUT1_VOLUME, 8, 1);

  return result1 + result2 + result3 + result4;
}

HAL_StatusTypeDef _setHeadphoneVolumeDB(WM8960_t *dev, float dB) {
  // Create an unsigned integer volume setting variable we can send to
  // setHeadphoneVolume()
  uint8_t volume =
      convertDBtoSetting(dB, WM8960_HP_GAIN_OFFSET, WM8960_HP_GAIN_STEPSIZE,
                         WM8960_HP_GAIN_MIN, WM8960_HP_GAIN_MAX);

  return _setHeadphoneVolume(dev, volume);
}

/*
 *   codec.enablePLL(); // Needed for class-d amp clock
  codec.setPLLPRESCALE(WM8960_PLLPRESCALE_DIV_2);
  codec.setSMD(WM8960_PLL_MODE_FRACTIONAL);
  codec.setCLKSEL(WM8960_CLKSEL_PLL);
  codec.setSYSCLKDIV(WM8960_SYSCLK_DIV_BY_2);
  codec.setBCLKDIV(4);
  codec.setDCLKDIV(WM8960_DCLKDIV_16);
  codec.setPLLN(7);
  codec.setPLLK(0x86, 0xC2, 0x26); // PLLK=86C226h
  //codec.setADCDIV(0); // Default is 000 (what we need for 44.1KHz)
  //codec.setDACDIV(0); // Default is 000 (what we need for 44.1KHz)
  codec.setWL(WM8960_WL_16BIT);

 */

/** Set the codec in Master mode, slave by default */
HAL_StatusTypeDef _MS_setMaster(WM8960_t *dev) {
  return _writeBit(dev, WM8960_R07_AUDIO_INTERFACE_1, 6, 1);
}

HAL_StatusTypeDef _MS_setSlave(WM8960_t *dev) {
  return _writeBit(dev, WM8960_R07_AUDIO_INTERFACE_1, 6, 0);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
HAL_StatusTypeDef _setLB2LOVOL(WM8960_t *dev, uint8_t volume) {
  if (volume > 7)
    volume = 7; // Limit incoming values max
  return _writeMultiBits(dev, WM8960_R45_BYPASS_1, 6, 4, volume);
}

// Valid inputs are 0-7. 0 = 0dB ...3dB steps... 7 = -21dB
HAL_StatusTypeDef _setRB2ROVOL(WM8960_t *dev, uint8_t volume) {
  if (volume > 7)
    volume = 7; // Limit incoming values max
  return _writeMultiBits(dev, WM8960_R46_BYPASS_2, 6, 4, volume);
}

// }}}

// Public API {{{

bool WM8960_isReady(I2C_HandleTypeDef *hi2c) {
  return HAL_I2C_IsDeviceReady(hi2c, WM8960_I2C_ADDR, 5, 100) == HAL_OK;
}

HAL_StatusTypeDef WM8960_init(I2C_HandleTypeDef *hi2c, WM8960_t **o_dev) {
  *o_dev = malloc(sizeof(WM8960_t));

  (*o_dev)->i2cHandle = hi2c;

  memcpy((*o_dev)->registerState, REGISTER_DEFAULTS, sizeof(REGISTER_DEFAULTS));

  HAL_StatusTypeDef halReturnStatus = HAL_OK;

  // Reset the codec settings
  halReturnStatus += _writeBit(*o_dev, WM8960_R15_RESET, BIT_RESET, 1);

  /**
   * This enables the whole chip.
   * It defaults to 0 when the chip powers up, and can be set to 0 to save
   * power. See P. 64.
   */
  halReturnStatus += _writeBit(*o_dev, WM8960_R25_PWR_MGMT_1, BIT_VREF, ENABLE);

  // P.64
  halReturnStatus +=
      _writeMultiBits(*o_dev, WM8960_R25_PWR_MGMT_1, BIT_VMIDSEL_1,
                      BIT_VMIDSEL_0, WM8960_SETTING_VMIDSEL_2X50KOHM);

  // Input signal config {{{
  // Left/Right channel input PGA enable
  // P.20
  halReturnStatus += _writeBit(*o_dev, WM8960_R47_PWR_MGMT_3, BIT_RMIC, ENABLE);
  halReturnStatus += _writeBit(*o_dev, WM8960_R47_PWR_MGMT_3, BIT_LMIC, ENABLE);

  // Left/Right channel input PGA and boost stage enable
  // P.20
  halReturnStatus += _writeBit(*o_dev, WM8960_R25_PWR_MGMT_1, BIT_AINL, ENABLE);
  halReturnStatus += _writeBit(*o_dev, WM8960_R25_PWR_MGMT_1, BIT_AINR, ENABLE);

  // Disconnect LINPUT1/RINPUT1
  // P.21
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R32_ADCL_SIGNAL_PATH, BIT_LMN1, DISABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R33_ADCR_SIGNAL_PATH, BIT_RMN1, DISABLE);

  // Plug INPUT2 to PGA  (P.21)
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R32_ADCL_SIGNAL_PATH, BIT_LMP2, ENABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R33_ADCR_SIGNAL_PATH, BIT_RMP2, ENABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R33_ADCR_SIGNAL_PATH, BIT_RMP2, ENABLE);

  // Plug PGA to Boost mixer
  // P.21
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R32_ADCL_SIGNAL_PATH, BIT_LMIC2B, ENABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R33_ADCR_SIGNAL_PATH, BIT_RMIC2B, ENABLE);

  // P.22 {{{
  // Left/Right Input PGA Analogue Mute
  // `1` must be written to the IPVU bit to update the unmuted state
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R00_LEFT_INPUT_VOLUME, BIT_LINMUTE, DISABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R00_LEFT_INPUT_VOLUME, BIT_IPVU, ENABLE);

  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R01_RIGHT_INPUT_VOLUME, BIT_RINMUTE, DISABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R01_RIGHT_INPUT_VOLUME, BIT_IPVU, ENABLE);
  // }}}

  // P.22
  halReturnStatus += _setLINVOLDB((*o_dev), 0.00);
  halReturnStatus += _setRINVOLDB((*o_dev), 0.00);

  // P.23
  halReturnStatus += _writeMultiBits(*o_dev, WM8960_R32_ADCL_SIGNAL_PATH, 5, 4,
                                     WM8960_SETTING_MIC_BOOST_GAIN_0DB);
  halReturnStatus += _writeMultiBits(*o_dev, WM8960_R33_ADCR_SIGNAL_PATH, 5, 4,
                                     WM8960_SETTING_MIC_BOOST_GAIN_0DB);

  // }}}

  // LB2LO DIS (See P.35)
  halReturnStatus += _writeBit(*o_dev, WM8960_R45_BYPASS_1, 7, DISABLE);
  // RB2LO DIS (See P.35)
  halReturnStatus += _writeBit(*o_dev, WM8960_R46_BYPASS_2, 7, DISABLE);

  // LD2LO EN (p. 35)
  halReturnStatus += _writeBit(*o_dev, WM8960_R34_LEFT_OUT_MIX_1, 8, ENABLE);
  // RD2LO EN (p. 35)
  halReturnStatus += _writeBit(*o_dev, WM8960_R37_RIGHT_OUT_MIX_2, 8, ENABLE);

  halReturnStatus += _setLB2LOVOL((*o_dev), WM8960_OUTPUT_MIXER_GAIN_0DB);
  halReturnStatus += _setRB2ROVOL((*o_dev), WM8960_OUTPUT_MIXER_GAIN_0DB);

  // LOMIX/ROMIX
  // P.35
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R47_PWR_MGMT_3, BIT_LOMIX, ENABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R47_PWR_MGMT_3, BIT_ROMIX, ENABLE);

  halReturnStatus += _writeMultiBits(*o_dev, WM8960_R07_AUDIO_INTERFACE_1,
                                     BIT_WL_1, BIT_WL_0, WORD_LENGTH_16);

  // Format i2S
  halReturnStatus += _writeMultiBits(*o_dev, WM8960_R07_AUDIO_INTERFACE_1,
                                     BIT_FORMAT_1, BIT_FORMAT_0, FORMAT_I2S);

  halReturnStatus += _MS_setSlave((*o_dev));
  // halReturnStatus += _MS_setMaster(*o_dev);

  // ADCL/ADCR
  halReturnStatus += _writeBit(*o_dev, WM8960_R25_PWR_MGMT_1, BIT_ADCL, ENABLE);
  halReturnStatus += _writeBit(*o_dev, WM8960_R25_PWR_MGMT_1, BIT_ADCR, ENABLE);

  // No loopback
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R09_AUDIO_INTERFACE_2, BIT_LOOPBACK, DISABLE);

  // R/LOUT1 (P.41)
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R26_PWR_MGMT_2, BIT_ROUT1, ENABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R26_PWR_MGMT_2, BIT_LOUT1, ENABLE);

  // OUT3
  halReturnStatus += _writeBit(*o_dev, WM8960_R26_PWR_MGMT_2, BIT_OUT3, ENABLE);

  // Disable mono output mixer
  // P.37
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R38_MONO_OUT_MIX_1, BIT_L2MO, DISABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R39_MONO_OUT_MIX_2, BIT_R2MO, DISABLE);

  // DACL
  halReturnStatus += _writeBit(*o_dev, WM8960_R26_PWR_MGMT_2, BIT_DACL, ENABLE);

  // DACVU
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R10_LEFT_DAC_VOLUME, BIT_DACVU, ENABLE);
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R11_RIGHT_DAC_VOLUME, BIT_DACVU, ENABLE);

  // DACMU
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R05_ADC_DAC_CTRL_1, BIT_DACMU, DISABLE);

  // DACR
  halReturnStatus += _writeBit(*o_dev, WM8960_R26_PWR_MGMT_2, BIT_DACR, ENABLE);

  // DACL
  halReturnStatus += _writeBit(*o_dev, WM8960_R26_PWR_MGMT_2, BIT_DACL, ENABLE);

  // ALRCGPIO, set this to 1 so that we can share the WS for ADC and DAC
  // P.47
  halReturnStatus +=
      _writeBit(*o_dev, WM8960_R09_AUDIO_INTERFACE_2, BIT_ALRCGPIO, ENABLE);
  halReturnStatus +=
      _writeMultiBits(*o_dev, WM8960_R48_ADDITIONAL_CONTROL_4, 6, 4, 0b100);

  halReturnStatus += _setHeadphoneVolumeDB(*o_dev, 0.00);

  return halReturnStatus;
}

// }}}
