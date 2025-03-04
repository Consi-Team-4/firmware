/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

// Modified by Gideon Rabson, spring 2025

#include "LSM6DSOX.h"
#include "LSM6DSOXdefines.h"
#include "math.h"

LSM6DSOX::LSM6DSOX() :
  _i2c_dma(nullptr),
  _slaveAddress(LSM6DSOX_ADDRESS)
{
}

LSM6DSOX::LSM6DSOX(i2c_dma_t *i2c_dma) :
  _i2c_dma(i2c_dma),
  _slaveAddress(LSM6DSOX_ADDRESS)
{
}

LSM6DSOX::LSM6DSOX(i2c_dma_t *i2c_dma, uint8_t slaveAddress) :
  _i2c_dma(i2c_dma),
  _slaveAddress(slaveAddress)
{
}

LSM6DSOX::~LSM6DSOX()
{
}

// int LSM6DSOX::begin()
// {
//   if (readRegister(REG_WHO_AM_I) != WHO_AM_I_VAL) {
//     return -1;
//   }

//   //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
//   writeRegister(LSM6DSOX_CTRL2_G, 0x4C);

//   // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
//   // low pass filter (check figure9 of LSM6DSOX's datasheet)
//   writeRegister(LSM6DSOX_CTRL1_XL, 0x4A);

//   // set gyroscope power mode to high performance and bandwidth to 16 MHz
//   writeRegister(LSM6DSOX_CTRL7_G, 0x00);

//   // Set the ODR config register to ODR/4
//   writeRegister(LSM6DSOX_CTRL8_XL, 0x09);

//   return 1;
// }

// void LSM6DSOX::end()
// {
//   writeRegister(LSM6DSOX_CTRL2_G, 0x00);
//   writeRegister(LSM6DSOX_CTRL1_XL, 0x00);
// }

// int LSM6DSOX::readAcceleration(float& x, float& y, float& z)
// {
//   int16_t data[3];

//   if (!readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
//     x = NAN;
//     y = NAN;
//     z = NAN;

//     return 0;
//   }

//   x = data[0] * 4.0 / 32768.0;
//   y = data[1] * 4.0 / 32768.0;
//   z = data[2] * 4.0 / 32768.0;

//   return 1;
// }

// int LSM6DSOX::accelerationAvailable()
// {
//   if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
//     return 1;
//   }

//   return 0;
// }

// float LSM6DSOX::accelerationSampleRate()
// {
//   return 104.0F;
// }

// int LSM6DSOX::readGyroscope(float& x, float& y, float& z)
// {
//   int16_t data[3];

//   if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
//     x = NAN;
//     y = NAN;
//     z = NAN;

//     return 0;
//   }

//   x = data[0] * 2000.0 / 32768.0;
//   y = data[1] * 2000.0 / 32768.0;
//   z = data[2] * 2000.0 / 32768.0;

//   return 1;
// }

// int LSM6DSOX::gyroscopeAvailable()
// {
//   if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
//     return 1;
//   }

//   return 0;
// }

// int LSM6DSOX::readTemperature(int& temperature_deg)
// {
//   float temperature_float = 0;
//   readTemperatureFloat(temperature_float);

//   temperature_deg = static_cast<int>(temperature_float);

//   return 1;
// }

// int LSM6DSOX::readTemperatureFloat(float& temperature_deg)
// {
//   /* Read the raw temperature from the sensor. */
//   int16_t temperature_raw = 0;

//   if (readRegisters(LSM6DSOX_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
//     return 0;
//   }

//   /* Convert to °C. */
//   static int const TEMPERATURE_LSB_per_DEG = 256;
//   static int const TEMPERATURE_OFFSET_DEG = 25;

//   temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

//   return 1;
// }

// int LSM6DSOX::temperatureAvailable()
// {
//   if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
//     return 1;
//   }

//   return 0;
// }

// float LSM6DSOX::gyroscopeSampleRate()
// {
//   return 104.0F;
// }

int LSM6DSOX::readRegisters(uint8_t regAddress, uint8_t* buf, size_t length)
{
  int result = i2c_dma_write_read(_i2c_dma, _slaveAddress, &regAddress, 1, buf, length);
  return result;
}

int LSM6DSOX::writeRegister(uint8_t regAddress, uint8_t value)
{
  uint8_t buff[2] = {regAddress, value};
  int result = i2c_dma_write(_i2c_dma, _slaveAddress, buff, 2);
  return result;
}
