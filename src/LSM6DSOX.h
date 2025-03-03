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

#ifndef LSM6DSOX_H
#define LSM6DSOX_H

#include "i2c_dma.h"

class LSM6DSOX {
  public:
    LSM6DSOX();
    LSM6DSOX(i2c_dma_t *i2c_dma);
    LSM6DSOX(i2c_dma_t *i2c_dma, uint8_t slaveAddress);
    ~LSM6DSOX();

    // int begin();
    // void end();

    // // Accelerometer
    // int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
    // float accelerationSampleRate(); // Sampling rate of the sensor.
    // int accelerationAvailable(); // Check for available data from accelerometer

    // // Gyroscope
    // int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    // float gyroscopeSampleRate(); // Sampling rate of the sensor.
    // int gyroscopeAvailable(); // Check for available data from gyroscope

    // // Temperature
    // int readTemperature(int& temperature_deg);
    // int readTemperatureFloat(float& temperature_deg);
    // int temperatureAvailable();

    int readRegisters(uint8_t regAddress, uint8_t* buf, size_t length);
    int writeRegister(uint8_t regAddress, uint8_t value);


  private:
    i2c_dma_t *_i2c_dma;
    uint8_t _slaveAddress;
};

#endif