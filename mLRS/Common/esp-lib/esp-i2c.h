//*******************************************************
// Copyright (c) OlliW, OlliW42, www.olliw.eu
// GPL3
// https://www.gnu.org/licenses/gpl-3.0.de.html
//*******************************************************
// ESP I2C
//*******************************************************
#ifndef ESPLIB_I2C_H
#define ESPLIB_I2C_H
#ifdef __cplusplus
extern "C" {
#endif


//-------------------------------------------------------
// Defines
//-------------------------------------------------------

#include "esp-peripherals.h"
#include <Wire.h>
// https://github.com/espressif/arduino-esp32/blob/master/libraries/Wire/src/Wire.h


//-------------------------------------------------------
//  I2C user routines
//-------------------------------------------------------

uint8_t i2c_dev_adr;


// call this before transaction
IRAM_ATTR void i2c_setdeviceadr(uint8_t dev_adr)
{
    i2c_dev_adr = dev_adr;
}


IRAM_ATTR uint8_t i2c_getdeviceadr(void)
{
    return i2c_dev_adr;
}


IRAM_ATTR HAL_StatusTypeDef i2c_put_blocked(uint8_t reg_adr, uint8_t* buf, uint16_t len)
{
    Wire.beginTransmission(i2c_dev_adr);
    Wire.write(reg_adr);
    Wire.write(buf, len);
    uint8_t error = Wire.endTransmission(true);
    return (error == 0) ? HAL_OK : HAL_ERROR;
}


IRAM_ATTR HAL_StatusTypeDef i2c_put(uint8_t reg_adr, uint8_t* buf, uint16_t len)
{
    Wire.beginTransmission(i2c_dev_adr);
    Wire.write(reg_adr);
    Wire.write(buf, len);
    uint8_t error = Wire.endTransmission(true);
    return (error == 0) ? HAL_OK : HAL_ERROR;
}


IRAM_ATTR HAL_StatusTypeDef i2c_device_ready(void)
{
    Wire.beginTransmission(i2c_dev_adr);
    uint8_t error = Wire.endTransmission(true);
    return (error == 0) ? HAL_OK : HAL_ERROR;
}


//-------------------------------------------------------
// INIT routines
//-------------------------------------------------------

void i2c_init(void)
{
    Wire.begin(I2C_SDA_IO, I2C_SCL_IO, I2C_CLOCKSPEED);
    Wire.setBufferSize(I2C_BUFFER_SIZE);
    Wire.setTimeout(2);
    i2c_dev_adr = 0;
}


#ifdef __cplusplus
}
#endif
#endif // ESPLIB_I2C_H