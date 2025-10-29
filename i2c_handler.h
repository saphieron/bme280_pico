//
// Created by saphieron on 4/1/21.
//

#ifndef BME_I2C_HANDLER_H
#define BME_I2C_HANDLER_H

#include <stdint.h>


// uint32_t i2c_handler_initialise(uint32_t baudrate);
// void i2c_handler_disable(void);
// int32_t i2c_handler_selectHwInstance(uint8_t device_num);
// uint32_t i2c_handler_set_baudrate(uint32_t baudrate);
int32_t i2c_handler_write(uint8_t addr, uint8_t* buffer, uint32_t amount);
int32_t i2c_handler_read(uint8_t addr, uint8_t* buffer, uint32_t amount);
// void i2c_handler_scanForDevices(void);

#endif //BME_I2C_HANDLER_H
