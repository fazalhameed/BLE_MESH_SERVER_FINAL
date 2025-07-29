// i2c_wrapper.h
#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include "esp_err.h"
#include "driver/i2c_master.h"
#include <string.h>
#include "esp_log.h"

#define I2C_TICKS_TO_WAIT 1000

#if CONFIG_I2C_PORT_0
    #define I2C_NUM I2C_NUM_0
#elif CONFIG_I2C_PORT_1
    #define I2C_NUM I2C_NUM_1
#else
    #define I2C_NUM I2C_NUM_0
#endif

extern i2c_master_bus_handle_t i2c_bus_handle;  // Declare it here
typedef enum {
    I2C_BUS_SPEED_400k = 400000,
    I2C_BUS_SPEED_100k = 100000
} i2c_bus_speed_t;

esp_err_t i2c_master_init(i2c_master_bus_handle_t *i2c_bus_handle, gpio_num_t sda, gpio_num_t scl);
esp_err_t i2c_write_byte(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t data);
esp_err_t i2c_write_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data,  size_t len);
esp_err_t i2c_read_byte(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data,  size_t len);
esp_err_t i2c_read_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data,  size_t len);
esp_err_t begin(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev, uint8_t address);


#endif // I2C_WRAPPER_H
