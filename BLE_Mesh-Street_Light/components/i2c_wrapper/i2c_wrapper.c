#include "i2c_wrapper.h"

#define I2C_TICKS_TO_WAIT 1000
#define TAG "I2C_WRAPPER"




esp_err_t i2c_device_add(i2c_master_bus_handle_t i2c_bus_handle, i2c_master_dev_handle_t *i2c_dev_handle, uint8_t device_address)
{
    if (!i2c_dev_handle) {
        return ESP_ERR_INVALID_ARG;
    }
    i2c_device_config_t dev_cfg = {0};                          // Ensure zero initialization
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = device_address;
    dev_cfg.scl_speed_hz = I2C_BUS_SPEED_100k;
    return i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, i2c_dev_handle);
}

esp_err_t i2c_write_byte(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t data)
{
    uint8_t out_buf[2] = {reg, data};
    return i2c_master_transmit(i2c_dev_handle, out_buf, sizeof(out_buf), I2C_TICKS_TO_WAIT);
}

esp_err_t i2c_write_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t out_buf[len + 1];
    out_buf[0] = reg;
    memcpy(&out_buf[1], data, len);
    return i2c_master_transmit(i2c_dev_handle, out_buf, len + 1, I2C_TICKS_TO_WAIT);
}

// esp_err_t i2c_read_byte(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t len)
// {
//     esp_err_t res = i2c_master_transmit(i2c_dev_handle, &reg, 1, I2C_TICKS_TO_WAIT);
//     if (res != ESP_OK)
//         return res;
//     return i2c_master_receive(i2c_dev_handle, data, 1, I2C_TICKS_TO_WAIT);
// }

esp_err_t i2c_read_byte(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(i2c_dev_handle, &reg, 1, data, len, I2C_TICKS_TO_WAIT);

    // esp_err_t res = i2c_master_transmit(i2c_dev_handle, &reg, 1, I2C_TICKS_TO_WAIT);
    // if (res != ESP_OK)
    //     return res;
    // return i2c_master_receive(i2c_dev_handle, data, 1, I2C_TICKS_TO_WAIT);
}

// esp_err_t i2c_read_bytes(i2c_master_dev_handle_t i2c_dev_handle, uint8_t reg, uint8_t *data, size_t len)
// {
//     esp_err_t res = i2c_master_transmit(i2c_dev_handle, &reg, 1, I2C_TICKS_TO_WAIT);
//     if (res != ESP_OK)
//         return res;
//     return i2c_master_receive(i2c_dev_handle, data, len, I2C_TICKS_TO_WAIT);
// }

esp_err_t begin(i2c_master_bus_handle_t bus, i2c_master_dev_handle_t *dev, uint8_t address) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    return i2c_device_add(bus, dev, address);
}