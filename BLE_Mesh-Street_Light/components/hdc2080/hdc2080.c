#include "hdc2080.h"
#include "i2c_wrapper.h"
#include "esp_err.h"
#include "driver/i2c_master.h" 
#include "freertos/FreeRTOS.h"

i2c_master_bus_handle_t mst_bus_handle;
extern i2c_master_dev_handle_t mst_dev_handle;
/*
  I2C Mater bus configuration
*/
// esp_err_t i2c_master_init(i2c_master_bus_handle_t *i2c_bus_handle, gpio_num_t sda, gpio_num_t scl)
// {
//     i2c_master_bus_config_t bus_cnfg = {
//         .i2c_port = I2C_PORT_NUM,
//         .sda_io_num = I2C_SDA_PIN,
//         .scl_io_num = I2C_SCL_PIN,
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true
//     };
//     if(i2c_new_master_bus(&bus_cnfg, &mst_bus_handle) != ESP_OK){
//         return ESP_FAIL;    
//     }
//     i2c_device_config_t dev_cnfg = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = HDC2080_ADDR,
//         .scl_speed_hz = 100000               // 100kHz
//     };
//     if(i2c_master_bus_add_device(mst_bus_handle, &dev_cnfg, &mst_dev_handle) != ESP_OK){
//         return ESP_FAIL;
//     }
//     return ESP_OK;
// }

// static esp_err_t hdc2080_write_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t data){
//    uint8_t buf[2] = {reg, data};
//    return i2c_master_transmit(dev_handle, buf, sizeof(buf),  pdMS_TO_TICKS(1000));
// }

// static esp_err_t hdc2080_read_register(i2c_master_dev_handle_t dev_handle, uint8_t reg, uint8_t* data, size_t size){
//     return i2c_master_transmit_receive(dev_handle, &reg, 1, data, size, pdMS_TO_TICKS(1000));
// }

// ------  Low Level I2C Functions ------- //

esp_err_t hdc2080_begin(i2c_master_bus_handle_t i2c_bus_handle, i2c_master_dev_handle_t *i2c_dev_handle, uint8_t device_address){
    if(i2c_bus_handle == NULL){
        ESP_LOGE("HDC2080", "Master Not init");
        return ESP_FAIL;
    }
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_address,
        .scl_speed_hz = 100000
    };

    if(i2c_master_bus_add_device(i2c_bus_handle, &dev_cfg, i2c_dev_handle) != ESP_OK){
         ESP_LOGE("HDC2080", "Add Device Fail");
        return ESP_FAIL;
    }
    uint16_t id;
    esp_err_t err = hdc2080_get_id((i2c_master_dev_handle_t)*i2c_dev_handle , &id);
    if (err != ESP_OK) {
        ESP_LOGE("HDC2080", "Read Device ID Fail");
        return err; // Return error if ID read fails
    }
    if (id != HDC2080_DEV_ID) {
        return ESP_ERR_INVALID_RESPONSE; // Optional: if device ID doesn't match
    }
    return ESP_OK;
}

esp_err_t hdc2080_get_id(i2c_master_dev_handle_t mst_dev_handle, uint16_t* id){
    if (!id) return ESP_ERR_INVALID_ARG;
    uint8_t data[2];
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, DEVICE_ID_L, data, 2);
    if (err != ESP_OK){
      *id = 0;
      return err;
    }

    *id = data[0] | (data[1] <<8);
    if (*id != HDC2080_DEV_ID){
        *id = 0;
        return ESP_ERR_INVALID_RESPONSE;
    }
    return ESP_OK;
}

esp_err_t hdc2080_read_temp(i2c_master_dev_handle_t mst_dev_handle, float *temperature){
    if (!temperature) return ESP_ERR_INVALID_ARG;

    uint8_t data[2];
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, TEMP_LOW, data, 2) ;
    if (err != ESP_OK){
        *temperature = 0.0f;
        return err;
    }
    uint16_t raw_temp = data[0] | (data[1] << 8);
    *temperature = (float)(((raw_temp * 165.0f) / 65536.0f)  - 40.5f);
    return ESP_OK;
}

esp_err_t hdc2080_read_temp_offset_adjust(i2c_master_dev_handle_t mst_dev_handle, uint8_t *offset){
 if (offset == NULL) {
        return ESP_ERR_INVALID_ARG; // Check for null pointer
    }
    return hdc2080_read_byte(mst_dev_handle, TEMP_OFFSET_ADJUST, offset, 1) ; 
}
  
esp_err_t hdc2080_set_temp_offset_adjust(i2c_master_dev_handle_t mst_dev_handle, uint8_t *offset){
    if (offset == NULL) {
        return ESP_ERR_INVALID_ARG; // Check for null pointer
    }
     esp_err_t err = hdc2080_write_byte(mst_dev_handle, TEMP_OFFSET_ADJUST, *offset);
     if (err!= ESP_OK){
        return err;
    }
    return hdc2080_read_temp_offset_adjust(mst_dev_handle, offset);
}

esp_err_t hdc2080_read_humidity(i2c_master_dev_handle_t mst_dev_handle, float *humidity){
    if (!humidity) return ESP_ERR_INVALID_ARG;
    uint8_t data[2];
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, HUMID_LOW, data, 2) ;
    if(err != ESP_OK){
        *humidity = 0.0f;
        return err;
    }
    uint16_t raw_humid = data[0] | (data[1] << 8);
    *humidity = (float)(((raw_humid * 100.0f) / 65536.0f));
    return ESP_OK;
}

esp_err_t hdc2080_read_humidity_offset_adjust(i2c_master_dev_handle_t mst_dev_handle, uint8_t *offset){
   if (offset == NULL) {
        return ESP_ERR_INVALID_ARG; // Check for null pointer
    }
    return hdc2080_read_byte(mst_dev_handle, HUM_OFFSET_ADJUST, offset, 1); 
  
}

esp_err_t hdc2080_set_humidity_offset_adjust(i2c_master_dev_handle_t mst_dev_handle, uint8_t *offset){
       if (offset == NULL) {
        return ESP_ERR_INVALID_ARG; // Check for null pointer
    }
     esp_err_t err = hdc2080_write_byte(mst_dev_handle, HUM_OFFSET_ADJUST, *offset); 
     if (err != ESP_OK){
        return err;
    }
    return hdc2080_read_humidity_offset_adjust(mst_dev_handle, offset);
}

esp_err_t hdc2080_read_measurements(i2c_master_dev_handle_t mst_dev_handle,float *temperature, float *humidity) {
    
    esp_err_t err_tem = hdc2080_read_temp(mst_dev_handle, temperature);
    if (err_tem != ESP_OK) {
        return err_tem; // Return error if temperature read fails
    }
    esp_err_t err_hum = hdc2080_read_humidity(mst_dev_handle, humidity);
    if (err_hum != ESP_OK) {
        return err_hum; // Return error if humidity read fails
    }
   
    return ESP_OK;
}

esp_err_t hdc2080_enable_heater(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data;

    esp_err_t err = hdc2080_read_byte(mst_dev_handle, CONFIG, &data, 1);
    if (err != ESP_OK){
        return err;
    }
    data |= (1 << 3);
    err = hdc2080_write_byte(mst_dev_handle, CONFIG, data);
    return err;
}

esp_err_t hdc2080_disable_heater(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data;

    esp_err_t err =hdc2080_read_byte(mst_dev_handle, CONFIG, &data, 1) ;

    if(err != ESP_OK){
        return err;
    }
    data = (data & ~ 0xF7);
    err = hdc2080_write_byte(mst_dev_handle, CONFIG, data);
    return err;
}

esp_err_t hdc2080_set_low_temp(i2c_master_dev_handle_t mst_dev_handle, float temp){
    if (temp < -40.5f || temp > 124.5f) {
        return ESP_ERR_INVALID_ARG; // Temperature out of range
    }
    uint8_t temp_low_thres = (uint8_t)( 256.0f *(temp + 40.5f) / 165.0f);
    return hdc2080_write_byte(mst_dev_handle, TEMP_THR_L, temp_low_thres);
}

esp_err_t hdc2080_set_high_temp(i2c_master_dev_handle_t mst_dev_handle, float temp){
       if (temp < -40.5f || temp > 124.5f) {
        return ESP_ERR_INVALID_ARG; // Temperature out of range
    }
    uint8_t temp_high_thres = (uint8_t)( 256.0f *(temp + 40.5f) / 165.0f);
    return hdc2080_write_byte(mst_dev_handle, TEMP_THR_H, temp_high_thres);
}

esp_err_t hdc2080_set_high_humidity(i2c_master_dev_handle_t mst_dev_handle, float humid){
    if (humid < 0.0f || humid > 100.0f) {
        return ESP_ERR_INVALID_ARG; // Humidity out of range
    }
    uint8_t humid_high_thres = (uint8_t)( 256.0f * humid / 100.0f);
    return hdc2080_write_byte(mst_dev_handle, HUMID_THR_H, humid_high_thres);
}

esp_err_t hdc2080_set_low_humidity(i2c_master_dev_handle_t mst_dev_handle, float humid){
      if (humid < 0.0f || humid > 100.0f) {
        return ESP_ERR_INVALID_ARG; // Humidity out of range
    }
    
    uint8_t humid_low_thres = (uint8_t)( 256.0f * humid / 100.0f);
    return hdc2080_write_byte(mst_dev_handle, HUMID_THR_L, humid_low_thres);
}

esp_err_t hdc2080_read_low_humidity_threshold(i2c_master_dev_handle_t mst_dev_handle, float *humid){
    if (!humid) return ESP_ERR_INVALID_ARG;
    uint8_t data;
    esp_err_t err= hdc2080_read_byte(mst_dev_handle, HUMID_THR_L, &data, 1);
    if(err != ESP_OK){
        *humid =0.0;
        return err;
    }
    *humid = (float)(data * 100.0f / 256.0f);
    return ESP_OK;
}

esp_err_t hdc2080_read_high_humidity_threshold(i2c_master_dev_handle_t mst_dev_handle, float *humid){
    if (!humid) return ESP_ERR_INVALID_ARG;
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, HUMID_THR_H, &data, 1);
    if(err != ESP_OK){
        *humid = 0.0;
        return err;
    }
    *humid = (float)(data * 100.0f / 256.0f);
    return ESP_OK;
}

esp_err_t hdc2080_read_low_temp_threshold(i2c_master_dev_handle_t mst_dev_handle, float *temp){
    uint8_t data;
     esp_err_t err = hdc2080_read_byte(mst_dev_handle, TEMP_THR_L, &data, 1);
     if(err != ESP_OK){
        *temp = 0.0;
        return err;
    }
    
    *temp =(float)((data * 165.0f / 256.0f) - 40.5f);
    return ESP_OK;
}

esp_err_t hdc2080_read_high_temp_threshold(i2c_master_dev_handle_t mst_dev_handle, float *temp){
    if (!temp) return ESP_ERR_INVALID_ARG;
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, TEMP_THR_H, &data, 1);
    if(err  != ESP_OK){
        *temp = 0.0;
        return err;
    }
    *temp = (float)((data * 165.0f / 256.0f) - 40.5f);
    return ESP_OK;
}

esp_err_t hdc2080_trigger_measurement(i2c_master_dev_handle_t mst_dev_handle){

    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, MEASUREMENT_CONFIG, &data, 1);
     if(err != ESP_OK){
        return err;
    }
    data |= 0x01;
    return hdc2080_write_byte(mst_dev_handle, MEASUREMENT_CONFIG, data);
}

esp_err_t hdc2080_reset(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, CONFIG, &data, 1);
    if(err != ESP_OK){
        return err;
    }
    data |= 0x80;
    return hdc2080_write_byte(mst_dev_handle, CONFIG, data);
}

esp_err_t hdc2080_enable_interrupt(i2c_master_dev_handle_t mst_dev_handle){

    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, INTERRUPT_CONFIG, &data, 1);
    if (err != ESP_OK){
        return err;
    }
    data |= 0x04;
    err = hdc2080_write_byte(mst_dev_handle, INTERRUPT_CONFIG, data);
    return err;
}

esp_err_t hdc2080_disable_interrupt(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, INTERRUPT_CONFIG, &data, 1);
    if (err != ESP_OK){
        return err;
    }
    data &= 0xFB;
    err = hdc2080_write_byte(mst_dev_handle, INTERRUPT_CONFIG, data);
    return err;
}

esp_err_t hdc2080_read_interrupt_status(i2c_master_dev_handle_t mst_dev_handle, uint8_t *data){
    if (!data){ return ESP_ERR_INVALID_ARG;
    }
  return hdc2080_read_byte(mst_dev_handle, INTERRUPT_DRDY, data, 1);
 }

esp_err_t hdc2080_clear_max_temp(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data = 0x00;
    return hdc2080_write_byte(mst_dev_handle, TEMP_MAX, data);
}

esp_err_t hdc2080_clear_max_humidity(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data = 0x00;
    return hdc2080_write_byte(mst_dev_handle, HUMID_MAX, data);
}

esp_err_t hdc2080_read_max_temp(i2c_master_dev_handle_t mst_dev_handle, float *temp){
    if (!temp) return ESP_ERR_INVALID_ARG;
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, TEMP_MAX, &data, 1);
    if (err != ESP_OK){
        *temp = 0.0f;
        return err;
    }
    *temp = (float)(((data * 165.0f) / 256.0f)  - 40.5f);
    return ESP_OK;
}

esp_err_t hdc2080_read_max_humidity(i2c_master_dev_handle_t mst_dev_handle, float *humid){
    if (!humid) return ESP_ERR_INVALID_ARG;
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, HUMID_MAX, &data, 1);
    if (err != ESP_OK){
        *humid = 0.0f;
        return err;
    }
    *humid =(float)(((data * 100.0f) / 256.0f));
    return ESP_OK;
}

esp_err_t hdc2080_enable_threshold_interrupt(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, INTERRUPT_CONFIG, &data, 1);
    if (err != ESP_OK){      
        return err;
    }
    data |= 0x78;
    return hdc2080_write_byte(mst_dev_handle, INTERRUPT_CONFIG, data);
}

esp_err_t hdc2080_disable_threshold_interrupt( i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, INTERRUPT_CONFIG, &data, 1); 
    if (err != ESP_OK){
        return err;
    }
    data &= 0x87;
    return hdc2080_write_byte(mst_dev_handle, INTERRUPT_CONFIG, data);
}

esp_err_t hdc2080_enable_DRDY_interrupt(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data;
    esp_err_t err =hdc2080_read_byte(mst_dev_handle, INTERRUPT_CONFIG, &data, 1);
    if (err != ESP_OK){
        return err;
    }
    data |= 0x80;
    return hdc2080_write_byte(mst_dev_handle, INTERRUPT_CONFIG, data);
}

esp_err_t hdc2080_disable_DRDY_interrupt(i2c_master_dev_handle_t mst_dev_handle){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, INTERRUPT_CONFIG, &data, 1); 
    if( err != ESP_OK){
        return err;
    }
    data &= 0x7F;
    return hdc2080_write_byte(mst_dev_handle, INTERRUPT_CONFIG, data);
}

esp_err_t  hdc2080_set_temp_resolution(i2c_master_dev_handle_t mst_dev_handle, int resolution){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, MEASUREMENT_CONFIG, &data, 1);
    if (err != ESP_OK){
        return err;
    }
    switch (resolution)
    {
    case FOURTEEN_BIT:
        data &= 0x3F;
        break;
    case ELEVEN_BIT:    
        data &= 0x7F;
        data |= 0x40;
        break;
    case NINE_BIT:
        data &= 0xBF;
        data |= 0x80;
        break;
    default:
        data &= 0x3F;
        break;
    }
    return hdc2080_write_byte(mst_dev_handle, MEASUREMENT_CONFIG, data);
}

esp_err_t hdc2080_set_humid_resolution(i2c_master_dev_handle_t mst_dev_handle, int resolution){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, MEASUREMENT_CONFIG, &data, 1);
    if (err  != ESP_OK){
        return err;
    }
    switch (resolution)
    {
    case FOURTEEN_BIT:
        data &= 0xCF;
        break;
    case ELEVEN_BIT:    
        data &= 0xDF;
        data |= 0x10;
        break;
    case NINE_BIT:
        data &= 0xEF;
        data |= 0x20;
        break;
    default:
        data &= 0xCF;
        break;
    }
    return hdc2080_write_byte(mst_dev_handle, MEASUREMENT_CONFIG, data);
}

esp_err_t hdc2080_set_measurement_mode(i2c_master_dev_handle_t mst_dev_handle, int mode){
    uint8_t data;
    esp_err_t err =hdc2080_read_byte(mst_dev_handle, MEASUREMENT_CONFIG, &data, 1);
    if (err != ESP_OK){
        return err;
    }
    switch (mode)
    {
    case TEMP_AND_HUMID:
        data &= 0xF9;
        break;
    case TEMP_ONLY:    
        data &= 0xFC;
        data |= 0x02;
        break;
    case HUMID_ONLY:
        data &= 0xFD;
        data |= 0x04;
        break;
    default:
        data &= 0xF9;
        break;
    }
    return hdc2080_write_byte(mst_dev_handle, MEASUREMENT_CONFIG, data);
}

esp_err_t  hdc2080_set_interrupt_polarity(i2c_master_dev_handle_t mst_dev_handle, int polarity){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, INTERRUPT_CONFIG, &data, 1);
    if (err != ESP_OK){
        return err;
    }
    switch (polarity)
    {
    case ACTIVE_LOW:
        data &= 0xFD;
        break;
    case ACTIVE_HIGH:    
        data |= 0x02;
        break;
    default:
        data &= 0xFD;
        break;
    }
    return hdc2080_write_byte(mst_dev_handle, INTERRUPT_CONFIG, data);
}

esp_err_t  hdc2080_set_interrupt_mode(i2c_master_dev_handle_t mst_dev_handle, int mode){
    uint8_t data;
    esp_err_t err = hdc2080_read_byte(mst_dev_handle, INTERRUPT_CONFIG, &data, 1); 
    if (err!= ESP_OK){
        return err;
    }
    switch (mode)
    {
    case LEVEL_MODE:
        data &= 0xFE;
        break;
    case COMPARATOR_MODE:    
        data |= 0x01;
        break;
    default:
        data &= 0xFE;
        break;
    }
    return hdc2080_write_byte(mst_dev_handle, INTERRUPT_CONFIG, data);
}

esp_err_t  hdc2080_set_rate(i2c_master_dev_handle_t mst_dev_handle,int rate){
    uint8_t data;
      esp_err_t err = hdc2080_read_byte(mst_dev_handle, MEASUREMENT_CONFIG, &data, 1);
    if (err!= ESP_OK){
        return err;
    }
    switch (rate)
    {
    case MANUAL:
        data &= 0x8F;
        break;
    case TWO_MINS:    
        data &= 0x9F;
        data |= 0x10;
        break;
    case ONE_MINS:
        data &= 0xAF;
        data |= 0x20;
        break;
    case TEN_SECONDS:
        data &= 0xBF;
        data |= 0x30;
        break;
    case FIVE_SECONDS:
        data &= 0xCF;
        data |= 0x40;
        break;
    case ONE_HZ:
        data &= 0xDF;
        data |= 0x50;
        break;
    case TWO_HZ:
        data &= 0xEF;
        data |= 0x60;
        break;
    case FIVE_HZ:
        data |= 0x70;
        break;
    default:
        data &= 0x8F;
        break;
    }
    return hdc2080_write_byte(mst_dev_handle, MEASUREMENT_CONFIG, data);
}



