#include <stdint.h>
#include <stdbool.h>
#ifndef __HDC2080_H__
#define __HDC2080_H__
#include "i2c_wrapper.h"

#ifdef __cplusplus
extern "C" {
#endif

//  Constants for setting measurement resolution
#define FOURTEEN_BIT 0
#define ELEVEN_BIT 1
#define NINE_BIT 2

//  Constants for setting sensor mode
#define TEMP_AND_HUMID 0
#define TEMP_ONLY 1
#define HUMID_ONLY 2
#define ACTIVE_LOW 0
#define ACTIVE_HIGH 1
#define LEVEL_MODE 0
#define COMPARATOR_MODE 1

//  Constants for setting sample rate
#define MANUAL 0
#define TWO_MINS 1
#define ONE_MINS 2
#define TEN_SECONDS 3
#define FIVE_SECONDS 4
#define ONE_HZ 5
#define TWO_HZ 6
#define FIVE_HZ 7

//Define Register Map
#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03
#define INTERRUPT_DRDY 0x04
#define TEMP_MAX 0x05
#define HUMID_MAX 0x06
#define INTERRUPT_CONFIG 0x07
#define TEMP_OFFSET_ADJUST 0x08
#define HUM_OFFSET_ADJUST 0x09
#define TEMP_THR_L 0x0A
#define TEMP_THR_H 0x0B
#define HUMID_THR_L 0x0C
#define HUMID_THR_H 0x0D
#define CONFIG 0x0E
#define MEASUREMENT_CONFIG 0x0F
#define MID_L 0xFC
#define MID_H 0xFD
#define DEVICE_ID_L 0xFE
#define DEVICE_ID_H 0xFF
#define HDC2080_DEV_ID 0x07D0U

// I2C Bus Configuration
#define I2C_TIMEOUT 1000
#define I2C_PORT_NUM I2C_NUM_0       // default I2C port number
#define I2C_SCL_PIN 9               // default I2C SCL pin
#define I2C_SDA_PIN 8             // default I2C SDA pin
#define HDC2080_ADDR 0x40            // default HDC2080 address
#define HDC2080_INT_PIN 21      // default HDC2080 interrupt pin

#define hdc2080_write_byte i2c_write_byte 
#define hdc2080_read_byte i2c_read_byte
extern i2c_master_bus_handle_t mst_bus_handle;
extern i2c_master_dev_handle_t mst_dev_handle;


// Function Naming Convention			          
esp_err_t hdc2080_begin(i2c_master_bus_handle_t i2c_bus_handle, i2c_master_dev_handle_t *i2c_dev_handle, uint8_t device_address);    // Join I2C bus
esp_err_t hdc2080_get_id(i2c_master_dev_handle_t mst_dev_handle, uint16_t* id);                                         // Gives the device ID 
esp_err_t hdc2080_read_temp(i2c_master_dev_handle_t mst_dev_handle, float *temperature);		                      // Returns the temperature in degrees C
esp_err_t hdc2080_read_temp_offset_adjust(i2c_master_dev_handle_t mst_dev_handle, uint8_t *offset);		        // Returns the offset adjust in binary
esp_err_t hdc2080_set_temp_offset_adjust(i2c_master_dev_handle_t mst_dev_handle, uint8_t *offset);  	               // Set and return the offset adjust in binary
esp_err_t hdc2080_read_humidity(i2c_master_dev_handle_t mst_dev_handle, float *humidity);		                      // Returns the relative humidity
esp_err_t hdc2080_read_measurements(i2c_master_dev_handle_t mst_dev_handle,float *temperature, float *humidity);        // Returns the temperature and humidity
esp_err_t hdc2080_read_humidity_offset_adjust(i2c_master_dev_handle_t mst_dev_handle, uint8_t *offset);                 // Returns the offset adjust in binary
esp_err_t hdc2080_set_humidity_offset_adjust(i2c_master_dev_handle_t mst_dev_handle, uint8_t *offset);                  // Set and return the offset adjust in binary
esp_err_t hdc2080_enable_heater(i2c_master_dev_handle_t mst_dev_handle);			                             // Enables the heating element
esp_err_t hdc2080_disable_heater(i2c_master_dev_handle_t mst_dev_handle);			                             // Disables the heating element
esp_err_t hdc2080_set_low_temp(i2c_master_dev_handle_t mst_dev_handle, float temp);			                      // Sets low threshold temperature (in c)
esp_err_t hdc2080_set_high_temp(i2c_master_dev_handle_t mst_dev_handle, float temp);			               // Sets high threshold temperature (in c)
esp_err_t hdc2080_set_high_humidity(i2c_master_dev_handle_t mst_dev_handle, float humid);		                      // Sets high Humiditiy threshold
esp_err_t hdc2080_set_low_humidity(i2c_master_dev_handle_t mst_dev_handle, float humid);		                      // Sets low Humidity threshold
esp_err_t hdc2080_read_low_humidity_threshold(i2c_master_dev_handle_t mst_dev_handle, float *humid);	               // Returns contents of low humidity threshold register
esp_err_t hdc2080_read_high_humidity_threshold(i2c_master_dev_handle_t mst_dev_handle, float *humid);  	               // Returns contents of high humidity threshold register
esp_err_t hdc2080_read_low_temp_threshold(i2c_master_dev_handle_t mst_dev_handle, float *temp);		               // Returns contents of low temperature threshold register (in C)
esp_err_t hdc2080_read_high_temp_threshold(i2c_master_dev_handle_t mst_dev_handle, float *temp);		               // Returns contents of high temperature threshold register (in C)
esp_err_t hdc2080_trigger_measurement(i2c_master_dev_handle_t mst_dev_handle);			                      // Triggers a manual temperature/humidity reading
esp_err_t hdc2080_reset(i2c_master_dev_handle_t mst_dev_handle);					                      // Triggers a software reset
esp_err_t hdc2080_enable_interrupt(i2c_master_dev_handle_t mst_dev_handle);			                             // Enables the interrupt/DRDY pin
esp_err_t hdc2080_disable_interrupt(i2c_master_dev_handle_t mst_dev_handle);			                             // Disables the interrupt/DRDY pin (High Z)
esp_err_t hdc2080_read_interrupt_status(i2c_master_dev_handle_t mst_dev_handle,uint8_t *data);		               // Reads the status of the interrupt register
esp_err_t hdc2080_clear_max_temp(i2c_master_dev_handle_t mst_dev_handle);				                      // Clears the Maximum temperature register
esp_err_t hdc2080_clear_max_humidity(i2c_master_dev_handle_t mst_dev_handle);			                      // Clears the Maximum humidity register
esp_err_t hdc2080_read_max_temp(i2c_master_dev_handle_t mst_dev_handle, float *temp);				        // Reads the maximum temperature register
esp_err_t hdc2080_read_max_humidity(i2c_master_dev_handle_t mst_dev_handle, float *humid);			               // Reads the maximum humidity register
esp_err_t hdc2080_enable_threshold_interrupt(i2c_master_dev_handle_t mst_dev_handle);	                             // Enables high and low temperature/humidity interrupts
esp_err_t hdc2080_disable_threshold_interrupt( i2c_master_dev_handle_t mst_dev_handle);	                             // Disables high and low temperature/humidity interrupts
esp_err_t hdc2080_enable_DRDY_interrupt(i2c_master_dev_handle_t mst_dev_handle);			                      // Enables data ready interrupt
esp_err_t hdc2080_disable_DRDY_interrupt(i2c_master_dev_handle_t mst_dev_handle);		                             // Disables data ready interrupt
       
/* Sets Temperature & Humidity Resolution, 3 options
       0 - 14 bit
       1 - 11 bit
       2 - 9 bit
       default - 14 bit							*/
esp_err_t  hdc2080_set_temp_resolution(i2c_master_dev_handle_t mst_dev_handle, int resolution);
esp_err_t  hdc2080_set_humid_resolution(i2c_master_dev_handle_t mst_dev_handle, int resolution);

/* Sets measurement mode, 3 options
       0 - Temperature and Humidity
       1 - Temperature only
       2 - Humidity only
       default - Temperature & Humidity			*/
esp_err_t hdc2080_set_measurement_mode(i2c_master_dev_handle_t mst_dev_handle, int mode);

/* Sets reading rate, 8 options
       0 - Manual
       1 - reading every 2 minutes
       2 - reading every minute
       3 - reading every ten seconds
       4 - reading every 5 seconds
       5 - reading every second
       6 - reading at 2Hz
       7 - reading at 5Hz
       default - Manual		*/
esp_err_t  hdc2080_set_rate(i2c_master_dev_handle_t mst_dev_handle,int rate);

/* Sets Interrupt polarity, 2 options
       0 - Active Low
       1 - Active High
       default - Active Low			*/
esp_err_t  hdc2080_set_interrupt_polarity(i2c_master_dev_handle_t mst_dev_handle, int polarity);

/* Sets Interrupt mode, 2 options
       0 - Level sensitive
       1 - Comparator mode
       default - Level sensitive	*/
esp_err_t  hdc2080_set_interrupt_mode(i2c_master_dev_handle_t mst_dev_handle, int mode);

#ifdef __cplusplus
}
#endif

#endif // __HDC2080_H__