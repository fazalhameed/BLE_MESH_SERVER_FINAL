/**
 * @file board.h
 * @brief Board-specific hooks and LED configuration
 * 
 * This header file defines board-specific configurations for different ESP32 variants,
 * particularly focusing on LED control and PWM settings for BLE Mesh applications.
 */

/* board.h - Board-specific hooks */

#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif /**< __cplusplus */

#include "driver/gpio.h"

// PWM configuration for LED control
#define LED_MAX_DUTY       1023       // Max duty cycle for 10-bit PWM resolution
#define LED_PWM_FREQ       5000                         // PWM frequency: 5 kHz
#define LED_PWM_RESOLUTION LEDC_TIMER_10_BIT            // PWM resolution: 10-bit
#define LED_PWM_CHANNEL    LEDC_CHANNEL_0               // PWM channel 0
#define LED_PWM_TIMER      LEDC_TIMER_0                 // PWM timer 0
#define LED_PWM_MODE       LEDC_LOW_SPEED_MODE          // PWM mode: low speed

#define LDR_ADC_CHANNEL   ADC1_CHANNEL_7   // GPIO5 → ADC1_CH0
#define LDR_THRESHOLD     2000             // Adjust experimentally (0-4095)

/**
 * @brief Board-specific LED pin configuration
 * 
 * Defines RGB LED pins for different ESP32 development boards.
 * Note: Some boards may have shared pins for multiple LEDs.
 */
#if defined(CONFIG_BLE_MESH_ESP_WROOM_32)
    // ESP32-WROOM-32 configuration
    #define LED_R GPIO_NUM_4 // Red LED pin
    #define LED_G GPIO_NUM_4  // Green LED pin (shared with Red)
    #define LED_B GPIO_NUM_22   // Blue LED pin
  
#elif defined(CONFIG_BLE_MESH_ESP_WROVER)
    // ESP-WROVER-KIT configuration
    #define LED_R GPIO_NUM_0    // Red LED pin
    #define LED_G GPIO_NUM_2    // Green LED pin
    #define LED_B GPIO_NUM_4    // Blue LED pin
    
#elif defined(CONFIG_BLE_MESH_ESP32C3_DEV)
    // ESP32-C3-DevKit configuration
    #define LED_R GPIO_NUM_8    // All LEDs share the same pin
    #define LED_G GPIO_NUM_8
    #define LED_B GPIO_NUM_8
    
#elif defined(CONFIG_BLE_MESH_ESP32S3_DEV)
    // ESP32-S3-DevKit configuration
    #define LED_R GPIO_NUM_47
    #define LED_G GPIO_NUM_47
    #define LED_B GPIO_NUM_47

#elif defined(CONFIG_BLE_MESH_ESP32C6_DEV)
    // ESP32-C6-DevKit configuration
    #define LED_R GPIO_NUM_8
    #define LED_G GPIO_NUM_8
    #define LED_B GPIO_NUM_8

#elif defined(CONFIG_BLE_MESH_ESP32C61_DEV)
    // ESP32-C61-DevKit configuration
    #define LED_R GPIO_NUM_8
    #define LED_G GPIO_NUM_8
    #define LED_B GPIO_NUM_8

#elif defined(CONFIG_BLE_MESH_ESP32H2_DEV)
    // ESP32-H2-DevKit configuration
    #define LED_R GPIO_NUM_8
    #define LED_G GPIO_NUM_8
    #define LED_B GPIO_NUM_8

#elif defined(CONFIG_BLE_MESH_ESP32C5_DEV)
    // ESP32-C5-DevKit configuration
    #define LED_R GPIO_NUM_8
    #define LED_G GPIO_NUM_8
    #define LED_B GPIO_NUM_8
#endif

// LED state definitions
#define LED_ON  1   // Logical ON state for LED
#define LED_OFF 0   // Logical OFF state for LED

/**
 * @brief LED state structure
 * 
 * Tracks current and previous states of an LED along with its pin and name.
 */
struct _led_state {
    uint8_t current;    // Current state of the LED (ON/OFF)
    uint8_t previous;   // Previous state of the LED
    uint8_t pin;        // GPIO pin number for the LED
    char *name;         // Descriptive name for the LED
};

/**
 * @brief Control an LED's state
 * @param pin GPIO pin number of the LED
 * @param onoff Desired state (LED_ON or LED_OFF)
 */
void board_led_operation(uint8_t pin, uint8_t onoff);

/**
 * @brief Set LED brightness using PWM
 * @param duty PWM duty cycle (0-1023 for 10-bit resolution)
 */
void board_led_set_brightness(uint32_t duty);

/**
 * @brief Initialize PWM for LED control
 */
void pwm_init(void);

/**
 * @brief Board initialization function
 * 
 * Should be called once at startup to configure board-specific hardware.
 */
void board_init(void);

#ifdef __cplusplus
}
#endif /**< __cplusplus */

#endif /* _BOARD_H_ */