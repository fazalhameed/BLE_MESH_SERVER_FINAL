/* board.c - Board-specific hooks */

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"
#include "driver/ledc.h"
#include <inttypes.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"

#define TAG "BOARD"

// #define LDR_ADC_CHANNEL    ADC1_CHANNEL_6  // GPIO3 = ADC1_CH2
// #define LDR_THRESHOLD      2000             // Adjust based on testing (0–4095)

// PWM configuration constants
#define LED_MAX_DUTY       1023       // Max duty cycle for 10-bit PWM resolution
//#define FADE_STEP          8          // PWM increment/decrement step size
//#define FADE_DELAY_MS      10         // Milliseconds between fade steps
#define FADE_CYCLES        6          // Number of complete fade in/out cycles

// Task handle for LED fade animation
static TaskHandle_t led_fade_task_handle = NULL;

// ============= LED State Management ===============
// Array to track state of RGB LEDs (red, green, blue)

struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },    // Red LED state
    { LED_OFF, LED_OFF, LED_G, "green" },    // Green LED state  
    { LED_OFF, LED_OFF, LED_B, "blue"  },    // Blue LED state
};

// =========== PWM Brightness Control ============
/**
 * @brief Set LED brightness using PWM duty cycle
 * @param duty PWM duty cycle (0-1023 for 10-bit resolution)
 */

void board_led_set_brightness(uint32_t duty) {
    // Clamp duty cycle to maximum allowed value
    
    if (duty > LED_MAX_DUTY) duty = LED_MAX_DUTY; 
    // Update PWM duty cycle and apply changes
    ledc_set_duty(LED_PWM_MODE, LED_PWM_CHANNEL, duty);
    ledc_update_duty(LED_PWM_MODE, LED_PWM_CHANNEL);
}

// ================= PWM Setup =================
/**
 * @brief Initialize LEDC PWM for LED brightness control
 * Configures timer and channel for smooth LED dimming
 */

void pwm_init(void) {
    ESP_LOGI(TAG, "Configuring LEDC timer and channel");
    // Configure LEDC timer for PWM generation
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LED_PWM_MODE,        // High-speed mode
        .timer_num        = LED_PWM_TIMER,       // Timer selection
        .duty_resolution  = LED_PWM_RESOLUTION,  // 10-bit resolution (0-1023)
        .freq_hz          = LED_PWM_FREQ,        // PWM frequency in Hz
        .clk_cfg          = LEDC_AUTO_CLK        // Auto clock selection
    };
    ledc_timer_config(&ledc_timer);
    // Configure LEDC channel for red LED
    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LED_R,            // Red LED GPIO pin
        .speed_mode     = LED_PWM_MODE,     // Match timer speed mode
        .channel        = LED_PWM_CHANNEL,  // Channel number
        .timer_sel      = LED_PWM_TIMER,    // Associated timer
        .duty           = 0,                // Start with LED off
        .hpoint         = 0                 // High point (phase shift)
    };
    ledc_channel_config(&ledc_channel);

    // Install fade function for smooth transitions
    ledc_fade_func_install(0);
    ESP_LOGI(TAG, "PWM initialized on GPIO %d", LED_R);
}

// ============== Fade Animation Task ==============
/**
 * @brief FreeRTOS task for LED fade in/out animation
 * Creates smooth breathing effect with multiple cycles
 */

void led_fade_task(void *param) {
    
    ESP_LOGI(TAG, "Waiting before fade-in...");
    vTaskDelay(pdMS_TO_TICKS(2000));  // 2 second startup delay

    ESP_LOGI(TAG, "Starting fade-in sequence");
    // Execute multiple fade cycles for breathing effect
    for (int cycle = 0; cycle < FADE_CYCLES; cycle++) {
        // Fade up: 0% to 100% brightness over 1 second
        ledc_set_fade_with_time(LED_PWM_MODE, LED_PWM_CHANNEL, LED_MAX_DUTY, 1000);
        ledc_fade_start(LED_PWM_MODE, LED_PWM_CHANNEL, LEDC_FADE_WAIT_DONE);

        // Fade down: 100% to 0% brightness over 1 second
        ledc_set_fade_with_time(LED_PWM_MODE, LED_PWM_CHANNEL, 0, 1000);
        ledc_fade_start(LED_PWM_MODE, LED_PWM_CHANNEL, LEDC_FADE_WAIT_DONE);
    }
    
    // Ensure LED is completely off after animation
    board_led_set_brightness(0);

    ESP_LOGI(TAG, "Fade sequence complete, LED OFF");

    // Clean up task handle and delete task
    led_fade_task_handle = NULL;
    vTaskDelete(NULL);
}

// ============ LED Control Operations ============
/**
 * @brief Control LED on/off state with PWM support for red LED
 * @param pin GPIO pin number of the LED to control
 * @param onoff LED state: 1=ON, 0=OFF
 */

void board_led_operation(uint8_t pin, uint8_t onoff) {
    // Find LED in state array
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;  // Skip if not the target LED
        }
        // Check if LED is already in requested state
        if (onoff == led_state[i].previous) {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
        // Update LED state tracking
        led_state[i].previous = onoff;
        // Special handling for PWM-controlled red LED
        if (pin == LED_R) {
            if (onoff) {
                // Turn ON: Cancel any running fade and set full brightness
                if (led_fade_task_handle != NULL) {
                    vTaskDelete(led_fade_task_handle);
                    led_fade_task_handle = NULL;
                }
                    // ledc_stop(LED_PWM_MODE, LED_PWM_CHANNEL, 1);
                    board_led_set_brightness(LED_MAX_DUTY);  // Full brightness
            } else {
                // Turn OFF: Start fade animation instead of instant off
                //  ledc_stop(LED_PWM_MODE, LED_PWM_CHANNEL, 0); // Output LOW
                    board_led_set_brightness(0);
                if (led_fade_task_handle == NULL) {
                    xTaskCreate(led_fade_task, "led_fade_task", 2048, NULL, 5, &led_fade_task_handle);
                }
            }
        } else {
            // For non-PWM LEDs (green/blue): use simple GPIO control
            gpio_set_level(pin, onoff);
        }
        ESP_LOGI(TAG, "LED %s turned %s", led_state[i].name, onoff ? "ON" : "OFF");
        return;
    }
    // Error: LED pin not found in configuration
    ESP_LOGE(TAG, "LED is not found!");
}

// ============== LED Initialization ===============
/**
 * @brief Initialize all GPIO pins for LED control
 * Sets up red, green, and blue LEDs as outputs in OFF state
 */

static void board_led_init(void) {
    // Configure each LED pin as output and set initial state
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(led_state[i].pin);                        // Reset pin configuration
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);  // Set as output
        gpio_set_level(led_state[i].pin, LED_OFF);               // Turn LED off initially
        led_state[i].previous = LED_OFF;                         // Update state tracking
    }
}

// // ============== LDR Task ==============
// static void ldr_task(void *pv) {
//     while (1) {
//         int raw = adc1_get_raw(LDR_ADC_CHANNEL);
//         ESP_LOGI(TAG, "LDR Value: %d", raw);

//         if (raw < LDR_THRESHOLD) {
//             board_led_operation(LED_R, 1);  // Dark -> Turn ON
//         } else {
//             board_led_operation(LED_R, 0);  // Bright -> Turn OFF
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000));  // check every 1s
//     }
// }

// =========== Board Initialization ============
/**
 * @brief Initialize all board peripherals
 * Sets up LEDs and PWM functionality
 */

void board_init(void) {
    board_led_init();  // Initialize LED GPIO pins
    pwm_init();        // Initialize PWM for brightness control

    //  // Init ADC for LDR
    // adc1_config_width(ADC_WIDTH_BIT_12);
    // adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // // Start LDR task
    // xTaskCreate(ldr_task, "ldr_task", 2048, NULL, 5, NULL);

}
// ===================== END ====================