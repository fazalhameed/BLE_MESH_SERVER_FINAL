/* board.c - Board-specific hooks */


#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"
#include "driver/ledc.h"
#include <inttypes.h> 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "BOARD"

// ================= Configuration =================
#define LED_MAX_DUTY       1023       // Max duty for 10-bit resolution
#define FADE_STEP          8         // PWM step size for fade
#define FADE_DELAY_MS      10          // Delay between steps
#define FADE_CYCLES        6          // How many fade in/out cycles

// ==================================================

struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};

static TaskHandle_t led_fade_task_handle = NULL;

void board_led_set_brightness(uint32_t duty) {
   
    if (duty > LED_MAX_DUTY) duty = LED_MAX_DUTY;  // Limit to max
    //ESP_LOGI(TAG, "Setting LED brightness: %lu", (unsigned long)duty);
    ledc_set_duty(LED_PWM_MODE, LED_PWM_CHANNEL, duty);
    ledc_update_duty(LED_PWM_MODE, LED_PWM_CHANNEL);
}

void pwm_init(void) {

    ESP_LOGI(TAG, "Configuring LEDC timer and channel");

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LED_PWM_MODE,
        .timer_num        = LED_PWM_TIMER,
        .duty_resolution  = LED_PWM_RESOLUTION,
        .freq_hz          = LED_PWM_FREQ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num       = LED_R,
        .speed_mode     = LED_PWM_MODE,
        .channel        = LED_PWM_CHANNEL,
        .timer_sel      = LED_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
    ledc_fade_func_install(0);

     ESP_LOGI(TAG, "PWM initialized on GPIO %d",LED_R);
}

// Fade task (runs when LED turns off)
void led_fade_task(void *param) {
    ESP_LOGI(TAG, "Waiting before fade-in...");
    vTaskDelay(pdMS_TO_TICKS(2000));  // Wait 5 seconds before starting fade

    ESP_LOGI(TAG, "Starting fade-in sequence");

for (int cycle = 0; cycle < FADE_CYCLES; cycle++) {
    // Fade up
    ledc_set_fade_with_time(LED_PWM_MODE, LED_PWM_CHANNEL, LED_MAX_DUTY, 500); // 500ms
    ledc_fade_start(LED_PWM_MODE, LED_PWM_CHANNEL, LEDC_FADE_WAIT_DONE);

    // Fade down
    ledc_set_fade_with_time(LED_PWM_MODE, LED_PWM_CHANNEL, 0, 500);
    ledc_fade_start(LED_PWM_MODE, LED_PWM_CHANNEL, LEDC_FADE_WAIT_DONE);
}
    // Ensure LED is fully off
    board_led_set_brightness(0);

    ESP_LOGI(TAG, "Fade sequence complete, LED OFF");

    led_fade_task_handle = NULL;
    vTaskDelete(NULL);
}

void board_led_operation(uint8_t pin, uint8_t onoff)
{
    for (int i = 0; i < 3; i++) {
        if (led_state[i].pin != pin) {
            continue;
        }
        if (onoff == led_state[i].previous) {
            ESP_LOGW(TAG, "led %s is already %s",
                     led_state[i].name, (onoff ? "on" : "off"));
            return;
        }
      // gpio_set_level(pin, onoff);
       led_state[i].previous = onoff;

      // If this LED is the PWM LED, use brightness control
       if (pin == LED_R) {
            if (onoff) {
                // Cancel any fade in progress
                if (led_fade_task_handle != NULL) {
                    vTaskDelete(led_fade_task_handle);
                    led_fade_task_handle = NULL;
                }
                board_led_set_brightness(LED_MAX_DUTY); // Full brightness
            } else {
                  board_led_set_brightness(0);
                // Start fade sequence instead of instant OFF
                if (led_fade_task_handle == NULL) {
                    xTaskCreate(led_fade_task, "led_fade_task", 2048, NULL, 5, &led_fade_task_handle);
                }
            }
        } else {
            // For non-PWM LEDs, just use GPIO level
            gpio_set_level(pin, onoff);
        }

     //   led_state[i].previous = onoff;
       
        ESP_LOGI(TAG, "LED %s turned %s", led_state[i].name, onoff ? "ON" : "OFF");
        return;
    }
    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(led_state[i].pin);   
        gpio_set_direction(led_state[i].pin, GPIO_MODE_OUTPUT);
        gpio_set_level(led_state[i].pin, LED_OFF);
        led_state[i].previous = LED_OFF;
    }
}

void board_init(void)
{
    board_led_init();
    pwm_init();
}
