/* board.c - Board-specific hooks */


#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "board.h"
#include "driver/ledc.h"
#include <inttypes.h> 

#define TAG "BOARD"


#define LED_PWM_GPIO       4     
#define LED_PWM_FREQ       5000                         // 5 kHz
#define LED_PWM_RESOLUTION LEDC_TIMER_10_BIT            // 10-bit resolution
#define LED_PWM_CHANNEL    LEDC_CHANNEL_0
#define LED_PWM_TIMER      LEDC_TIMER_0
#define LED_PWM_MODE       LEDC_LOW_SPEED_MODE

struct _led_state led_state[3] = {
    { LED_OFF, LED_OFF, LED_R, "red"   },
    { LED_OFF, LED_OFF, LED_G, "green" },
    { LED_OFF, LED_OFF, LED_B, "blue"  },
};


void board_led_set_brightness(uint32_t duty) {
   
    if (duty > 1023) duty = 1023;  // Limit to max
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
        .gpio_num       = LED_PWM_GPIO,
        .speed_mode     = LED_PWM_MODE,
        .channel        = LED_PWM_CHANNEL,
        .timer_sel      = LED_PWM_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ledc_channel);
     ESP_LOGI(TAG, "PWM initialized on GPIO %d", LED_PWM_GPIO);
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
        // ledc_set_duty(LEDC_LOW_SPEED_MODE, i, onoff ? 1023 : 0);
        // ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
         gpio_set_level(pin, onoff);
        led_state[i].previous = onoff;

           // Control PWM brightness together with LED
        if (onoff) {
            board_led_set_brightness(1023);  // Full brightness
        } else {
            board_led_set_brightness(0);     // Turn off PWM
        }
        return;
    }

    ESP_LOGE(TAG, "LED is not found!");
}

static void board_led_init(void)
{
    for (int i = 0; i < 3; i++) {
        gpio_reset_pin(led_state[i].pin);
        // board_led_pwm_init(led_state[i].pin, i); // i == LEDC_CHANNEL_0, _1, _2
        // ledc_set_duty(LEDC_LOW_SPEED_MODE, i, 0);
        // ledc_update_duty(LEDC_LOW_SPEED_MODE, i);
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
