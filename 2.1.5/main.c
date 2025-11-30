#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "rom/ets_sys.h"
#include "esp_timer.h"

#define TRIG_GPIO 7
#define ECHO_GPIO 1
#define ECHO_TIMEOUT_US 30000 

static const char *TAG = "HC-SR04";

bool wait_for_pin_level(gpio_num_t pin, uint32_t level, uint32_t timeout_us) {
    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(pin) != level) {
        if ((esp_timer_get_time() - start_time) > timeout_us) {
            return false;
        }
    }
    return true; 
}

void app_main(void)
{
    gpio_reset_pin(TRIG_GPIO);
    gpio_set_direction(TRIG_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ECHO_GPIO);
    gpio_set_direction(ECHO_GPIO, GPIO_MODE_INPUT);

    while (1) {
        gpio_set_level(TRIG_GPIO, 0);
        ets_delay_us(2);
        gpio_set_level(TRIG_GPIO, 1);
        ets_delay_us(10);
        gpio_set_level(TRIG_GPIO, 0);

        if (!wait_for_pin_level(ECHO_GPIO, 1, ECHO_TIMEOUT_US)) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        int64_t pulse_start = esp_timer_get_time();

        if (!wait_for_pin_level(ECHO_GPIO, 0, ECHO_TIMEOUT_US)) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        int64_t pulse_end = esp_timer_get_time();

        int64_t duration_us = pulse_end - pulse_start;
        
        if (duration_us > 0 && duration_us < ECHO_TIMEOUT_US) {
            float distance_cm = (duration_us / 2.0f) / 29.1f;
            ESP_LOGI(TAG, "Расстояние: %.2f cm", distance_cm);
        } 

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}