#include <stdio.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>

static esp_timer_handle_t debounce_timer = NULL;
static esp_timer_handle_t double_click_timer = NULL;
static uint32_t debounce_time = 10000;
static uint32_t click_count = 0;
static uint32_t last_click_time = 0;
static uint32_t double_click_timeout = 1000000;

static void debounceTimer(void* args) {
    gpio_intr_disable(GPIO_NUM_5);
    uint32_t current_state = gpio_get_level(GPIO_NUM_5);

    if (current_state == 0) {
        uint64_t current_time = esp_timer_get_time();
        
        if ((current_time - last_click_time) > debounce_time) {
            click_count++;
            
            if (click_count == 1) {
                esp_timer_start_once(double_click_timer, double_click_timeout);
            } else if (click_count == 2) {
                uint64_t click_interval = current_time - last_click_time;
                if (click_interval < double_click_timeout) {
                    ESP_LOGW("MAIN", "Double click detected! Interval: %llu us", click_interval);
                    click_count = 0;
                    esp_timer_stop(double_click_timer);
                }
            }
            
            last_click_time = current_time;
            ESP_LOGI("MAIN", "Click count: %d", click_count);
        }
    }

    gpio_intr_enable(GPIO_NUM_5);
}

static void doubleClickTimeout(void* args) {
    if (click_count == 1) {
        ESP_LOGI("MAIN", "Single click detected");
    }
    click_count = 0;
}

static void gpioISRHandle(void* args) {
    gpio_intr_disable(GPIO_NUM_5);
    esp_timer_start_once(debounce_timer, debounce_time);
}

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_5),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&io_conf);

    // Create debounce timer
    esp_timer_create_args_t debounce_tmr_cfg = {
        .callback = debounceTimer,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "debounce",
        .skip_unhandled_events = false
    };
    esp_timer_create(&debounce_tmr_cfg, &debounce_timer);

    // Create double click timeout timer
    esp_timer_create_args_t dbl_click_tmr_cfg = {
        .callback = doubleClickTimeout,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "dblclick",
        .skip_unhandled_events = false
    };
    esp_timer_create(&dbl_click_tmr_cfg, &double_click_timer);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(GPIO_NUM_5, gpioISRHandle, NULL);
    gpio_intr_enable(GPIO_NUM_5);

    ESP_LOGI("MAIN", "Double click detector started");
    ESP_LOGI("MAIN", "Double click timeout: %d ms", double_click_timeout / 1000);
}