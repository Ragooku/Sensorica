#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"

// пины
#define BUTTON_PIN GPIO_NUM_5
#define DEBOUNCE_TIME_MS 50
#define DOUBLE_CLICK_TIME_MS 200  

static int64_t last_click_time = 0;
static int click_count = 0;
static QueueHandle_t gpio_evt_queue = NULL;
static uint8_t last_button_state = 1; // нач сост

// Прерывания
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void button_task(void* arg) {
    uint32_t io_num;
    int64_t current_time;
    uint8_t current_state;
    
    while(1) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            current_time = esp_timer_get_time() / 1000; // время в мс
            current_state = gpio_get_level(BUTTON_PIN);
            
            //даблкличит
            if(current_time - last_click_time < DEBOUNCE_TIME_MS) {
                continue; 
            }
            
            if(last_button_state == 1 && current_state == 0) {
                printf("Кнопка нажата, клик %d\n", click_count + 1);
                
                if(current_time - last_click_time < DOUBLE_CLICK_TIME_MS) {
                    click_count++;
                } else {
                    click_count = 1;
                }
                
                last_click_time = current_time;
                
                // даблклик текст
                if(click_count == 2) {
                    printf("Двойной клик!\n");
                    click_count = 0;
                }
            }
            last_button_state = current_state;
        }
    }
}

void app_main() {
    // Настройка GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE // Прерывание по спадающему фронту
    };
    gpio_config(&io_conf);
    
    // Инициализируем начальное состояние
    last_button_state = gpio_get_level(BUTTON_PIN);
    
    // Создаем очередь для событий GPIO
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    
    // Устанавливаем обработчик прерывания
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, gpio_isr_handler, (void*)BUTTON_PIN);
    
    // Создаем задачу для обработки кнопки
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    printf("Старт\n");
}