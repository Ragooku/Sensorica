#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"

#define I2C_MASTER_SDA_IO 9
#define I2C_MASTER_SCL_IO 8
#define I2C_MASTER_FREQ_HZ 40000
#define I2C_MASTER_NUM I2C_NUM_0  

// WS2812
#define LED_GPIO 3
#define LED_NUM 64
#define LED_MATRIX_SIZE 8

// Акселерометр MPU6050
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
} accel_type_t;

// Структура для наклона
typedef struct {
    float pitch;  // тангаж (наклон вперед/назад)
    float roll;   // крен (наклон влево/вправо)
} tilt_type_t;

// Глобальные переменные
static accel_type_t accel_data;
static tilt_type_t device_tilt = {0, 0};
static uint8_t led_matrix[LED_NUM * 3];
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

// Инициализация I2C
static void i2c_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// Запись в регистр MPU6050
static esp_err_t mpu6050_write_reg(uint8_t addr, uint8_t data) {
    uint8_t buf[2] = {addr, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, 0x68, buf, sizeof(buf), pdMS_TO_TICKS(1000));
}

// Чтение регистров MPU6050
static esp_err_t mpu6050_read_reg(uint8_t addr, uint8_t *data, uint8_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, 0x68, &addr, 1, data, len, pdMS_TO_TICKS(1000));
}

// Инициализация MPU6050
static void mpu6050_init() {
    mpu6050_write_reg(0x6B, 0x00); // Выход из спящего режима
    mpu6050_write_reg(0x1C, 0x08); // Акселерометр: ±4g
}

// Чтение данных акселерометра
static esp_err_t mpu6050_read_accel(accel_type_t *data) {
    uint8_t buf[6];
    esp_err_t err = mpu6050_read_reg(0x3B, buf, sizeof(buf));

    data->accel_x = (buf[0] << 8) | buf[1];
    data->accel_y = (buf[2] << 8) | buf[3];
    data->accel_z = (buf[4] << 8) | buf[5];

    return err;
}

// Расчет углов наклона из данных акселерометра
static void calculate_tilt() {
    // Конвертация в g (ускорение свободного падения)
    float ax = accel_data.accel_x / 8192.0f;  // ±4g диапазон
    float ay = accel_data.accel_y / 8192.0f;
    float az = accel_data.accel_z / 8192.0f;
    
    // Расчет углов (в радианах)
    device_tilt.roll = atan2f(ay, az);                    // Крен
    device_tilt.pitch = atan2f(-ax, sqrtf(ay*ay + az*az)); // Тангаж
    
    // Конвертация в градусы
    device_tilt.roll = device_tilt.roll * 180.0f / M_PI;
    device_tilt.pitch = device_tilt.pitch * 180.0f / M_PI;
    
    // Ограничение углов
    if (device_tilt.roll > 45) device_tilt.roll = 45;
    if (device_tilt.roll < -45) device_tilt.roll = -45;
    if (device_tilt.pitch > 45) device_tilt.pitch = 45;
    if (device_tilt.pitch < -45) device_tilt.pitch = -45;
}

// Инициализация RMT для WS2812
static void ws2812_init() {
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz = 10000000,
        .trans_queue_depth = 4,
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = 3,
            .level1 = 0,
            .duration1 = 9,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = 9,
            .level1 = 0,
            .duration1 = 3,
        },
    };
    ESP_ERROR_CHECK(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder));

    ESP_ERROR_CHECK(rmt_enable(led_chan));
}

// Установка цвета пикселя в матрице
static void set_pixel_color(uint8_t x, uint8_t y, uint8_t r, uint8_t g, uint8_t b) {
    if (x >= LED_MATRIX_SIZE || y >= LED_MATRIX_SIZE) return;
    
    uint16_t index;
    if (y % 2 == 0) {
        index = y * LED_MATRIX_SIZE + x;
    } else {
        index = y * LED_MATRIX_SIZE + (LED_MATRIX_SIZE - 1 - x);
    }
    
    led_matrix[index * 3 + 0] = g;
    led_matrix[index * 3 + 1] = r;
    led_matrix[index * 3 + 2] = b;
}

// Очистка матрицы
static void clear_matrix() {
    memset(led_matrix, 0, sizeof(led_matrix));
}

// Обновление LED матрицы
static void update_matrix() {
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_matrix, sizeof(led_matrix), &tx_config));
    vTaskDelay(pdMS_TO_TICKS(10));
}

// Индикация наклона для оценки 3 (светодиоды с яркостью)
static void draw_tilt_indicators_grade3() {
    clear_matrix();
    
    // Центр матрицы
    int center_x = LED_MATRIX_SIZE / 2;
    int center_y = LED_MATRIX_SIZE / 2;
    
    // Определяем направление и степень наклона
    float abs_pitch = fabsf(device_tilt.pitch);
    float abs_roll = fabsf(device_tilt.roll);
    
    // Яркость зависит от степени наклона (0-255)
    uint8_t brightness = (uint8_t)((fmaxf(abs_pitch, abs_roll) / 45.0f) * 255.0f);
    
    // Количество загорающихся светодиодов зависит от степени наклона
    int num_leds = (int)((fmaxf(abs_pitch, abs_roll) / 45.0f) * 3) + 1;
    
    // Цвет: зеленый для малого наклона, желтый для среднего, красный для большого
    uint8_t r, g, b;
    if (fmaxf(abs_pitch, abs_roll) < 15.0f) {
        r = 0; g = brightness; b = 0; // Зеленый
    } else if (fmaxf(abs_pitch, abs_roll) < 30.0f) {
        r = brightness; g = brightness; b = 0; // Желтый
    } else {
        r = brightness; g = 0; b = 0; // Красный
    }
    
    // Индикация наклона вперед/назад
    if (device_tilt.pitch > 5.0f) {
        // Наклон вперед - загораются нижние светодиоды
        for (int i = 0; i < num_leds && (center_y + i + 1) < LED_MATRIX_SIZE; i++) {
            set_pixel_color(center_x, center_y + i + 1, r, g, b);
        }
    } else if (device_tilt.pitch < -5.0f) {
        // Наклон назад - загораются верхние светодиоды
        for (int i = 0; i < num_leds && (center_y - i - 1) >= 0; i++) {
            set_pixel_color(center_x, center_y - i - 1, r, g, b);
        }
    }
    
    // Индикация наклона влево/вправо
    if (device_tilt.roll > 5.0f) {
        // Наклон вправо - загораются правые светодиоды
        for (int i = 0; i < num_leds && (center_x + i + 1) < LED_MATRIX_SIZE; i++) {
            set_pixel_color(center_x + i + 1, center_y, r, g, b);
        }
    } else if (device_tilt.roll < -5.0f) {
        // Наклон влево - загораются левые светодиоды
        for (int i = 0; i < num_leds && (center_x - i - 1) >= 0; i++) {
            set_pixel_color(center_x - i - 1, center_y, r, g, b);
        }
    }
    
    // Центральный светодиод всегда горит белым
    set_pixel_color(center_x, center_y, 255, 255, 255);
}

// Отрисовка стрелки для оценки 4-5
static void draw_tilt_arrow_grade45() {
    clear_matrix();
    
    // Центр матрицы
    int center_x = LED_MATRIX_SIZE / 2;
    int center_y = LED_MATRIX_SIZE / 2;
    
    // Длина стрелки зависит от степени наклона (1-3 клетки)
    int arrow_length = (int)((fmaxf(fabsf(device_tilt.pitch), fabsf(device_tilt.roll)) / 45.0f) * 3) + 1;
    if (arrow_length > 3) arrow_length = 3;
    
    // Цвет стрелки: от зеленого к красному в зависимости от наклона
    uint8_t r, g, b;
    float max_tilt = fmaxf(fabsf(device_tilt.pitch), fabsf(device_tilt.roll));
    if (max_tilt < 15.0f) {
        r = 0; g = 255; b = 0; // Зеленый
    } else if (max_tilt < 30.0f) {
        r = 255; g = 255; b = 0; // Желтый
    } else {
        r = 255; g = 0; b = 0; // Красный
    }
    
    // Определяем направление стрелки
    int end_x = center_x;
    int end_y = center_y;
    
    if (fabsf(device_tilt.pitch) > fabsf(device_tilt.roll)) {
        // Преобладает наклон вперед/назад
        if (device_tilt.pitch > 5.0f) {
            end_y = center_y + arrow_length; // Вперед
        } else if (device_tilt.pitch < -5.0f) {
            end_y = center_y - arrow_length; // Назад
        }
    } else {
        // Преобладает наклон влево/вправо
        if (device_tilt.roll > 5.0f) {
            end_x = center_x + arrow_length; // Вправо
        } else if (device_tilt.roll < -5.0f) {
            end_x = center_x - arrow_length; // Влево
        }
    }
    
    // Рисуем линию стрелки (алгоритм Брезенхема)
    int dx = abs(end_x - center_x);
    int dy = abs(end_y - center_y);
    int sx = (center_x < end_x) ? 1 : -1;
    int sy = (center_y < end_y) ? 1 : -1;
    int err = dx - dy;
    
    int x = center_x;
    int y = center_y;
    
    while (true) {
        set_pixel_color(x, y, r, g, b);
        
        if (x == end_x && y == end_y) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
    
    // Рисуем наконечник стрелки
    if (end_x != center_x || end_y != center_y) {
        // Наконечник в направлении, противоположном направлению стрелки
        if (end_y > center_y) {
            // Стрелка вниз - наконечник вверх
            if (end_y - 1 >= 0) set_pixel_color(end_x, end_y - 1, r, g, b);
            if (end_x - 1 >= 0) set_pixel_color(end_x - 1, end_y, r, g, b);
            if (end_x + 1 < LED_MATRIX_SIZE) set_pixel_color(end_x + 1, end_y, r, g, b);
        } else if (end_y < center_y) {
            // Стрелка вверх - наконечник вниз
            if (end_y + 1 < LED_MATRIX_SIZE) set_pixel_color(end_x, end_y + 1, r, g, b);
            if (end_x - 1 >= 0) set_pixel_color(end_x - 1, end_y, r, g, b);
            if (end_x + 1 < LED_MATRIX_SIZE) set_pixel_color(end_x + 1, end_y, r, g, b);
        } else if (end_x > center_x) {
            // Стрелка вправо - наконечник влево
            if (end_x - 1 >= 0) set_pixel_color(end_x - 1, end_y, r, g, b);
            if (end_y - 1 >= 0) set_pixel_color(end_x, end_y - 1, r, g, b);
            if (end_y + 1 < LED_MATRIX_SIZE) set_pixel_color(end_x, end_y + 1, r, g, b);
        } else if (end_x < center_x) {
            // Стрелка влево - наконечник вправо
            if (end_x + 1 < LED_MATRIX_SIZE) set_pixel_color(end_x + 1, end_y, r, g, b);
            if (end_y - 1 >= 0) set_pixel_color(end_x, end_y - 1, r, g, b);
            if (end_y + 1 < LED_MATRIX_SIZE) set_pixel_color(end_x, end_y + 1, r, g, b);
        }
    }
    
    // Центральный светодиод всегда горит белым
    set_pixel_color(center_x, center_y, 255, 255, 255);
}

// Задача для работы с MPU6050
static void mpu6050_task(void *pvParameter) {
    while (true) {
        if (mpu6050_read_accel(&accel_data) == ESP_OK) {
            calculate_tilt();
            printf("Accel: X=%d, Y=%d, Z=%d | Pitch=%.1f°, Roll=%.1f°\n", 
                   accel_data.accel_x, accel_data.accel_y, accel_data.accel_z,
                   device_tilt.pitch, device_tilt.roll);
        } else {
            printf("Error reading accel data\n");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Задача для индикации наклона
static void tilt_indicator_task(void *pvParameter) {
    int mode = 0; // 0 - оценка 3, 1 - оценка 4-5
    
    while (true) {
        // Переключение режима по нажатию (в реальности можно добавить кнопку)
        // Здесь просто переключаем каждые 10 секунд для демонстрации
        static int counter = 0;
        if (counter++ > 200) { // 200 * 50ms = 10 seconds
            counter = 0;
            mode = !mode;
            printf("Switched to mode: %s\n", mode ? "Grade 4-5 (Arrow)" : "Grade 3 (LEDs)");
        }
        
        if (mode == 0) {
            // Режим для оценки 3
            draw_tilt_indicators_grade3();
        } else {
            // Режим для оценки 4-5
            draw_tilt_arrow_grade45();
        }
        
        update_matrix();
        vTaskDelay(pdMS_TO_TICKS(50)); // 20 FPS
    }
}

void app_main(void) {
    // Инициализация периферии
    i2c_init();
    mpu6050_init();
    ws2812_init();
    
    printf("Tilt Indicator Demo Started\n");
    printf("Mode 1 (Grade 3): LEDs light up in tilt direction with brightness\n");
    printf("Mode 2 (Grade 4-5): Arrow shows tilt direction with length\n");
    printf("Mode switches automatically every 10 seconds\n");
    
    // Создание задач
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 2, NULL);
    xTaskCreate(tilt_indicator_task, "tilt_task", 4096, NULL, 2, NULL);
    
    // Бесконечный цикл
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}