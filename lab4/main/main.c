#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#define SERVO_PIN 18
#define ENC_CLK 17
#define ENC_DT 5
#define ENC_SW 16

#define BTN_DEBOUNCE 40
#define ENC_DEBOUNCE 2

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500

typedef enum {
    MODE_RUN = 0,
    MODE_CW,
    MODE_CCW,
    MODE_MIN,   // теперь "качание 40–120"
    MODE_MAX    // теперь "качание 10–150"
} mode_t;

static volatile mode_t mode = MODE_RUN;

// параметры
static int speed_cw  = 5;
static int speed_ccw = 5;
static int min_angle = 0;
static int max_angle = 180;
static int angle     = 90;
static bool forward  = true;

// антидребезг кнопки
static uint32_t last_btn_time = 0;
static int last_btn_state = 1;

// энкодер
static uint32_t last_enc_time = 0;
static int last_clk = 1;

static inline uint32_t millis() {
    return esp_timer_get_time() / 1000;
}

//---------------------------------------------
// SERVO
//---------------------------------------------
static void servo_init() {
    ledc_timer_config_t t = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t c = {
        .gpio_num = SERVO_PIN,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&c);
}

static void servo_set_angle(int a) {
    if (a < 0) a = 0;
    if (a > 180) a = 180;

    uint32_t pulse = SERVO_MIN_US + (a * (SERVO_MAX_US - SERVO_MIN_US)) / 180;
    uint32_t duty = (pulse * 8191) / 20000;

    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
}

//---------------------------------------------
// BUTTON WITH DEBOUNCE
//---------------------------------------------
bool button_pressed() {
    int current = gpio_get_level(ENC_SW);
    uint32_t now = millis();

    if (current != last_btn_state) {
        last_btn_state = current;
        last_btn_time = now;
    }

    if (current == 0 && (now - last_btn_time) > BTN_DEBOUNCE) {
        while (gpio_get_level(ENC_SW) == 0) vTaskDelay(1);
        return true;
    }
    return false;
}

//---------------------------------------------
// ENCODER
//---------------------------------------------
int encoder_get() {
    int clk = gpio_get_level(ENC_CLK);
    int dt  = gpio_get_level(ENC_DT);

    if (clk != last_clk) {
        last_clk = clk;

        uint32_t now = millis();
        if (now - last_enc_time < ENC_DEBOUNCE) return 0;
        last_enc_time = now;

        return (clk != dt) ? 1 : -1;
    }
    return 0;
}

//---------------------------------------------
// MAIN
//---------------------------------------------
void app_main() {
    gpio_set_direction(ENC_CLK, GPIO_MODE_INPUT);
    gpio_set_direction(ENC_DT, GPIO_MODE_INPUT);
    gpio_set_direction(ENC_SW, GPIO_MODE_INPUT);

    gpio_set_pull_mode(ENC_CLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENC_DT, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(ENC_SW, GPIO_PULLUP_ONLY);

    servo_init();
    servo_set_angle(angle);

    printf("System started\n");

    uint32_t last_move = millis();

    while (1) {

        // --- BUTTON MODE SWITCH ---
        if (button_pressed()) {
            mode = (mode + 1) % 5;
            printf("[MODE] %d\n", mode);
            forward = true;   // сброс направления при смене режима
        }

        // --- ENCODER ADJUSTMENTS ---
        int dir = encoder_get();
        if (dir != 0) {
            switch (mode) {
                case MODE_CW:
                    speed_cw += dir;
                    if (speed_cw < 1) speed_cw = 1;
                    if (speed_cw > 20) speed_cw = 20;
                    break;

                case MODE_CCW:
                    speed_ccw += dir;
                    if (speed_ccw < 1) speed_ccw = 1;
                    if (speed_ccw > 20) speed_ccw = 20;
                    break;

                default:
                    break;  // в mode 3 и 4 энкодер ничего не регулирует
            }
        }

        // --- SERVO MOVEMENT ---
        uint32_t now = millis();
        if (now - last_move > 30) {
            last_move = now;

            switch (mode) {

                case MODE_RUN:
                    if (forward) {
                        angle += speed_cw;
                        if (angle >= max_angle) { angle = max_angle; forward = false; }
                    } else {
                        angle -= speed_ccw;
                        if (angle <= min_angle) { angle = min_angle; forward = true; }
                    }
                    servo_set_angle(angle);
                    break;

                case MODE_CW:
                    angle += speed_cw;
                    if (angle > 180) angle = 180;
                    servo_set_angle(angle);
                    break;

                case MODE_CCW:
                    angle -= speed_ccw;
                    if (angle < 0) angle = 0;
                    servo_set_angle(angle);
                    break;

                // --- NEW SPECIAL MODES ---
                case MODE_MIN: {   // качание 40–120
                    int lo = 40, hi = 120;
                    if (forward) {
                        angle += speed_cw;
                        if (angle >= hi) { angle = hi; forward = false; }
                    } else {
                        angle -= speed_ccw;
                        if (angle <= lo) { angle = lo; forward = true; }
                    }
                    servo_set_angle(angle);
                }
                break;

                case MODE_MAX: {   // качание 10–150
                    int lo = 10, hi = 150;
                    if (forward) {
                        angle += speed_cw;
                        if (angle >= hi) { angle = hi; forward = false; }
                    } else {
                        angle -= speed_ccw;
                        if (angle <= lo) { angle = lo; forward = true; }
                    }
                    servo_set_angle(angle);
                }
                break;
            }
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
