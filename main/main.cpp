#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_mac.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "pyDrone";

// --- Pin Definitions ---
#define MOTOR1_PIN GPIO_NUM_4
#define MOTOR2_PIN GPIO_NUM_5
#define MOTOR3_PIN GPIO_NUM_40
#define MOTOR4_PIN GPIO_NUM_41

#define I2C_SDA_PIN GPIO_NUM_16
#define I2C_SCL_PIN GPIO_NUM_17

#define VBAT_ADC_CHANNEL ADC_CHANNEL_1 // GPIO2

#define LED_BLUE GPIO_NUM_46
#define LED_GREEN GPIO_NUM_42

// --- ESP-NOW Global State ---
static uint8_t controller_mac[6] = {0};
static bool controller_connected = false;
static uint32_t last_packet_time = 0;

// Motor Control Target Outputs
static int16_t current_rol = 0;
static int16_t current_pit = 0;
static int16_t current_yaw = 0;
static int16_t current_thr = 0;

// --- State Machine & Flight Variables ---
static bool is_flying = false;
static int16_t target_height_cm = 0;
static float current_height_cm = 0.0f; // Altitude tracking (simulated/sensor)

// Simple dead reckoning position tracking relative to starting position
static float current_x = 0.0f; 
static float current_y = 0.0f;

static bool returning_to_origin = false;
static uint32_t origin_reached_time = 0;

// Button Edge Detection Helper
static uint8_t last_btns = 0;

// Initialize PWM for 4 Brushless/Brushed Motors
void init_motors() {
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    const gpio_num_t motor_pins[4] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};
    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t channel_conf = {
            .gpio_num = motor_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };
        ledc_channel_config(&channel_conf);
    }
}

// Set Motor PWM Duty (0 - 1023)
void set_motor_speed(int motor_idx, uint32_t duty) {
    if (duty > 1023) duty = 1023;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)motor_idx, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)motor_idx);
}

// Stop all motors instantly
void stop_motors() {
    for (int i = 0; i < 4; i++) {
        set_motor_speed(i, 0);
    }
}

// Initialize I2C Bus for the on-board MPU6050 and other sensors
void init_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = { .clk_speed = 400000 },
        .clk_flags = 0
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

// Initialize ADC for Battery reading
adc_oneshot_unit_handle_t init_adc() {
    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_0,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, VBAT_ADC_CHANNEL, &config));
    return adc_handle;
}

// Maps [0 - 255] byte received into [-100, 100] target control axes
int16_t parse_axis(uint8_t val) {
    if (val > 100 && val < 155) return 0;
    if (val <= 100) return (int16_t)val - 100; // -100 to 0
    return (int16_t)val - 155; // 0 to 100
}

// --- ESP-NOW Receive Callback ---
void on_data_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (data_len == 16 && memcmp(data, "pyDRONE_DISCOVER", 16) == 0) {
        // Send ACK back to complete pairing Handshake
        esp_now_peer_info_t peer_info = {};
        peer_info.channel = 1;
        peer_info.encrypt = false;
        memcpy(peer_info.peer_addr, esp_now_info->src_addr, 6);
        if (!esp_now_is_peer_exist(esp_now_info->src_addr)) {
            esp_now_add_peer(&peer_info);
        }
        memcpy(controller_mac, esp_now_info->src_addr, 6);
        controller_connected = true;
        
        esp_now_send(controller_mac, (const uint8_t*)"pyDRONE_ACK", 11);
        ESP_LOGI(TAG, "Sent Discovery ACK to Controller");
        return;
    }

    if (data_len == 6 && data[0] == 67) { // Standard Control Packet ('C' = 67)
        last_packet_time = xTaskGetTickCount();

        // Extract Joystick commands
        int16_t rc_rol = parse_axis(data[1]);
        int16_t rc_pit = parse_axis(data[2]);
        current_yaw = parse_axis(data[3]);
        current_thr = parse_axis(data[4]);
        
        uint8_t btns = data[5];

        // Edge detection triggers (Activates strictly on key release-to-press transition)
        bool press_y = (btns & (1 << 4)) && !(last_btns & (1 << 4)); // Y Press
        bool press_b = (btns & (1 << 5)) && !(last_btns & (1 << 5)); // B Press
        bool press_a = (btns & (1 << 6)) && !(last_btns & (1 << 6)); // A Press
        bool press_x = (btns & (1 << 7)) && !(last_btns & (1 << 7)); // X Press

        last_btns = btns; // Update state tracking

        // --- BUTTON STATE MACHINE ---
        
        // Y: Hover to 30 cm height
        if (press_y) {
            target_height_cm = 30;
            is_flying = true;
            returning_to_origin = false;
            ESP_LOGI(TAG, "Y pressed -> Elevating to 30cm Hover height.");
        }

        // B: Rise 30 cm higher (can be pressed continuously)
        if (press_b) {
            if (is_flying) {
                target_height_cm += 30;
                ESP_LOGI(TAG, "B pressed -> Target height adjusted up to: %d cm", target_height_cm);
            }
        }

        // A: Descend 30 cm lower (can be pressed continuously)
        if (press_a) {
            if (is_flying) {
                target_height_cm -= 30;
                if (target_height_cm < 0) target_height_cm = 0; // Prevent descending below ground
                ESP_LOGI(TAG, "A pressed -> Target height adjusted down to: %d cm", target_height_cm);
            }
        }

        // X: Return to origin/starting coordinate (0,0) and turn off motors after 2 seconds
        if (press_x) {
            if (is_flying) {
                returning_to_origin = true;
                ESP_LOGI(TAG, "X pressed -> Initiating Return-To-Origin sequence.");
            }
        }

        // Handle flight controls
        if (is_flying) {
            if (returning_to_origin) {
                // Return to Origin (RTL): Overwrite manual controls to steer back to coordinate (0,0)
                float error_x = 0.0f - current_x;
                float error_y = 0.0f - current_y;

                float kp_pos = 1.5f; // Proportional steer coefficient
                current_rol = (int16_t)(error_x * kp_pos);
                current_pit = (int16_t)(error_y * kp_pos);

                // Safe roll & pitch clamp limits during automatic RTL
                if (current_rol > 30)  current_rol = 30;
                if (current_rol < -30) current_rol = -30;
                if (current_pit > 30)  current_pit = 30;
                if (current_pit < -30) current_pit = -30;

                // Check if we are physically back within tolerance of origin
                if (fabs(error_x) < 2.0f && fabs(error_y) < 2.0f) {
                    current_rol = 0;
                    current_pit = 0;

                    if (origin_reached_time == 0) {
                        origin_reached_time = xTaskGetTickCount();
                        ESP_LOGI(TAG, "Arrived at origin! Starting 2-second countdown to land.");
                    } else if (pdTICKS_TO_MS(xTaskGetTickCount() - origin_reached_time) >= 2000) {
                        // 2 seconds have elapsed after returning: turn off motors
                        stop_motors();
                        is_flying = false;
                        returning_to_origin = false;
                        origin_reached_time = 0;
                        target_height_cm = 0;
                        ESP_LOGI(TAG, "Landing timer expired. Motors turned off.");
                    }
                }
            } else {
                // Manual Flight Mode: Accept raw inputs from controller sticks
                current_rol = rc_rol;
                current_pit = rc_pit;
                origin_reached_time = 0; // Reset timer if aborted
            }
        }
    }
}

extern "C" void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_reset_pin(LED_BLUE);
    gpio_set_direction(LED_BLUE, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LED_GREEN);
    gpio_set_direction(LED_GREEN, GPIO_MODE_OUTPUT);
    
    gpio_set_level(LED_BLUE, 1);
    gpio_set_level(LED_GREEN, 0);

    init_motors();
    init_i2c();
    adc_oneshot_unit_handle_t adc_handle = init_adc();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_data_recv));

    ESP_LOGI(TAG, "pyDrone C++ Firmware initialized.");

    const TickType_t interval = pdMS_TO_TICKS(50); // 20Hz physics tick
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, interval);

        // --- PHYSICAL DYNAMICS SIMULATION & PID PLACEHOLDER ---
        if (is_flying) {
            // Dead Reckoning: Track simulated X and Y spatial displacement
            // Pitch tilts drone on Y axis, Roll tilts drone on X axis
            float dt = 0.05f; 
            current_x += (current_rol / 100.0f) * 40.0f * dt;
            current_y += (current_pit / 100.0f) * 40.0f * dt;

            // Height Simulation tracking target height (To be swapped with Barometer sensor reading)
            if (current_height_cm < target_height_cm) {
                current_height_cm += 2.0f; // Climb rate
                if (current_height_cm > target_height_cm) current_height_cm = target_height_cm;
            } else if (current_height_cm > target_height_cm) {
                current_height_cm -= 2.0f; // Fall rate
                if (current_height_cm < target_height_cm) current_height_cm = target_height_cm;
            }

            // Simple Height PID Loop Proportional Feedback
            float alt_error = target_height_cm - current_height_cm;
            float kp_alt = 6.0f;
            int base_throttle = 450 + (alt_error * kp_alt); // Hover base index ~450
            if (base_throttle < 100) base_throttle = 100;

            // Output mixing to motors
            set_motor_speed(0, base_throttle + current_rol + current_pit); // Motor 1
            set_motor_speed(1, base_throttle - current_rol + current_pit); // Motor 2
            set_motor_speed(2, base_throttle - current_rol - current_pit); // Motor 3
            set_motor_speed(3, base_throttle + current_rol - current_pit); // Motor 4

            ESP_LOGI(TAG, "Telemetry -> Pos: (X:%.2f, Y:%.2f) | Height: %.1fcm / Target: %dcm", 
                     current_x, current_y, current_height_cm, target_height_cm);
        }

        // Failsafe connection check (Timeout triggers disarm if controller cuts out)
        if (controller_connected) {
            uint32_t now = xTaskGetTickCount();
            if (pdTICKS_TO_MS(now - last_packet_time) > 500) {
                stop_motors();
                is_flying = false;
                controller_connected = false;
                gpio_set_level(LED_GREEN, 0);
                ESP_LOGW(TAG, "Connection Lost! Motors powered down.");
            } else {
                gpio_set_level(LED_GREEN, 1);
            }
        }

        gpio_set_level(LED_BLUE, !gpio_get_level(LED_BLUE));
    }
}