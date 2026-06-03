// PyDrone-cpp/main/main.cpp
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

// --- Pin Definitions (From Schematic) ---
#define MOTOR1_PIN GPIO_NUM_4
#define MOTOR2_PIN GPIO_NUM_5
#define MOTOR3_PIN GPIO_NUM_40
#define MOTOR4_PIN GPIO_NUM_41

#define I2C_SDA_PIN GPIO_NUM_16
#define I2C_SCL_PIN GPIO_NUM_15

#define VBAT_ADC_CHANNEL ADC_CHANNEL_1 // GPIO2
#define MPU6050_ADDR 0x68

#define LED_BLUE GPIO_NUM_46
#define LED_GREEN GPIO_NUM_42

// --- ESP-NOW Global State ---
static const uint8_t broadcast_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool controller_connected = false;
static uint32_t last_packet_time = 0;

// Motor Control Target Outputs
static int16_t current_rol = 0;
static int16_t current_pit = 0;
static int16_t current_yaw = 0;
static int16_t current_thr = 0;

// Hardware telemetry
static float drone_rol = 0.0f;
static float drone_pit = 0.0f;
static float drone_yaw = 0.0f;

// Calibration offsets
static float calib_rol_offset = 0.0f;
static float calib_pit_offset = 0.0f;

// --- State Machine & Flight Variables ---
static bool is_flying = false;
static int16_t target_height_cm = 0;
static float current_height_cm = 0.0f; 

static float current_x = 0.0f; 
static float current_y = 0.0f;
static bool returning_to_origin = false;
static uint32_t origin_reached_time = 0;
static uint8_t last_btns = 8; // Default POV Hat is 8 (Neutral)

// Track active, ramped motor speeds globally to preserve state between loops
static float active_motor_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

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

void set_motor_speed(int motor_idx, uint32_t duty) {
    if (duty > 1023) duty = 1023;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)motor_idx, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)motor_idx);
}

void stop_motors() {
    for (int i = 0; i < 4; i++) {
        active_motor_speed[i] = 0.0f; // Clear current soft-start tracking memory
        set_motor_speed(i, 0);
    }
}

// Initialize I2C Bus and Wake up MPU6050
void init_i2c_and_mpu6050() {
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

    // Wake MPU6050 from Sleep Mode
    vTaskDelay(pdMS_TO_TICKS(20)); // Small delay to let MPU stabilize
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x6B, true); // PWR_MGMT_1 Register
    i2c_master_write_byte(cmd, 0x00, true); // Wake up
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) ESP_LOGE(TAG, "MPU6050 Init Failed!");
}

// Read raw hardware axis to variables
void read_mpu6050() {
    uint8_t data[14];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x3B, true); // ACCEL_XOUT_H
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, 13, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + 13, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    if (i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(100)) == ESP_OK) {
        int16_t ax = (data[0] << 8) | data[1];
        int16_t ay = (data[2] << 8) | data[3];
        int16_t az = (data[4] << 8) | data[5];
        int16_t gz = (data[12] << 8) | data[13];

        // Accelerometer-based static tilt
        float raw_pit = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
        float raw_rol = atan2(ay, az) * 180.0 / M_PI;

        // Apply dynamic offsets (calibrated via DPAD UP)
        drone_pit = raw_pit - calib_pit_offset;
        drone_rol = raw_rol - calib_rol_offset;

        // Simple integration loop for yaw
        float gz_dps = gz / 131.0f;
        drone_yaw += gz_dps * 0.05f; // 50ms tick
        if (drone_yaw > 180.0f) drone_yaw -= 360.0f;
        if (drone_yaw < -180.0f) drone_yaw += 360.0f;
    }
    i2c_cmd_link_delete(cmd);
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
        .atten = ADC_ATTEN_DB_0, // Max scale ~1.1V for Voltage divider precision
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, VBAT_ADC_CHANNEL, &config));
    return adc_handle;
}

// Maps [0 - 255] byte received into [-100, 100] target control axes
int16_t parse_axis(uint8_t val) {
    if (val > 100 && val < 155) return 0;
    if (val <= 100) return (int16_t)val - 100;
    return (int16_t)val - 155;
}

// --- ESP-NOW Receive Callback ---
void on_data_recv(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    if (data_len >= 16 && memcmp(data, "pyDRONE_DISCOVER", 16) == 0) {
        controller_connected = true;
        last_packet_time = xTaskGetTickCount(); // Important: Stop failsafe from instantly triggering
        
        // Add the controller to the ESP-NOW peer list so that the drone stack accepts unicast control packets
        if (!esp_now_is_peer_exist(esp_now_info->src_addr)) {
            esp_now_peer_info_t peer_info = {};
            peer_info.channel = 1;
            peer_info.encrypt = false;
            memcpy(peer_info.peer_addr, esp_now_info->src_addr, 6);
            esp_err_t err = esp_now_add_peer(&peer_info);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Failed to add controller peer: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG, "Added Controller to Peer List");
            }
        }

        esp_now_send(broadcast_mac, (const uint8_t*)"pyDRONE_ACK", 11);
        ESP_LOGI(TAG, "Sent Discovery ACK to Controller");
        return;
    }

    if (data_len >= 6 && data[0] == 67) { 
        last_packet_time = xTaskGetTickCount();
        controller_connected = true; // Refresh connection state just in case

        int16_t rc_rol = parse_axis(data[1]);
        int16_t rc_pit = parse_axis(data[2]);
        current_yaw = parse_axis(data[3]);
        current_thr = parse_axis(data[4]);
        
        uint8_t btns = data[5];

        // --- DECODE POV HAT & ACTION BUTTONS ---
        uint8_t pov = btns & 0x0F;
        uint8_t last_pov = last_btns & 0x0F;
        
        // 0=UP, 1=UP-RIGHT, 7=UP-LEFT
        bool is_up = (pov == 0 || pov == 1 || pov == 7);
        bool was_up = (last_pov == 0 || last_pov == 1 || last_pov == 7);
        bool press_up = is_up && !was_up; 

        // 4=DOWN, 3=DOWN-RIGHT, 5=DOWN-LEFT
        bool is_down = (pov == 3 || pov == 4 || pov == 5);
        bool was_down = (last_pov == 3 || last_pov == 4 || last_pov == 5);
        bool press_down = is_down && !was_down; 

        // Action buttons are bitmasks on the upper nibble
        bool press_y  = (btns & (1 << 4)) && !(last_btns & (1 << 4)); // Y Press
        bool press_b  = (btns & (1 << 5)) && !(last_btns & (1 << 5)); // B Press
        bool press_a  = (btns & (1 << 6)) && !(last_btns & (1 << 6)); // A Press
        bool press_x  = (btns & (1 << 7)) && !(last_btns & (1 << 7)); // X Press

        last_btns = btns; 

        // 1. KILL SWITCH (Highest Priority)
        if (press_x) {
            is_flying = false;
            returning_to_origin = false;
            target_height_cm = 0;
            current_height_cm = 0;
            stop_motors(); // Extremely fast hardware PWM halt
            ESP_LOGW(TAG, "EMERGENCY STOP (X pressed)! Motors killed instantly.");
        }
        
        // 2. Gyro Calibration (Triggered by DPAD UP)
        if (press_up) {
            calib_rol_offset += drone_rol;
            calib_pit_offset += drone_pit;
            drone_yaw = 0.0f; // Zero out yaw memory
            ESP_LOGI(TAG, "Gyro Calibrated! ROL Offset: %.2f | PIT Offset: %.2f", calib_rol_offset, calib_pit_offset);
        }

        // 3. Return to Origin (Triggered by DPAD DOWN)
        if (press_down && is_flying) {
            returning_to_origin = true;
            ESP_LOGI(TAG, "Return to Origin Initiated!");
        }

        // 4. Flight Commands
        if (press_y) {
            if (!is_flying) {
                // If taking off from ground, make current spot the new (0, 0) origin point
                current_x = 0.0f; 
                current_y = 0.0f;
            }
            target_height_cm = 120; // Take off to 120cm
            is_flying = true;
            returning_to_origin = false;
        }

        if (press_b && is_flying) target_height_cm += 30;
        
        if (press_a && is_flying) {
            target_height_cm -= 30;
            if (target_height_cm < 0) target_height_cm = 0; 
        }

        // 5. Automatic Flight / Joystick Override Control
        if (is_flying && returning_to_origin) {
            float error_x = 0.0f - current_x;
            float error_y = 0.0f - current_y;

            float kp_pos = 1.5f; 
            current_rol = (int16_t)(error_x * kp_pos);
            current_pit = (int16_t)(error_y * kp_pos);

            // Constraint boundaries to prevent aggressive tilt during RTH
            if (current_rol > 30)  current_rol = 30;
            if (current_rol < -30) current_rol = -30;
            if (current_pit > 30)  current_pit = 30;
            if (current_pit < -30) current_pit = -30;

            if (fabs(error_x) < 2.0f && fabs(error_y) < 2.0f) {
                current_rol = 0;
                current_pit = 0;

                if (origin_reached_time == 0) {
                    origin_reached_time = xTaskGetTickCount();
                } else if (pdTICKS_TO_MS(xTaskGetTickCount() - origin_reached_time) >= 2000) {
                    // Hovered at origin for 2 seconds, auto-land.
                    stop_motors();
                    is_flying = false;
                    returning_to_origin = false;
                    origin_reached_time = 0;
                    target_height_cm = 0;
                }
            }
        } else {
            // Apply joystick inputs continuously even when not actively flying or returning
            // This allows the Controller Screen to display accurate realtime targets
            current_rol = rc_rol;
            current_pit = rc_pit;
            origin_reached_time = 0; 
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
    init_i2c_and_mpu6050();
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

    // Add broadcast address to peers list so we can transmit without failing out.
    esp_now_peer_info_t bcast_peer = {};
    bcast_peer.channel = 1;
    bcast_peer.encrypt = false;
    memcpy(bcast_peer.peer_addr, broadcast_mac, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&bcast_peer));

    ESP_LOGI(TAG, "pyDrone C++ Firmware initialized.");

    const TickType_t interval = pdMS_TO_TICKS(50); // 20Hz physics tick
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        vTaskDelayUntil(&last_wake_time, interval);

        // Run I2C Sensor reads continuously to update hardware telemetry payload
        read_mpu6050();

        if (is_flying) {
            // Positional Integration (Track drift relative to takeoff origin)
            float dt = 0.05f; 
            current_x += (current_rol / 100.0f) * 40.0f * dt;
            current_y += (current_pit / 100.0f) * 40.0f * dt;

            // Height Management
            if (current_height_cm < target_height_cm) {
                current_height_cm += 2.0f;
                if (current_height_cm > target_height_cm) current_height_cm = target_height_cm;
            } else if (current_height_cm > target_height_cm) {
                current_height_cm -= 2.0f;
                if (current_height_cm < target_height_cm) current_height_cm = target_height_cm;
            }

            float alt_error = target_height_cm - current_height_cm;
            float kp_alt = 6.0f;
            int base_throttle = 450 + (alt_error * kp_alt); 
            
            // Limit base_throttle step to 750 max to prevent massive immediate current draw on takeoff
            if (base_throttle > 750) base_throttle = 750;
            if (base_throttle < 100) base_throttle = 100;

            int target_speed[4];
            target_speed[0] = base_throttle + current_rol + current_pit; 
            target_speed[1] = base_throttle - current_rol + current_pit; 
            target_speed[2] = base_throttle - current_rol - current_pit; 
            target_speed[3] = base_throttle + current_rol - current_pit; 

            // Smooth ramping (slew-rate limiting) to prevent battery brownouts
            const float max_increase_per_tick = 35.0f; // Soft-start ramp rate
            for (int i = 0; i < 4; i++) {
                if (target_speed[i] > 780) target_speed[i] = 780; // Hard clamp max current draw on battery
                if (target_speed[i] < 0) target_speed[i] = 0;

                if (target_speed[i] > active_motor_speed[i]) {
                    active_motor_speed[i] += max_increase_per_tick;
                    if (active_motor_speed[i] > target_speed[i]) {
                        active_motor_speed[i] = (float)target_speed[i];
                    }
                } else {
                    // Instantly reduce speed if targets drop (for safety and responsiveness)
                    active_motor_speed[i] = (float)target_speed[i];
                }
                set_motor_speed(i, (uint32_t)active_motor_speed[i]);
            }
        } else {
            // Ensure motor speeds are kept at 0 when not flying
            for (int i = 0; i < 4; i++) {
                active_motor_speed[i] = 0.0f;
                set_motor_speed(i, 0);
            }
        }

        // --- ENCODE AND SEND TELEMETRY PACKET (18-Byte Big-Endian Offset) ---
        // MOVED OUTSIDE `if (controller_connected)` so it always broadcasts!
        
        // Read ADC for battery 40.2K/10K voltage divider scaling
        int vbat_raw;
        adc_oneshot_read(adc_handle, VBAT_ADC_CHANNEL, &vbat_raw);
        float vbat = ((float)vbat_raw / 4095.0f) * 1.1f * 5.02f;

        int16_t state_buf_local[9];
        state_buf_local[0] = (int16_t)(drone_rol * 100); 
        state_buf_local[1] = (int16_t)(drone_pit * 100); 
        state_buf_local[2] = (int16_t)(drone_yaw * 100); 
        state_buf_local[3] = current_rol * 10;  
        state_buf_local[4] = current_pit * 10;  
        state_buf_local[5] = current_yaw * 200; 
        state_buf_local[6] = (int16_t)((current_thr + 100) / 2); 
        state_buf_local[7] = (int16_t)(vbat * 100);               
        state_buf_local[8] = (int16_t)(current_height_cm); 

        uint8_t tx_buf[18];
        for (int i = 0; i < 9; i++) {
            uint16_t val_offset = (uint16_t)(state_buf_local[i] + 32768);
            tx_buf[i * 2] = (val_offset >> 8) & 0xFF;
            tx_buf[i * 2 + 1] = val_offset & 0xFF;
        }
        
        // Send telemetry directly to Broadcast MAC unconditionally 
        esp_now_send(broadcast_mac, tx_buf, sizeof(tx_buf));

        // Handle failsafe shutdown separately
        if (controller_connected) {
            uint32_t now = xTaskGetTickCount();
            if (pdTICKS_TO_MS(now - last_packet_time) > 500) {
                stop_motors();
                is_flying = false;
                returning_to_origin = false;
                controller_connected = false; // Note: Telemetry broadcast keeps running!
                gpio_set_level(LED_GREEN, 0);
                ESP_LOGW(TAG, "Connection Lost! Motors powered down.");
            } else {
                gpio_set_level(LED_GREEN, 1);
            }
        }

        gpio_set_level(LED_BLUE, !gpio_get_level(LED_BLUE));
    }
}