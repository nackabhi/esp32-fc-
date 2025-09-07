#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "driver/i2c.h"
#include "driver/gpio.h"


#define SERVO_FREQ_HZ      50
#define SERVO_PERIOD_US    20000


static const int esc_gpio[4]               = {18, 19, 4, 5};
static const mcpwm_unit_t esc_unit[4]      = {MCPWM_UNIT_0, MCPWM_UNIT_0, MCPWM_UNIT_0, MCPWM_UNIT_0};
static const mcpwm_timer_t esc_timer[4]    = {MCPWM_TIMER_0, MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_1};
static const mcpwm_operator_t esc_opr[4]   = {MCPWM_OPR_A, MCPWM_OPR_B, MCPWM_OPR_A, MCPWM_OPR_B};
static const mcpwm_io_signals_t esc_sig[4] = {MCPWM0A, MCPWM0B, MCPWM1A, MCPWM1B};


#define I2C_MASTER_NUM     I2C_NUM_0
#define I2C_MASTER_SCL_IO  22
#define I2C_MASTER_SDA_IO  21
#define I2C_MASTER_FREQ_HZ 400000
#define MPU6050_ADDR       0x68
#define I2C_TIMEOUT_MS     1000


#define WIFI_SSID   "ESP32-DRONE-SIMPLE"
#define WIFI_PASS   "drone123"

static const char *TAG = "SIMPLE_DRONE";

float gyro_roll = 0, gyro_pitch = 0, gyro_yaw = 0;
float gyro_cal_roll = 0, gyro_cal_pitch = 0, gyro_cal_yaw = 0;
uint16_t motor_values[4] = {1000, 1000, 1000, 1000};
bool system_armed = false;
int base_throttle = 1000;


static void set_motor_pulse(int motor, uint16_t pulse_us)
{
    if (motor < 0 || motor >= 4) return;
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;
    
    float duty = (100.0f * pulse_us) / SERVO_PERIOD_US;
    mcpwm_set_duty(esc_unit[motor], esc_timer[motor], esc_opr[motor], duty);
    mcpwm_set_duty_type(esc_unit[motor], esc_timer[motor], esc_opr[motor], MCPWM_DUTY_MODE_0);
}

static void update_all_motors()
{
    for (int i = 0; i < 4; i++) {
        set_motor_pulse(i, motor_values[i]);
    }
}

static void stop_all_motors()
{
    for (int i = 0; i < 4; i++) {
        motor_values[i] = 1000;
    }
    update_all_motors();
}

static esp_err_t mpu6050_write_reg(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static void read_gyro()
{
    uint8_t data[6];
    if (mpu6050_read_reg(0x43, data, 6) == ESP_OK) {
        int16_t gx = (data[0] << 8) | data[1];
        int16_t gy = (data[2] << 8) | data[3];
        int16_t gz = (data[4] << 8) | data[5];
        
        gyro_roll = (float)gx / 65.5f;
        gyro_pitch = (float)gy / 65.5f;
        gyro_yaw = (float)gz / 65.5f;
    }
}

static void calibrate_gyro()
{
    printf("Calibrating gyro - keep still!\n");
    gyro_cal_roll = gyro_cal_pitch = gyro_cal_yaw = 0;
    
    for (int i = 0; i < 500; i++) {
        read_gyro();
        gyro_cal_roll += gyro_roll;
        gyro_cal_pitch += gyro_pitch;
        gyro_cal_yaw += gyro_yaw;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    gyro_cal_roll /= 500;
    gyro_cal_pitch /= 500;
    gyro_cal_yaw /= 500;
    
    printf("Gyro calibrated: %.2f, %.2f, %.2f\n", gyro_cal_roll, gyro_cal_pitch, gyro_cal_yaw);
}


static void flight_control_loop()
{
    if (!system_armed) {
        stop_all_motors();
        return;
    }
    
 
    read_gyro();
    
 
    float roll_rate = gyro_roll - gyro_cal_roll;
    float pitch_rate = gyro_pitch - gyro_cal_pitch;
    float yaw_rate = gyro_yaw - gyro_cal_yaw;
    
    if (base_throttle > 1050) {  
        
        float roll_correction = -roll_rate * 0.5f;   
        float pitch_correction = -pitch_rate * 0.5f;
        float yaw_correction = -yaw_rate * 1.0f;
        
        // Limit corrections
        if (roll_correction > 50) roll_correction = 50;
        if (roll_correction < -50) roll_correction = -50;
        if (pitch_correction > 50) pitch_correction = 50;
        if (pitch_correction < -50) pitch_correction = -50;
        if (yaw_correction > 30) yaw_correction = 30;
        if (yaw_correction < -30) yaw_correction = -30;
        
        
        motor_values[0] = base_throttle + roll_correction - pitch_correction - yaw_correction; 
        motor_values[1] = base_throttle + roll_correction + pitch_correction + yaw_correction;   
        motor_values[2] = base_throttle - roll_correction + pitch_correction - yaw_correction; 
        motor_values[3] = base_throttle - roll_correction - pitch_correction + yaw_correction; 
        
        
        for (int i = 0; i < 4; i++) {
            if (motor_values[i] < 1000) motor_values[i] = 1000;
            if (motor_values[i] > 1800) motor_values[i] = 1800;
        }
    } else {
        stop_all_motors();
    }
    
    update_all_motors();
}


static const char *html_page = 
"<!DOCTYPE html><html><head><title>Simple Drone Control</title></head><body>"
"<h2>Simple Drone Control</h2>"
"<p><button onclick=\"fetch('/arm')\">ARM</button> "
"<button onclick=\"fetch('/disarm')\">DISARM</button> "
"<button onclick=\"fetch('/emergency')\">EMERGENCY</button></p>"
"<p>Throttle: <input type=\"range\" min=\"1000\" max=\"1600\" value=\"1000\" "
"oninput=\"fetch('/throttle?val='+this.value)\"> <span id=\"val\">1000</span></p>"
"<p><button onclick=\"takeoff()\">Test Takeoff</button></p>"
"<script>"
"function takeoff() {"
"  fetch('/arm').then(()=>{"
"    setTimeout(()=>fetch('/throttle?val=1200'), 500);"
"    setTimeout(()=>fetch('/throttle?val=1000'), 3500);"
"    setTimeout(()=>fetch('/disarm'), 4000);"
"  });"
"}"
"document.querySelector('input').oninput=function(){"
"  document.getElementById('val').textContent=this.value;"
"  fetch('/throttle?val='+this.value);"
"};"
"</script></body></html>";


static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

static esp_err_t arm_handler(httpd_req_t *req)
{
    system_armed = true;
    printf("SYSTEM ARMED\n");
    httpd_resp_sendstr(req, "ARMED");
    return ESP_OK;
}

static esp_err_t disarm_handler(httpd_req_t *req)
{
    system_armed = false;
    base_throttle = 1000;
    stop_all_motors();
    printf("SYSTEM DISARMED\n");
    httpd_resp_sendstr(req, "DISARMED");
    return ESP_OK;
}

static esp_err_t emergency_handler(httpd_req_t *req)
{
    system_armed = false;
    base_throttle = 1000;
    stop_all_motors();
    printf("EMERGENCY STOP\n");
    httpd_resp_sendstr(req, "EMERGENCY STOP");
    return ESP_OK;
}

static esp_err_t throttle_handler(httpd_req_t *req)
{
    char query[64];
    char value[16];
    
    if (httpd_req_get_url_query_len(req) + 1 > sizeof(query)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Query too long");
        return ESP_OK;
    }
    
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "val", value, sizeof(value)) == ESP_OK) {
            int throttle = atoi(value);
            if (throttle >= 1000 && throttle <= 1600) {
                base_throttle = throttle;
                printf("Throttle set to: %d\n", throttle);
                httpd_resp_sendstr(req, "OK");
                return ESP_OK;
            }
        }
    }
    
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid throttle");
    return ESP_OK;
}

static void setup_i2c()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void setup_mcpwm()
{
    mcpwm_config_t pwm_config = {
        .frequency = SERVO_FREQ_HZ,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    
    for (int i = 0; i < 4; i++) {
        mcpwm_gpio_init(esc_unit[i], esc_sig[i], esc_gpio[i]);
    }
    
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_0);
    mcpwm_start(MCPWM_UNIT_0, MCPWM_TIMER_1);
}

static void setup_wifi()
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    wifi_config_t ap_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = 1,
            .password = WIFI_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    printf("WiFi AP started. SSID: %s, Password: %s\n", WIFI_SSID, WIFI_PASS);
}

static void start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
        httpd_uri_t arm = {.uri = "/arm", .method = HTTP_GET, .handler = arm_handler};
        httpd_uri_t disarm = {.uri = "/disarm", .method = HTTP_GET, .handler = disarm_handler};
        httpd_uri_t emergency = {.uri = "/emergency", .method = HTTP_GET, .handler = emergency_handler};
        httpd_uri_t throttle = {.uri = "/throttle", .method = HTTP_GET, .handler = throttle_handler};
        
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &arm);
        httpd_register_uri_handler(server, &disarm);
        httpd_register_uri_handler(server, &emergency);
        httpd_register_uri_handler(server, &throttle);
        
        printf("Web server started on http://192.168.4.1/\n");
    } else {
        printf("Failed to start web server\n");
    }
}


static void flight_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); 
    
    while (1) {
        flight_control_loop();
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void status_task(void *pvParameters)
{
    while (1) {
        if (system_armed) {
            read_gyro();
            float roll = gyro_roll - gyro_cal_roll;
            float pitch = gyro_pitch - gyro_cal_pitch;
            float yaw = gyro_yaw - gyro_cal_yaw;
            
            printf("Armed:%d Throttle:%d Roll:%.1f Pitch:%.1f Yaw:%.1f Motors:[%d,%d,%d,%d]\n",
                   system_armed, base_throttle, roll, pitch, yaw,
                   motor_values[0], motor_values[1], motor_values[2], motor_values[3]);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); 
    }
}


void app_main(void)
{
    printf("Starting Simple Drone Controller\n");
    
  
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    
  
    setup_i2c();
    setup_mcpwm();
    
    
    vTaskDelay(pdMS_TO_TICKS(100));
    mpu6050_write_reg(0x6B, 0x00); 
    mpu6050_write_reg(0x1A, 0x05); 
    mpu6050_write_reg(0x1B, 0x08); 
    vTaskDelay(pdMS_TO_TICKS(100));
    
   
    stop_all_motors();
    
    
    printf("Arming ESCs...\n");
    for (int i = 0; i < 150; i++) {
        stop_all_motors();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    printf("ESCs armed\n");
    
    
    calibrate_gyro();
    
    
    setup_wifi();
    start_webserver();
    
    
    xTaskCreate(flight_task, "flight_task", 4096, NULL, 10, NULL);
    xTaskCreate(status_task, "status_task", 2048, NULL, 5, NULL);
    
    printf("=== SIMPLE DRONE CONTROLLER READY ===\n");
    printf("Connect to WiFi: %s / %s\n", WIFI_SSID, WIFI_PASS);
    printf("Open browser: http://192.168.4.1/\n");
    printf("SAFETY: Test without propellers first!\n");
    printf("1. Click ARM\n");
    printf("2. Use throttle slider or Test Takeoff button\n");
    printf("3. Click DISARM or EMERGENCY to stop\n");
    
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (system_armed) {
            printf("System running - Armed: %s, Throttle: %d\n", 
                   system_armed ? "YES" : "NO", base_throttle);
        }
    }
}
