#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "nvs_flash.h"

#define WIFI_SSID "ESP32-DRONE-CONTROL"
#define WIFI_PASS "drone123"

static const char *TAG = "DRONE_CONTROL";

#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 400000

#define MPU6050_ADDRESS 0x68
#define BMP280_ADDRESS 0x76

#define MOTOR1_PIN 4 //fl
#define MOTOR2_PIN 19 // fr
#define MOTOR3_PIN 5 // bl
#define MOTOR4_PIN 18 // br
 
struct PID_Controller {
    float kp;
    float ki; 
    float kd;
    float previous_error;
    float integral;
    float output;
};

struct PID_Controller roll_pid;
struct PID_Controller pitch_pid; 
struct PID_Controller yaw_pid;
struct PID_Controller alt_pid;

float current_roll = 0;
float current_pitch = 0;
float current_yaw = 0;
float current_altitude = 0;

float target_roll = 0;
float target_pitch = 0;
float target_yaw = 0;
float target_altitude = 0;

int base_throttle = 1000;
int motor_speeds[4];
int manual_throttle = 1000;
bool manual_mode = true;
bool system_armed = false;

float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float pressure, temperature;

uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

void setup_i2c() {
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ
    };
    
    i2c_param_config(0, &i2c_config);
    i2c_driver_install(0, I2C_MODE_MASTER, 0, 0, 0);
}

void i2c_write_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void i2c_read_reg(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, int len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_READ, true);
    
    for(int i = 0; i < len - 1; i++) {
        i2c_master_read_byte(cmd, &data[i], I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len-1], I2C_MASTER_NACK);
    
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(0, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

void setup_mpu6050() {
    i2c_write_reg(MPU6050_ADDRESS, 0x6B, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    i2c_write_reg(MPU6050_ADDRESS, 0x19, 0x07);
    i2c_write_reg(MPU6050_ADDRESS, 0x1B, 0x08);
    i2c_write_reg(MPU6050_ADDRESS, 0x1C, 0x10);
    
    printf("MPU6050 setup done\n");
}

void read_mpu6050() {
    uint8_t data[14];
    i2c_read_reg(MPU6050_ADDRESS, 0x3B, data, 14);
    
    int16_t ax = (data[0] << 8) | data[1];
    int16_t ay = (data[2] << 8) | data[3]; 
    int16_t az = (data[4] << 8) | data[5];
    int16_t gx = (data[8] << 8) | data[9];
    int16_t gy = (data[10] << 8) | data[11];
    int16_t gz = (data[12] << 8) | data[13];
    
    accel_x = ax / 4096.0;
    accel_y = ay / 4096.0;
    accel_z = az / 4096.0;
    gyro_x = gx / 65.5;
    gyro_y = gy / 65.5; 
    gyro_z = gz / 65.5;
}
/*
void setup_bmp280() {
    uint8_t id;
    i2c_read_reg(BMP280_ADDRESS, 0xD0, &id, 1);
    if(id != 0x58) {
        printf("BMP280 not found!\n");
        return;
    }
    
    uint8_t calib[24];
    i2c_read_reg(BMP280_ADDRESS, 0x88, calib, 24);
    
    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];
    
    i2c_write_reg(BMP280_ADDRESS, 0xF4, 0xFF);
    i2c_write_reg(BMP280_ADDRESS, 0xF5, 0x00);
    
    printf("BMP280 setup done\n");
}

void read_bmp280() {
    uint8_t data[6];
    i2c_read_reg(BMP280_ADDRESS, 0xF7, data, 6);
    
    int32_t raw_pressure = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;
    int32_t raw_temp = ((data[3] << 16) | (data[4] << 8) | data[5]) >> 4;
    
    int32_t var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) >> 8;
    temperature = temperature / 100.0;
    
    int64_t var1_p = ((int64_t)t_fine) - 128000;
    int64_t var2_p = var1_p * var1_p * (int64_t)dig_P6;
    var2_p = var2_p + ((var1_p * (int64_t)dig_P5) << 17);
    var2_p = var2_p + (((int64_t)dig_P4) << 35);
    var1_p = ((var1_p * var1_p * (int64_t)dig_P3) >> 8) + ((var1_p * (int64_t)dig_P2) << 12);
    var1_p = (((((int64_t)1) << 47) + var1_p)) * ((int64_t)dig_P1) >> 33;
    
    if (var1_p != 0) {
        int64_t p = 1048576 - raw_pressure;
        p = (((p << 31) - var2_p) * 3125) / var1_p;
        var1_p = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2_p = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + var1_p + var2_p) >> 8) + (((int64_t)dig_P7) << 4);
        pressure = (float)p / 256.0;
        
        current_altitude = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
    }
}
*/
void setup_motors() {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); 
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
    
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR1_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, MOTOR2_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, MOTOR3_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, MOTOR4_PIN);
    
    printf("Motors setup done\n");
}

void set_motor(int motor_num, int speed) {
    float duty = (speed / 20000.0) * 100.0;
    
    switch(motor_num) {
        case 0: mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty); break;
        case 1: mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty); break;
        case 2: mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty); break;
        case 3: mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty); break;
    }
}

void calculate_angles() {
    current_roll = atan2(accel_y, accel_z) * 180.0 / M_PI;
    current_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / M_PI;
    
    static float last_roll = 0, last_pitch = 0;
    current_roll = 0.98 * (last_roll + gyro_x * 0.01) + 0.02 * current_roll;
    current_pitch = 0.98 * (last_pitch + gyro_y * 0.01) + 0.02 * current_pitch;
    current_yaw += gyro_z * 0.01;
    
    last_roll = current_roll;
    last_pitch = current_pitch;
}

float calculate_roll_pid(float target, float current) {
    float error = target - current;
    
    float p_term = roll_pid.kp * error;
    
    roll_pid.integral += error * 0.01;
    float i_term = roll_pid.ki * roll_pid.integral;
    
    float d_term = roll_pid.kd * (error - roll_pid.previous_error) / 0.01;
    roll_pid.previous_error = error;
    
    roll_pid.output = p_term + i_term + d_term;
    
    if(roll_pid.output > 400) roll_pid.output = 400;
    if(roll_pid.output < -400) roll_pid.output = -400;
    
    return roll_pid.output;
}

float calculate_pitch_pid(float target, float current) {
    float error = target - current;
    
    float p_term = pitch_pid.kp * error;
    
    pitch_pid.integral += error * 0.01;
    float i_term = pitch_pid.ki * pitch_pid.integral;
    
    float d_term = pitch_pid.kd * (error - pitch_pid.previous_error) / 0.01;
    pitch_pid.previous_error = error;
    
    pitch_pid.output = p_term + i_term + d_term;
    
    if(pitch_pid.output > 400) pitch_pid.output = 400;
    if(pitch_pid.output < -400) pitch_pid.output = -400;
    
    return pitch_pid.output;
}

float calculate_yaw_pid(float target, float current) {
    float error = target - current;
    
    float p_term = yaw_pid.kp * error;
    
    yaw_pid.integral += error * 0.01;
    float i_term = yaw_pid.ki * yaw_pid.integral;
    
    float d_term = yaw_pid.kd * (error - yaw_pid.previous_error) / 0.01;
    yaw_pid.previous_error = error;
    
    yaw_pid.output = p_term + i_term + d_term;
    
    if(yaw_pid.output > 400) yaw_pid.output = 400;
    if(yaw_pid.output < -400) yaw_pid.output = -400;
    
    return yaw_pid.output;
}

float calculate_altitude_pid(float target, float current) {
    float error = target - current;
    
    float p_term = alt_pid.kp * error;
    
    alt_pid.integral += error * 0.01;
    float i_term = alt_pid.ki * alt_pid.integral;
    
    float d_term = alt_pid.kd * (error - alt_pid.previous_error) / 0.01;
    alt_pid.previous_error = error;
    
    
    alt_pid.output = p_term + i_term + d_term;
    
    if(alt_pid.output > 300) alt_pid.output = 300;
    if(alt_pid.output < -300) alt_pid.output = -300;
    
    return alt_pid.output;
}

void mix_motors() {
    float roll_correction = calculate_roll_pid(target_roll, current_roll);
    float pitch_correction = calculate_pitch_pid(target_pitch, current_pitch); 
    float yaw_correction = calculate_yaw_pid(target_yaw, current_yaw);
    float alt_correction = calculate_altitude_pid(target_altitude, current_altitude);
    
    int total_throttle = base_throttle + (int)alt_correction;
    
    motor_speeds[0] = total_throttle - roll_correction + pitch_correction - yaw_correction;
    motor_speeds[1] = total_throttle + roll_correction + pitch_correction + yaw_correction;
    motor_speeds[2] = total_throttle - roll_correction - pitch_correction + yaw_correction;
    motor_speeds[3] = total_throttle + roll_correction - pitch_correction - yaw_correction;
    
    for(int i = 0; i < 4; i++) {
        if(motor_speeds[i] < 1000) motor_speeds[i] = 1000;
        if(motor_speeds[i] > 2000) motor_speeds[i] = 2000;
    }
}

void stop_all_motors() {
    for(int i = 0; i < 4; i++) {
        set_motor(i, 1000);
        motor_speeds[i] = 1000;
    }
    manual_throttle = 1000;
    printf("EMERGENCY STOP - All motors stopped\n");
}

void set_all_motors_manual(int throttle) {
    if(throttle < 1000) throttle = 1000;
    if(throttle > 2000) throttle = 2000;
    
    if(system_armed) {
        for(int i = 0; i < 4; i++) {
            set_motor(i, throttle);
        }
        manual_throttle = throttle;
    } else {
        for(int i = 0; i < 4; i++) {
            set_motor(i, 1000);
        }
    }
}

void update_motors() {
    if(manual_mode) {
        set_all_motors_manual(manual_throttle);
    } else {
        if(system_armed) {
            for(int i = 0; i < 4; i++) {
                set_motor(i, motor_speeds[i]);
            }
        } else {
            stop_all_motors();
        }
    }
}

static const char *html_page = 
"<!DOCTYPE html><html><head><title>Drone Motor Control</title>"
"<style>"
"body{font-family:Arial;text-align:center;padding:20px;background:#f0f0f0;}"
".container{max-width:600px;margin:0 auto;background:white;padding:30px;border-radius:10px;box-shadow:0 4px 8px rgba(0,0,0,0.1);}"
".status{padding:15px;margin:10px 0;border-radius:5px;font-weight:bold;}"
".armed{background:#ffe6e6;color:#cc0000;}"
".disarmed{background:#e6ffe6;color:#006600;}"
".manual{background:#e7f3ff;border-left:5px solid #2196F3;}"
".auto{background:#e8f5e8;border-left:5px solid #4CAF50;}"
".button{background:#4CAF50;border:none;color:white;padding:15px 32px;text-align:center;font-size:16px;margin:10px;border-radius:5px;cursor:pointer;}"
".button:hover{background:#45a049;}"
".danger{background:#f44336;}"
".danger:hover{background:#da190b;}"
".warning{background:#ff9800;}"
".warning:hover{background:#f57c00;}"
".slider{width:100%;height:30px;border-radius:15px;background:#ddd;outline:none;}"
".slider::-webkit-slider-thumb{appearance:none;width:30px;height:30px;border-radius:50%;background:#4CAF50;cursor:pointer;}"
".value-display{font-size:24px;font-weight:bold;color:#333;margin:10px 0;}"
"</style></head><body>"
"<div class='container'>"
"<h1> Drone Motor Controller</h1>"
"<div id='armStatus' class='status disarmed'>SYSTEM DISARMED</div>"
"<div id='modeStatus' class='status manual'>Manual Mode - Direct Motor Control</div>"
"<div style='margin:20px 0;'>"
"<button class='button' onclick='armSystem()' id='armButton'>ARM SYSTEM</button>"
"<button class='button danger' onclick='emergencyStop()'>EMERGENCY STOP</button>"
"</div>"
"<div style='margin:20px 0;'>"
"<h3>Throttle Control (All Motors)</h3>"
"<input type='range' min='1000' max='1800' value='1000' class='slider' id='throttleSlider'>"
"<div class='value-display' id='throttleValue'>1000</div>"
"</div>"
"<button class='button' onclick='toggleMode()' id='modeButton'>Switch to PID Mode</button>"
"<button class='button warning' onclick='testTakeoff()'>Test Takeoff (3 sec)</button>"
"<div style='margin-top:20px;'>"
"<p><strong> SAFETY WARNING:</strong> Always remove propellers for testing!</p>"
"<p><strong>Range:</strong> 1000 = Motors Off, 1800 = Safe Max Speed</p>"
"</div></div>"
"<script>"
"const slider = document.getElementById('throttleSlider');"
"const valueDisplay = document.getElementById('throttleValue');"
"const armStatus = document.getElementById('armStatus');"
"const modeStatus = document.getElementById('modeStatus');"
"const armButton = document.getElementById('armButton');"
"const modeButton = document.getElementById('modeButton');"
"let isArmed = false;"
"let isManualMode = true;"
"slider.oninput = function() {"
"  valueDisplay.innerHTML = this.value;"
"  fetch('/throttle?val=' + this.value);"
"};"
"function armSystem() {"
"  if(!isArmed) {"
"    fetch('/arm').then(response => response.text()).then(result => {"
"      if(result.includes('ARMED')) {"
"        isArmed = true;"
"        armStatus.className = 'status armed';"
"        armStatus.innerHTML = 'SYSTEM ARMED - READY TO FLY';"
"        armButton.innerHTML = 'DISARM SYSTEM';"
"        slider.disabled = false;"
"      }"
"    });"
"  } else {"
"    disarmSystem();"
"  }"
"}"
"function disarmSystem() {"
"  fetch('/disarm').then(() => {"
"    isArmed = false;"
"    armStatus.className = 'status disarmed';"
"    armStatus.innerHTML = 'SYSTEM DISARMED';"
"    armButton.innerHTML = 'ARM SYSTEM';"
"    slider.value = 1000;"
"    valueDisplay.innerHTML = '1000';"
"    slider.disabled = true;"
"  });"
"}"
"function emergencyStop() {"
"  fetch('/emergency');"
"  disarmSystem();"
"  alert('EMERGENCY STOP ACTIVATED! ');"
"}"
"function toggleMode() {"
"  fetch('/toggle_mode').then(response => response.text()).then(mode => {"
"    isManualMode = (mode === 'manual');"
"    if(isManualMode) {"
"      modeStatus.className = 'status manual';"
"      modeStatus.innerHTML = 'Manual Mode - Direct Motor Control';"
"      modeButton.innerHTML = 'Switch to PID Mode';"
"      slider.style.display = 'block';"
"    } else {"
"      modeStatus.className = 'status auto';"
"      modeStatus.innerHTML = 'PID Mode - Stabilization Active';"
"      modeButton.innerHTML = 'Switch to Manual Mode';"
"      slider.style.display = 'none';"
"    }"
"  });"
"}"
"function testTakeoff() {"
"  if(!isArmed) {"
"    alert('Please ARM system first!');"
"    return;"
"  }"
"  fetch('/arm').then(()=>{"
"    slider.value = 1200;"
"    valueDisplay.innerHTML = '1200';"
"    fetch('/throttle?val=1200');"
"    setTimeout(()=>{"
"      slider.value = 1000;"
"      valueDisplay.innerHTML = '1000';"
"      fetch('/throttle?val=1000');"
"    }, 3000);"
"  });"
"}"
"slider.disabled = true;"
"</script></body></html>";

esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, strlen(html_page));
    return ESP_OK;
}

esp_err_t arm_handler(httpd_req_t *req) {
    system_armed = true;
    printf("SYSTEM ARMED\n");
    httpd_resp_sendstr(req, "SYSTEM ARMED");
    return ESP_OK;
}

esp_err_t disarm_handler(httpd_req_t *req) {
    system_armed = false;
    manual_throttle = 1000;
    base_throttle = 1000;
    stop_all_motors();
    printf("SYSTEM DISARMED\n");
    httpd_resp_sendstr(req, "SYSTEM DISARMED");
    return ESP_OK;
}

esp_err_t emergency_handler(httpd_req_t *req) {
    system_armed = false;
    manual_throttle = 1000;
    base_throttle = 1000;
    stop_all_motors();
    printf(" EMERGENCY STOP ACTIVATED! \n");
    httpd_resp_sendstr(req, "EMERGENCY STOP ACTIVATED");
    return ESP_OK;
}

esp_err_t throttle_handler(httpd_req_t *req) {
    char query[64];
    char value[16];
    
    if (httpd_req_get_url_query_len(req) + 1 > sizeof(query)) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Query too long");
        return ESP_OK;
    }
    
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        if (httpd_query_key_value(query, "val", value, sizeof(value)) == ESP_OK) {
            int throttle = atoi(value);
            if (throttle >= 1000 && throttle <= 1800) {
                manual_throttle = throttle;
                if(manual_mode) {
                    base_throttle = throttle;
                }
                printf("Web throttle command: %d\n", throttle);
                httpd_resp_sendstr(req, "OK");
                return ESP_OK;
            }
        }
    }
    
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid throttle");
    return ESP_OK;
}

esp_err_t mode_toggle_handler(httpd_req_t *req) {
    manual_mode = !manual_mode;
    
    if(manual_mode) {
        printf("Switched to Manual Mode\n");
        httpd_resp_sendstr(req, "manual");
    } else {
        printf("Switched to PID Mode\n");
        httpd_resp_sendstr(req, "pid");
    }
    
    return ESP_OK;
}

void setup_wifi() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    
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
    
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();
    
    printf("WiFi AP started. SSID: %s, Password: %s\n", WIFI_SSID, WIFI_PASS);
    printf("Connect and go to: http://192.168.4.1/\n");
}

httpd_handle_t setup_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    
    httpd_handle_t server = NULL;
    
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {.uri = "/", .method = HTTP_GET, .handler = root_handler};
        httpd_register_uri_handler(server, &root);
        
        httpd_uri_t arm = {.uri = "/arm", .method = HTTP_GET, .handler = arm_handler};
        httpd_register_uri_handler(server, &arm);
        
        httpd_uri_t disarm = {.uri = "/disarm", .method = HTTP_GET, .handler = disarm_handler};
        httpd_register_uri_handler(server, &disarm);
        
        httpd_uri_t emergency = {.uri = "/emergency", .method = HTTP_GET, .handler = emergency_handler};
        httpd_register_uri_handler(server, &emergency);
        
        httpd_uri_t throttle = {.uri = "/throttle", .method = HTTP_GET, .handler = throttle_handler};
        httpd_register_uri_handler(server, &throttle);
        
        httpd_uri_t mode = {.uri = "/toggle_mode", .method = HTTP_GET, .handler = mode_toggle_handler};
        httpd_register_uri_handler(server, &mode);
        
        printf("Web server started on http://192.168.4.1/\n");
    } else {
        printf("Failed to start web server\n");
    }
    
    return server;
}

void control_loop_task(void *params) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20);
    
    while(1) {
        if(!manual_mode && system_armed) {
            read_mpu6050();
            //read_bmp280();
            calculate_angles();
            mix_motors();
        }
        
        update_motors();
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void status_task(void *params) {
    while(1) {
        if(system_armed) {
            if(manual_mode) {
                printf("ARMED | MANUAL MODE | Throttle: %d\n", manual_throttle);
            } else {
                read_mpu6050();
                calculate_angles();
                printf("ARMED | PID MODE | Roll: %.1f | Pitch: %.1f | Yaw: %.1f\n", 
                       current_roll, current_pitch, current_yaw);
                printf("Motors: [%d,%d,%d,%d]\n", 
                       motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
            }
        } else {
            printf("DISARMED | System Safe\n");
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main() {
    printf("Starting Drone Controller with Web Interface...\n");
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }
    
    roll_pid.kp = 2.0; roll_pid.ki = 0.1; roll_pid.kd = 0.5;
    pitch_pid.kp = 2.0; pitch_pid.ki = 0.1; pitch_pid.kd = 0.5;  
    yaw_pid.kp = 1.5; yaw_pid.ki = 0.05; yaw_pid.kd = 0.3;
    alt_pid.kp = 3.0; alt_pid.ki = 0.2; alt_pid.kd = 1.0;
    
    setup_i2c();
    setup_mpu6050();
   // setup_bmp280();
    setup_motors();
    
    printf("Initializing ESCs...\n");
    for(int i = 0; i < 100; i++) {
        stop_all_motors();
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    printf("ESCs initialized\n");
    
    setup_wifi();
    httpd_handle_t server = setup_webserver();
    
    if(server) {
        printf("=== DRONE CONTROLLER READY ===\n");
        printf("WiFi AP: %s / %s\n", WIFI_SSID, WIFI_PASS);
        printf("Web Interface: http://192.168.4.1/\n");
        printf("===============================\n");
        printf("SAFETY INSTRUCTIONS:\n");
        printf("1. Remove propellers for testing!\n");
        printf("2. System starts DISARMED\n");
        printf("3. Use web to ARM/DISARM\n");
        printf("4. Emergency stop available\n");
        printf("===============================\n");
    }
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    xTaskCreate(control_loop_task, "flight_control", 4096, NULL, 10, NULL);
    xTaskCreate(status_task, "status_task", 2048, NULL, 5, NULL);
    
    printf("System ready! Connect to web interface.\n");
    
    while(1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
