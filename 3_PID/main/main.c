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

// Basic defines
#define I2C_SDA_PIN 21
#define I2C_SCL_PIN 22
#define I2C_FREQ 400000

#define MPU6050_ADDRESS 0x68
#define BMP280_ADDRESS 0x76

#define MOTOR1_PIN 16  // front left
#define MOTOR2_PIN 17  // front right  
#define MOTOR3_PIN 18  // back left
#define MOTOR4_PIN 19  // back right

// Simple PID struct
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

// Sensor data
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float pressure, temperature;

// BMP280 calibration - just the basic ones we need
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// Simple I2C functions
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

// Read multiple bytes from I2C device
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

// Setup MPU6050 sensor
void setup_mpu6050() {
    
    i2c_write_reg(MPU6050_ADDRESS, 0x6B, 0x00);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    
    i2c_write_reg(MPU6050_ADDRESS, 0x19, 0x07);
    
    
    i2c_write_reg(MPU6050_ADDRESS, 0x1B, 0x08);
    
    
    i2c_write_reg(MPU6050_ADDRESS, 0x1C, 0x10);
    
    printf("MPU6050 setup done\n");
}

// Read data from MPU6050
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
/**
// Setup BMP280 sensor
void setup_bmp280() {
    // Check if sensor is there
    uint8_t id;
    i2c_read_reg(BMP280_ADDRESS, 0xD0, &id, 1);
    if(id != 0x58) {
        printf("BMP280 not found!\n");
        return;
    }
    
    // Read calibration data
    uint8_t calib[24];
    i2c_read_reg(BMP280_ADDRESS, 0x88, calib, 24);
    
    // Parse calibration values
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
    
    // Configure sensor
    i2c_write_reg(BMP280_ADDRESS, 0xF4, 0xFF); // temp and pressure oversampling
    i2c_write_reg(BMP280_ADDRESS, 0xF5, 0x00); // config
    
    printf("BMP280 setup done\n");
}

// Read pressure and calculate altitude
void read_bmp280() {
    uint8_t data[6];
    i2c_read_reg(BMP280_ADDRESS, 0xF7, data, 6);
    
    int32_t raw_pressure = ((data[0] << 16) | (data[1] << 8) | data[2]) >> 4;
    int32_t raw_temp = ((data[3] << 16) | (data[4] << 8) | data[5]) >> 4;
    
    // Temperature calculation (simplified)
    int32_t var1 = ((((raw_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((raw_temp >> 4) - ((int32_t)dig_T1)) * ((raw_temp >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    temperature = (t_fine * 5 + 128) >> 8;
    temperature = temperature / 100.0;
    
    // Pressure calculation (basic version)
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
        
        // Simple altitude calculation
        current_altitude = 44330.0 * (1.0 - pow(pressure / 101325.0, 0.1903));
    }
}
*/
// Setup motors with MCPWM
void setup_motors() {
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;
    pwm_config.cmpr_a = 0;
    pwm_config.cmpr_b = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    // Setup each motor
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

// Set individual motor speed
void set_motor(int motor_num, int speed) {
    // Convert 1000-2000 to duty cycle
    float duty = (speed / 20000.0) * 100.0;
    
    switch(motor_num) {
        case 0: mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, duty); break;
        case 1: mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, duty); break;
        case 2: mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, duty); break;
        case 3: mcpwm_set_duty(MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, duty); break;
    }
}

// Calculate angles from accelerometer 
void calculate_angles() {
    // Basic tilt calculation
    current_roll = atan2(accel_y, accel_z) * 180.0 / M_PI;
    current_pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / M_PI;
    
    // Simple complementary filter
    static float last_roll = 0, last_pitch = 0;
    current_roll = 0.98 * (last_roll + gyro_x * 0.01) + 0.02 * current_roll;
    current_pitch = 0.98 * (last_pitch + gyro_y * 0.01) + 0.02 * current_pitch;
    current_yaw += gyro_z * 0.01; // just integrate gyro
    
    last_roll = current_roll;
    last_pitch = current_pitch;
}

// Basic PID calculation
float calculate_pid(struct PID_Controller *pid, float target, float current) {
    float error = target - current;
    
    // Proportional
    float p_term = pid->kp * error;
    
    // Integral  
    pid->integral += error * 0.01; // dt = 0.01s
    float i_term = pid->ki * pid->integral;
    
    // Derivative
    float d_term = pid->kd * (error - pid->previous_error) / 0.01;
    pid->previous_error = error;
    
    pid->output = p_term + i_term + d_term;
    
    // Basic limits
    if(pid->output > 400) pid->output = 400;
    if(pid->output < -400) pid->output = -400;
    
    return pid->output;
}

// Mix PID outputs to motor speeds
void mix_motors() {
    float roll_correction = calculate_pid(&roll_pid, target_roll, current_roll);
    float pitch_correction = calculate_pid(&pitch_pid, target_pitch, current_pitch); 
    float yaw_correction = calculate_pid(&yaw_pid, target_yaw, current_yaw);
    float alt_correction = calculate_pid(&alt_pid, target_altitude, current_altitude);
    
    int total_throttle = base_throttle + (int)alt_correction;
    
    // Basic X-quad mixing
    motor_speeds[0] = total_throttle - roll_correction + pitch_correction - yaw_correction; // FL
    motor_speeds[1] = total_throttle + roll_correction + pitch_correction + yaw_correction; // FR
    motor_speeds[2] = total_throttle - roll_correction - pitch_correction + yaw_correction; // BL
    motor_speeds[3] = total_throttle + roll_correction - pitch_correction - yaw_correction; // BR
    
    // Keep motors in range
    for(int i = 0; i < 4; i++) {
        if(motor_speeds[i] < 1000) motor_speeds[i] = 1000;
        if(motor_speeds[i] > 2000) motor_speeds[i] = 2000;
    }
}

// Update all motors
void update_motors() {
    for(int i = 0; i < 4; i++) {
        set_motor(i, motor_speeds[i]);
    }
}

// Main control loop
void control_loop_task(void *params) {
    while(1) {
        // Read all sensors
        read_mpu6050();
       // read_bmp280();
        
        // Calculate current attitude
        calculate_angles();
        
        // Calculate motor outputs
        mix_motors();
        
        // Send to motors
        update_motors();
        
        // Print some debug info
        static int counter = 0;
        counter++;
        if(counter >= 100) { // every 1 second
            printf("Roll: %.1f, Pitch: %.1f, Yaw: %.1f, Alt: %.1f\n", 
                   current_roll, current_pitch, current_yaw, current_altitude);
            printf("Motors: %d %d %d %d\n", 
                   motor_speeds[0], motor_speeds[1], motor_speeds[2], motor_speeds[3]);
            counter = 0;
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz loop
    }
}

void app_main() {
    printf("Starting drone controller...\n");
    
    // Initialize PID controllers with some basic values
    roll_pid.kp = 2.0; roll_pid.ki = 0.1; roll_pid.kd = 0.5;
    pitch_pid.kp = 2.0; pitch_pid.ki = 0.1; pitch_pid.kd = 0.5;  
    yaw_pid.kp = 1.5; yaw_pid.ki = 0.05; yaw_pid.kd = 0.3;
    alt_pid.kp = 3.0; alt_pid.ki = 0.2; alt_pid.kd = 1.0;
    
    // Setup hardware
    setup_i2c();
    setup_mpu6050();
    setup_bmp280();
    setup_motors();
    
    // Wait a bit for everything to stabilize
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    printf("All systems ready!\n");
    
    // Start control loop
    xTaskCreate(control_loop_task, "control", 4096, NULL, 5, NULL);
}