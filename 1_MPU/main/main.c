#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define I2C_MASTER_NUM         I2C_NUM_0
#define I2C_MASTER_SCL_IO      22
#define I2C_MASTER_SDA_IO      21
#define I2C_MASTER_FREQ_HZ     400000
#define MPU6050_ADDR           0x68
#define I2C_TIMEOUT_MS         1000


float RateRoll = 0, RatePitch = 0, RateYaw = 0;
float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;

static esp_err_t mpu6050_write(uint8_t reg_addr, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t mpu6050_read(uint8_t reg_addr, uint8_t *data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    
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

void gyro_signals() {
    mpu6050_write(0x1A, 0x05);
    mpu6050_write(0x1B, 0x08); 

    uint8_t data[6];
    mpu6050_read(0x43, data, 6);

    int16_t GyroX = (data[0] << 8) | data[1];
    int16_t GyroY = (data[2] << 8) | data[3];
    int16_t GyroZ = (data[4] << 8) | data[5];

    RateRoll  = (float)GyroX / 65.5f;
    RatePitch = (float)GyroY / 65.5f;
    RateYaw   = (float)GyroZ / 65.5f;
}

void init_i2c() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void calibrate_gyro() {
    int samples = 2000;
    RateCalibrationRoll = 0;
    RateCalibrationPitch = 0;
    RateCalibrationYaw = 0;

    for (int i = 0; i < samples; i++) {
        gyro_signals();
        RateCalibrationRoll += RateRoll;
        RateCalibrationPitch += RatePitch;
        RateCalibrationYaw += RateYaw;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    RateCalibrationRoll /= samples;
    RateCalibrationPitch /= samples;
    RateCalibrationYaw /= samples;
}

void app_main() {
    
    init_i2c();
    vTaskDelay(pdMS_TO_TICKS(250));
    mpu6050_write(0x6B, 0x00);

    
    calibrate_gyro();

    while (1) {
        gyro_signals();
        float roll = RateRoll - RateCalibrationRoll;
        float pitch = RatePitch - RateCalibrationPitch;
        float yaw = RateYaw - RateCalibrationYaw;

        printf("Roll rate [°/s]= %.2f, Pitch rate [°/s]= %.2f, Yaw rate [°/s]= %.2f\n",
               roll, pitch, yaw);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
