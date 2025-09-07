**ESP32 Flight Controller**  

A lightweight, modular flight-controller stack for the ESP32. It supports RC receiver inputs, ESC/motor outputs, and is structured to add IMU-based stabilization and navigation features. Built on ESP-IDF with FreeRTOS.
Status: Experimental. Not intended for production or safety-critical use.

**Features** 

1.Receiver Inputs: currently via wifi A/P mode 

2.ESC/Motor Control: PWM generation for up to four motors (quad-class), with arming/disarm guard and min/max pulse clamping.

3.ESP-IDF Native: No Arduino shim, MCPWM/LEDC drivers, and idf.py workflow.

4.Currently only read mpu-6050 values , calibrate and try to make the drone hower 

5.Easy to understand 


**HARDWARE**

1. ESP32 development board (wroom32)
2. ESCs/Motors: 4 × brushless ESCs 30A simonk (standard RC PWM, ~50–400 Hz) and BLDC motors 2200kv.
3. Power: LiPo battery with a proper power distribution board
4. IMU : MPU-6050 (currently) , barometer bmp-280 (soon)

**FIREMWARE SETUP**

1. Install ESP-IDF ( v 5.2 )
2. clone the repository
   ```bash
   
   git clone https://github.com/nackabhi/esp32-fc-.git
   cd esp32-fc-
   
   ```
3. setup env
   ```bash
   . $HOME/esp/esp-idf/export.sh
   idf.py --version
    ```
   
