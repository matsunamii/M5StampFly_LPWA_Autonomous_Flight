/*
 * MIT License
 *
 * Copyright (c) 2024 Kouhei Ito
 * Copyright (c) 2024 M5Stack
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "sensor.hpp"
#include "mag.hpp"
#include "imu.hpp"
#include "tof.hpp"
#include "stampfly.hpp"


Madgwick ahrs;
INA3221 ina3221(INA3221_ADDR40_GND);  // Set I2C address to 0x40 (A0 pin -> GND)

// Sensor data
float ax, ay, az, gx, gy, gz, x, y, z;
float roll_rate_offset, pitch_rate_offset, yaw_rate_offset;
float roll_angle_offset, pitch_angle_offset;
float sensor[16];
volatile float Roll_angle=0.0f, Pitch_angle=0.0f, Yaw_angle=0.0f;
volatile float Roll_rate, Pitch_rate, Yaw_rate;
volatile float Altitude2 = 0.0f;
volatile float Alt_velocity = 0.0f;
uint8_t OverG_flag = 0;
uint16_t rate_offset_counter = 0;
uint16_t angle_offset_counter = 0;

uint8_t scan_i2c() {
    USBSerial.println("I2C scanner. Scanning ...");
    delay(50);
    byte count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        Wire1.beginTransmission(i);        // Begin I2C transmission Address (i)
        if (Wire1.endTransmission() == 0)  // Receive 0 = success (ACK response)
        {
            USBSerial.print("Found address: ");
            USBSerial.print(i, DEC);
            USBSerial.print(" (0x");
            USBSerial.print(i, HEX);
            USBSerial.println(")");
            count++;
        }
    }
    USBSerial.print("Found ");
    USBSerial.print(count, DEC);  // numbers of devices
    USBSerial.println(" device(s).");
    return count;
}

void test_voltage(void) {
    for (uint16_t i = 0; i < 1000; i++) {
        USBSerial.printf("Voltage[%03d]:%f\n\r", i, ina3221.getVoltage(INA3221_CH2));
    }
}

void ahrs_reset(void) {
    ahrs.reset();
}

void sensor_init() {
    Wire1.begin(SDA_PIN, SCL_PIN, 400000UL);
    if (scan_i2c() == 0) {
        USBSerial.printf("No I2C device!\r\n");
        USBSerial.printf("Can not boot AtomFly2.\r\n");
        while (1);
    }
    mag_init();
    tof_init();
    imu_init();
    ina3221.begin(&Wire1);
    ina3221.reset();
    ahrs.begin(400.0);

    uint16_t cnt = 0;
    while (cnt < 10) {
        if (ToF_bottom_data_ready_flag) {
            ToF_bottom_data_ready_flag = 0;
            cnt++;
            USBSerial.printf("%d %d\n\r", cnt, tof_bottom_get_range());
        }
    }
    delay(10);
    USBSerial.printf("Finish sensor init!\r\n");
}

void sensor_calc_offset_rate_avarage(void) {
    roll_rate_offset  = (rate_offset_counter * roll_rate_offset  + gx) / (rate_offset_counter + 1);
    pitch_rate_offset = (rate_offset_counter * pitch_rate_offset + gy) / (rate_offset_counter + 1);
    yaw_rate_offset   = (rate_offset_counter * yaw_rate_offset   + gz) / (rate_offset_counter + 1);
    rate_offset_counter++;
}
void sensor_calc_offset_angle_avarage(void) {
    roll_angle_offset = (angle_offset_counter * roll_angle_offset + Roll_angle) / (angle_offset_counter + 1);
    pitch_angle_offset = (angle_offset_counter * pitch_angle_offset + Pitch_angle) / (angle_offset_counter + 1);
    angle_offset_counter++;
}

void sensor_read(sensor_value_t* data) {
    float acc_x, acc_y, acc_z, roll_rate, pitch_rate, yaw_rate, dir_x, dir_y, dir_z;
    float roll_angle, pitch_angle, yaw_angle;
    uint16_t bottom_tof_range;
    float voltage;
    static float previous_altitude2 = 0.0f;
    static uint16_t debug_print_counter = 0;
    constexpr uint16_t debug_print_interval = 100;

    // 以下では航空工学の座標軸の取り方に従って
    // X軸：前後（前が正）左肩上がりが回転の正
    // Y軸：右左（右が正）頭上げが回転の正
    // Z軸：下上（下が正）右回りが回転の正
    // となる様に軸の変換を施しています
    // BMI270の座標軸の撮り方は
    // X軸：右左（右が正）頭上げが回転の正
    // Y軸：前後（前が正）左肩上がりが回転の正
    // Z軸：上下（上が正）左回りが回転の正

    // Get IMU raw data
    imu_update();  // IMUの値を読む前に必ず実行
    ax  =   imu_get_acc_y();
    ay  =   imu_get_acc_x();
    az  = - imu_get_acc_z();
    gx =   imu_get_gyro_y();
    gy =   imu_get_gyro_x();
    gz = - imu_get_gyro_z();
    mag_update();

    if (StampFly.flag.mode >= AVERAGE_MODE) {
        acc_x = ax;
        acc_y = ay;
        acc_z = az;
        roll_rate  = gx - roll_rate_offset;
        pitch_rate = gy - pitch_rate_offset;
        yaw_rate   = gz - yaw_rate_offset;
        // Align magnetometer axes with transformed accel/gyro axes
        dir_x = imu_get_mag_y();
        dir_y = imu_get_mag_x();
        dir_z = - imu_get_mag_z();
        
        if (mag_available()) {
            ahrs.update( roll_rate  * (float)RAD_TO_DEG, 
                         pitch_rate * (float)RAD_TO_DEG,
                         yaw_rate   * (float)RAD_TO_DEG, 
                         -acc_x, 
                         -acc_y,
                         -acc_z,
                         dir_x,
                         dir_y,
                         dir_z);
        } else {
        ahrs.updateIMU( roll_rate  * (float)RAD_TO_DEG, 
                        pitch_rate * (float)RAD_TO_DEG,
                        yaw_rate   * (float)RAD_TO_DEG, 
                        -acc_x, 
                        -acc_y,
                        -acc_z);
        }

        roll_angle  = ahrs.getRoll()  * (float)DEG_TO_RAD;
        pitch_angle = ahrs.getPitch() * (float)DEG_TO_RAD;
        yaw_angle   = ahrs.getYaw()   * (float)DEG_TO_RAD;

        Roll_angle = roll_angle;
        Pitch_angle = pitch_angle;
        Yaw_angle = yaw_angle;
        Roll_rate = roll_rate;
        Pitch_rate = pitch_rate;
        Yaw_rate = yaw_rate;
    }
    // Battery voltage check
    voltage   = ina3221.getVoltage(INA3221_CH2);

    if (ToF_bottom_data_ready_flag) {
        ToF_bottom_data_ready_flag = 0;
        bottom_tof_range = tof_bottom_get_range();
    } else {
        bottom_tof_range = data->bottom_tof_range;
    }
    Altitude2 = (float)bottom_tof_range * 0.001f;
    if (StampFly.times.interval_time > 0.0f) {
        Alt_velocity = (Altitude2 - previous_altitude2) / StampFly.times.interval_time;
    } else {
        Alt_velocity = 0.0f;
    }
    previous_altitude2 = Altitude2;

    if (acc_x * acc_x + acc_y * acc_y + acc_z * acc_z > 9.0f) {
        OverG_flag = 1;
    }

    // set value
    data->accx = acc_x;
    data->accy = acc_y;
    data->accz = acc_z;
    data->roll_rate = roll_rate;
    data->pitch_rate = pitch_rate;
    data->yaw_rate = yaw_rate;
    data->dir_x = dir_x;
    data->dir_y = dir_y;
    data->dir_z = dir_z;
    data->roll_angel = roll_angle;
    data->pitch_angle = pitch_angle;
    data->yaw_angle = yaw_angle;
    data->voltage = voltage;
    data->bottom_tof_range = bottom_tof_range;
    
    // // Periodic debug output: Roll/Pitch/Yaw (deg), accel XYZ (m/s^2), |acc|, battery
    // if ((debug_print_counter % debug_print_interval) == 0) {
    //     float acc_norm = sqrtf(data->accx * data->accx + data->accy * data->accy + data->accz * data->accz);
    //     USBSerial.printf("R:%.2f P:%.2f Y:%.2f  ax:%.3f ay:%.3f az:%.3f V:%.3f\n\r",
    //         Roll_angle * (float)RAD_TO_DEG,
    //         Pitch_angle * (float)RAD_TO_DEG,
    //         Yaw_angle * (float)RAD_TO_DEG,
    //         data->accx,
    //         data->accy,
    //         data->accz,
    //         data->voltage);
    // }
    debug_print_counter++;

    return;
}

void bottom_tof_read(sensor_value_t* data) {
    uint16_t range = data->bottom_tof_range;
    if (ToF_bottom_data_ready_flag) {
        //dcnt = 0u;
        ToF_bottom_data_ready_flag = 0;
        // 距離の値の更新
        range = tof_bottom_get_range();
        //USBSerial.printf("%04d\n\r", bottom_tof_range);
    }
    data->bottom_tof_range = range;   
}
