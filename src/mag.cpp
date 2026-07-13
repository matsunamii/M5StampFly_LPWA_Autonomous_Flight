#include <Arduino.h>
#include "mag.hpp"
#include <Wire.h>
#include "bmm150.h"

static struct bmm150_dev dev;
static uint8_t bmm150_addr = BMM150_DEFAULT_I2C_ADDRESS;

static bool mag_ok = false;

// raw values (uT)
static float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;

static BMM150_INTF_RET_TYPE bmm150_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t addr = *(uint8_t*)intf_ptr;
    
    Wire1.beginTransmission(addr);
    Wire1.write(reg_addr);
    if (Wire1.endTransmission(false) != 0) {
        return BMM150_E_COM_FAIL; // error
    }
    uint32_t got = Wire1.requestFrom(addr, (uint8_t)len);
    if (got ==0) return BMM150_E_COM_FAIL;

    uint32_t i = 0;
    unsigned long start = micros();
    while (i < len){
        if (Wire1.available()){
            reg_data[i++] = Wire1.read();
        }else {
            if ((micros() - start) > 1000UL * len) break;
        }
    }
    return (i == len) ? BMM150_INTF_RET_SUCCESS : BMM150_E_COM_FAIL;
}

static BMM150_INTF_RET_TYPE bmm150_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    if (intf_ptr == NULL) return BMM150_E_NULL_PTR;
    uint8_t addr = *(uint8_t*)intf_ptr;
    if (len == 0) return BMM150_INTF_RET_SUCCESS;
    
    Wire1.beginTransmission(addr);
    Wire1.write(reg_addr);
    Wire1.write(reg_data, len);
    if (Wire1.endTransmission() != 0) {
        return BMM150_E_COM_FAIL; // error
    }
    return BMM150_INTF_RET_SUCCESS;
}

static void bmm150_delay_us(uint32_t period, void *intf_ptr)
{
    delayMicroseconds(period);
}

void mag_init(void)
{
    dev.intf = BMM150_I2C_INTF;
    dev.intf_ptr = &bmm150_addr;
    dev.read = bmm150_i2c_read;
    dev.write = bmm150_i2c_write;
    dev.delay_us = bmm150_delay_us;

    int8_t rslt = bmm150_init(&dev);
    if (rslt == BMM150_OK)
    {
        struct bmm150_settings settings;
        settings.pwr_mode = BMM150_POWERMODE_NORMAL;
        bmm150_set_op_mode(&settings, &dev);
        settings.preset_mode = BMM150_PRESETMODE_REGULAR;
        bmm150_set_presetmode(&settings, &dev);
        mag_ok = true;
    }
    else
    {
        mag_ok = false;
    }
}

void mag_update(void)
{
    struct bmm150_mag_data magdata;
    int8_t rslt = bmm150_read_mag_data(&magdata, &dev);
    if (rslt == BMM150_OK) {
        mag_x = (float)magdata.x;
        mag_y = (float)magdata.y;
        mag_z = (float)magdata.z;
        mag_ok = true;
    } else {
        mag_ok = false;
        // try a single re-init to recover transient I2C errors
        int8_t r2 = bmm150_init(&dev);
        if (r2 == BMM150_OK) {
            mag_ok = true;
        }
    }
}

float imu_get_mag_x(void)
{
    return mag_x;
}

float imu_get_mag_y(void)
{
    return mag_y;
}

float imu_get_mag_z(void)
{
    return mag_z;
}

bool mag_available(void)
{
    return mag_ok;
}

void mag_test(void) {
    if (mag_available()) {
        mag_update();
        Serial.print("Mag: ");
        Serial.print(imu_get_mag_x());
        Serial.print(", ");
        Serial.print(imu_get_mag_y());
        Serial.print(", ");
        Serial.println(imu_get_mag_z());
    } else {
        Serial.println("Mag not available");
    }
}
