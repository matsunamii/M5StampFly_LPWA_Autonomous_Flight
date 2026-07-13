#ifndef MAG_HPP
#define MAG_HPP
#include <Arduino.h>
#include "bmm150_defs.h"

void mag_init(void);
void mag_update(void);
float imu_get_mag_x(void);
float imu_get_mag_y(void);
float imu_get_mag_z(void);
bool mag_available(void);
// Calibration and debug
void mag_start_calibration(void);
void mag_stop_calibration(void);
void mag_reset_calibration(void);
void mag_print_debug(void);
float mag_get_bias_x(void);
float mag_get_bias_y(void);
float mag_get_bias_z(void);

#endif // MAG_HPP