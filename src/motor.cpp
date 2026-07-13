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

#include "motor.hpp"
#include <Arduino.h>

// 制御周期
// Control period
extern float Control_period;

void motor_set_duty_fr(float duty) {
    ledcWrite(CH_FRONT_RIGHT, (uint32_t)(255 * duty));
}
void motor_set_duty_fl(float duty) {
    ledcWrite(CH_FRONT_LEFT, (uint32_t)(255 * duty));
}
void motor_set_duty_rr(float duty) {
    ledcWrite(CH_REAR_RIGHT, (uint32_t)(255 * duty));
}
void motor_set_duty_rl(float duty) {
    ledcWrite(CH_REAR_LEFT, (uint32_t)(255 * duty));
}

void motor_init(void) {
    ledcSetup(CH_FRONT_LEFT,  PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_FRONT_RIGHT, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_REAR_LEFT,   PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(CH_REAR_RIGHT,  PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_FRONT_LEFT,  CH_FRONT_LEFT);
    ledcAttachPin(PIN_FRONT_RIGHT, CH_FRONT_RIGHT);
    ledcAttachPin(PIN_REAR_LEFT,   CH_REAR_LEFT);
    ledcAttachPin(PIN_REAR_RIGHT,  CH_REAR_RIGHT);
}

void motor_stop(void) {
    motor_set_duty_fr(0.0);
    motor_set_duty_fl(0.0);
    motor_set_duty_rr(0.0);
    motor_set_duty_rl(0.0);
}