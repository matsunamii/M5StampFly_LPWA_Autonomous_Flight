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

//
// StampFly Flight Control Main Module
//
// Desigend by Kouhei Ito 2023~2024
//
// 2024-08-11 StampFly 自己開発用のスケルトンプログラム制作開始

#include "main_loop.hpp"
#include "motor.hpp"
#include "rc.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "led.hpp"
#include "telemetry.hpp"
#include "button.hpp"
#include "buzzer.h"
#include "alt_kalman.hpp"
#include "stampfly.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


volatile uint8_t armButtonState = 0;
volatile uint8_t armButtonPressedAndRerleased = 0;
volatile uint8_t previousArmButtonState = 0; 
//Times
volatile float Elapsed_time=0.0f;
volatile float Old_Elapsed_time=0.0f;
volatile float Interval_time=0.0f;
volatile uint32_t S_time=0,E_time=0,D_time=0,Dt_time=0;
float Control_period = 0.0025f; // 400Hz

//Machine state & flag
uint8_t Control_mode = ANGLECONTROL;
float Motor_on_duty_threshold = 0.1f;

//Motor Duty 
volatile float FrontRight_motor_duty=0.0f;
volatile float FrontLeft_motor_duty=0.0f;
volatile float RearRight_motor_duty=0.0f;
volatile float RearLeft_motor_duty=0.0f;

//PID Gain
//Rate control PID gain
const float Roll_rate_kp = 0.6f;
const float Roll_rate_ti = 0.7f;
const float Roll_rate_td = 0.01;
const float Roll_rate_eta = 0.125f;

const float Pitch_rate_kp = 0.75f;
const float Pitch_rate_ti = 0.7f;
const float Pitch_rate_td = 0.025f;
const float Pitch_rate_eta = 0.125f;

const float Yaw_rate_kp = 3.0f;
const float Yaw_rate_ti = 0.8f;
const float Yaw_rate_td = 0.01f;
const float Yaw_rate_eta = 0.125f;

//Angle control PID gain
const float Rall_angle_kp = 8.0f;
const float Rall_angle_ti = 4.0f;
const float Rall_angle_td = 0.04f;
const float Rall_angle_eta = 0.125f;

const float Pitch_angle_kp = 8.0f;
const float Pitch_angle_ti = 4.0f;
const float Pitch_angle_td = 0.04f;
const float Pitch_angle_eta = 0.125f;

//Altitude control PID gain
const float alt_kp = 0.65f;
const float alt_ti = 200.0f;
const float alt_td = 0.0f;
const float alt_eta = 0.125f;
const float alt_period = 0.0333;
static uint8_t alt_counter = 0;
static uint8_t z_dot_counter = 0;

const float Thrust0_nominal = 0.63;
const float z_dot_kp = 0.15f;
const float z_dot_ti = 13.5f;
const float z_dot_td = 0.005f;
const float z_dot_eta = 0.125f;

//Counter
uint8_t AngleControlCounter=0;
uint16_t RateControlCounter=0;
uint16_t OffsetCounter=0;

//制御目標
//PID Control reference
//角速度目標値
//Rate reference
volatile float Roll_rate_reference=0.0f, Pitch_rate_reference=0.0f, Yaw_rate_reference=0.0f;
//角度目標値
//Angle reference
volatile float Roll_angle_reference=0.0f, Pitch_angle_reference=0.0f, Yaw_angle_reference=0.0f;
//舵角指令値
//Commanad
//スロットル指令値
//Throttle
volatile float Thrust_command=0.0f, Thrust_command2 = 0.0f;
//角速度指令値
//Rate command
volatile float Roll_rate_command=0.0f, Pitch_rate_command=0.0f, Yaw_rate_command=0.0f;
//角度指令値
//Angle comannd
volatile float Roll_angle_command=0.0f, Pitch_angle_command=0.0f, Yaw_angle_command=0.0f;

//Offset
volatile float Elevator_center=0.0f, Aileron_center=0.0f, Rudder_center=0.0f;

int counter_loop = 0;

//PID object and etc.
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
//PID alt;
PID alt_pid;
PID z_dot_pid;
Alt_kalman alt_kalman;
Filter Thrust_filtered;
Filter Duty_fr;
Filter Duty_fl;
Filter Duty_rr;
Filter Duty_rl;

volatile float Altitude_kf = 0.0f;
volatile float Alt_velocity_kf = 0.0f;

volatile float Thrust0=0.0;
uint8_t Alt_flag = 0;
float Alt_max = 0.5;

//速度目標Z
float Z_dot_ref = 0.0f;
static uint16_t takeoff_count = 0;

//高度目標
const float Alt_ref_min = 0.3;
volatile float Alt_ref = 0.5;

void IRAM_ATTR onTimer(void);
void init_copter(void);
void update_loop400Hz(void);
void init_mode(void);
void average_mode(void);
void flight_mode(void);
void parking_mode(void);
void loop_400Hz(void);
float limit(float value, float min, float max);
void angle_control(void);
void rate_control(void);
void reset_rate_control(void);
void reset_angle_control(void);
void control_init();
void get_command(void);

// Main loop
void loop_400Hz(void) {
    // 400Hzで以降のコードが実行

    update_loop400Hz();
    
    // Mode select
    if (StampFly.flag.mode == INIT_MODE) 
        init_mode();
    else if (StampFly.flag.mode == AVERAGE_MODE)
        average_mode();
    else if (StampFly.flag.mode == FLIGHT_MODE)
        flight_mode();
    else if (StampFly.flag.mode == PARKING_MODE)
        parking_mode();

    //// Telemetry
    telemetry();
    StampFly.flag.oldmode = StampFly.flag.mode;  // Memory now mode
    
    // End of Loop_400Hz function    
}

// 割り込み関数
// Intrupt function
hw_timer_t* timer = NULL;
void IRAM_ATTR onTimer(void) {
    StampFly.flag.loop = 1;
}

// Initialize StampFly
void init_copter(void) {
    //disableCore1WDT();
    // Initialize Mode
    StampFly.flag.mode = INIT_MODE;
    StampFly.flag.loop = 0;
    // Initialaze LED function
    led_init();
    // Initialize Serial communication
    USBSerial.begin(115200);
    USBSerial.setTxTimeoutMs(0);
    delay(1500);
    USBSerial.printf("Start StampFly! Skeleton\r\n");
    motor_init();
    sensor_init();
    rc_init();
    //PID GAIN and etc. Init
    control_init();

    // init button G0
    init_button();
    setup_pwm_buzzer();
    USBSerial.printf("Finish StampFly init!\r\n");
    start_tone();

    // 割り込み設定
    // Initialize intrupt
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2500, true);
    timerAlarmEnable(timer);
}

//loop400Hzの更新関数
void update_loop400Hz(void) {
    uint32_t now_time;

    //タイマ割り込みを待機 400Hzでループするために、割り込みフラグが立つのを待つ
    while (StampFly.flag.loop == 0);
    StampFly.flag.loop = 0;

    #if 0
    USBSerial.printf("%9.4f %9.4f %04d\n\r", 
        StampFly.times.elapsed_time, 
        StampFly.times.interval_time,
        StampFly.sensor.bottom_tof_range);
    #endif

    //Clock
    now_time = micros();
    StampFly.times.old_elapsed_time = StampFly.times.elapsed_time;
    StampFly.times.elapsed_time = 1e-6 * (now_time - StampFly.times.start_time);
    StampFly.times.interval_time = StampFly.times.elapsed_time - StampFly.times.old_elapsed_time;
    Interval_time = StampFly.times.interval_time;
    
    // Read Sensor Value
    sensor_read(&StampFly.sensor);
    bottom_tof_read(&StampFly.sensor);
    
    // Update Altitude Kalman Filter (融合: TOF + 加速度)
    alt_kalman.update(Altitude2, 0.0f, Interval_time);
    Altitude_kf = alt_kalman.Altitude;   // Kalman推定高度
    Alt_velocity_kf = alt_kalman.Velocity; // Kalman推定速度
    
    // LED Drive
    led_drive();

    // Read Button Value
    armButtonState = Stick[BUTTON_ARM];
    if (armButtonState != previousArmButtonState) {
        if (armButtonState == 0) {
            armButtonPressedAndRerleased = 1;
        }
        previousArmButtonState = armButtonState;
    }
    // Centralize RC ARM button handling: on falling edge toggle modes
    if (armButtonPressedAndRerleased) {
        armButtonPressedAndRerleased = 0;
        if (StampFly.flag.mode == FLIGHT_MODE) {
            motor_stop();
            takeoff_count = 0;
            StampFly.flag.mode = PARKING_MODE;
            USBSerial.printf("RC: switch to PARKING_MODE\n\r");
        } else {
            motor_stop();
            reset_angle_control();
            reset_rate_control();
            Roll_angle_command = 0.0f;
            Pitch_angle_command = 0.0f;
            Alt_ref = 0.50f;
            Alt_flag = 0;
            Thrust0 = 0.71f;
            Thrust_command = Thrust0 * BATTERY_VOLTAGE;
            alt_kalman = Alt_kalman();  // Kalmanフィルターをリセット
            takeoff_count = 0;
            StampFly.flag.mode = FLIGHT_MODE;
            StampFly.times.start_time = micros();
            counter_loop = 0;
            USBSerial.printf("RC: switch to FLIGHT_MODE\n\r");
        }
    }
}

void init_mode(void) {
    motor_stop();
    StampFly.counter.offset = 0;
    //Mode change
    StampFly.flag.mode = AVERAGE_MODE;
    return;

}

void average_mode(void) {
    // Gyro offset Estimate 角速度のオフセットを取得
    // Set LED Color
    onboard_led1(PERPLE, 1);
    onboard_led2(PERPLE, 1);

    if (StampFly.counter.offset < AVERAGENUM) {
        // 前半：角速度オフセットを平均
        sensor_calc_offset_rate_avarage();
        StampFly.counter.offset++;
        return;
    }

    if (StampFly.counter.offset < 2 * AVERAGENUM) {
        // 後半：角度オフセットを平均
        sensor_calc_offset_angle_avarage();
        StampFly.counter.offset++;
        return;
    }
    // Mode change
    StampFly.flag.mode   = PARKING_MODE;
    StampFly.times.start_time = micros();
    StampFly.times.old_elapsed_time = 0.0f;
    return;
}

void flight_mode(void) {
    //飛行するためのコードを以下に記述する
    // Set LED Color

    onboard_led1(PERPLE, 1);
    onboard_led2(PERPLE, 1);
    // Get Command from RC
    get_command();
    //Angle Control
    angle_control();

    //Rate Control
    rate_control();
}

void get_command(void)
{
  float th;
  float takeoff_throttle;

  const float target_altitude = 0.50f;
  const float takeoff_throttle_min = 0.30f;
  const float takeoff_throttle_max = 0.72f;
  const float ramp_step = 0.0005f;

  Control_mode = ANGLECONTROL;

  Roll_angle_command  = Aileron_center;
  Pitch_angle_command = Elevator_center;
  Yaw_angle_command   = Rudder_center;
  Yaw_rate_reference  = 0.0f;

  Alt_ref = target_altitude;

  if (Alt_flag == 0)
  {
    takeoff_throttle = takeoff_throttle_min + ramp_step * takeoff_count;

    if (takeoff_throttle > takeoff_throttle_max)
    {
      takeoff_throttle = takeoff_throttle_max;
    }

    th = takeoff_throttle * BATTERY_VOLTAGE;
    Thrust_command = Thrust_filtered.update(th, Interval_time);

    Thrust0 = Thrust_command / BATTERY_VOLTAGE;

    alt_pid.reset();
    z_dot_pid.reset();
    
    if (Altitude2 >= Alt_ref)
    {
      Alt_flag = 1;
      Z_dot_ref = 0.0f;

      alt_pid.reset();
      z_dot_pid.reset();
    }
    else
    {
      takeoff_count++;
    }
  }
  else
  {
    // 高度保持中は推力を上書きしない
  }
}

void parking_mode(void) {
    //着陸している時に行う処理を記述する
    // Set LED Color
    onboard_led1(GREEN, 1);
    onboard_led2(GREEN, 1);

    // Call motor_stop() only when entering PARKING_MODE to avoid repeated calls/log spam
    if (StampFly.flag.oldmode != PARKING_MODE) {
        motor_stop();
        // Capture current attitude as level reference while the craft is on the ground.
        Alt_flag = 0;
        Thrust0 = 0.0f;
        Z_dot_ref = 0.0f;
        Alt_ref = Alt_ref_min;
        alt_pid.reset();
        z_dot_pid.reset();
        Thrust_filtered.reset();
        alt_kalman = Alt_kalman();  // Kalmanフィルターをリセット
    }
    // Mode change on ARM is handled centrally in update_loop400Hz()
}

float limit(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void control_init(void)
{
  //Rate control
  p_pid.set_parameter(Roll_rate_kp, Roll_rate_ti, Roll_rate_td, Roll_rate_eta, Control_period);//Roll rate control gain
  q_pid.set_parameter(Pitch_rate_kp, Pitch_rate_ti, Pitch_rate_td, Pitch_rate_eta, Control_period);//Pitch rate control gain
  r_pid.set_parameter(Yaw_rate_kp, Yaw_rate_ti, Yaw_rate_td, Yaw_rate_eta, Control_period);//Yaw rate control gain

  //Angle control
  phi_pid.set_parameter(Rall_angle_kp, Rall_angle_ti, Rall_angle_td, Rall_angle_eta, Control_period);//Roll angle control gain
  theta_pid.set_parameter(Pitch_angle_kp, Pitch_angle_ti, Pitch_angle_td, Pitch_angle_eta, Control_period);//Pitch angle control gain

  //Altitude control
  alt_pid.set_parameter(alt_kp, alt_ti, alt_td, alt_eta, alt_period);
  z_dot_pid.set_parameter(z_dot_kp, z_dot_ti, z_dot_td, alt_eta, alt_period);

  Duty_fl.set_parameter(0.003, Control_period);
  Duty_fr.set_parameter(0.003, Control_period);
  Duty_rl.set_parameter(0.003, Control_period);
  Duty_rr.set_parameter(0.003, Control_period);

}

void reset_angle_control(void)
{
    Roll_rate_reference=0.0f;
    Pitch_rate_reference=0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
    /////////////////////////////////////
    // 以下の処理で、角度制御が有効になった時に
    // 急激な目標値が発生して機体が不安定になるのを防止する
    // When no external controller is used, keep centers at zero
    Aileron_center  = 0.0f;
    Elevator_center = 0.0f;
    /////////////////////////////////////
}

void reset_rate_control(void)
{
    motor_stop();
    FrontRight_motor_duty = 0.0;
    FrontLeft_motor_duty = 0.0;
    RearRight_motor_duty = 0.0;
    RearLeft_motor_duty = 0.0;
    Duty_fr.reset();
    Duty_fl.reset();
    Duty_rr.reset();
    Duty_rl.reset();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    alt_pid.reset();
    z_dot_pid.reset();
    Roll_rate_reference = 0.0f;
    Pitch_rate_reference = 0.0f;
    Yaw_rate_reference = 0.0f;
    Rudder_center   = Yaw_angle_command;
    //angle control value reset
    Roll_rate_reference=0.0f;
    Pitch_rate_reference=0.0f;
    phi_pid.reset();
    theta_pid.reset();
    phi_pid.set_error(Roll_angle_reference);
    theta_pid.set_error(Pitch_angle_reference);
}



void rate_control(void)
{
    float p_rate, q_rate, r_rate;// p=Roll軸角速度、q=Pitch、r=Yaw
    float p_ref, q_ref, r_ref;//角速度の目標値（参照値）を格納
    float p_err, q_err, r_err, z_dot_err;//各軸の誤差（目標−実測）と、上昇速度（Z_dot）の誤差を格納

  //Control main
  if(Thrust_command/BATTERY_VOLTAGE < Motor_on_duty_threshold)
  {
    reset_rate_control();
  }
  else
  {
    //Control angle velocity
    p_rate = Roll_rate;//Roll軸角速度
    q_rate = Pitch_rate;//Pitch軸角速度
    r_rate = Yaw_rate;//Yaw軸角速度

    //Get reference
    p_ref = Roll_rate_reference;//Roll軸角速度の目標値
    q_ref = Pitch_rate_reference;//Pitch軸角速度の目標値
    r_ref = Yaw_rate_reference;//Yaw軸角速度の目標値

    //Error
    p_err = p_ref - p_rate;//Roll軸角速度の誤差
    q_err = q_ref - q_rate;//Pitch軸角速度の誤差
    r_err = r_ref - r_rate;//Yaw軸角速度の誤差
    z_dot_err = Z_dot_ref - Alt_velocity_kf;//上昇速度の誤差

    //Rate Control PID
    Roll_rate_command = p_pid.update(p_err, Interval_time);// Roll軸角速度の指令値を更新
    Pitch_rate_command = q_pid.update(q_err, Interval_time);// Pitch軸角速度の指令値を更新
    Yaw_rate_command = r_pid.update(r_err, Interval_time);// Yaw軸角速度の指令値を更新

    // (debug injection removed)
    if (Alt_flag == 1)
      {
        if (z_dot_counter++ >= 13)  // 30Hz: 400Hz / 30Hz ≈ 13
        {
          z_dot_counter = 0;
          float z_dot_out = z_dot_pid.update(z_dot_err, alt_period);
          float thrust0_cmd = Thrust0 + z_dot_out;
          if (thrust0_cmd < 0.30f) thrust0_cmd = 0.30f;
          if (thrust0_cmd > 0.80f) thrust0_cmd = 0.80f;
          Thrust_command = thrust0_cmd * BATTERY_VOLTAGE;
        }
      }

    //Motor Control
    //正規化Duty
    float preFR = (Thrust_command +(-Roll_rate_command +Pitch_rate_command +Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE;
    float preFL = (Thrust_command +( Roll_rate_command +Pitch_rate_command -Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE;
    float preRR = (Thrust_command +(-Roll_rate_command -Pitch_rate_command -Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE;
    float preRL = (Thrust_command +( Roll_rate_command -Pitch_rate_command +Yaw_rate_command)*0.25f)/BATTERY_VOLTAGE;

    FrontRight_motor_duty = Duty_fr.update(preFR, Interval_time);
    FrontLeft_motor_duty  = Duty_fl.update(preFL, Interval_time);
    RearRight_motor_duty  = Duty_rr.update(preRR, Interval_time);
    RearLeft_motor_duty   = Duty_rl.update(preRL, Interval_time);

    const float minimum_duty=0.0f;
    const float maximum_duty=0.95f;

    if (FrontRight_motor_duty < minimum_duty) FrontRight_motor_duty = minimum_duty;
    if (FrontRight_motor_duty > maximum_duty) FrontRight_motor_duty = maximum_duty;

    if (FrontLeft_motor_duty < minimum_duty) FrontLeft_motor_duty = minimum_duty;
    if (FrontLeft_motor_duty > maximum_duty) FrontLeft_motor_duty = maximum_duty;

    if (RearRight_motor_duty < minimum_duty) RearRight_motor_duty = minimum_duty;
    if (RearRight_motor_duty > maximum_duty) RearRight_motor_duty = maximum_duty;

    if (RearLeft_motor_duty < minimum_duty) RearLeft_motor_duty = minimum_duty;
    if (RearLeft_motor_duty > maximum_duty) RearLeft_motor_duty = maximum_duty;

        //Duty set
        if (OverG_flag==0){
            motor_set_duty_fr(FrontRight_motor_duty);
            motor_set_duty_fl(FrontLeft_motor_duty);
            motor_set_duty_rr(RearRight_motor_duty);
            motor_set_duty_rl(RearLeft_motor_duty);      
        }
        else 
        {
            FrontRight_motor_duty = 0.0;
            FrontLeft_motor_duty = 0.0;
            RearRight_motor_duty = 0.0;
            RearLeft_motor_duty = 0.0;
            motor_stop();
            OverG_flag=0;
            StampFly.flag.mode = PARKING_MODE;
        }
  }
}

void angle_control(void)
{
  //ロール角誤差,ピッチ角誤差,高度誤差
  float phi_err, theta_err, alt_err;

  if (Control_mode == RATECONTROL) return;

  //PID Control
  if ((Thrust_command/BATTERY_VOLTAGE < Motor_on_duty_threshold))//Angle_control_on_duty_threshold))
  {
    //Initialize
    reset_angle_control();
  }
  else
  {
    //Get Roll and Pitch angle ref 
    Roll_angle_reference  = 0.5f * PI * (Roll_angle_command - Aileron_center);
    Pitch_angle_reference = 0.5f * PI * (Pitch_angle_command - Elevator_center);
    if (Roll_angle_reference > (30.0f*PI/180.0f) ) Roll_angle_reference = 30.0f*PI/180.0f;
    if (Roll_angle_reference <-(30.0f*PI/180.0f) ) Roll_angle_reference =-30.0f*PI/180.0f;
    if (Pitch_angle_reference > (30.0f*PI/180.0f) ) Pitch_angle_reference = 30.0f*PI/180.0f;
    if (Pitch_angle_reference <-(30.0f*PI/180.0f) ) Pitch_angle_reference =-30.0f*PI/180.0f;

    //Error
    phi_err   = Roll_angle_reference  - (Roll_angle  - roll_angle_offset );
    theta_err = Pitch_angle_reference - (Pitch_angle - pitch_angle_offset);
    alt_err = Alt_ref - Altitude_kf;

    //Altitude COntrol PID
    Roll_rate_reference = phi_pid.update(phi_err, Interval_time);// Roll軸角速度の目標値を更新
    Pitch_rate_reference = theta_pid.update(theta_err, Interval_time);// Pitch軸角速度の目標値を更新

    if(Alt_flag==1)
    {
        if (alt_counter++ >= 13){
            alt_counter = 0;
            Z_dot_ref = alt_pid.update(alt_err, alt_period);// 上昇速度の目標値を更新 (30Hz)
        }
    }
  }
}