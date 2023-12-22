/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_MotorsTailsitter.cpp - ArduCopter motors library for tailsitters and bicopters
 *
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsTailsitter.h"
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // 这里使用了APM分配的通道名称
    // 该函数用于下面判断是否需要修正增益
    _has_diff_thrust = SRV_Channels::function_assigned(SRV_Channel::k_motor1) || SRV_Channels::function_assigned(SRV_Channel::k_motor2) || SRV_Channels::function_assigned(SRV_Channel::k_motor3) || SRV_Channels::function_assigned(SRV_Channel::k_motor4);

    // 设置电机默认通道
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_1);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_2);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor3, CH_3);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor4, CH_4);

    // 设置舵机默认通道
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor5, CH_5);
    
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor6, CH_6);
  
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor7, CH_7);
   
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor8, CH_8);
   
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor9, CH_9);
 
    SRV_Channels::set_output_min_max(SRV_Channel::k_motor5, 1206, 1834);//为控制舵机限位相同并考虑中立位偏置，在这里修改范围

    SRV_Channels::set_output_min_max(SRV_Channel::k_motor6, 600, 1820);

    SRV_Channels::set_output_min_max(SRV_Channel::k_motor7, 1196, 1824);

    SRV_Channels::set_output_min_max(SRV_Channel::k_motor8, 860, 2080);

    SRV_Channels::set_output_min_max(SRV_Channel::k_motor9, 986, 2014);

    _mav_type = MAV_TYPE_VTOL_TILTROTOR;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_TAILSITTER);
}


/// Constructor
AP_MotorsTailsitter::AP_MotorsTailsitter(uint16_t speed_hz) :
    AP_MotorsMulticopter(speed_hz)
{
    set_update_rate(speed_hz);
}


// set update rate to motors - a value in hertz
void AP_MotorsTailsitter::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    // 设置电机响应速度
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor1, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor2, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor3, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_motor4, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }
    // 先判断摇杆位置对应的输出
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0] = 0.0f;
            _actuator[1] = 0.0f;
            _actuator[2] = 0.0f;
            _actuator[3] = 0.0f;
            // 舵机的输出 0..1
            _tilt[0] = 0.0f;
            _tilt[1] = 0.0f;
            _tilt[2] = 0.0f;
            _tilt[3] = 0.0f;
            _tilt[4] = 0.0f;

            _external_min_throttle = 0.0;
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[3], actuator_spin_up_to_ground_idle());
            // 地面状态舵机输出为零
            _tilt[0] = 0.0f;
            _tilt[1] = 0.0f;
            _tilt[2] = 0.0f;
            _tilt[3] = 0.0f;
            _tilt[4] = 0.0f;

            _external_min_throttle = 0.0;
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            set_actuator_with_slew(_actuator[0], thrust_to_actuator(_thrust[0]));
            set_actuator_with_slew(_actuator[1], thrust_to_actuator(_thrust[1]));
            set_actuator_with_slew(_actuator[2], thrust_to_actuator(_thrust[2]));
            set_actuator_with_slew(_actuator[3], thrust_to_actuator(_thrust[3]));
            break;
    }

    // 电机输出
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, output_to_pwm(_actuator[1]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, output_to_pwm(_actuator[2]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, output_to_pwm(_actuator[3]));


    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _actuator[2]*100);

    // 舵机输出限制后的值
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor5, output_to_tilt1(_tilt[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor6, output_to_tilt2(_tilt[1]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor7, output_to_tilt3(_tilt[2]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor8, output_to_tilt4(_tilt[3]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor9, output_to_tilt5(_tilt[4]));

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
// 未使用
uint32_t AP_MotorsTailsitter::get_motor_mask()
{
    uint32_t motor_mask = 0;
    uint8_t chan;
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleLeft, chan)) {
        motor_mask |= 1U << chan;
    }
    if (SRV_Channels::find_channel(SRV_Channel::k_throttleRight, chan)) {
        motor_mask |= 1U << chan;
    }

    // add parent's mask
    motor_mask |= AP_MotorsMulticopter::get_motor_mask();

    return motor_mask;
}

//控制舵机角度，同时进行中立位微调
int16_t AP_MotorsTailsitter::output_to_tilt1(float actuator)
{
    float pwm_output;
    if (AP_Motors::_spool_state == AP_Motors::SpoolState::SHUT_DOWN)
    {
        // in shutdown mode, use PWM 0 or minimum PWM
        pwm_output = 1520;
    }
    else
    {
        // in all other spool modes, covert to desired PWM
        pwm_output = 1520 + 1000 * actuator;
    }

    return pwm_output;
}

int16_t AP_MotorsTailsitter::output_to_tilt2(float actuator)
{
    float pwm_output;
    if (AP_Motors::_spool_state == AP_Motors::SpoolState::SHUT_DOWN)
    {
        // in shutdown mode, use PWM 0 or minimum PWM
        pwm_output = 1210;
    }
    else
    {
        // in all other spool modes, covert to desired PWM
        pwm_output = 1210 + 1000 * actuator;
    }

    return pwm_output;
}

int16_t AP_MotorsTailsitter::output_to_tilt3(float actuator)
{
    float pwm_output;
    if (AP_Motors::_spool_state == AP_Motors::SpoolState::SHUT_DOWN)
    {
        // in shutdown mode, use PWM 0 or minimum PWM
        pwm_output = 1510;
    }
    else
    {
        // in all other spool modes, covert to desired PWM
        pwm_output = 1510 + 1000 * actuator;
    }

    return pwm_output;
}

int16_t AP_MotorsTailsitter::output_to_tilt4(float actuator)
{
    float pwm_output;
    if (AP_Motors::_spool_state == AP_Motors::SpoolState::SHUT_DOWN)
    {
        // in shutdown mode, use PWM 0 or minimum PWM
        pwm_output = 1470;
    }
    else
    {
        // in all other spool modes, covert to desired PWM
        pwm_output = 1470 + 1000 * actuator;
    }

    return pwm_output;
}

int16_t AP_MotorsTailsitter::output_to_tilt5(float actuator)
{
    float pwm_output;
    if (AP_Motors::_spool_state == AP_Motors::SpoolState::SHUT_DOWN)
    {
        // in shutdown mode, use PWM 0 or minimum PWM
        pwm_output = 1500;
    }
    else
    {
        // in all other spool modes, covert to desired PWM
        pwm_output = 1500 + 1000 * actuator;
    }

    return pwm_output;
}

#define fx (force_body[0])
#define fy (force_body[1])
#define fz (force_body[2])
#define mx roll_thrust
#define my pitch_thrust
#define mz yaw_thrust
#include <stdio.h>
#include <stdarg.h>
#include <GCS_MAVLink/GCS.h>

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    uint8_t i;

    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   thrust_max;                 // highest motor value
    float   thrust_min;                 // lowest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy
    float   forward_thrust;             // 纵向推力输入, +/- 1.0
    float   lateral_thrust;             // 侧向推力输入, +/- 1.0

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = _pitch_in + _pitch_in_ff;
    yaw_thrust = _yaw_in + _yaw_in_ff;
    throttle_thrust = get_throttle() * compensation_gain;
    forward_thrust = get_forward() * throttle_thrust;// 将控制器输出的力分解为纵向和横向分力
    lateral_thrust = get_lateral() * throttle_thrust;
    gcs().send_text(MAV_SEVERITY_INFO,"%0.2f",forward_thrust);

    const float max_boost_throttle = _throttle_avg_max * compensation_gain;

    // never boost above max, derived from throttle mix params
    const float min_throttle_out = MIN(_external_min_throttle, max_boost_throttle);
    const float max_throttle_out = _throttle_thrust_max * compensation_gain;

    // sanity check throttle is above min and below current limited throttle
    if (throttle_thrust <= min_throttle_out) {
        throttle_thrust = min_throttle_out;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= max_throttle_out) {
        throttle_thrust = max_throttle_out;
        limit.throttle_upper = true;
    }

    if (roll_thrust >= 1.0) {
        // cannot split motor outputs by more than 1
        roll_thrust = 1;
        limit.roll = true;
    }

    /*
    extern float aim_pitch_deg;
    extern float aim_roll_deg;
    float sinrad = sinf(radians(aim_pitch_deg));
    float cosrad = cosf(radians(aim_pitch_deg));
    */
    extern float ahrs_pitch_deg;
    extern float ahrs_roll_deg;
    

    Quaternion quat;
    quat.from_euler(radians(-ahrs_roll_deg), radians(-ahrs_pitch_deg), 0.0f);
    
    Vector3f force_nav{forward_thrust, lateral_thrust, -throttle_thrust};
    Vector3f force_body = quat * force_nav;

    float Ct = 0.5f;
    float Cm = 0.005f;
    float L = 1.0f;
    

    // 线性化控制分配
    /*
    _virtual_thrust[0] = (1.414f*C_factor*mz)/(8*mz_factor) - my*(Cm*Cm*Cm/my_factor - (Cm*mx_factor)/(2*my_factor)) - fx/(4*Ct) - mx*(Cm*Cm*Cm/my_factor - (Cm*mx_factor)/(2*my_factor));
    _virtual_thrust[1] = - fy/(4*Ct) - (1.414f*C_factor*mz)/(8*mz_factor);
    _virtual_thrust[2] = my*((1.414f*C_factor*mx_factor)/(4*my_factor) + (1.414f*Cm*Cm*C_factor)/(2*my_factor)) - mx*((1.414f*C_factor*mx_factor)/(4*my_factor) + (1.414f*Cm*Cm*C_factor)/(2*my_factor)) - fz/(4*Ct) + (Cm*mz)/(4*mz_factor);
    _virtual_thrust[3] = - mx*(Cm*Cm*Cm/my_factor - (Cm*mx_factor)/(2*my_factor)) - my*(Cm*Cm*Cm/my_factor - (Cm*mx_factor)/(2*my_factor)) - fx/(4*Ct) - (1.414f*C_factor*mz)/(8*mz_factor);
    _virtual_thrust[4] = (1.414f*C_factor*mz)/(8*mz_factor) - fy/(4*Ct);
    _virtual_thrust[5] = mx*((1.414f*C_factor*mx_factor)/(4*my_factor) + (1.414f*Cm*Cm*C_factor)/(2*my_factor)) - fz/(4*Ct) - my*((1.414f*C_factor*mx_factor)/(4*my_factor) + (1.414f*Cm*Cm*C_factor)/(2*my_factor)) + (Cm*mz)/(4*mz_factor);
    _virtual_thrust[6] = mx*(Cm*Cm*Cm/my_factor - (Cm*mx_factor)/(2*my_factor)) + my*(Cm*Cm*Cm/my_factor - (Cm*mx_factor)/(2*my_factor)) - fx/(4*Ct) + (1.414f*C_factor*mz)/(8*mz_factor);
    _virtual_thrust[7] = (1.414f*C_factor*mz)/(8*mz_factor) - fy/(4*Ct);
    _virtual_thrust[8] = - fz/(4*Ct) - mx*((1.414f*C_factor*mx_factor)/(4*my_factor) - (1.414f*Cm*Cm*C_factor)/(2*my_factor)) - my*((1.414f*C_factor*mx_factor)/(4*my_factor) - (1.414f*Cm*Cm*C_factor)/(2*my_factor)) - (Cm*mz)/(4*mz_factor);
    _virtual_thrust[9] = mx*(Cm*Cm*Cm/my_factor - (Cm*mx_factor)/(2*my_factor)) + my*(Cm*Cm*Cm/my_factor - (Cm*mx_factor)/(2*my_factor)) - fx/(4*Ct) - (1.414f*C_factor*mz)/(8*mz_factor);
    _virtual_thrust[10] = - fy/(4*Ct) - (1.414f*C_factor*mz)/(8*mz_factor);
    _virtual_thrust[11] = mx*((1.414f*C_factor*mx_factor)/(4*my_factor) - (1.414f*Cm*Cm*C_factor)/(2*my_factor)) - fz/(4*Ct) + my*((1.414f*C_factor*mx_factor)/(4*my_factor) - (1.414f*Cm*Cm*C_factor)/(2*my_factor)) - (Cm*mz)/(4*mz_factor);
    */

    _virutal_thrust[0] = (1.414f*Ct*L*mz)/(8*(Cm*Cm + Ct*Ct*L*L)) - my*(Cm*Cm*Cm/(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L) - (Cm*(2*Cm*Cm + Ct*Ct*L*L))/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - fx/(4*Ct) - mx*(Cm*Cm*Cm/(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L) - (Cm*(2*Cm*Cm + Ct*Ct*L*L))/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)));
    _virutal_thrust[1] = - fy/(4*Ct) - (1.414f*Ct*L*mz)/(8*(Cm*Cm + Ct*Ct*L*L));
    _virutal_thrust[2] = my*((1.414f*Ct*L*(2*Cm*Cm + Ct*Ct*L*L))/(4*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)) + (1.414f*Cm*Cm*Ct*L)/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - mx*((1.414f*Ct*L*(2*Cm*Cm + Ct*Ct*L*L))/(4*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)) + (1.414f*Cm*Cm*Ct*L)/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - fz/(4*Ct) + (Cm*mz)/(4*(Cm*Cm + Ct*Ct*L*L));
    _virutal_thrust[3] = - mx*(Cm*Cm*Cm/(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L) - (Cm*(2*Cm*Cm + Ct*Ct*L*L))/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - my*(Cm*Cm*Cm/(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L) - (Cm*(2*Cm*Cm + Ct*Ct*L*L))/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - fx/(4*Ct) - (1.414f*Ct*L*mz)/(8*(Cm*Cm + Ct*Ct*L*L));
    _virutal_thrust[4] = (1.414f*Ct*L*mz)/(8*(Cm*Cm + Ct*Ct*L*L)) - fy/(4*Ct);
    _virutal_thrust[5] = mx*((1.414f*Ct*L*(2*Cm*Cm + Ct*Ct*L*L))/(4*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)) + (1.414f*Cm*Cm*Ct*L)/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - fz/(4*Ct) - my*((1.414f*Ct*L*(2*Cm*Cm + Ct*Ct*L*L))/(4*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)) + (1.414f*Cm*Cm*Ct*L)/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) + (Cm*mz)/(4*(Cm*Cm + Ct*Ct*L*L));
    _virutal_thrust[6] = mx*(Cm*Cm*Cm/(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L) - (Cm*(2*Cm*Cm + Ct*Ct*L*L))/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) + my*(Cm*Cm*Cm/(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L) - (Cm*(2*Cm*Cm + Ct*Ct*L*L))/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - fx/(4*Ct) + (1.414f*Ct*L*mz)/(8*(Cm*Cm + Ct*Ct*L*L));
    _virutal_thrust[7] = (1.414f*Ct*L*mz)/(8*(Cm*Cm + Ct*Ct*L*L)) - fy/(4*Ct);
    _virutal_thrust[8] = - fz/(4*Ct) - mx*((1.414f*Ct*L*(2*Cm*Cm + Ct*Ct*L*L))/(4*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)) - (1.414f*Cm*Cm*Ct*L)/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - my*((1.414f*Ct*L*(2*Cm*Cm + Ct*Ct*L*L))/(4*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)) - (1.414f*Cm*Cm*Ct*L)/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - (Cm*mz)/(4*(Cm*Cm + Ct*Ct*L*L));
    _virutal_thrust[9] = mx*(Cm*Cm*Cm/(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L) - (Cm*(2*Cm*Cm + Ct*Ct*L*L))/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) + my*(Cm*Cm*Cm/(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L) - (Cm*(2*Cm*Cm + Ct*Ct*L*L))/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - fx/(4*Ct) - (1.414f*Ct*L*mz)/(8*(Cm*Cm + Ct*Ct*L*L));
    _virutal_thrust[10] = - fy/(4*Ct) - (1.414f*Ct*L*mz)/(8*(Cm*Cm + Ct*Ct*L*L));
    _virutal_thrust[11] = mx*((1.414f*Ct*L*(2*Cm*Cm + Ct*Ct*L*L))/(4*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)) - (1.414f*Cm*Cm*Ct*L)/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - fz/(4*Ct) + my*((1.414f*Ct*L*(2*Cm*Cm + Ct*Ct*L*L))/(4*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L)) - (1.414f*Cm*Cm*Ct*L)/(2*(4*Cm*Cm*Ct*Ct*L*L + Ct*Ct*Ct*Ct*L*L*L*L))) - (Cm*mz)/(4*(Cm*Cm + Ct*Ct*L*L));

    // 控制解耦
    // 根据虚拟力求解四个电机转速输出和俯仰舵机输出
    for (i = 0; i < 4; i++)
    {
        _thrust[i] = sqrtf(_virutal_thrust[3 * i] * _virutal_thrust[2 * i] + _virutal_thrust[2 * i + 1] * _virutal_thrust[2 * i + 1] + _virutal_thrust[2 * i + 2] * _virutal_thrust[2 * i + 2]);
        if (throttle_thrust > 0.15f)
        {
            _tilt[i] = (degrees(acosf(_virutal_thrust[3 * i + 2]/_thrust[i])) - 90.0f) / 90.0f;
            
        }
        else
        {
            _tilt[i] = 0.0f;
        }
    }
    
    // 偏航舵机的输出
    _tilt[4] = 0.0f;
    for (i = 0; i < 4; i++)
    {
    _tilt[4] += (degrees(atan2f(_virutal_thrust[3 * i + 1], _virutal_thrust[3 * i] ))) / 180.0f;
    }
    _tilt[4] = 0.25f * _tilt[4];

    // 重新限幅
    thrust_max = 1.0f;
    thrust_min = 0.0f;

    for (i = 0; i < 4; i++)
    {
        thrust_max = MAX(_thrust[i], thrust_max);
        thrust_min = MIN(_thrust[i], thrust_min);
    }

    if (thrust_max > 1.0f) 
    {
        // if max thrust is more than one reduce average throttle
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
    } else if (thrust_min < 0.0) 
    {
        // if min thrust is less than 0 increase average throttle
        // but never above max boost
        thr_adj = -thrust_min;
        if ((throttle_thrust + thr_adj) > max_boost_throttle) {
            thr_adj = MAX(max_boost_throttle - throttle_thrust, 0.0);
            // in this case we throw away some roll output, it will be uneven
            // constraining the lower motor more than the upper
            // this unbalances torque, but motor torque should have significantly less control power than tilts / control surfaces
            // so its worth keeping the higher roll control power at a minor cost to yaw
            limit.roll = true;
        }
        limit.throttle_lower = true;
    }

    // Add adjustment to reduce average throttle
    // 最终推力限幅
    for (i = 0; i < 4; i++)
    {
        _thrust[i] = constrain_float(_thrust[i] + thr_adj, 0.0f, 1.0f);
    }

    _throttle = throttle_thrust;

    // compensation_gain can never be zero
    // ensure accurate representation of average throttle output, this value is used for notch tracking and control surface scaling
    if (_has_diff_thrust) {
        _throttle_out = (throttle_thrust + thr_adj) / compensation_gain;
    } else {
        _throttle_out = throttle_thrust / compensation_gain;
    }

}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsTailsitter::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // right throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, pwm);
            break;
        case 2:
            // right tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, pwm);
            break;
        case 3:
            // left throttle
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, pwm);
            break;
        case 4:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, pwm);
            break;
        case 5:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor5, pwm);
            break;
        case 6:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor6, pwm);
            break;
        case 7:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor7, pwm);
            break;
        case 8:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor8, pwm);
            break;
        case 9:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_motor9, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
