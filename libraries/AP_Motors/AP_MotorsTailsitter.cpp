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
#include <RC_Channel/RC_Channel.h>

extern const AP_HAL::HAL& hal;

#define SERVO_OUTPUT_RANGE  4500

// init
void AP_MotorsTailsitter::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // setup default motor and servo mappings
    _has_diff_thrust = SRV_Channels::function_assigned(SRV_Channel::k_motor1) || SRV_Channels::function_assigned(SRV_Channel::k_motor2) || SRV_Channels::function_assigned(SRV_Channel::k_motor3)  || SRV_Channels::function_assigned(SRV_Channel::k_motor4);

    // 四个电机通道 33 34 35 36
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor1, CH_1);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor2, CH_2);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor3, CH_3);

    SRV_Channels::set_aux_channel_default(SRV_Channel::k_motor4, CH_4);

    // 机构俯仰倾转舵机 48
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltpitch, CH_5);
    SRV_Channels::set_angle(SRV_Channel::k_tiltpitch, SERVO_OUTPUT_RANGE);

    // 机构偏航倾转舵机 49
    SRV_Channels::set_aux_channel_default(SRV_Channel::k_tiltyaw, CH_6);
    SRV_Channels::set_angle(SRV_Channel::k_tiltyaw, SERVO_OUTPUT_RANGE);

    _mav_type = MAV_TYPE_VTOL_DUOROTOR;

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

    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleLeft, speed_hz);
    SRV_Channels::set_rc_frequency(SRV_Channel::k_throttleRight, speed_hz);
}

void AP_MotorsTailsitter::output_to_motors()
{
    if (!initialised_ok()) {
        return;
    }

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            _actuator[0] = 0.0f;
            _actuator[1] = 0.0f;
            _actuator[2] = 0.0f;
            _actuator[3] = 0.0f;
            _external_min_throttle = 0.0;
            break;
        case SpoolState::GROUND_IDLE:
            set_actuator_with_slew(_actuator[0], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[1], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[2], actuator_spin_up_to_ground_idle());
            set_actuator_with_slew(_actuator[3], actuator_spin_up_to_ground_idle());
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

    SRV_Channels::set_output_pwm(SRV_Channel::k_motor1, output_to_pwm(_actuator[0]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor2, output_to_pwm(_actuator[1]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor3, output_to_pwm(_actuator[2]));
    SRV_Channels::set_output_pwm(SRV_Channel::k_motor4, output_to_pwm(_actuator[3]));

    // use set scaled to allow a different PWM range on plane forward throttle, throttle range is 0 to 100
    //SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, _actuator[2]*100);

    //始终输出给倾转舵机
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltpitch, _tilt_pitch*SERVO_OUTPUT_RANGE);
    SRV_Channels::set_output_scaled(SRV_Channel::k_tiltyaw, _tilt_yaw*SERVO_OUTPUT_RANGE);

}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
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

// calculate outputs to the motors
void AP_MotorsTailsitter::output_armed_stabilizing()
{
    uint8_t i;
    
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0
    float   rotate_angle_pitch;               // 手动控制旋转角
    float   rotate_angle_yaw;
    float   rate;                       // 舵机俯仰控制权重，水平为0，朝下为1，朝上为-1
    float   m_rate;
    float   s_rate;
    //float   y_rate;
    //float   x_rate;
    float   thrust_max;                 // highest motor value
    float   thrust_min;                 // lowest motor value
    float   thr_adj = 0.0f;             // the difference between the pilot's desired throttle and throttle_thrust_best_rpy

    // apply voltage and air pressure compensation
    const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;
    pitch_thrust = _pitch_in + _pitch_in_ff;
    yaw_thrust = _yaw_in + _yaw_in_ff;
    throttle_thrust = get_throttle() * compensation_gain;
    const float max_boost_throttle = _throttle_avg_max * compensation_gain;

    // never boost above max, derived from throttle mix params
    const float min_throttle_out = MIN(_external_min_throttle, max_boost_throttle);
    const float max_throttle_out = _throttle_thrust_max * compensation_gain;

    // 旋转角与俯仰控制权重
    rotate_angle_pitch= RC_Channels::get_radio_in(CH_6);// 获取遥控器第6通道值
    rotate_angle_pitch= (rotate_angle_pitch -1500) *0.002f;// -1..1
    rate = rotate_angle_pitch;

    // 电机俯仰控制权重恒正
    if (rate >= 0.0f) 
    {
        m_rate= 1.0f -rate;
        s_rate= rate;  
        }
    if (rate <  0.0f) 
    {
        m_rate= 1.0f +rate;
        s_rate=-rate;
        }

    // 旋转角与偏航控制权重
    rotate_angle_yaw= RC_Channels::get_radio_in(CH_5);
    rotate_angle_yaw= (rotate_angle_yaw -1500) *0.002f;
    
    /*rate = rotate_angle_yaw;

    if (rate >= 0.0f)
    {
        x_rate = 1.0f - rate;
        y_rate = rate;
    }
    if (rate < 0.0f)
    {
        x_rate = 1.0f + rate;
        y_rate = - rate;
    }*/

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

    // calculate left and right throttle outputs
    _thrust[0]  = throttle_thrust - roll_thrust * 0.45f + m_rate * pitch_thrust * 0.4f + yaw_thrust * 0.45f;
    _thrust[1]  = throttle_thrust + roll_thrust * 0.45f - m_rate * pitch_thrust * 0.4f + yaw_thrust * 0.45f;
    _thrust[2]  = throttle_thrust + roll_thrust * 0.45f + m_rate * pitch_thrust * 0.4f - yaw_thrust * 0.45f;
    _thrust[3]  = throttle_thrust - roll_thrust * 0.45f - m_rate * pitch_thrust * 0.4f - yaw_thrust * 0.45f;

    thrust_max = MAX(_thrust[0], _thrust[1]);
    thrust_min = MIN(_thrust[0], _thrust[1]);
    if (thrust_max > 1.0f) {
        // if max thrust is more than one reduce average throttle
        thr_adj = 1.0f - thrust_max;
        limit.throttle_upper = true;
    } else if (thrust_min < 0.0) {
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
    for(i = 0; i < 4 ; i++)
    {
    _thrust[i]  = constrain_float(_thrust[i]  + thr_adj, 0.0f, 1.0f);
    }

    // compensation_gain can never be zero
    // ensure accurate representation of average throttle output, this value is used for notch tracking and control surface scaling
    if (_has_diff_thrust) {
        _throttle_out = (throttle_thrust + thr_adj) / compensation_gain;
    } else {
        _throttle_out = throttle_thrust / compensation_gain;
    }

    

    _tilt_pitch  = - rotate_angle_pitch * 0.7f - s_rate * pitch_thrust * 0.3f;
    _tilt_yaw = rotate_angle_yaw * 0.6f;
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
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltpitch, pwm);
            break;
        case 6:
            // left tilt servo
            SRV_Channels::set_output_pwm(SRV_Channel::k_tiltyaw, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
