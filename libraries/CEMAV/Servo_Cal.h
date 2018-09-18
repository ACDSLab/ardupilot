//
// Created by mgandhi3 on 9/17/18.
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_math.h>


#ifndef ARDUPILOT_SERVO_CAL_H
#define ARDUPILOT_SERVO_CAL_H

class Flap {
public:
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // PWM values corresponding to different flap angles
    AP_Int16        _pwm_0deg;
    AP_Int16        _pwm_15deg;
    AP_Int16        _pwm_30deg;
    AP_Int16        _pwm_45deg;
    AP_Int16        _pwm_60deg;
    AP_Int16        _pwm_75deg;
    AP_Int16        _pwm_90deg;
};

class Rudder {
public:
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // PWM values corresponding to different rudder angles
    AP_Int16        _pwm_n40deg;
    AP_Int16        _pwm_n30deg;
    AP_Int16        _pwm_n15deg;
    AP_Int16        _pwm_0deg;
    AP_Int16        _pwm_15deg;
    AP_Int16        _pwm_30deg;
    AP_Int16        _pwm_40deg;
};
#endif //ARDUPILOT_SERVO_CAL_H
