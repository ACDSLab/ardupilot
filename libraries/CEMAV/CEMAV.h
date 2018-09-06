/*
 * Coanda_MAV.h
 *
 *  Created on: Sep 5, 2018
 *      Author: Manan Gandhi
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#ifndef CEMAV_H_
#define CEMAV_H_

const int COANDA_MAX_RATE  = 20;
const int CEMAV_MAX_YAW_RATE = 2; // max yaw rate in rads/sec
const int CEMAV_MAX_P_RATE = 2; // max pitch rate in rads/sec
const int CEMAV_MAX_Q_RATE = 2; // max roll rate in rads/sec
const int CEMAV_MAX_PITCH = 1; // max pitch angle in rads
const int CEMAV_MAX_ROLL = 1; // max roll angle in rads
const int CEMAV_MAX_THROTTLE = 1; // max yaw rate in throttle percent

// Define new parameters:
#define MY_NEW_PARAM_DEFAULT 20
#define MY_NEW_PARAM_DEFAULT2 21

//#define MAX_YAW_RATE_DEFAULT 2.0

#define COANDA_MAX_YAW_RATE 2;

class CEMAV {
public:
	float get_pilot_des_yaw_rate(int16_t stick_input);// { return stick_input / MAX_YAW_STICK_INPUT * CEMAV_MAX_YAW_RATE; };
	float get_pilot_des_p_rate(int16_t stick_input);  // X axis body rate desired
	float get_pilot_des_q_rate(int16_t stick_input);  // Y axis body rate desired
	float get_pilot_des_pitch(int16_t stick_input);
	float get_pilot_des_roll(int16_t stick_input);
	float get_pilot_des_throttle(int16_t stick_input);

    static const struct AP_Param::GroupInfo var_info[];  // Contains the information for parameters


private:
	const int16_t MAX_YAW_STICK_INPUT = 1900;
	const int16_t MAX_P_STICK_INPUT = 1900;
	const int16_t MAX_Q_STICK_INPUT = 1900;
	const int16_t MAX_PITCH_STICK_INPUT = 1900;
	const int16_t MAX_ROLL_STICK_INPUT = 1900;
	const int16_t MAX_THROTTLE_STICK_INPUT = 1900;


protected:
    AP_Float my_new_lib_parameter; // new param
    AP_Float my_new_lib_parameter2; // new param

//    AP_Float MAX_YAW_RATE;
};



#endif /* CEMAV_H_ */
