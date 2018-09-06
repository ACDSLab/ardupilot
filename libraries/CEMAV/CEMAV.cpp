/*
 * Coanda_MAV.cpp
 *
 *  Created on: Sep 5, 2018
 *      Author: Manan Gandhi
 */

#include "CEMAV.h"

/* Define functions to parse stick inputs (PWM). Functions are seperate in case
we want separate logic for each pilot input
 */

float CEMAV::get_pilot_des_yaw_rate(int16_t stick_input) {

	return stick_input / MAX_YAW_STICK_INPUT * CEMAV_MAX_YAW_RATE;
}

float CEMAV::get_pilot_des_p_rate(int16_t stick_input) {

	return stick_input / MAX_P_STICK_INPUT * CEMAV_MAX_P_RATE;
}

float CEMAV::get_pilot_des_q_rate(int16_t stick_input) {

	return stick_input / MAX_Q_STICK_INPUT * CEMAV_MAX_Q_RATE;
}

float CEMAV::get_pilot_des_pitch(int16_t stick_input) {

	return stick_input / MAX_PITCH_STICK_INPUT * CEMAV_MAX_PITCH;
}

float CEMAV::get_pilot_des_roll(int16_t stick_input) {

	return stick_input / MAX_ROLL_STICK_INPUT * CEMAV_MAX_ROLL;
}

float CEMAV::get_pilot_des_throttle(int16_t stick_input) {

	return stick_input / MAX_THROTTLE_STICK_INPUT * CEMAV_MAX_THROTTLE;
}

