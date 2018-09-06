/*
 * Coanda_MAV.cpp
 *
 *  Created on: Sep 5, 2018
 *      Author: Manan Gandhi
 */

#include "CEMAV.h"

// Add parameters to the var_info table
const AP_Param::GroupInfo CEMAV::var_info[] = {
		// @Param: MY_NEW_PARAM
		// @DisplayName: Compass3 device id expected
		// @Description: The expected value of COMPASS_DEV_ID3, used by arming checks. Setting this to -1 means "don't care."
		AP_GROUPINFO("NEW_param", 0, CEMAV, my_new_lib_parameter, 20),

        // @Param: MY_NEW_PARAM2
        // @DisplayName: Compass3 device id expected
        // @Description: The expected value of COMPASS_DEV_ID3, used by arming checks. Setting this to -1 means "don't care."
        AP_GROUPINFO("NEW_param2", 1, CEMAV, my_new_lib_parameter2, MY_NEW_PARAM_DEFAULT2),

//        // @Param: MAX_YAW_RATE
//        // @DisplayName: Maximum body yaw rate
//        // @Description: Maximum body yaw rate of the cemav vehicle
//        AP_GROUPINFO("MAX_YAW_RATE", 0, CEMAV, MAX_YAW_RATE, MAX_YAW_RATE_DEFAULT),

		AP_GROUPEND

};


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

