/*
 * Coanda_MAV.h
 *
 *  Created on: Sep 5, 2018
 *      Author: Manan Gandhi
 */
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h> // Define parameters
#include <AP_Math/AP_Math.h> // Use common math functions
#include <AP_AHRS/AP_AHRS_View.h> // Get a view of the ekf
#include <AC_PID/AC_PID.h>  // Get the PID controller class
#include <SRV_Channel/SRV_Channel.h> // Set servo angle max


#ifndef CEMAV_H_
#define CEMAV_H_

class CEMAV {
public:
    // Constructor
    CEMAV(AP_AHRS_View &ahrs, float dt);

    // empty destructor to suppress compiler warning
    virtual ~CEMAV() {}

    // Functions to get pilot desired input given system parameters and normalized stick input
    float get_pilot_des_yaw_rate(float norm_stick_input);// { return stick_input / MAX_YAW_STICK_INPUT * CEMAV_MAX_YAW_RATE; };
//	float get_pilot_des_p_rate(int16_t norm_stick_input);  // X axis body rate desired
//	float get_pilot_des_q_rate(int16_t norm_stick_input);  // Y axis body rate desired
//	float get_pilot_des_pitch(int16_t norm_stick_input);
//	float get_pilot_des_roll(int16_t norm_stick_input);
	float get_pilot_des_crpm(float stick_percent_input);

	// Algorithms to compute control inputs required for desired rate
	float compute_yaw_rate_control(float des_yaw_rate);
    float compute_rpm_control(float des_rpm, float curr_rpm);


        // pid accessors
//    AC_PID& get_rate_roll_pid() { return _pid_rate_roll; }
//    AC_PID& get_rate_pitch_pid() { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() { return _pid_rate_yaw; }
    AC_PID& get_rpm_pid() {return _pid_rpm; }

    static const struct AP_Param::GroupInfo var_info[];  // Contains the information for parameters

private:

    // References to external libraries
    const AP_AHRS_View&  _ahrs;

    // Yaw Rate Parameters
    AP_Float _max_yaw_ds; // Maximum yaw rate in degrees per second
    AP_Float _max_rpm; // Maximum rpm in rev per minute
    AC_PID   _pid_rate_yaw; // Parameters for AC_PID class yaw channel
    AC_PID   _pid_rpm;
	AP_Int16 _f1_scale;
	AP_Int16 _f2_scale;
	AP_Int16 _f3_scale;
	AP_Int16 _f4_scale;
	AP_Int16 _f5_scale;
	AP_Int16 _f6_scale;
    AP_Int16 _f7_scale;
    AP_Int16 _f8_scale;

    AP_Int16 _r_scale;
    AP_Int16 _t_scale;

};



#endif /* CEMAV_H_ */
