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
#include <CEMAV/Servo_Cal.h>


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
	float get_pilot_des_rpm(uint8_t throttle_stick_percent);

	// Algorithms to compute control inputs required for desired rate
	float compute_yaw_rate_control(float des_yaw_rate);
    float compute_rpm_control(float des_rpm, float curr_rpm);


    // pid accessors
//    AC_PID& get_rate_roll_pid() { return _pid_rate_roll; }
//    AC_PID& get_rate_pitch_pid() { return _pid_rate_pitch; }
    AC_PID& get_rate_yaw_pid() { return _pid_rate_yaw; }
    AC_PID& get_rpm_pid() {return _pid_rpm; }

    // servo cal accessors
    Flap& get_flap1(){ return _flap1;}
    Flap& get_flap2(){ return _flap2;}
    Flap& get_flap3(){ return _flap3;}
    Flap& get_flap4(){ return _flap4;}

    Rudder& get_rudder(){return _rudder;}

    // Control surface deflection angle to PWM values
    uint16_t flap_angle_to_pwm(float angle, int flap_number);
    uint16_t rudder_angle_to_pwm(float angle);

    //

    static const struct AP_Param::GroupInfo var_info[];  // Contains the information for parameters

private:

    // References to external libraries
    const AP_AHRS_View&  _ahrs;

    // Yaw Rate Parameters
    AP_Float _max_yaw_ds; // Maximum yaw rate in degrees per second
    AP_Float _yaw_rate_control_scale; // Scaling the error before putting it into the PID class

    // RPM Parameters
    AP_Float _max_rpm; // Maximum angular speed of the motor in revolutions per minute
    AP_Float _rpm_control_scale; // Scaling for RPM controller before transforming into a pwm value

    // PID controllers
    AC_PID   _pid_rate_yaw; // Parameters for AC_PID class yaw channel
    AC_PID   _pid_rpm;

    // Servo Calibration for 4 flaps and 1 rudder
    Flap _flap1;
    Flap _flap2;
    Flap _flap3;
    Flap _flap4;

    Rudder _rudder;


};



#endif /* CEMAV_H_ */
