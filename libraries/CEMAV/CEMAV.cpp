/*
 * Coanda_MAV.cpp
 *
 *  Created on: Sep 5, 2018
 *      Author: Manan Gandhi
 */


#include "CEMAV.h"

// Add parameters to the var_info table
const AP_Param::GroupInfo CEMAV::var_info[] = {
		// @Param: MAX_YAW_DS
		// @DisplayName: Maximum yaw rate in deg/s
        // @Description: Maximum Yaw rate in deg/s. Max stick input commands max yaw rate.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("MAX_YAW_DS", 0, CEMAV, _max_yaw_ds, 360.0f),

        // @Param: MAX_RPM
        // @DisplayName: Maximum motor speed in rpm
        // @Description: Maximum motor speed in rpm of the main engine for the CE vehicle.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("MAX_RPM", 1, CEMAV, _max_rpm, 3000.0f),


        // @Param: RAT_YAW_P
        // @DisplayName: Yaw axis rate controller P gain
        // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
        // @Range: 0.10 2.50
        // @Increment: 0.005
        // @User: Standard

        // @Param: RAT_YAW_I
        // @DisplayName: Yaw axis rate controller I gain
        // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
        // @Range: 0.010 1.0
        // @Increment: 0.01
        // @User: Standard

        // @Param: RAT_YAW_IMAX
        // @DisplayName: Yaw axis rate controller I gain maximum
        // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
        // @Range: 0 1
        // @Increment: 0.01
        // @Units: %
        // @User: Standard

        // @Param: RAT_YAW_D
        // @DisplayName: Yaw axis rate controller D gain
        // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
        // @Range: 0.000 0.02
        // @Increment: 0.001
        // @User: Standard

        // @Param: RAT_YAW_FF
        // @DisplayName: Yaw axis rate controller feed forward
        // @Description: Yaw axis rate controller feed forward
        // @Range: 0 0.5
        // @Increment: 0.001
        // @User: Standard

        // @Param: RAT_YAW_FILT
        // @DisplayName: Yaw axis rate controller input frequency in Hz
        // @Description: Yaw axis rate controller input frequency in Hz
        // @Range: 1 10
        // @Increment: 1
        // @Units: Hz
        // @User: Standard
        AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 2, CEMAV, AC_PID),

        // @Param: RAT_YAW_P
        // @DisplayName: Yaw axis rate controller P gain
        // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
        // @Range: 0.10 2.50
        // @Increment: 0.005
        // @User: Standard

        // @Param: RAT_YAW_I
        // @DisplayName: Yaw axis rate controller I gain
        // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
        // @Range: 0.010 1.0
        // @Increment: 0.01
        // @User: Standard

        // @Param: RAT_YAW_IMAX
        // @DisplayName: Yaw axis rate controller I gain maximum
        // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
        // @Range: 0 1
        // @Increment: 0.01
        // @Units: %
        // @User: Standard

        // @Param: RAT_YAW_D
        // @DisplayName: Yaw axis rate controller D gain
        // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
        // @Range: 0.000 0.02
        // @Increment: 0.001
        // @User: Standard

        // @Param: RAT_YAW_FF
        // @DisplayName: Yaw axis rate controller feed forward
        // @Description: Yaw axis rate controller feed forward
        // @Range: 0 0.5
        // @Increment: 0.001
        // @User: Standard

        // @Param: RAT_YAW_FILT
        // @DisplayName: Yaw axis rate controller input frequency in Hz
        // @Description: Yaw axis rate controller input frequency in Hz
        // @Range: 1 10
        // @Increment: 1
        // @Units: Hz
        // @User: Standard
        AP_SUBGROUPINFO(_pid_rpm, "RPM_", 3, CEMAV, AC_PID),
		AP_GROUPINFO("M1_SCALE", 4, CEMAV, _m1_scale, 720),
		AP_GROUPINFO("M2_SCALE", 5, CEMAV, _m2_scale, 720),
		AP_GROUPINFO("M3_SCALE", 6, CEMAV, _m3_scale, 720),
		AP_GROUPINFO("M4_SCALE", 7, CEMAV, _m4_scale, 720),
		AP_GROUPINFO("M5_SCALE", 8, CEMAV, _m5_scale, 720),
		AP_GROUPINFO("M6_SCALE", 9, CEMAV, _m6_scale, 9000),


		AP_GROUPEND

};

// CEMAV Constructor
CEMAV::CEMAV(AP_AHRS_View &ahrs, float dt) :
    _ahrs(ahrs),
    _pid_rate_yaw(10, 1, 0, 0.5, 5, dt),
    _pid_rpm(10, 0, 0, 0.5, 5, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    SRV_Channels::set_angle(SRV_Channel::k_motor5, _m5_scale);
    SRV_Channels::set_range(SRV_Channel::k_motor6, _m6_scale);
    SRV_Channels::set_angle(SRV_Channel::k_motor1, _m1_scale);
    SRV_Channels::set_angle(SRV_Channel::k_motor2, _m2_scale);
    SRV_Channels::set_angle(SRV_Channel::k_motor3, _m3_scale);
    SRV_Channels::set_angle(SRV_Channel::k_motor4, _m4_scale)
}


/* Define functions to parse stick inputs (PWM). Functions are seperate in case
 * we want separate logic for each pilot input
 */

float CEMAV::get_pilot_des_yaw_rate(float norm_stick_input) {

	return norm_stick_input * _max_yaw_ds;
}

//float CEMAV::get_pilot_des_p_rate(int16_t norm_stick_input) {
//
//	return norm_stick_input  * CEMAV_MAX_P_RATE;
//}
//
//float CEMAV::get_pilot_des_q_rate(int16_t norm_stick_input) {
//
//	return norm_stick_input  * CEMAV_MAX_Q_RATE;
//}
//
//float CEMAV::get_pilot_des_pitch(int16_t norm_stick_input) {
//
//	return norm_stick_input  * CEMAV_MAX_PITCH;
//}
//
//float CEMAV::get_pilot_des_roll(int16_t norm_stick_input) {
//
//	return norm_stick_input  * CEMAV_MAX_ROLL;
//}
//
float CEMAV::get_pilot_des_rpm(float norm_stick_input) {

	return norm_stick_input  * _max_rpm;
}

/* Define functions to compute the PWM values for the inner control loop for
 * body rates.
 */
float CEMAV::compute_yaw_rate_control(float des_yaw_rate) {
    // Get the current yaw rate
    float curr_yaw_rate = _ahrs.get_gyro()[2] * RAD_TO_DEG;
//    float curr_yaw_rate = _ahrs.yaw* RAD_TO_DEG;
    // Get the desired yaw rate
    float error = des_yaw_rate - curr_yaw_rate;
    _pid_rate_yaw.set_input_filter_d(error);

    return _pid_rate_yaw.get_pid();
}

float CEMAV::compute_rpm_control(float des_rpm, float curr_rpm) {
    float error = des_rpm - curr_rpm;
    _pid_rpm.set_input_filter_d(error);

    return _pid_rpm.get_pid();
}

