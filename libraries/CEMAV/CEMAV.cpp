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
        AP_GROUPINFO("RAT_MAX_YAW", 0, CEMAV, _max_yaw_ds, 720.0f),

        // @Param: MAX_RPM
        // @DisplayName: Maximum motor speed in rpm
        // @Description: Maximum motor speed in rpm of the main engine for the CE vehicle.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("MAX_RPM", 1, CEMAV, _max_rpm, 9000.0f),

        // @Param: RPM_SC
        // @DisplayName: Rotor speed control scaling
        // @Description: The control output from the PID controller is scaled by this value so that controller outputs are bounded before being converted to PWM
        // @Range: 1 10000
        // @Increment 10
        // @User: Advanced
        AP_GROUPINFO("RPM_SC", 2, CEMAV, _rpm_control_scale, 1000.0f),

        // @Param: YAW_RAT_SC
        // @DisplayName: Yaw rate control scaling
        // @Description: The control output from the PID controller is normalized by this number so that resulting PWM values are bounded.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("YAW_RAT_SC", 3, CEMAV, _yaw_rate_control_scale, 720.0f),


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
        AP_SUBGROUPINFO(_pid_rate_yaw, "RAT_YAW_", 4, CEMAV, AC_PID),

        // @Param: RPM_P
        // @DisplayName: Yaw axis rate controller P gain
        // @Description: Yaw axis rate controller P gain.  Converts the difference between desired yaw rate and actual yaw rate into a motor speed output
        // @Range: 0.10 2.50
        // @Increment: 0.005
        // @User: Standard

        // @Param: RPM_I
        // @DisplayName: Yaw axis rate controller I gain
        // @Description: Yaw axis rate controller I gain.  Corrects long-term difference in desired yaw rate vs actual yaw rate
        // @Range: 0.010 1.0
        // @Increment: 0.01
        // @User: Standard

        // @Param: RPM_IMAX
        // @DisplayName: Yaw axis rate controller I gain maximum
        // @Description: Yaw axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
        // @Range: 0 1
        // @Increment: 0.01
        // @Units: %
        // @User: Standard

        // @Param: RPM_D
        // @DisplayName: Yaw axis rate controller D gain
        // @Description: Yaw axis rate controller D gain.  Compensates for short-term change in desired yaw rate vs actual yaw rate
        // @Range: 0.000 0.02
        // @Increment: 0.001
        // @User: Standard

        // @Param: RPM_FF
        // @DisplayName: Yaw axis rate controller feed forward
        // @Description: Yaw axis rate controller feed forward
        // @Range: 0 0.5
        // @Increment: 0.001
        // @User: Standard

        // @Param: RPM_FILT
        // @DisplayName: Yaw axis rate controller input frequency in Hz
        // @Description: Yaw axis rate controller input frequency in Hz
        // @Range: 1 10
        // @Increment: 1
        // @Units: Hz
        // @User: Standard
        AP_SUBGROUPINFO(_pid_rpm, "RPM_", 5, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_flap1, "FL1_", 6, CEMAV, Flap),
        AP_SUBGROUPINFO(_flap2, "FL2_", 7, CEMAV, Flap),
        AP_SUBGROUPINFO(_flap3, "FL3_", 8, CEMAV, Flap),
        AP_SUBGROUPINFO(_flap4, "FL4_", 9, CEMAV, Flap),
        AP_SUBGROUPINFO(_flap5, "FL5_", 10, CEMAV, Flap),
        AP_SUBGROUPINFO(_flap6, "FL6_", 11, CEMAV, Flap),
        AP_SUBGROUPINFO(_flap7, "FL7_", 12, CEMAV, Flap),
        AP_SUBGROUPINFO(_flap8, "FL8_", 13, CEMAV, Flap),

        AP_SUBGROUPINFO(_rudder, "RUD_", 14, CEMAV, Rudder),

        // @Param: MAX_P_DS
        // @DisplayName: Maximum p angular velocity in deg/s
        // @Description: Maximum p angular velocity in deg/s. Max stick input commands max roll rate.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("RAT_MAX_P", 15, CEMAV, _max_p_ds, 720.0f),

        // @Param: MAX_Q_DS
        // @DisplayName: Maximum q angular velocity in deg/s
        // @Description: Maximum q angular velocity in deg/s. Max stick input commands max pitch rate.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("RAT_MAX_Q", 16, CEMAV, _max_q_ds, 720.0f),

        AP_SUBGROUPINFO(_lqr, "PQ_", 17, CEMAV, LQR),

        // @Param: MAX_PIT
        // @DisplayName: Maximum pitch angle in deg
        // @Description: Maximum q angular velocity in deg/s. Max stick input commands max yaw rate.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("AT_MAX_PIT", 18, CEMAV, _max_pitch_angle, 45.0f),

        // @Param: MAX_ROL
        // @DisplayName: Maximum roll angle in deg
        // @Description: Maximum roll angle in deg.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("AT_MAX_ROL", 19, CEMAV, _max_roll_angle, 45.0f),

        // @Param: MAX_D_YAW
        // @DisplayName: Maximum change in yaw angle in delta deg
        // @Description: Maximum roll angle in deg.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("MAX_D_YAW", 20, CEMAV, _max_delta_yaw_angle, 720.0f),

        AP_SUBGROUPINFO(_pid_nil_pitch, "AT_NI_PI_", 21, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_pid_nil_roll, "AT_NI_RO_", 22, CEMAV, AC_PID),

        AP_GROUPINFO("COUNTER", 23, CEMAV, _count_max, 4),

        AP_GROUPINFO("YAW_TRIM", 24, CEMAV, _yaw_trim_angle, 0.0f),

        AP_GROUPINFO("MAX_FLAP_ANG", 25, CEMAV, _max_flap_angle, 90.0f),

        AP_GROUPINFO("MIN_FLAP_ANG", 26, CEMAV, _min_flap_angle, 30.0f),

        AP_SUBGROUPINFO(_cf, "LM_", 27, CEMAV, CrossFeed),
		
		AP_SUBGROUPINFO(_pid_rate_lat, "RAT_LAT_", 28, CEMAV, AC_PID),

		AP_SUBGROUPINFO(_pid_rate_long, "RAT_LONG_", 29, CEMAV, AC_PID),
		
		AP_SUBGROUPINFO(_pid_yaw, "AT_YAW_", 30, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_pid_il_pitch, "AT_IL_PI_", 31, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_pid_il_roll, "AT_IL_RO_", 32, CEMAV, AC_PID),

        AP_GROUPEND

};

// CEMAV Constructor
CEMAV::CEMAV(AP_AHRS_View &ahrs, float dt) :
    _ahrs(ahrs),
    _pid_rate_yaw(1, 1, 0, 0.5, 5, dt),
    _pid_rate_lat(1, 1, 0, 0.5, 5, dt),
    _pid_rate_long(1, 1, 0, 0.5, 5, dt),

    _pid_rpm(10, 0, 0, 0.5, 5, dt),
    _pid_nil_pitch(1,1,0,0.5,5,dt),
    _pid_nil_roll(1,1,0,0.5,5,dt),
    _pid_il_pitch(1,1,0,0.5,5,dt),
    _pid_il_roll(1,1,0,0.5,5,dt),
	_pid_yaw(1,1,0,0.1,5,dt)
{
    AP_Param::setup_object_defaults(this, var_info);

}


/* Define functions to parse stick inputs (PWM). Functions are seperate in case
 * we want separate logic for each pilot input
 */

float CEMAV::get_pilot_des_yaw_rate(float norm_stick_input) {

	return norm_stick_input * _max_yaw_ds * DEG_TO_RAD;
}

float CEMAV::get_pilot_des_p(float norm_stick_input) {

	return norm_stick_input  * _max_p_ds * DEG_TO_RAD;
}

float CEMAV::get_pilot_des_q(float norm_stick_input) {

	return norm_stick_input  * _max_q_ds * DEG_TO_RAD;
}

float CEMAV::get_pilot_des_pitch(float norm_stick_input) {

	return norm_stick_input  * _max_pitch_angle * DEG_TO_RAD;
}

float CEMAV::get_pilot_des_roll(float norm_stick_input) {

	return norm_stick_input  * _max_roll_angle * DEG_TO_RAD;
}

float CEMAV::get_pilot_des_rpm(uint8_t throttle_stick_percent) {

    return ( (float) throttle_stick_percent/ 100.0) * _max_rpm;

}

/* Define functions to compute the PWM values for the inner control loop for
 * body rates.
 */


float CEMAV::compute_rpm_control(float des_rpm, float curr_rpm) {
    float error = (des_rpm - curr_rpm);
    _pid_rpm.set_input_filter_all(error);

    return (_pid_rpm.get_pid()) / _rpm_control_scale; // Compute then scale the output control
}

uint16_t CEMAV::flap_angle_to_pwm(float angle, int flap_number) {

    uint16_t pwm = 0;

	switch (flap_number) {
        case 1:
            pwm = _flap1.flap_angle_to_pwm(angle);
            break;
        case 2:
            pwm = _flap2.flap_angle_to_pwm(angle);
            break;
        case 3:
            pwm = _flap3.flap_angle_to_pwm(angle);
            break;
        case 4:
            pwm = _flap4.flap_angle_to_pwm(angle);
            break;
	    case 5:
	        pwm = _flap5.flap_angle_to_pwm(angle);
	        break;
        case 6:
            pwm = _flap6.flap_angle_to_pwm(angle);
            break;
        case 7:
            pwm = _flap7.flap_angle_to_pwm(angle);
            break;
	    case 8:
	        pwm = _flap8.flap_angle_to_pwm(angle);
	        break;
	}
	return pwm;
}

uint16_t CEMAV::rudder_angle_to_pwm(float angle) {
    return _rudder.rudder_angle_to_pwm(angle);
}

void CEMAV::pq_feedback_flaps(float des_p, float des_q, float (&flap_angles)[8]) {
    float cur_p = _ahrs.get_gyro()[0];
    float cur_q = _ahrs.get_gyro()[1];
    _lqr.compute_flaps_pq(cur_p, cur_q, des_p, des_q, flap_angles);
}

void CEMAV::pq_feedback_commands(float des_p, float des_q, float (&commands)[2]) {
    float cur_p = _ahrs.get_gyro()[0];
    float cur_q = _ahrs.get_gyro()[1];
    _lqr.compute_twostate_pq(cur_p, cur_q, des_p, des_q, commands);
}

float CEMAV::compute_yaw_rate_control(float des_yaw_rate) {
    // Get the current yaw rate
    float curr_yaw_rate = _ahrs.get_gyro()[2];

    // Get the desired yaw rate
    float error = des_yaw_rate - curr_yaw_rate; // diff in rad/sec
    _pid_rate_yaw.set_input_filter_d(error); // Filter the error signal

    return (_pid_rate_yaw.get_pid()) - _yaw_trim_angle; // Compute then scale the output control
}

float CEMAV::compute_lat_rate_control(float des_lat_rate) {
    // Get the current lateral rate
    float curr_lat_rate = _ahrs.get_gyro()[0];

    // Get the desired lateral rate
    float error = des_lat_rate - curr_lat_rate; // diff in rad/sec
    _pid_rate_lat.set_input_filter_d(error); // Filter the error signal

    return (_pid_rate_lat.get_pid()); // Compute then scale the output control
}


float CEMAV::compute_long_rate_control(float des_long_rate) {
    // Get the current longitudinal rate
    float curr_long_rate = _ahrs.get_gyro()[1];

    // Get the desired longitudinal rate
    float error = des_long_rate - curr_long_rate; // diff in rad/sec
    _pid_rate_long.set_input_filter_d(error); // Filter the error signal

    return (_pid_rate_long.get_pid()); // Compute then scale the output control
}


void CEMAV::compute_NIL_pitch_roll(float des_pitch, float des_roll, float (&commands)[2]) {
    // Compute the error in both pitch and roll
    float err_pitch = des_pitch - _ahrs.pitch;
    float err_roll = des_roll - _ahrs.roll;

    // Set and then compute the pid terms, the derivative portion gives us proportional rate control.
    _pid_nil_pitch.set_input_filter_all(err_pitch);
    _pid_nil_roll.set_input_filter_all(err_roll);
    commands[0] = _pid_nil_roll.get_pid(); // L_c
    commands[1] = _pid_nil_pitch.get_pid();  // M_c

}

void CEMAV::compute_IL_pitch_roll(float des_pitch, float des_roll, float (&commands)[2]) {
    float err_pitch = des_pitch - _ahrs.pitch;
    float err_roll = des_roll - _ahrs.roll;

    // Set and then compute the att PID terms, the output is desired rates in p and q
    _pid_il_pitch.set_input_filter_all(err_pitch);
    _pid_il_roll.set_input_filter_all(err_roll);

    // Set and then compute the angular velocity PID terms, the output is L_c, and M_c
    float err_lat_rate = _pid_il_roll.get_pid() - _ahrs.get_gyro()[0];
    float err_long_rate = _pid_il_pitch.get_pid() - _ahrs.get_gyro()[1];

    _pid_rate_lat.set_input_filter_all(err_lat_rate);
    _pid_rate_long.set_input_filter_all(err_long_rate);

    commands[0] = _pid_rate_lat.get_pid(); // L_c
    commands[1] = _pid_rate_long.get_pid();  // M_c
}

void CEMAV::compute_crossfeed_LM(float lat_c, float lon_c, float& cf_L, float& cf_M) {
    _cf.compute_crossfeed_moments(lat_c, lon_c, cf_L, cf_M);
}

float CEMAV::rescale_flaps(float input_command) {
    return (_max_flap_angle - _min_flap_angle) * input_command + _min_flap_angle;
}
