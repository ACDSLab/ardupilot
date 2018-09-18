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
        AP_GROUPINFO("MAX_YAW_DS", 0, CEMAV, _max_yaw_ds, 720.0f),

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

        AP_SUBGROUPINFO(_rudder, "RUD_", 10, CEMAV, Rudder),

        // @Param: MAX_P_DS
        // @DisplayName: Maximum p angular velocity in deg/s
        // @Description: Maximum p angular velocity in deg/s. Max stick input commands max yaw rate.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("MAX_P_DS", 11, CEMAV, _max_p_ds, 720.0f),

        // @Param: MAX_Q_DS
        // @DisplayName: Maximum q angular velocity in deg/s
        // @Description: Maximum q angular velocity in deg/s. Max stick input commands max yaw rate.
        // @Range: 200 720
        // @Increment 1
        // @User: Advanced
        AP_GROUPINFO("MAX_Q_DS", 12, CEMAV, _max_q_ds, 720.0f),

        AP_SUBGROUPINFO(_lqr, "PQ_", 13, CEMAV, LQR),

		AP_GROUPEND

};

// CEMAV Constructor
CEMAV::CEMAV(AP_AHRS_View &ahrs, float dt) :
    _ahrs(ahrs),
    _pid_rate_yaw(10, 1, 0, 0.5, 5, dt),
    _pid_rpm(10, 0, 0, 0.5, 5, dt)
{
    AP_Param::setup_object_defaults(this, var_info);

}


/* Define functions to parse stick inputs (PWM). Functions are seperate in case
 * we want separate logic for each pilot input
 */

float CEMAV::get_pilot_des_yaw_rate(float norm_stick_input) {

	return norm_stick_input * _max_yaw_ds;
}

float CEMAV::get_pilot_des_p(float norm_stick_input) {

	return norm_stick_input  * _max_p_ds;
}

float CEMAV::get_pilot_des_q(float norm_stick_input) {

	return norm_stick_input  * _max_q_ds;
}

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
float CEMAV::get_pilot_des_rpm(uint8_t throttle_stick_percent) {

    return ( (float) throttle_stick_percent/ 100.0) * _max_rpm;

}

/* Define functions to compute the PWM values for the inner control loop for
 * body rates.
 */
float CEMAV::compute_yaw_rate_control(float des_yaw_rate) {
    // Get the current yaw rate
    float curr_yaw_rate = _ahrs.get_gyro()[2] * RAD_TO_DEG;

    // Get the desired yaw rate
    float error = des_yaw_rate - curr_yaw_rate;
    _pid_rate_yaw.set_input_filter_d(error); // Filter the error signal

    return (_pid_rate_yaw.get_pid()) / _yaw_rate_control_scale; // Compute then scale the output control
}

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
	}
	return pwm;
}

uint16_t CEMAV::rudder_angle_to_pwm(float angle) {
    return _rudder.rudder_angle_to_pwm(angle);
}

void compute_control_pq(float des_p, float des_q, uint16_t (&flap_pwms)[4]) {
  float cur_p = _ahrs.get_gyro()[0] * RAD_TO_DEG;
  float cur_q = _ahrs.get_gyro()[1] * RAD_TO_DEG;
  _lqr.compute_control_pq(cur_p, cur_q, des_p, des_q, flap_pwms);
}
