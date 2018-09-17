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

float CEMAV::flap_angle_to_pwm(float angle, uint8_t flap_number) {

	float pwm = 0;
	
	if(flap_number==1) {
		float pwm0deg =	_pwm0deg_1;	
		float pwm15deg = _pwm15deg_1;
		float pwm30deg = _pwm30deg_1;
		float pwm45deg = _pwm45deg_1;
		float pwm60deg = _pwm60deg_1;
		float pwm75deg = _pwm75deg_1;
		float pwm90deg = _pwm90deg_1;
	} else if (flap_number==2) {
		float pwm0deg =	_pwm0deg_2;	
		float pwm15deg = _pwm15deg_2;
		float pwm30deg = _pwm30deg_2;
		float pwm45deg = _pwm45deg_2;
		float pwm60deg = _pwm60deg_2;
		float pwm75deg = _pwm75deg_2;
		float pwm90deg = _pwm90deg_2;
	} else if (flap_number==3) {
		float pwm0deg =	_pwm0deg_3;
		float pwm15deg = _pwm15deg_3;
		float pwm30deg = _pwm30deg_3;
		float pwm45deg = _pwm45deg_3;
		float pwm60deg = _pwm60deg_3;
		float pwm75deg = _pwm75deg_3;
		float pwm90deg = _pwm90deg_3;
	} else if (flap_number==4) {
		float pwm0deg =	_pwm0deg_4;	
		float pwm15deg = _pwm15deg_4;
		float pwm30deg = _pwm30deg_4;
		float pwm45deg = _pwm45deg_4;
		float pwm60deg = _pwm60deg_4;
		float pwm75deg = _pwm75deg_4;
		float pwm90deg = _pwm90deg_4;
	} else if (flap_number==5) {
		float pwm0deg =	_pwm0deg_5;	
		float pwm15deg = _pwm15deg_5;
		float pwm30deg = _pwm30deg_5;
		float pwm45deg = _pwm45deg_5;
		float pwm60deg = _pwm60deg_5;
		float pwm75deg = _pwm75deg_5;
		float pwm90deg = _pwm90deg_5;
	} else if (flap_number==6) {
		float pwm0deg =	_pwm0deg_6;	
		float pwm15deg = _pwm15deg_6;
		float pwm30deg = _pwm30deg_6;
		float pwm45deg = _pwm45deg_6;
		float pwm60deg = _pwm60deg_6;
		float pwm75deg = _pwm75deg_6;
		float pwm90deg = _pwm90deg_6;
	} else if (flap_number==7) {
		float pwm0deg =	_pwm0deg_7;	
		float pwm15deg = _pwm15deg_7;
		float pwm30deg = _pwm30deg_7;
		float pwm45deg = _pwm45deg_7;
		float pwm60deg = _pwm60deg_7;
		float pwm75deg = _pwm75deg_7;
		float pwm90deg = _pwm90deg_7;
	} else if (flap_number==8) {
		float pwm0deg =	_pwm0deg_8;	
		float pwm15deg = _pwm15deg_8;
		float pwm30deg = _pwm30deg_8;
		float pwm45deg = _pwm45deg_8;
		float pwm60deg = _pwm60deg_8;
		float pwm75deg = _pwm75deg_8;
		float pwm90deg = _pwm90deg_8;
	}

	if (angle == 0) {
		pwm = pwm0deg;
	} else if (angle == 15) {
		pwm = pwm15deg;
	} else if (angle == 30) {
		pwm = pwm30deg;
	} else if (angle == 45) {
		pwm = pwm45deg;
	} else if (angle == 60) {
		pwm = pwm60deg;
	} else if (angle == 75) {
		pwm = pwm75deg;
	} else if (angle == 90) {
		pwm = pwm90deg;
	} else if (angle > 0 && angle < 15) {
		pwm = pwm0deg + angle/15*(pwm15deg - pwm0deg);
	} else if (angle > 15 && angle < 30) {
		pwm = pwm15deg + (angle - 15)/15*(pwm30deg - pwm15deg);
	} else if (angle > 30 && angle < 45) {
		pwm = pwm30deg + (angle - 30)/15 * (pwm45deg - pwm30deg);
	} else if (angle > 45 && angle < 60) {
		pwm = pwm45deg + (angle - 45)/15 * (pwm60deg - pwm45deg);
	} else if (angle > 60 && angle < 75) {
		pwm = pwm60deg + (angle - 60)/15 * (pwm75deg - pwm60deg);
	} else if (angle > 75 && angle < 90) {
		pwm = pwm75deg + (angle - 75)/15 * (pwm90deg - pwm75deg);
	} else if (angle < 0) {
		pwm = pwm0deg;
	} else if (angle > 90) {
		pwm = pwm90deg;
	} 

	return pwm;

}

float CEMAV::fishtail_angle_to_pwm(float angle)

	float pwm = 0;

	if (angle == -40) {
		pwm = _pwm_n40deg_ft;
	} else if (angle == -30) {
		pwm = _pwm_n30deg_ft;
	} else if (angle == -15) {
		pwm = _pwm_n15deg_ft;
	} else if (angle == 0) {
		pwm = _pwm_0deg_ft;
	} else if (angle == 15) {
		pwm = _pwm_15deg_ft;
	} else if (angle == 30) {
		pwm = _pwm_30deg_ft;
	} else if (angle == 40) {
		pwm = _pwm_40deg_ft;
	} else if (angle > -40 && angle < -30) {
		pwm = _pwm_n40deg_ft + angle/10*(_pwm_n30deg_ft - _pwm_n40deg_ft);
	} else if (angle > -30 && angle < -15) {
		pwm = _pwm_n30deg_ft + angle/15*(_pwm_n15deg_ft - _pwm_n30deg_ft);
	} else if (angle > -15 && angle < 0) {
		pwm = _pwm_n15deg_ft + angle/15*(_pwm_n0deg_ft - _pwm_n15deg_ft);
	} else if (angle > 0 && angle < 15) {
		pwm = _pwm_0deg_ft + angle/15*(_pwm_15deg_ft - _pwm_0deg_ft);
	} else if (angle > 15 && angle < 30) {
		pwm = _pwm_15deg_ft + angle/15*(_pwm_30deg_ft - _pwm_15deg_ft);
	} else if (angle > 30 && angle < 40) {
		pwm = _pwm_30deg_ft + angle/10*(_pwm_40deg_ft - _pwm_30deg_ft);
	} else if (angle < -40) {
		pwm = _pwm_n40deg_ft;
	} else if (angle > 40) {
		pwm = _pwm_40deg_ft;
	} 

	return pwm;

}
