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
        AP_GROUPINFO("RAT_MAX_R", 0, CEMAV, _max_yaw_ds, 720.0f),

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

        AP_SUBGROUPINFO(_lqr, "RAT_", 17, CEMAV, LQR),

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

        AP_SUBGROUPINFO(_pid_nil_pitch, "AT_NI_PI_", 21, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_pid_nil_roll, "AT_NI_RO_", 22, CEMAV, AC_PID),

        AP_GROUPINFO("COUNTER", 23, CEMAV, _count_max, 4),

        AP_GROUPINFO("MAX_FLAP_ANG", 25, CEMAV, _max_flap_angle, 90.0f),

        AP_GROUPINFO("MIN_FLAP_ANG", 26, CEMAV, _min_flap_angle, 30.0f),

        AP_SUBGROUPINFO(_cf, "LM_", 27, CEMAV, CrossFeed),
		
		AP_SUBGROUPINFO(_pid_rate_lat, "RAT_LAT_", 28, CEMAV, AC_PID),

		AP_SUBGROUPINFO(_pid_rate_long, "RAT_LONG_", 29, CEMAV, AC_PID),
		
		AP_SUBGROUPINFO(_pid_yaw, "AT_YAW_", 30, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_pid_il_pitch, "AT_IL_PI_", 31, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_pid_il_roll, "AT_IL_RO_", 32, CEMAV, AC_PID),

        AP_GROUPINFO("AT_CTRL", 33, CEMAV, _att_controller_type, 0),

        AP_GROUPINFO("RAT_CTRL", 34, CEMAV, _rate_controller_type, 1),

        AP_SUBGROUPINFO(_pid_di_lat, "RAT_DI_P_", 35, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_pid_di_long, "RAT_DI_Q_", 36, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_di, "RAT_", 37, CEMAV, Dynamic_Inversion),


        AP_GROUPEND

};

// CEMAV Constructor
CEMAV::CEMAV(AP_AHRS_View &ahrs, float dt) :
    _ahrs(ahrs),
    _pid_rate_yaw(1, 1, 0, 0.5, 5, dt),
    _pid_rate_lat(1, 1, 0, 0.5, 5, dt),
    _pid_rate_long(1, 1, 0, 0.5, 5, dt),
    _pid_di_lat(1, 1, 0, 0.5, 5, dt),
    _pid_di_long(1, 1, 0, 0.5, 5, dt),
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




float CEMAV::compute_rpm_control(float des_rpm, float curr_rpm) {
    float error = (des_rpm - curr_rpm);
    _pid_rpm.set_input_filter_all(error);

    return (_pid_rpm.get_pid()) / _rpm_control_scale; // Compute then scale the output control
}

// Define functions to compute the PWM values for given flap deflections.

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


// Yaw rate controller PID
float CEMAV::compute_yaw_rate_control(float des_yaw_rate) {
    // Get the current yaw rate
    float curr_yaw_rate = _ahrs.get_gyro()[2];

    // Get the desired yaw rate
    float error = des_yaw_rate - curr_yaw_rate; // diff in rad/sec
    _pid_rate_yaw.set_input_filter_d(error); // Filter the error signal

    return (_pid_rate_yaw.get_pid()); // Compute then scale the output control
}



// Define functions to compute commands for state feedback

void CEMAV::pq_feedback_flaps(float des_p, float des_q, float (&flap_angles)[8]) {
    float cur_p = _ahrs.get_gyro()[0];
    float cur_q = _ahrs.get_gyro()[1];
    _lqr.compute_flaps_pq(cur_p, cur_q, des_p, des_q, flap_angles);
}

// Rate controller

void CEMAV::compute_pq_rate_commands(float des_lat_rate, float des_long_rate, float cur_rpm, float (&commands)[2],
                                     int rate_ctrl) {

    float cur_p = _ahrs.get_gyro()[0];
    float cur_q = _ahrs.get_gyro()[1];
	
	// Compute the derivatives of the p and q_dot_c
	uint32_t t1 = AP_HAL::micros();  // time of current measurement
    p_dot_filter.update(cur_p, t1);
	q_dot_filter.update(cur_q, t1);
	
	float curr_p_dot = p_dot_filter.slope();
	float curr_q_dot - q_dot_filter.slope();
	
    int rate_switch;
    if (rate_ctrl == 0) {
        rate_switch = _rate_controller_type;
    } else {
        rate_switch = rate_ctrl;
    }

    switch (rate_switch) {
        case 1: { // PID Inner Loop
            // PID controller takes in desired lat_rate and des_long_rate, and outputs p_dot_c and q_dot_c
            float lat_error = des_lat_rate - cur_p; // diff in rad/sec
            float long_error = des_long_rate - cur_q; // diff in rad/sec

            _pid_rate_lat.set_input_filter_d(lat_error); // Filter the error signal
            _pid_rate_long.set_input_filter_d(long_error); // Filter the error signal

            // Set and then compute the angular velocity PID terms, the output is L_c, and M_c
            commands[0] = _pid_rate_lat.get_pid(); // L_c
            commands[1] = _pid_rate_long.get_pid();  // M_c
            break;
        }

        case 2: { // LQR state feedback inner loop
            _lqr.compute_twostate_pq(cur_p, cur_q, des_lat_rate, des_long_rate, commands);
            break;
        }

        case 3: { // NDI inner loop stage 1
            float cur_omega = float(cur_rpm) * 2 * M_PI / 60.0; // Cur RPM is in Rev per minute convert to rad/s

            // Compute the commands based on sensor data
            _di.compute_DI_pq(cur_p, cur_q, cur_omega, des_lat_rate, des_long_rate, commands);
            break;
			
		case 4: { // NDI inner loop stage 2 (omega_dot feedback)
            float cur_omega = float(cur_rpm) * 2 * M_PI / 60.0; // Cur RPM is in Rev per minute convert to rad/s

            // PID controller takes in desired lat_rate and des_long_rate, and outputs p_dot_c and q_dot_c
            float lat_error = des_lat_rate - curr_p_dot; // diff in rad/sec
            float long_error = des_long_rate - curr_q_dot; // diff in rad/sec

            _pid_di_lat.set_input_filter_d(lat_error);
            _pid_di_long.set_input_filter_d(long_error);

            float p_dot_c = _pid_di_lat.get_pid() + des_lat_rate;
            float q_dot_c = _pid_di_long.get_pid() + des_long_rate;

            // Compute the commands based on PID compensator output and sensor data
            _di.compute_DI_pq(cur_p, cur_q, cur_omega, des_lat_rate, des_long_rate, commands);
            break;
        }

        default: {
            commands[0] = 0;
            commands[1] = 0;
            break;
        }
    }
}


// Attitude controller

void CEMAV::compute_pitch_roll_commands(float des_pitch, float des_roll, float cur_rpm, float (&commands)[2]) {
    float err_pitch = des_pitch - _ahrs.pitch;
    float err_roll = des_roll - _ahrs.roll;

    if (_att_controller_type == 0) {
        // No inner loop
        // Set and then compute the angular velocity PID terms, the output is L_c, and M_c
        _pid_nil_pitch.set_input_filter_all(err_pitch);
        _pid_nil_roll.set_input_filter_all(err_roll);
        commands[0] = _pid_nil_roll.get_pid(); // L_c
        commands[1] = _pid_nil_pitch.get_pid();  // M_c
    } else { // 1: PID inner loop, 2: Statefeedback inner loop, 3: DI inner loop 1, 4: DI inner loop 2
        // Set and then compute the att PID terms, the output is desired rates in p and q
        _pid_il_pitch.set_input_filter_all(err_pitch);
        _pid_il_roll.set_input_filter_all(err_roll);

        float des_lat_rate = _pid_il_roll.get_pid();
        float des_long_rate = _pid_il_pitch.get_pid();

        compute_pq_rate_commands(des_lat_rate, des_long_rate, cur_rpm, commands, _att_controller_type);
    }

}

// Crossfeed functions
void CEMAV::compute_crossfeed_LM(float lat_c, float lon_c, float& cf_L, float& cf_M) {
    _cf.compute_crossfeed_moments(lat_c, lon_c, cf_L, cf_M);
}

// Flap scaling for inputs
float CEMAV::rescale_flaps(float input_command) {
    return (_max_flap_angle - _min_flap_angle) * input_command + _min_flap_angle;
}
