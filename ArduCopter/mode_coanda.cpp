#include "Copter.h"

/*
 * Init and run calls for mode coanda flight mode
 */


// stabilize_init - initialise stabilize controller
bool Copter::ModeCoanda::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
        (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);


    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::ModeCoanda::run()
{
	
    // if not armed set throttle to zero and exit immediately
    if (!motors->armed())  {
        // zero_throttle_and_relax_ac();
	    SRV_Channels::set_output_pwm(SRV_Channel::k_motor6, 900);
        return;
    }
   
	// Get the pilot input from rudder channel: 4
	float yaw_rate_stick_norm = channel_yaw->norm_input_dz();

	float des_yaw = cemav->get_pilot_des_yaw_rate(yaw_rate_stick_norm);
    // Get the current state from the EKF

    // Use the PID controller in CEMAV.cpp to compute the output for the yaw rate controller
    float u_yaw_rate = cemav->compute_yaw_rate_control(des_yaw);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor5, u_yaw_rate);
//      SRV_Channels::set_output_pwm(SRV_Channel::k_motor5, des_yaw);

    // Get rpm value from RPM pin (the sensor is in AP_RPM)
    float curr_rpm = copter.rpm_sensor.get_rpm(0);

    // Get the pilot input percentage
    float throttle_stick_percent = channel_throttle->percent_input();

    float des_rpm = cemav->get_pilot_des_rpm(throttle_stick_percent);

    // Use the PID controller to compute the output for the rpm controller
    float u_rpm = cemav->compute_rpm_control(des_rpm, curr_rpm);
    SRV_Channels::set_output_scaled(SRV_Channel::k_motor6, u_rpm);

}

//int Copter::ModeCoanda::scale_input_to_pwm(input, input_range, center_input) {
//    double max_r_rate = 20;  // Assume the angular rate goes from -20 to 20
//    return (input / max_r_rate) * input_range + center_input;
//}


//// Basic servo controller based on attitude
//int u = _attitude_target_euler_angle[0] / 3.1412 * (400) + 1500;
//
//
//
//if (motors->armed()) {
//// Pass the controls to the servos without scaling. u[0] - u[4]
//SRV_Channels::set_output_pwm(SRV_Channel::k_aileron, u);
//}
