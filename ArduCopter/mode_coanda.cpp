#include "Copter.h"

/*
 * Init and run calls for mode coanda flight mode
 */


// stabilize_init - initialise stabilize controller
bool Copter::ModeCoanda::init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    //if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
    //    (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
    //    return false;
    //}
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
	    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, 900);
        return;
    }
	
	// clear landing flag
    set_land_complete(false);
   
	// Get the pilot input from rudder channel: 4
	float yaw_rate_stick_norm = channel_yaw->norm_input_dz();

	float des_yaw = cemav->get_pilot_des_yaw_rate(yaw_rate_stick_norm);
    // Get the current state from the EKF

    // Use the PID controller in CEMAV.cpp to compute the output for the yaw rate controller
    float u_yaw_rate = cemav->compute_yaw_rate_control(des_yaw);
    SRV_Channels::set_output_scaled(SRV_Channel::k_cemav_rudder, u_yaw_rate);

    // Get rpm value from RPM pin (the sensor is in AP_RPM)
    float curr_rpm = copter.rpm_sensor.get_rpm(0); // RPM in centi revolutions per minute

    // Get the pilot input percentage
    uint8_t throttle_stick_percent = channel_throttle->percent_input();  // This gives us something 0 - 100

    float des_rpm = cemav->get_pilot_des_rpm(throttle_stick_percent);  // Desired RPM

    // Use the PID controller to compute the output for the rpm controller
    float u_rpm = cemav->compute_rpm_control(des_rpm, curr_rpm); // Kp * (RPM_error)
    float u_pwm = constrain_value(u_rpm, (float) 0, (float) 1) * (channel_throttle->get_radio_max() - channel_throttle->get_radio_min()) + channel_throttle->get_radio_min();

    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, (int) u_pwm);


	// Add manual "passthrough pwm" flap control
	SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap1, channel_roll->get_radio_in());
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap2, channel_pitch->get_radio_in());
	SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap3, channel_roll->get_radio_in());
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap4, channel_pitch->get_radio_in());


    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap5, (int) des_rpm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap6, (int) u_rpm*9000.0);


    SRV_Channels::calc_pwm();

    SRV_Channels::output_ch_all();
}

