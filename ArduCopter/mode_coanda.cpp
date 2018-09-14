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


    /**************************
    * Yaw Rate Controller
    ***************************/
	// Get the pilot input from rudder channel: 4
	float yaw_rate_stick_norm = channel_yaw->norm_input_dz();  // -1 to 1

	float des_yaw = cemav->get_pilot_des_yaw_rate(yaw_rate_stick_norm); // -720 deg per sec to 720

    // Use the PID controller in CEMAV.cpp to compute the output for the yaw rate controller
    float u_yaw_rate = cemav->compute_yaw_rate_control(des_yaw); // (Kp * (yaw_rate_error) + Ki * int(yaw_rate_error)) / _yaw_control_scale

    // Convert u_yaw rate to a PWM.
//    uint16_t yaw_r_min_pwm = SRV_Channels::srv_channel(5)->get_output_min();
    uint16_t yaw_r_max_pwm = SRV_Channels::srv_channel(5)->get_output_max();
    uint16_t yaw_r_trim_pwm = SRV_Channels::srv_channel(5)->get_trim();

    // Get the rpm fraction then scale it by the pwm range, finally constrain the change in throttle to be between 100
    float u_yaw_rate_bounded = constrain_value(u_yaw_rate, (float)-1, (float) 1); // Bound the yaw rate


    // Bounded yaw rate will never result in a negative float being converted to unsigned integer.
    uint16_t u_yaw_rate_pwm = u_yaw_rate_bounded* (yaw_r_max_pwm - yaw_r_trim_pwm) + yaw_r_trim_pwm;

    // Set rudder angle pwm
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_rudder, u_yaw_rate_pwm);


    /**************************
    * RPM Controller
    ***************************/
    // Get rpm value from RPM pin (the sensor is in AP_RPM)
    float curr_rpm = copter.rpm_sensor.get_rpm(0); // RPM in centi revolutions per minute

    // Get the pilot input percentage
    uint8_t throttle_stick_percent = channel_throttle->percent_input();  // This gives us something 0 - 100
    float des_rpm = cemav->get_pilot_des_rpm(throttle_stick_percent);  // Desired RPM


    // Use the PID controller to compute the output for the rpm controller
    float u_rpm = cemav->compute_rpm_control(des_rpm, curr_rpm); // (Kp * (RPM_error) + Ki * int(RPM_error)) / _rpm_control_scale


    // Convert du_rpm to a PWM change,
    uint16_t thr_min_pwm = SRV_Channels::srv_channel(4)->get_output_min();
    uint16_t thr_max_pwm = SRV_Channels::srv_channel(4)->get_output_max();

    // Make sure that the new throttle value is greater than 0 before casting to uint16
    uint16_t u_pwm = constrain_value((int) (u_rpm * (thr_max_pwm - thr_min_pwm) + thr_min_pwm), (int)thr_min_pwm, (int)thr_max_pwm);

    // Set the throttle pwm
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, u_pwm);

    /**************************
    * Roll and Pitch Pass-through
    ***************************/
	// Add manual "passthrough pwm" flap control
	SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap1, channel_roll->get_radio_in());
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap2, channel_pitch->get_radio_in());
	SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap3, channel_roll->get_radio_in());
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap4, channel_pitch->get_radio_in());

    /**************************
    * Debug printing
    ***************************/
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap5, (int) curr_rpm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap6, (int) des_rpm);

}

