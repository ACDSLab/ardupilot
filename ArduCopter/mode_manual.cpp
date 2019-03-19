#include "Copter.h"


/*
 * Init and run calls for manual flight mode
 */

// manual_init - initialise manual controller
bool Copter::ModeManual::init(bool ignore_checks)
{
    // // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    // if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            // (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        // return false;
    // }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    // Set the min and max angle from cemav
    max_angle = cemav->get_max_flap_angle();
    min_angle = cemav->get_min_flap_angle();


    return true;
}

float Copter::ModeManual::rescale_flaps(float input_command) {
    return (max_angle - min_angle) * input_command + min_angle;
}

// manual_run - runs the main manual function that passes rc input as motor commands
// should be called at 100hz or more
void Copter::ModeManual::run()
{

    // clear landing flag
    set_land_complete(false);

        // From a function called void Plane::set_servos_manual_passthrough(void) in servos.cpp in ArduPlane
        // channel_roll to channel_throttle are channels on the radio. We get the controls from rc_in, and then
        // pass them to the respect servo channels k_rcin1 - 4
//    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap1, channel_roll->get_radio_in()); // roll +
//    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap2, channel_pitch->get_radio_in()); // pitch +
//	SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap3, channel_roll->get_radio_in());  // roll -
//    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap4, channel_pitch->get_radio_in()); // pitch -


//    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_rudder, channel_yaw->get_radio_in());


        // Servo Cal Rudder!
//    float yaw_angle_input =  40*(channel_yaw->norm_input_dz()); // -40 to 40
//    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_rudder, cemav->rudder_angle_to_pwm(yaw_angle_input));

	/**************************
	* Yaw Rate Controller
	***************************/
	// Get the pilot input from rudder channel: 4
	float yaw_rate_stick_norm = channel_yaw->norm_input_dz();  // -1 to 1

	float des_yaw = cemav->get_pilot_des_yaw_rate(yaw_rate_stick_norm); // -2pi rad per sec to 2pi

	// Use the PID controller in CEMAV.cpp to compute the output for the yaw rate controller
	float u_rudder_angle = cemav->compute_yaw_rate_control(des_yaw); // (Kp * (yaw_rate_error) + Ki * int(yaw_rate_error)) - _yaw_trim_angle

/*     // Convert u_yaw rate to a PWM.
//    uint16_t yaw_r_min_pwm = SRV_Channels::srv_channel(5)->get_output_min();
    uint16_t yaw_r_max_pwm = SRV_Channels::srv_channel(5)->get_output_max();
    uint16_t yaw_r_trim_pwm = SRV_Channels::srv_channel(5)->get_trim();

    // Get the rpm fraction then scale it by the pwm range, finally constrain the change in throttle to be between 100
    float u_yaw_rate_bounded = -1*constrain_value(u_yaw_rate, (float)-1, (float) 1); // Bound the yaw rate

    // Bounded yaw rate will never result in a negative float being converted to unsigned integer.
    uint16_t u_yaw_rate_pwm = u_yaw_rate_bounded* (yaw_r_max_pwm - yaw_r_trim_pwm) + yaw_r_trim_pwm;

    // Set rudder angle pwm
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_rudder, u_yaw_rate_pwm); */

	// Servo Cal Flaps -> stick goes from min flap angle to max flap angle
	float lateral_command = channel_roll->norm_input_dz();
	float longitudinal_command = channel_pitch->norm_input_dz();
	// Fore and Aft Flap Pairs
	float F1_c = rescale_flaps(constrain_value(-lateral_command, (float) 0, (float) 1));
	float F8_c = F1_c;

	float F4_c = rescale_flaps(constrain_value(lateral_command, (float) 0, (float) 1));
	float F5_c = F4_c;

	// Port and Starboard Flap Pairs
	float F2_c = rescale_flaps(constrain_value(longitudinal_command, (float) 0, (float) 1));
	float F3_c = F2_c;

	float F6_c = rescale_flaps(constrain_value(-longitudinal_command, (float) 0, (float) 1));
	float F7_c = F6_c;
		
    if (counter >= cemav->get_control_counter()) {
        counter = 1;
		
		   // if not armed set throttle to zero and exit immediately
		if (!motors->armed() ) {
			// zero_throttle_and_relax_ac();
			SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, 900);
			// Removed the return statement, because we want to be able to move the control surfaces even without arming.
		} else {
			SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, channel_throttle->get_radio_in());
		}
		
		SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_rudder, cemav->rudder_angle_to_pwm(u_rudder_angle));

        // Set the output PWM's for the 8 flap vehicle
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap1, cemav->flap_angle_to_pwm(F1_c, 1));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap2, cemav->flap_angle_to_pwm(F2_c, 2));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap3, cemav->flap_angle_to_pwm(F3_c, 3));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap4, cemav->flap_angle_to_pwm(F4_c, 4));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap5, cemav->flap_angle_to_pwm(F5_c, 5));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap6, cemav->flap_angle_to_pwm(F6_c, 6));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap7, cemav->flap_angle_to_pwm(F7_c, 7));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap8, cemav->flap_angle_to_pwm(F8_c, 8));

    } else {
        counter += 1;
    }




}
