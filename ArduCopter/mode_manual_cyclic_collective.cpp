#include "Copter.h"


/*
 * Init and run calls for manual flight mode
 */

// manual_init - initialise manual controller
bool Copter::ModeManualCF::init(bool ignore_checks)
{
    // // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    // if (motors->armed() && ap.land_complete && !copter.flightmode->has_manual_throttle() &&
            // (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        // return false;
    // }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);


    return true;
}


// manual_run - runs the main manual function that passes rc input as motor commands
// should be called at 100hz or more
void Copter::ModeManualCF::run()
{

    // clear landing flag
    set_land_complete(false);


	/**************************
	* Yaw Rate Controller
	***************************/
	// Get the pilot input from rudder channel: 4
	float yaw_rate_stick_norm = channel_yaw->norm_input_dz();  // -1 to 1

	float des_yaw = cemav->get_pilot_des_yaw_rate(yaw_rate_stick_norm); // -2pi rad per sec to 2pi

	// Use the PID controller in CEMAV.cpp to compute the output for the yaw rate controller
	float u_rudder_angle = cemav->compute_yaw_rate_control(des_yaw); // (Kp * (yaw_rate_error) + Ki * int(yaw_rate_error)) - _yaw_trim_angle


    /**************************
    * Cross Feed Inputs
    ***************************/
    // The input_L and input_M are going to be scaled lateral and longidinal commands (respectively)
	// Servo Cal Flaps -> stick goes from min flap angle to max flap angle
	float lateral_command = channel_roll->norm_input_dz();
	float longitudinal_command = -channel_pitch->norm_input_dz();


	// Declare the crossfed moment commands
    float cf_L;
    float cf_M;
    // compute the crossfed moment commands
    cemav->compute_crossfeed_LM(lateral_command, longitudinal_command, cf_L, cf_M);

    cf_L = constrain_value(cf_L, (float) -1, (float) 1);
    cf_M = constrain_value(cf_M, (float) -1, (float) 1);

    /********************************************
    * Collective Command from throttle stick
    ********************************************/
    float beta_collective = 1 - channel_throttle->norm_input_dz();  // Throttle down is 1 (full flaps out), throttle up is 0, (full flaps in)


    /*******************************
    * Combine cyclic with collective
    ********************************/
    float F1_c, F2_c, F3_c, F4_c, F5_c, F6_c, F7_c, F8_c;
    // Compute how much range is available for the flaps to increase or decrease
    float flap_increase_range = cemav->get_max_flap_angle()/90.0 - beta_collective;
    float flap_decrease_range = beta_collective - cemav->get_min_flap_angle()/90.0; // TODO check this!!
    float available_range = (flap_increase_range > flap_decrease_range) ? flap_decrease_range : flap_increase_range;

    // Longitudinal moment
    if (available_range >= std::abs(cf_M / 2.0)) { // We can split the moment differential equally between the two pairs of flaps
        F1_c = cemav->rescale_flaps(beta_collective - (cf_M / 2.0));
        F8_c = F1_c;

        F4_c = cemav->rescale_flaps(beta_collective + (cf_M / 2.0));
        F5_c = F4_c;
    } else { // One flap must be closed or open fully, the other flap should take up the rest of the differential
        // Fore and Aft Flap Pairs
        F1_c = cemav->rescale_flaps(constrain_value(-cf_M, (float) 0, (float) 1));
        F8_c = F1_c;

        F4_c = cemav->rescale_flaps(constrain_value(cf_M, (float) 0, (float) 1));
        F5_c = F4_c;
    }

    // Lateral moment
    if (available_range >= std::abs(cf_L / 2.0)) { // We can split the moment differential equally between the two pairs of flaps
        F2_c = cemav->rescale_flaps(beta_collective + (cf_L / 2.0));
        F3_c = F2_c;

        F6_c = cemav->rescale_flaps(beta_collective - (cf_L / 2.0));
        F7_c = F6_c;
    } else { // One flap must be closed or open fully, the other flap should take up the rest of the differential
        // Port and Starboard Flap Pairs
        F2_c = cemav->rescale_flaps(constrain_value(cf_L, (float) 0, (float) 1));
        F3_c = F2_c;

        F6_c = cemav->rescale_flaps(constrain_value(-cf_L, (float) 0, (float) 1));
        F7_c = F6_c;
    }




    if (counter >= cemav->get_control_counter()) {
        counter = 1;

		   // if not armed set throttle to zero and exit immediately
		if (!motors->armed() ) {
			// zero_throttle_and_relax_ac();
			SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, 900);
			// Removed the return statement, because we want to be able to move the control surfaces even without arming.
		} else {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, channel_collective->get_radio_in());
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
