#include "Copter.h"

/*
 * Init and run calls for mode pqfeedback flight mode
 */


// stabilize_init - initialise stabilize controller
bool Copter::ModePQFeedback::init(bool ignore_checks)
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
void Copter::ModePQFeedback::run()
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
	float u_rudder_angle = cemav->compute_yaw_rate_control(des_yaw); // (Kp * (yaw_rate_error) + Ki * int(yaw_rate_error))  - _yaw_trim_angle


	/**************************
	* RPM Controller
	***************************/
	// Get rpm value from RPM pin (the sensor is in AP_RPM)
//	float curr_rpm = copter.rpm_sensor.get_rpm(0); // RPM in centi revolutions per minute

/*    // Get the pilot input percentage
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
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, u_pwm);*/


	/**************************
	* Roll and Pitch Pass-through
	***************************/
	// Servo Cal Flaps
	// float roll_flap_input = 90*channel_roll->norm_input_dz();
	// float pitch_flap_input = 90*channel_pitch->norm_input_dz();
	//
	// SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap1, cemav->flap_angle_to_pwm(roll_flap_input, 1));
	// SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap2, cemav->flap_angle_to_pwm(pitch_flap_input, 2));
	// SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap3, cemav->flap_angle_to_pwm(-1*roll_flap_input, 3));
	// SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap4, cemav->flap_angle_to_pwm(-1*pitch_flap_input, 4));

	/********************************
	* Roll and Pitch PQ Feedback
	*********************************/
	// Get the pilot input from pitch channel
	float q_stick_norm = -1 * channel_pitch->norm_input_dz();  // -1 to 1 The stick is reversed!
	float p_stick_norm = channel_roll->norm_input_dz();  // -1 to 1
	float des_q = cemav->get_pilot_des_q(q_stick_norm); // rad/sec
	float des_p = cemav->get_pilot_des_p(p_stick_norm); // rad/sec
	//

	/* If we wanted to compute the flap angle values directly (crossfeed in its current form doesn't work for this case)
	float flap_angles[8];
	cemav->pq_feedback_flaps(des_p, des_q, flap_angles);
    */

	// Compute the longitudinal and lateral commands using pq feedback
	float commands[2];
	cemav->pq_feedback_commands(des_p, des_q, commands);

    // Declare the crossfed moment commands
    float cf_L;
    float cf_M;
	// Perform crossfeed on the desired inputs
    cemav->compute_crossfeed_LM(commands[0], commands[1], cf_L, cf_M);
    // Fore and Aft Flap Pairs
    float F1_c = cemav->rescale_flaps(constrain_value(-cf_M, (float) 0, (float) 1));
    float F8_c = F1_c;

    float F4_c = cemav->rescale_flaps(constrain_value(cf_M, (float) 0, (float) 1));
    float F5_c = F4_c;

    // Port and Starboard Flap Pairs
    float F2_c = cemav->rescale_flaps(constrain_value(cf_L, (float) 0, (float) 1));
    float F3_c = F2_c;

    float F6_c = cemav->rescale_flaps(constrain_value(-cf_L, (float) 0, (float) 1));
    float F7_c = F6_c;
		
	if (counter >= cemav->get_control_counter()) {
        counter = 1;
		
		    // if not armed set throttle to zero and exit immediately
		if (!motors->armed())  {
			// zero_throttle_and_relax_ac();
			SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, 900);
			// return;
		} else {
			SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, channel_throttle->get_radio_in());
		}
		
		// Set the rudder PWM
		SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_rudder, cemav->rudder_angle_to_pwm(u_rudder_angle));
		
//		float F1_c = constrain_value(flap_angles[0], (float) 30, (float) 90);
//		float F2_c = constrain_value(flap_angles[1], (float) 30, (float) 90);
//		float F3_c = constrain_value(flap_angles[2], (float) 30, (float) 90);
//		float F4_c = constrain_value(flap_angles[3], (float) 30, (float) 90);
//		float F5_c = constrain_value(flap_angles[4], (float) 30, (float) 90);
//		float F6_c = constrain_value(flap_angles[5], (float) 30, (float) 90);
//		float F7_c = constrain_value(flap_angles[6], (float) 30, (float) 90);
//		float F8_c = constrain_value(flap_angles[7], (float) 30, (float) 90);

        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap1, cemav->flap_angle_to_pwm(F1_c, 1));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap2, cemav->flap_angle_to_pwm(F2_c, 2));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap3, cemav->flap_angle_to_pwm(F3_c, 3));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap4, cemav->flap_angle_to_pwm(F4_c, 4));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap5, cemav->flap_angle_to_pwm(F5_c, 5));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap6, cemav->flap_angle_to_pwm(F6_c, 6));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap7, cemav->flap_angle_to_pwm(F7_c, 7));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap8, cemav->flap_angle_to_pwm(F8_c, 8));

        /**************************
        * Debug printing
        ***************************/
//        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap5, (int) curr_rpm);
//        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap6, (int) des_rpm);
    } else {
        counter += 1;
    }

}
