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


    if (counter == cemav->get_control_counter()) {
        counter = 1;
        /**************************
        * Yaw Rate Controller
        ***************************/
        // Get the pilot input from rudder channel: 4
        float yaw_rate_stick_norm = channel_yaw->norm_input_dz();  // -1 to 1

        float des_yaw = cemav->get_pilot_des_yaw_rate(yaw_rate_stick_norm); // -2pi rad per sec to 2pi

        // Use the PID controller in CEMAV.cpp to compute the output for the yaw rate controller
        float u_rudder_angle = cemav->compute_yaw_rate_control(des_yaw); // (Kp * (yaw_rate_error) + Ki * int(yaw_rate_error))  - _yaw_trim_angle

        // Set the rudder PWM
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_rudder, cemav->rudder_angle_to_pwm(u_rudder_angle));



        /**************************
        * RPM Controller
        ***************************/
        // Get rpm value from RPM pin (the sensor is in AP_RPM)
        float curr_rpm = copter.rpm_sensor.get_rpm(0); // RPM in centi revolutions per minute

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
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, channel_throttle->get_radio_in());


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

        /**************************
        * Roll and Pitch DI
        ***************************/
        // Get the pilot input from pitch channel
        float q_stick_norm = -1 * channel_pitch->norm_input_dz();  // -1 to 1 The stick is reversed!
        float p_stick_norm = channel_roll->norm_input_dz();  // -1 to 1
        float des_q = cemav->get_pilot_des_q(q_stick_norm); // rad/sec
        float des_p = cemav->get_pilot_des_p(p_stick_norm); // rad/sec
        //
        float flap_angles[4];
        // Compute the control on the rates
        cemav->compute_control_pq(des_p, des_q, flap_angles);
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap1, cemav->flap_angle_to_pwm(flap_angles[0], 1));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap2, cemav->flap_angle_to_pwm(flap_angles[1], 2));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap3, cemav->flap_angle_to_pwm(flap_angles[2], 3));
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap4, cemav->flap_angle_to_pwm(flap_angles[3], 4));

        /**************************
        * Debug printing
        ***************************/
        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap5, (int) curr_rpm);
//        SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap6, (int) des_rpm);
    } else {
        counter += 1;
    }

}
