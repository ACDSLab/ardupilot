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


    return true;
}

// manual_run - runs the main manual function that passes rc input as motor commands
// should be called at 100hz or more
void Copter::ModeManual::run()
{

   // if not armed set throttle to zero and exit immediately
   if (!motors->armed() ) {
       // zero_throttle_and_relax_ac();
	   SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, 900);
       return;
   }

    // clear landing flag
    set_land_complete(false);

    float throttle_stick_percent = channel_throttle->percent_input();

    float des_rpm = cemav->get_pilot_des_rpm(throttle_stick_percent);


    // From a function called void Plane::set_servos_manual_passthrough(void) in servos.cpp in ArduPlane
    // channel_roll to channel_throttle are channels on the radio. We get the controls from rc_in, and then
    // pass them to the respect servo channels k_rcin1 - 4
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap1, channel_roll->get_radio_in());
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap2, channel_pitch->get_radio_in());
	SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap3, channel_roll->get_radio_in());
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap4, channel_pitch->get_radio_in());
	
	
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_rudder, channel_yaw->get_radio_in());
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_throttle, channel_throttle->get_radio_in());

    // DEBUG
    float curr_rpm = copter.rpm_sensor.get_rpm(0);
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap5, (int) curr_rpm);
    SRV_Channels::set_output_pwm(SRV_Channel::k_cemav_flap6, (int) des_rpm);




}
