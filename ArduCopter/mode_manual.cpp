#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::ModeManual::init(bool ignore_checks)
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

// manual_run - runs the main manual function that passes rc input as motor commands
// should be called at 100hz or more
void Copter::ModeManual::run()
{

    // if not armed set throttle to zero and exit immediately
    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
        zero_throttle_and_relax_ac();
        return;
    }

    // clear landing flag
    set_land_complete(false);

    // Get the command from the roll channel
//    auto roll = channel_roll->get_control_in_zero_dz();
//    auto pitch = channel_pitch->get_control_in_zero_dz();
//    auto yaw = channel_yaw->get_control_in_zero_dz();
//    auto throttle = channel_throttle->get_control_in_zero_dz();


    // From a function called void Plane::set_servos_manual_passthrough(void) in servos.cpp in ArduPlane
    SRV_Channels::set_output_scaled(SRV_Channel::k_aileron, channel_roll->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_elevator, channel_pitch->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_rudder, channel_yaw->get_control_in_zero_dz());
    SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, channel_throttle->get_control_in_zero_dz());

    AP_Vehicle::MultiCopter &aparm = copter.aparm;



//    float target_roll, target_pitch;
//    float target_yaw_rate;
//    float pilot_throttle_scaled;
//
//    // if not armed set throttle to zero and exit immediately
//    if (!motors->armed() || ap.throttle_zero || !motors->get_interlock()) {
//        zero_throttle_and_relax_ac();
//        return;
//    }
//
//    // clear landing flag
//    set_land_complete(false);
//
//    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
//
//    // apply SIMPLE mode transform to pilot inputs
//    update_simple_mode();
//
//    AP_Vehicle::MultiCopter &aparm = copter.aparm;
//
//    // convert pilot input to lean angles
//    get_pilot_desired_lean_angles(target_roll, target_pitch, aparm.angle_max, aparm.angle_max);
//
//    // get pilot's desired yaw rate
//    target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
//
//    // get pilot's desired throttle
//    pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());
//
//    // call attitude controller
//    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
//
//    // body-frame rate controller is run directly from 100hz loop
//
//    // output pilot's throttle
//    attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
}
