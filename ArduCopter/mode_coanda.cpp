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
    // Get the state feedback gains from the copter parameters

    // Get the current state from the EKF
    Vector3f _attitude_target_euler_angle = Vector3f(ahrs.roll, ahrs.pitch, ahrs.yaw);
//    Vector3f _body_rates = ahrs.get_gyro();

    // Get rpm value from RPM pin (the sensor is in AP_RPM)
//    float rpm_value = rpm_sensor.get_rpm(0);


    // Compute the controls using the state feedback gain matrix (LQR) u = K*x

    int u = _attitude_target_euler_angle[0] / 3.1412 * (400) + 1500;



    if (motors->armed()) {
        // Pass the controls to the servos without scaling. u[0] - u[4]
        SRV_Channels::set_output_pwm(SRV_Channel::k_aileron, u);
    }


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
