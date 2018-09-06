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
	// Get the pilot input from rudder channel: 4
    // Get the state feedback gains from the copter parameters
	int16_t yaw_rate_stick = channel_yaw->get_control_in_zero_dz();
	float des_yaw = ModeCoanda::CEMAV_Vehicle::get_pilot_des_yaw_rate(yaw_rate_stick);
    // Get the current state from the EKF
//    Vector3f _attitude_target_euler_angle = Vector3f(ahrs.roll, ahrs.pitch, ahrs.yaw);
    Vector3f _body_rates = ahrs.get_gyro();

    // Get rpm value from RPM pin (the sensor is in AP_RPM)
//    float rpm_value = rpm_sensor.get_rpm(0);


    // Design a proporational controller for yaw rate. The setpoint is zero for this example
    float r = _body_rates[2];  // current yaw rate
    float des_r = des_yaw;  // desired yaw rate

    double K_r = 10; // gain on yaw rate proportional control
    int u = K_r*(des_r - r) / COANDA_MAX_RATE * 400 + 1500;
    SRV_Channels::set_output_pwm(SRV_Channel::k_aileron,
            constrain_value(u, 1100, 1900));


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
