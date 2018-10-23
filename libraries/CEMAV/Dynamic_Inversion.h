//
// Created by mgandhi3 on 10/15/18.
//

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h> // Define parameters
#include <AP_Math/AP_Math.h> // Use common math functions
#include <AP_AHRS/AP_AHRS_View.h> // Get a view of the ekf
#include <AC_PID/AC_PID.h>  // Get the PID controller class
#include <SRV_Channel/SRV_Channel.h> // Set servo angle max

#ifndef ARDUPILOT_DYNAMIC_INVERSION_H
#define ARDUPILOT_DYNAMIC_INVERSION_H

class DI {
public:
    static const struct AP_Param::GroupInfo var_info[];

    // Constructor
    DI(float dt);

    // Compute the control for roll and pitch rate
    void compute_control_pq(float curr_p, float des_p,
                            float curr_q, float des_q,
                            float curr_r, float curr_omega,
                            float curr_rud_angle_rad, float (&angles)[4]);
private:
    // Compute the nonlinearity
    void compute_g_x(float curr_p, float curr_q, float curr_r, float curr_omega, float (&g)[2]);

    // Compute a constant based on saturated rudder angle
    float compute_c2(float curr_omega, float curr_rudd_angle_deg);

    // Get desired moments from current and desired rates
    void compute_des_moments(float curr_p, float des_p,
                             float curr_q, float des_q,
                             float curr_r, float curr_omega,
                             float (&moments)[2]);

    // Convert desired moments to flap angles
    void moments_to_flapangles(float curr_rud_angle_rad, float curr_omega, float (&moments)[2], float (&angles)[4]);

protected:
    // Parameters for the system
    AP_Float _I_Bx;
    AP_Float _I_By;
    AP_Float _I_Bz;
    AP_Float _I_Rz;

    AP_Float _K_z;
    AP_Float _K_n;
    AP_Float _S_z;
    AP_Float _S_n;

    AP_Float _R;
    AP_Float _rz;

    // Constants used in the dynamic inversion
	float a = R / 8;
	float b = -1 * sqrtf(2 - sqrtf(2.0)) * _rz / M_2PI;
    // float C_1 = sqrtf(_R/8*_R/8 + (2 - sqrtf(2.0)) * _rz * _rz/ (M_2PI*M_2PI));
	float C_1 = sqrtf(a*a + b*b);
	float theta = atan2f(b, a);
    // float theta = atan2f(-1*sqrtf((2 - sqrtf(2.0))) * _rz / M_2PI, _R / 8 ); // radians

    // PID Compensator on Rate that outputs pseudocontrol v
    AC_PID _pid_v_pitch;
    AC_PID _pid_v_roll;
};
#endif //ARDUPILOT_DYNAMIC_INVERSION_H
