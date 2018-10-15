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

    void compute_g_x(float curr_p, float curr_q, float curr_r, float curr_omega, float (&g)[2]);
    double compute_c2(float curr_omega, float curr_rudd_angle_deg);

protected:
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

    double C_1 = sqrtf(_R/8*_R/8 + (2 - sqrtf(2.0)) * _rz * _rz/ (M_2PI*M_2PI));
    double theta = atan2f(-1*sqrtf((2 - sqrtf(2.0))) * _rz / M_2PI, _R / 8 );
    double C_2;
};
#endif //ARDUPILOT_DYNAMIC_INVERSION_H
