//
// Created by mgandhi3 on 4/9/19.
//

#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#ifndef ARDUPILOT_DYNAMIC_INVERSION_H
#define ARDUPILOT_DYNAMIC_INVERSION_H


class Dynamic_Inversion {

public:
    static const struct AP_Param::GroupInfo var_info[];

    void compute_DI_pq(float cur_p, float cur_q, float cur_omega, float p_dot_c, float q_dot_c, float (&commands)[2]);


protected:
    AP_Float        _Ixy;  // x-y body inertia of the vehicle
    AP_Float        _Kpq;      // pq damping
    AP_Float        _Ir;       // Rotor inertia
    AP_Float        _LM_max;  // Max L and M moment generated by vehicle


};


#endif //ARDUPILOT_DYNAMIC_INVERSION_H
