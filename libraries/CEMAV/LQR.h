//
// Created by mgandhi3 on 9/17/18.
//
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>


#ifndef ARDUPILOT_LQR_H
#define ARDUPILOT_LQR_H

class LQR {
public:
    static const struct AP_Param::GroupInfo var_info[];

    void compute_control_pq(float cur_p, float cur_q, float des_p, float des_q, float (&flap_angles)[4]);


protected:
    AP_Float        _lq_11;
    AP_Float        _lq_12;
    AP_Float        _lq_21;
    AP_Float        _lq_22;
    AP_Float        _lq_31;
    AP_Float        _lq_32;
    AP_Float        _lq_41;
    AP_Float        _lq_42;
};

#endif
