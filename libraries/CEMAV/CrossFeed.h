//
// Created by mgandhi3 on 3/19/19.
//

#ifndef ARDUPILOT_CROSSFEED_H
#define ARDUPILOT_CROSSFEED_H

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>


class CrossFeed {
public:
    static const struct AP_Param::GroupInfo var_info[];

    void compute_crossfeed_moments(float input_L, float input_M, float& cf_L, float& cf_M);

protected:
    AP_Float        _cf_l1;
    AP_Float        _cf_12;
    AP_Float        _cf_21;
    AP_Float        _cf_22;

};


#endif //ARDUPILOT_CROSSFEED_H
