//
// Created by mgandhi3 on 3/19/19.
//

#include "CrossFeed.h"

const AP_Param::GroupInfo CrossFeed::var_info[] = {
        // @Param: DAMP_C
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("DAMP_C", 0, CrossFeed, _c, 1),

        // @Param: MOTOR_H
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("MOTOR_H", 1, CrossFeed, _Hr, 1),

        // @Param: IB
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("IB", 2, CrossFeed, _Ib, 1),

        // @Param: LAT_SC
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LAT_SC", 3, CrossFeed, _lateral_scale, 1),

        // @Param: LON_SC
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LON_SC", 4, CrossFeed, _longitudinal_scale, 1),

        AP_GROUPEND
};

void CrossFeed::compute_crossfeed_moments(float input_L, float input_M, float& cf_L, float& cf_M) {
    float mu = _Hr / _Ib;
    cf_L = (_c*_c) / (_c*_c + mu*mu ) * input_L*_lateral_scale + (_c*mu) / (_c*_c + mu*mu ) * input_M*_longitudinal_scale;
    cf_M = (-_c*mu) / (_c*_c + mu*mu ) * input_L*_lateral_scale + ( _c*_c) / (_c*_c + mu*mu ) * input_M*_longitudinal_scale;
}
