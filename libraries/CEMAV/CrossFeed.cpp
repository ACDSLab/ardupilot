//
// Created by mgandhi3 on 3/19/19.
//

#include "CrossFeed.h"

const AP_Param::GroupInfo CrossFeed::var_info[] = {
        // @Param: CF_11
        // @DisplayName:
        // @Description: Crossfeed L mixer value: cf_L = input_L*_cf_11 + input_M*_cf_12;
        AP_GROUPINFO("CF_11", 0, CrossFeed, _cf_11, 1),

        // @Param: CR_12
        // @DisplayName:
        // @Description: Crossfeed L mixer value: cf_L = input_L*_cf_11 + input_M*_cf_12;
        AP_GROUPINFO("CF_12", 1, CrossFeed, _cf_12, 1),

        // @Param: CF_21
        // @DisplayName:
        // @Description: Crossfeed M mixer value: cf_L = input_L*_cf_11 + input_M*_cf_12;
        AP_GROUPINFO("CF_21", 2, CrossFeed, _cf_21, 1),

        // @Param: CF_22
        // @DisplayName:
        // @Description: Crossfeed M mixer value: cf_L = input_L*_cf_11 + input_M*_cf_12;
        AP_GROUPINFO("CF_22", 3, CrossFeed, _cf_22, 1),

        AP_GROUPEND
};

void CrossFeed::compute_crossfeed_moments(float input_L, float input_M, float& cf_L, float& cf_M) {
    cf_L = input_L*_cf_11 + input_M*_cf_12;
    cf_M = input_L*_cf_21 + input_M*_cf_22;
}
