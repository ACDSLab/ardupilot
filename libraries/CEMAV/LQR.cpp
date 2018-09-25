//
// Created by mgandhi3 on 9/17/18.
//

#include "LQR.h"

const AP_Param::GroupInfo LQR::var_info[] = {
        // @Param: LQ_11
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_11", 0, LQR, _lq_11, 0),

        // @Param: LQ_12
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_12", 1, LQR, _lq_12, 0),

        // @Param: LQ_21
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_21", 2, LQR, _lq_21, 0),

        // @Param: LQ_22
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_22", 3, LQR, _lq_22, 0),

        // @Param: LQ_31
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_31", 4, LQR, _lq_31, 0),

        // @Param: LQ_32
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_32", 5, LQR, _lq_32, 0),

        // @Param: LQ_41
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_41", 6, LQR, _lq_41, 0),

        // @Param: LQ_42
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_42", 7, LQR, _lq_42, 0),


        AP_GROUPEND
};

void LQR::compute_control_pq(float cur_p, float cur_q, float des_p, float des_q, float (&flap_angles)[4]) {
  float err_p = des_p - cur_p; // Difference is in rad/sec
  float err_q = des_q - cur_q; // Difference is in rad/sec
  flap_angles[0] = -(_lq_11*err_p + _lq_12*err_q);
  flap_angles[1] = -(_lq_21*err_p + _lq_22*err_q);
  flap_angles[2] = -(_lq_31*err_p + _lq_32*err_q);
  flap_angles[3] = -(_lq_41*err_p + _lq_42*err_q);
}
