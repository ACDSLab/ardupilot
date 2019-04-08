//
// Created by mgandhi3 on 9/17/18.
//

#include "LQR.h"

const AP_Param::GroupInfo LQR::var_info[] = {
        // @Param: LQ_11
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_11", 0, LQR, _lq_11, 1.4279),

        // @Param: LQ_12
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_12", 1, LQR, _lq_12, -3.4472),

        // @Param: LQ_21
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_21", 2, LQR, _lq_21, 3.4472),

        // @Param: LQ_22
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_22", 3, LQR, _lq_22, -1.4279),

        // @Param: LQ_31
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_31", 4, LQR, _lq_31, 3.4472),

        // @Param: LQ_32
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_32", 5, LQR, _lq_32, 1.4279),

        // @Param: LQ_41
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_41", 6, LQR, _lq_41, 1.4279),

        // @Param: LQ_42
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_42", 7, LQR, _lq_42, 3.4472),
		
		// @Param: LQ_51
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_51", 8, LQR, _lq_51, -1.4279),

        // @Param: LQ_52
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_52", 9, LQR, _lq_52, 3.4472),

        // @Param: LQ_61
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_61", 10, LQR, _lq_61, -3.4472),

        // @Param: LQ_62
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_62", 11, LQR, _lq_62, 1.4279),

        // @Param: LQ_71
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_71", 12, LQR, _lq_71, -3.4472),

        // @Param: LQ_72
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_72", 13, LQR, _lq_72, -1.4279),

        // @Param: LQ_81
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_81", 14, LQR, _lq_81, -1.4279),

        // @Param: LQ_82
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_82", 15, LQR, _lq_82, -3.4472),
		
		// @Param: FLAP_TRIM
        // @DisplayName:
        // @Description:
		AP_GROUPINFO("FLAP_TRIM", 16, LQR, _flap_trim_angle, 30.0f),

        // @Param: LQ_M_11
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_TWO_11", 17, LQR, _lq_twostate_11, -3.4472),

        // @Param: LQ_M_12
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_TWO_12", 18, LQR, _lq_twostate_12, -1.4279),

        // @Param: LQ_M_21
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_TWO_21", 19, LQR, _lq_twostate_21, -1.4279),

        // @Param: LQ_M_22
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("LQ_TWO_22", 20, LQR, _lq_twostate_22, -3.4472),

		
        AP_GROUPEND
};

void LQR::compute_flaps_pq(float cur_p, float cur_q, float des_p, float des_q, float (&flap_angles)[8]) {
    float err_p = des_p - cur_p; // Difference is in rad/sec
    float err_q = des_q - cur_q; // Difference is in rad/sec
    flap_angles[0] = (_lq_11*err_p + _lq_12*err_q) + _flap_trim_angle;
    flap_angles[1] = (_lq_21*err_p + _lq_22*err_q) + _flap_trim_angle;
    flap_angles[2] = (_lq_31*err_p + _lq_32*err_q) + _flap_trim_angle;
    flap_angles[3] = (_lq_41*err_p + _lq_42*err_q) + _flap_trim_angle;
    flap_angles[4] = (_lq_51*err_p + _lq_52*err_q) + _flap_trim_angle;
    flap_angles[5] = (_lq_61*err_p + _lq_62*err_q) + _flap_trim_angle;
    flap_angles[6] = (_lq_71*err_p + _lq_72*err_q) + _flap_trim_angle;
    flap_angles[7] = (_lq_81*err_p + _lq_82*err_q) + _flap_trim_angle;
}

void LQR::compute_twostate_pq(float cur_p, float cur_q, float des_p, float des_q, float (&commands)[2]) {
    // float err_p = des_p - cur_p; // Difference is in rad/sec
    // float err_q = des_q - cur_q; // Difference is in rad/sec

    // commands[0] = (_lq_twostate_11*err_p + _lq_twostate_12*err_q);
    // commands[1] = (_lq_twostate_21*err_p + _lq_twostate_22*err_q);
	commands[0] = (_lq_twostate_11*cur_p + _lq_twostate_12*cur_q) + des_p;
    commands[1] = (_lq_twostate_21*cur_p + _lq_twostate_22*cur_q) + des_q;
}

//void LQR::compute_fourstate_pq(float cur_p, float cur_q, float des_p, float des_q, float (&commands)[2]){
//
//}
//
//void LQR::compute_integral_pq(float cur_p, float cur_q, float des_p, float des_q, float (&commands)[2]) {
//
//}
