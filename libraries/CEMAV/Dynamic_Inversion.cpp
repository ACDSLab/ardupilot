//
// Created by mgandhi3 on 10/15/18.
//

#include "Dynamic_Inversion.h"

const AP_Param::GroupInfo DI::var_info[] = {
        // @Param: I_Bx
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("I_Bx", 0, DI, _I_Bx, 0),

        // @Param: I_By
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("I_By", 1, DI, _I_By, 0),

        // @Param: I_Bz
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("I_Bz", 2, DI, _I_Bz, 0),

        // @Param: I_Rz
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("I_Rz", 3, DI, _I_Rz, 0),

        // @Param: K_z
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("K_z", 4, DI, _K_z, 0),

        // @Param: K_n
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("K_n", 5, DI, _K_n, 0),

        // @Param: S_z
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("S_z", 6, DI, _S_z, 0),

        // @Param: S_n
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("S_n", 7, DI, _S_n, 0),

        // @Param: R
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("R", 8, DI, _R, 0),

        // @Param: rz
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("rz", 9, DI, _rz, 0),


        AP_GROUPEND
};

// Compute the nonlinear cross term and the gyroscopic term that we want to eliminate
void DI::compute_g_x(float curr_p, float curr_q, float curr_r, float curr_omega, float (&g)[2]) {
    g[0] = (_I_Bz - _I_By)*curr_q*curr_r + _I_Rz*curr_omega*curr_q;
    g[1] = (_I_Bx - _I_Bz)*curr_p*curr_r - _I_Rz*curr_omega*curr_p;
}

double DI::compute_c2(float curr_omega, float curr_rudd_angle_deg) {
    return _K_z * curr_omega * curr_omega * cosf(curr_rudd_angle_deg + _S_z); // cosine in radians
}