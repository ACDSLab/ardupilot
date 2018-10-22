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

        AP_SUBGROUPINFO(_pid_v_pitch, "V_PIT_", 10, CEMAV, AC_PID),

        AP_SUBGROUPINFO(_pid_v_roll, "V_ROL_", 11, CEMAV, AC_PID),


        AP_GROUPEND
};

// Compute the nonlinear cross term and the gyroscopic term that we want to eliminate
void DI::compute_g_x(float curr_p, float curr_q, float curr_r, float curr_omega, float (&g)[2]) {
    g[0] = (_I_Bz - _I_By)*curr_q*curr_r + _I_Rz*curr_omega*curr_q;
    g[1] = (_I_Bx - _I_Bz)*curr_p*curr_r - _I_Rz*curr_omega*curr_p;
}

void DI::compute_des_moments(float curr_p, float des_p,
                             float curr_q, float des_q,
                             float curr_r, float curr_omega,
                             float (&moments)[2]) {
    // Compute pseudoinput v from PID Compensators
    float p_error = des_p - curr_p; // diff in rad/sec
    _pid_v_roll.set_input_filter_d(p_error); // Filter the error signal

    float q_error = des_q - curr_q; // difference in pitch rate rad/sec
    _pid_v_pitch.set_input_filter_d(q_error);

    // Compute g(x)
    float g[2];
    compute_g_x(curr_p, curr_q, curr_r, curr_omega, g);

    moments[0] = _pid_v_roll.get_pid() + g[0];
    moments[1] = _pid_v_pitch.get_pid() + g[1];
}

void DI::moments_to_flapangles(float curr_rud_angle_rad, float curr_omega, float (&moments)[2], float (&angles)[4]) {
    // Compute C_2 based on current rudder_angle
    float C_2 = compute_c2(curr_omega, curr_rud_angle_rad);

    // The flaps are paired together, each flap corresponds to a roll or pitch moment, positive or negative
    if (moments[0]  > 0) { // Check roll and change flaps 2 and 4
        // Positive roll moment Flap 2 comes out and Flap 4 is all the way in
        float u4 = cosf(0 - theta);
        float u2 = -1*(moments[1] / C_1 / C_2 - u4);
        angles[3] = 0;
        angles[1] = (acosf(u2 + theta)) * RAD_TO_DEG;
    } else {
        // Negative roll moment Flap 4 comes out and Flap 2 is all the way in
        float u2 = cosf(0 - theta);
        float u4 = -1*(moments[1] / C_1 / C_2 - u4);
        angles[1] = 0;
        angles[2] = (acosf(u4 + theta)) * RAD_TO_DEG;
    }

    if (moments[1]  > 0) { // Check pitch and change flaps 1 and 3
        // Positive pitch moment Flap 3 comes out and Flap 1 is all the way in
        float u1 = cosf(0 - theta);
        float u3 = -1*(moments[1] / C_1 / C_2 - u1);
        angles[0] = 0;
        angles[2] = (acosf(u3 + theta)) * RAD_TO_DEG;
    } else {
        // Negative pitch moment Flap 1 comes out and Flap 3 is all the way in
        float u3 = cosf(0 - theta);
        float u1 = (moments[1] / C_1 / C_2 + u1);
        angles[2] = 0;
        angles[0] = (acosf(u1 + theta)) * RAD_TO_DEG;

    }
}

void DI::compute_control_pq(float curr_p, float des_p,
                            float curr_q, float des_q,
                            float curr_r, float curr_omega,
                            float curr_rud_angle_rad, float (&angles)[4]) {
    // We have a desired, and current rate for desired pitch and roll, use the compensator to get the desired moments
    float moments[2];
    compute_des_moments(curr_p, des_p, curr_q, des_q, curr_r, curr_omega, moments);

    // Compute the desired flap angles
    moments_to_flapangles(curr_rud_angle_rad, curr_omega, moments, angles)
}

float DI::compute_c2(float curr_omega, float curr_rud_angle_rad) {
    return _K_z * curr_omega * curr_omega * cosf(curr_rud_angle_rad + _S_z); // cosine in radians
}