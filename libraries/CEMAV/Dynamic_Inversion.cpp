//
// Created by mgandhi3 on 4/9/19.
//

#include "Dynamic_Inversion.h"

const AP_Param::GroupInfo Dynamic_Inversion::var_info[] = {
        // @Param: DI_IXY
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("DI_IXY", 0, Dynamic_Inversion, _Ixy, 0.15),

        // @Param: DI_KPQ
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("DI_KPQ", 1, Dynamic_Inversion, _Kpq, 2.25),

        // @Param: DI_IR
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("DI_IR", 2, Dynamic_Inversion, _Ir, 0.0017),

        // @Param: DI_LM_MAX
        // @DisplayName:
        // @Description:
        AP_GROUPINFO("DI_LM_MAX", 3, Dynamic_Inversion, _LM_max, 0.0017),

        AP_GROUPEND
};

void Dynamic_Inversion::compute_DI_pq(float cur_p, float cur_q, float cur_omega, float p_dot_c, float q_dot_c, float (&commands)[2]) {
    // commands[0] = (_Kpq*cur_p + _Ir*cur_omega*cur_q + _Ixy*p_dot_c) / _LM_max;
    // commands[1] = (_Kpq*cur_q - _Ir*cur_omega*cur_p + _Ixy*q_dot_c) / _LM_max;
	
	// Fit of pitch/roll control moment at max flap deflection to motor speed (rad/s, assumes constant 17 degree fishtail trim)
	// Max moment in absolute terms is 3.54Nm@6000rpm, 4.94Nm@7000rpm, 6.57Nm@8000rpm
	float LM_max = 0.00001052*cur_omega*cur_omega - 0.0009282*cur_omega; 
	commands[0] = (_Ir*cur_omega*cur_q + p_dot_c) / LM_max;
    commands[1] = (-_Ir*cur_omega*cur_p + q_dot_c) / LM_max;
}


