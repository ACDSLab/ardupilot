//
// Created by mgandhi3 on 9/17/18.
//

#include "Servo_Cal.h"

const AP_Param::GroupInfo Flap::var_info[] = {
        // @Param: PWM_0
        // @DisplayName: PWM for 0 Deg Flap deflection
        // @Description: Output PWM value for 0 degree flap deflection.
        AP_GROUPINFO("PWM_0",    0, Flap, _pwm_0deg, 900),

        // @Param: PWM_15
        // @DisplayName: PWM for 15 Deg Flap deflection
        // @Description: Output PWM value for 15 degree flap deflection.
        AP_GROUPINFO("PWM_15",    1, Flap, _pwm_15deg, 900),

        // @Param: PWM_30
        // @DisplayName: PWM for 30 Deg Flap deflection
        // @Description: Output PWM value for 30 degree flap deflection.
        AP_GROUPINFO("PWM_30",    2, Flap, _pwm_30deg, 900),

        // @Param: PWM_45
        // @DisplayName: PWM for 45 Deg Flap deflection
        // @Description: Output PWM value for 45 degree flap deflection.
        AP_GROUPINFO("PWM_45",    3, Flap, _pwm_45deg, 900),

        // @Param: PWM_60
        // @DisplayName: PWM for 60 Deg Flap deflection
        // @Description: Output PWM value for 60 degree flap deflection.
        AP_GROUPINFO("PWM_60",    4, Flap, _pwm_60deg, 900),

        // @Param: PWM_75
        // @DisplayName: PWM for 75 Deg Flap deflection
        // @Description: Output PWM value for 75 degree flap deflection.
        AP_GROUPINFO("PWM_75",    5, Flap, _pwm_75deg, 900),

        // @Param: PWM_90
        // @DisplayName: PWM for 90 Deg Flap deflection
        // @Description: Output PWM value for 90 degree flap deflection.
        AP_GROUPINFO("PWM_90",    6, Flap, _pwm_90deg, 900),


        AP_GROUPEND
};

const AP_Param::GroupInfo Rudder::var_info[] = {
        // @Param: PWM_N40
        // @DisplayName: PWM for -40 Deg rudder deflection
        // @Description: Output PWM value for -40 degree rudder deflection.
        AP_GROUPINFO("PWM_N40",    0, Rudder, _pwm_n40deg, 900),

        // @Param: PWM_N30
        // @DisplayName: PWM for -30 Deg rudder deflection
        // @Description: Output PWM value for -30 degree rudder deflection.
        AP_GROUPINFO("PWM_N30",    1, Rudder, _pwm_n30deg, 900),

        // @Param: PWM_N15
        // @DisplayName: PWM for -15 Deg rudder deflection
        // @Description: Output PWM value for -15 degree rudder deflection.
        AP_GROUPINFO("PWM_N15",    2, Rudder, _pwm_n15deg, 900),

        // @Param: PWM_0
        // @DisplayName: PWM for 0 Deg rudder deflection
        // @Description: Output PWM value for 0 degree rudder deflection.
        AP_GROUPINFO("PWM_0",    3, Rudder, _pwm_0deg, 900),

        // @Param: PWM_15
        // @DisplayName: PWM for 15 Deg rudder deflection
        // @Description: Output PWM value for 15 degree rudder deflection.
        AP_GROUPINFO("PWM_15",    4, Rudder, _pwm_15deg, 900),

        // @Param: PWM_30
        // @DisplayName: PWM for 30 Deg rudder deflection
        // @Description: Output PWM value for 30 degree rudder deflection.
        AP_GROUPINFO("PWM_30",    5, Rudder, _pwm_30deg, 900),

        // @Param: PWM_40
        // @DisplayName: PWM for 40 Deg rudder deflection
        // @Description: Output PWM value for 40 degree rudder deflection.
        AP_GROUPINFO("PWM_40",    6, Rudder, _pwm_40deg, 900),


        AP_GROUPEND
};

uint16_t Flap::flap_angle_to_pwm(float angle) {
    uint16_t pwm = 0;


	if (angle >= 0 && angle < 15) {
		pwm = _pwm_0deg + (angle/15)*(_pwm_15deg - _pwm_0deg);
	} else if (angle >= 15 && angle < 30) {
		pwm = _pwm_15deg + ((angle - 15)/15)*(_pwm_30deg - _pwm_15deg);
	} else if (angle >= 30 && angle < 45) {
		pwm = _pwm_30deg + ((angle - 30)/15) * (_pwm_45deg - _pwm_30deg);
	} else if (angle >= 45 && angle < 60) {
		pwm = _pwm_45deg + ((angle - 45)/15) * (_pwm_60deg - _pwm_45deg);
	} else if (angle >= 60 && angle < 75) {
		pwm = _pwm_60deg + ((angle - 60)/15) * (_pwm_75deg - _pwm_60deg);
	} else if (angle >= 75 && angle <= 90) {
		pwm = _pwm_75deg + ((angle - 75)/15) * (_pwm_90deg - _pwm_75deg);
	} else if (angle < 0) {
		pwm = _pwm_0deg;
	} else if (angle > 90) {
		pwm = _pwm_90deg;
	}

	return pwm;
}

uint16_t Rudder::rudder_angle_to_pwm(float angle) {
    uint16_t pwm = 0;


	if (angle >= -40 && angle < -30) {
		pwm = _pwm_n40deg + ((angle + 40)/10)*(_pwm_n30deg - _pwm_n40deg);
	} else if (angle >= -30 && angle < -15) {
		pwm = _pwm_n30deg + ((angle + 30)/15)*(_pwm_n15deg - _pwm_n30deg);
	} else if (angle >= -15 && angle < 0) {
		pwm = _pwm_n15deg + ((angle + 15)/15)*(_pwm_0deg - _pwm_n15deg);
	} else if (angle >= 0 && angle < 15) {
		pwm = _pwm_0deg + (angle/15)*(_pwm_15deg - _pwm_0deg);
	} else if (angle >= 15 && angle < 30) {
		pwm = _pwm_15deg + ((angle - 15)/15)*(_pwm_30deg - _pwm_15deg);
	} else if (angle >= 30 && angle <= 40) {
		pwm = _pwm_30deg + ((angle - 30)/10)*(_pwm_40deg - _pwm_30deg);
	} else if (angle < -40) {
		pwm = _pwm_n40deg;
	} else if (angle > 40) {
		pwm = _pwm_40deg;
	}

	return pwm;
}


//
//float CEMAV::fishtail_angle_to_pwm(float angle) {
//
//	float pwm = 0;
//
//	if (angle == -40) {
//		pwm = _pwm_n40deg_ft;
//	} else if (angle == -30) {
//		pwm = _pwm_n30deg_ft;
//	} else if (angle == -15) {
//		pwm = _pwm_n15deg_ft;
//	} else if (angle == 0) {
//		pwm = _pwm_0deg_ft;
//	} else if (angle == 15) {
//		pwm = _pwm_15deg_ft;
//	} else if (angle == 30) {
//		pwm = _pwm_30deg_ft;
//	} else if (angle == 40) {
//		pwm = _pwm_40deg_ft;
//	} else if (angle > -40 && angle < -30) {
//		pwm = _pwm_n40deg_ft + angle/10*(_pwm_n30deg_ft - _pwm_n40deg_ft);
//	} else if (angle > -30 && angle < -15) {
//		pwm = _pwm_n30deg_ft + angle/15*(_pwm_n15deg_ft - _pwm_n30deg_ft);
//	} else if (angle > -15 && angle < 0) {
//		pwm = _pwm_n15deg_ft + angle/15*(_pwm_n0deg_ft - _pwm_n15deg_ft);
//	} else if (angle > 0 && angle < 15) {
//		pwm = _pwm_0deg_ft + angle/15*(_pwm_15deg_ft - _pwm_0deg_ft);
//	} else if (angle > 15 && angle < 30) {
//		pwm = _pwm_15deg_ft + angle/15*(_pwm_30deg_ft - _pwm_15deg_ft);
//	} else if (angle > 30 && angle < 40) {
//		pwm = _pwm_30deg_ft + angle/10*(_pwm_40deg_ft - _pwm_30deg_ft);
//	} else if (angle < -40) {
//		pwm = _pwm_n40deg_ft;
//	} else if (angle > 40) {
//		pwm = _pwm_40deg_ft;
//	}
//
//	return pwm;
//
//}
