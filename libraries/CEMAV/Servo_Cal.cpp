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

