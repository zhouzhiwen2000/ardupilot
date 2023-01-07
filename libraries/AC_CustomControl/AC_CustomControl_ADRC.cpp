#include "AC_CustomControl_ADRC.h"

#if CUSTOMCONTROL_ADRC_ENABLED
#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC::var_info[] = {
    // @Param: RAT_RLL_WC
    // @DisplayName: ADRC roll axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_RLL_WO
    // @DisplayName: ADRC roll axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_RLL_B0
    // @DisplayName: ADRC roll axis control input gain
    // @User: Advanced

    // @Param: RAT_RLL_DELT
    // @DisplayName: ADRC roll axis control linear zone length
    // @User: Advanced

    // @Param: RAT_RLL_ORDR
    // @DisplayName: ADRC roll axis control model order
    // @User: Advanced

    // @Param: RAT_RLL_LM
    // @DisplayName: ADRC roll axis control output limit
    // @User: Advanced
    AP_SUBGROUPINFO(_roll_cont, "RAT_RLL_", 1, AC_CustomControl_ADRC, AC_ADRC),

    // @Param: RAT_PIT_WC
    // @DisplayName: ADRC pitch axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_PIT_WO
    // @DisplayName: ADRC pitch axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_PIT_B0
    // @DisplayName: ADRC pitch axis control input gain
    // @User: Advanced

    // @Param: RAT_PIT_DELT
    // @DisplayName: ADRC pitch axis control linear zone length
    // @User: Advanced

    // @Param: RAT_PIT_ORDR
    // @DisplayName: ADRC pitch axis control model order
    // @User: Advanced

    // @Param: RAT_PIT_LM
    // @DisplayName: ADRC pitch axis control output limit
    // @User: Advanced
    AP_SUBGROUPINFO(_pitch_cont, "RAT_PIT_", 2, AC_CustomControl_ADRC, AC_ADRC),

    // @Param: RAT_YAW_WC
    // @DisplayName: ADRC yaw axis control bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_YAW_WO
    // @DisplayName: ADRC yaw axis ESO bandwidth(rad/s)
    // @User: Advanced

    // @Param: RAT_YAW_B0
    // @DisplayName: ADRC yaw axis control input gain
    // @User: Advanced

    // @Param: RAT_YAW_DELT
    // @DisplayName: ADRC yaw axis control linear zone length
    // @User: Advanced

    // @Param: RAT_YAW_ORDR
    // @DisplayName: ADRC yaw axis control model order
    // @User: Advanced

    // @Param: RAT_YAW_LM
    // @DisplayName: ADRC yaw axis control output limit
    // @User: Advanced

    AP_GROUPEND
};

// initialize in the constructor
AC_CustomControl_ADRC::AC_CustomControl_ADRC(AC_CustomControl &frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt),
    _roll_cont(dt),
    _pitch_cont(dt)
{    
    AP_Param::setup_object_defaults(this, var_info);
}

// update controller
// return roll, pitch, yaw controller output
Vector3f AC_CustomControl_ADRC::update(void)
{
    // reset controller based on spool state
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            // We are still at the ground. Reset custom controller to avoid
            // build up, ex: integrator
            reset();
            break;

        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            // we are off the ground
            break;
    }

    // Active disturbance rejection controller
    // only rate control is implemented
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    attitude_target = _att_control->get_attitude_target_quat();
    Vector3f gyro_latest = _ahrs->get_gyro_latest();
    Vector3f error_att;
    error_att = calculate_att_error(attitude_target, attitude_body);
    float bth =  _motors->get_throttle_hover();
    Vector3f motor_out;

    motor_out.x = _roll_cont.update_all(error_att.x, gyro_latest.x, bth);
    motor_out.y = _pitch_cont.update_all(error_att.y, gyro_latest.y, bth);
    motor_out.z = _motors->get_yaw();
    static uint32_t tlast = 0;
    uint32_t tnow = AP_HAL::micros();
    if (tnow - tlast > 5000000)
    {
        gcs().send_text(MAV_SEVERITY_INFO, "ADRC controller working");//send notification every 5 seconds
        tlast = tnow;
    }
    return motor_out;
}

Vector3f AC_CustomControl_ADRC::calculate_att_error(Quaternion target, Quaternion measurment)
{   
    Quaternion att_cur_quat;

    Quaternion _att_target_quat = target;
    att_cur_quat = measurment;

    Vector3f e_cur_z, e_des_z;    
    
    Matrix3f att_cur_matrix;
    att_cur_quat.rotation_matrix(att_cur_matrix);

    e_cur_z = att_cur_matrix.colz();

    Matrix3f att_target_rot_matrix;
    _att_target_quat.rotation_matrix(att_target_rot_matrix);
    e_des_z = att_target_rot_matrix.colz();

    // the cross product of the desired and target thrust vector defines the rotation vector
    Vector3f thrust_correction_vec_cross = e_cur_z % e_des_z;

    // the dot product is used to calculate the angle between the target and desired thrust vectors
    float thrust_correction_vec_dot = acosf(constrain_float(e_cur_z * e_des_z, -1.0f, 1.0f));

    // Normalize the thrust rotation vector
    float thrust_correction_vec_length = thrust_correction_vec_cross.length();
    if (is_zero(thrust_correction_vec_length) || is_zero(thrust_correction_vec_dot)) {
        thrust_correction_vec_cross = Vector3f(0, 0, 1);
        thrust_correction_vec_dot = 0.0f;
    } else {
        thrust_correction_vec_cross /= thrust_correction_vec_length;
    }

    Quaternion thrust_vec_correction_quat;
    thrust_vec_correction_quat.from_axis_angle(thrust_correction_vec_cross, thrust_correction_vec_dot);

    // Rotate thrust_vec_correction_quat to the body frame
    thrust_vec_correction_quat = att_cur_quat.inverse() * thrust_vec_correction_quat * att_cur_quat;

    // calculate the remaining rotation required after thrust vector is rotated transformed to the body frame
    Quaternion yaw_vec_correction_quat = thrust_vec_correction_quat.inverse() * att_cur_quat.inverse() * _att_target_quat;    

    // calculate the angle error in x and y.
    Vector3f rotation, error_att;
    thrust_vec_correction_quat.to_axis_angle(rotation);
    error_att.x = rotation.x;
    error_att.y = rotation.y;

    // calculate the angle error in z (x and y should be zero here).
    yaw_vec_correction_quat.to_axis_angle(rotation);
    error_att.z = rotation.z;

    return error_att;
}


// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
// TODO: this doesn't result in bumpless transition. 
void AC_CustomControl_ADRC::reset(void)
{
    _roll_cont.reset();
    _pitch_cont.reset();
}

#endif
