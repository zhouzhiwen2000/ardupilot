#include "AC_CustomControl_ADRC.h"

#if CUSTOMCONTROL_EMPTY_ENABLED

#include <GCS_MAVLink/GCS.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_ADRC::var_info[] = {
    // @Param: ADRC_P_b
    // @DisplayName: b for pitch
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_b", 1, AC_CustomControl_ADRC, P_b, 5.5f),

    // @Param: PARAM2
    // @DisplayName: Empty param2
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_P1", 2, AC_CustomControl_ADRC, P_P1, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_P2", 3, AC_CustomControl_ADRC, P_P2, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_P3", 4, AC_CustomControl_ADRC, P_P3, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_P4", 5, AC_CustomControl_ADRC, P_P4, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_TD4P1", 6, AC_CustomControl_ADRC, P_TD4P1, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_TD4P2", 7, AC_CustomControl_ADRC, P_TD4P2, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_TD4P3", 8, AC_CustomControl_ADRC, P_TD4P3, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_P_TD4P4", 9, AC_CustomControl_ADRC, P_TD4P4, 25.0f),

    // @Param: ADRC_R_b
    // @DisplayName: b for roll
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_b", 1, AC_CustomControl_ADRC, R_b, 5.5f),

    // @Param: PARAM2
    // @DisplayName: Empty param2
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_P1", 2, AC_CustomControl_ADRC, R_P1, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_P2", 3, AC_CustomControl_ADRC, R_P2, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_P3", 4, AC_CustomControl_ADRC, R_P3, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_P4", 5, AC_CustomControl_ADRC, R_P4, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_TD4P1", 6, AC_CustomControl_ADRC, R_TD4P1, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_TD4P2", 7, AC_CustomControl_ADRC, R_TD4P2, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_TD4P3", 8, AC_CustomControl_ADRC, R_TD4P3, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_R_TD4P4", 9, AC_CustomControl_ADRC, R_TD4P4, 25.0f),

   // @Param: ADRC_Y_b
    // @DisplayName: b for yaw
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_b", 1, AC_CustomControl_ADRC, Y_b, 1.0f),

    // @Param: PARAM2
    // @DisplayName: Empty param2
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_P1", 2, AC_CustomControl_ADRC, Y_P1, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_P2", 3, AC_CustomControl_ADRC, Y_P2, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_P3", 4, AC_CustomControl_ADRC, Y_P3, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_P4", 5, AC_CustomControl_ADRC, Y_P4, 15.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_TD4P1", 6, AC_CustomControl_ADRC, Y_TD4P1, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_TD4P2", 7, AC_CustomControl_ADRC, Y_TD4P2, 10.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_TD4P3", 8, AC_CustomControl_ADRC, Y_TD4P3, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Y_TD4P4", 9, AC_CustomControl_ADRC, Y_TD4P4, 25.0f),

    // @Param: PARAM3
    // @DisplayName: Empty param3
    // @Description: Dumy parameter for empty custom controller backend
    // @User: Advanced
    AP_GROUPINFO("ADRC_Motor_T", 9, AC_CustomControl_ADRC, Motor_T, 0.1f),

    AP_GROUPEND
};

Quaternion Quaternion_get_PRQuat( Quaternion quat )
{
    float tqx;
    float tqy;
    float tqw;
    
    float qw2 = quat.q1 * quat.q1;
//    float qx2 = quat.q2 * quat.q2;
//    float qy2 = quat.q3 * quat.q3;
    float qz2 = quat.q4 * quat.q4;
    float qwx = quat.q1 * quat.q2;
    float qwy = quat.q1 * quat.q3;
//    float qwz = quat.q1 * quat.q4;
//    float qxy = quat.q2 * quat.q3;
    float qxz = quat.q2 * quat.q4;
    float qyz = quat.q3 * quat.q4;
    
    float qw2Pqz2 = ( qw2 + qz2 );
    if( !is_zero( qw2Pqz2 ) )
    {		
        tqw = safe_sqrt( qw2Pqz2 );
        float inv_tqw = 1.0f / tqw;
        tqx = ( qwx + qyz ) * inv_tqw;
        tqy = ( qwy - qxz ) * inv_tqw;					
    }
    else
    {
        tqw = 0.0f;
        tqx = quat.q2;	tqy = quat.q3;
    }
    Quaternion result = { tqw , tqx , tqy , 0 };
    return result;
}

// initialize in the constructor
AC_CustomControl_ADRC::AC_CustomControl_ADRC(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl_Multi*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    AP_Param::setup_object_defaults(this, var_info);
    disturbance_filter[0].set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(),10);
    disturbance_filter[1].set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(),10);
    disturbance_filter[2].set_cutoff_frequency(AP::scheduler().get_loop_rate_hz(),10);
    //初始化期望TD4滤波器
    Target_tracker[0].init( R_TD4P1 , R_TD4P2 , R_TD4P3 , R_TD4P4 );//roll
    Target_tracker[1].init( P_TD4P1 , P_TD4P2 , P_TD4P3 , P_TD4P4 );//pitch
    Target_tracker[2].init( Y_TD4P1 , Y_TD4P2 , Y_TD4P3 , Y_TD4P4 );//yaw
    
    //初始化姿态ESO
    ESO[0].init( Motor_T , R_b , 0.7f , 10 , 500 , 5000 );//roll
    ESO[1].init( Motor_T , P_b , 0.7f , 10 , 500 , 5000 );//pitch
    ESO[2].init( 1.0f/200 , Y_b , 0.7f , 10 , 200 , 200 );//yaw

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
   // run custom controller after here
    Quaternion attitude_body, attitude_target;
    float Ps = R_P1;
    Vector3f P2 = {R_P2,P_P2,Y_P2};
    Vector3f P3 = {R_P3,P_P3,Y_P3};   
//    Vector3f P4 = {R_P4,P_P4,Y_P4};
    _ahrs->get_quat_body_to_ned(attitude_body);
    attitude_target = _att_control->get_attitude_target_quat();
	Quaternion current_quat_PR = Quaternion_get_PRQuat( attitude_body );
	float Yaw = attitude_body.get_euler_yaw();
    Matrix3f Rotation_Matrix;
    Matrix3f Rotation_Matrix_P;//正向旋转
    current_quat_PR.rotation_matrix(Rotation_Matrix_P);
    current_quat_PR.inverse().rotation_matrix(Rotation_Matrix);

    Vector3f AngularRateCtrl = _ahrs->get_gyro_latest();
	//运行扩张状态观测器得到估计角速度、角加速度
	ESO[0].run( AngularRateCtrl.x , 1.0f / AP::scheduler().get_loop_rate_hz() );
	ESO[1].run( AngularRateCtrl.y , 1.0f / AP::scheduler().get_loop_rate_hz() );
	ESO[2].run( AngularRateCtrl.z , 1.0f / AP::scheduler().get_loop_rate_hz() );
	Vector3f angular_rate_ESO = 
	{
		(float)(ESO[0].get_EsAngularRate()),
		(float)(ESO[1].get_EsAngularRate()) ,
		(float)(ESO[2].get_EsAngularRate()) ,
	};
	Vector3f angular_acceleration_ESO = 
	{ 
		(float)(ESO[0].get_EsAngularAcceleration()) ,
		(float)(ESO[1].get_EsAngularAcceleration()) ,
		(float)(ESO[2].get_EsAngularAcceleration()) , 
	};
    Vector3f angular_rate_ENU;
    angular_rate_ENU = Rotation_Matrix_P*angular_rate_ESO;
    Vector3f angular_acceleration_ENU;
    angular_acceleration_ENU = Rotation_Matrix_P*angular_acceleration_ESO;
    float target_Roll,target_Pitch,target_Yaw;
    attitude_target.to_euler(target_Roll,target_Pitch,target_Yaw);
    Target_tracker[0].track4( target_Roll , 1.0f / AP::scheduler().get_loop_rate_hz() );
    Target_tracker[1].track4( target_Pitch , 1.0f / AP::scheduler().get_loop_rate_hz() );
    Quaternion target_quat_PR;
    target_quat_PR.from_euler( Target_tracker[0].x1 , Target_tracker[1].x1 , 0 );
    Quaternion PR_rotation_quat = target_quat_PR * current_quat_PR.inverse();
    Vector3f PR_rotation;
    PR_rotation_quat.to_axis_angle(PR_rotation);
    Vector3f feed_foward_ratePR = { (float)(Target_tracker[0].x2) , (float)(Target_tracker[1].x2) , 0 };
    Vector3f target_angular_rate_RP = PR_rotation * Ps + feed_foward_ratePR;

    float target_angular_rate_Y;
    Target_tracker[2].r2n = Target_tracker[2].r2p = 1.0f;
    Target_tracker[2].track4( target_Yaw , 1.0f / AP::scheduler().get_loop_rate_hz() );
    float angle_error = Target_tracker[2].x1 - Yaw;
    angle_error = wrap_PI(angle_error);
    target_angular_rate_Y = angle_error * Ps + Target_tracker[2].x2;
    target_angular_rate_Y = constrain_float( target_angular_rate_Y ,-2.5f ,2.5f );

    //计算前馈量
    float YawAngleP =  ( Target_tracker[2].tracking_mode == 4 ) ? ( Ps ) : 0;
    Vector3f Tv1_ENU = { Ps*( (float)(Target_tracker[0].x2) - angular_rate_ENU.x ) + (float)(Target_tracker[0].x3) ,
                                                        Ps*( (float)(Target_tracker[1].x2) - angular_rate_ENU.y ) + (float)(Target_tracker[1].x3) ,
                                                        YawAngleP*( (float)(Target_tracker[2].x2) - angular_rate_ENU.z ) + (float)(Target_tracker[2].x3) };
    Vector3f Tv2_ENU = { Ps*( (float)(Target_tracker[0].x3) - angular_acceleration_ENU.x ) + (float)(Target_tracker[0].x4) ,
                                                        Ps*( (float)(Target_tracker[1].x3) - angular_acceleration_ENU.y ) + (float)(Target_tracker[1].x4) ,
                                                        YawAngleP*( (float)(Target_tracker[2].x3) - angular_acceleration_ENU.z ) + (float)(Target_tracker[2].x4) };
    Vector3f Tv1 = Rotation_Matrix * Tv1_ENU;
    Vector3f Tv2 = Rotation_Matrix * Tv2_ENU;
    Vector3f Ta1 = { P2.x*( Tv1.x - angular_acceleration_ESO.x ) + Tv2.x ,
                                                P2.y*( Tv1.y - angular_acceleration_ESO.y ) + Tv2.y ,
                                                P2.z*( Tv1.z - angular_acceleration_ESO.z ) + Tv2.z };
    Vector3f target_angular_rate_ENU;
    target_angular_rate_ENU.x = target_angular_rate_RP.x;
    target_angular_rate_ENU.y = target_angular_rate_RP.y;
    target_angular_rate_ENU.z = target_angular_rate_RP.z + target_angular_rate_Y;
	Vector3f target_angular_rate_body;
    target_angular_rate_body = Rotation_Matrix * target_angular_rate_ENU;
    Vector3f target_angular_acceleration = target_angular_rate_body - angular_rate_ESO;
    target_angular_acceleration.x *= P2.x;
    target_angular_acceleration.y *= P2.y;
    target_angular_acceleration.z *= P2.z;
	target_angular_acceleration = target_angular_acceleration + Tv1;
	Vector3f angular_acceleration_error =  target_angular_acceleration - angular_acceleration_ESO ;
	Vector3f disturbance = { 
		(float)(ESO[0].get_EsDisturbance()) ,
		(float)(ESO[1].get_EsDisturbance()) ,
		(float)(ESO[2].get_EsDisturbance()) ,
	};
	// static Vector3f last_disturbance = { 0 , 0 , 0 };		
    // Vector3f disturbance_Derivative = (disturbance - last_disturbance)*(AP::scheduler().get_loop_rate_hz());
	// last_disturbance = disturbance;
	// float disturbance_x = disturbance_filter[0].apply(disturbance_Derivative.x);
	// float disturbance_y = disturbance_filter[1].apply(disturbance_Derivative.y);
	// float disturbance_z = disturbance_filter[2].apply(disturbance_Derivative.z);
    gcs().send_text(MAV_SEVERITY_INFO, "ADRC custom controller working");
    Vector3f motor_out;
    // reset controller based on spool state
    motor_out.x = 	( (float)(ESO[0].get_EsMainPower()) + ESO[0].T * ( angular_acceleration_error.x * P3.x + Ta1.x /*- disturbance_x*/ ) )/ESO[0].b;
    motor_out.y =	( (float)(ESO[1].get_EsMainPower()) + ESO[1].T * ( angular_acceleration_error.y * P3.y + Ta1.y /*- disturbance_y*/ ) )/ESO[1].b;
    motor_out.z = ( target_angular_acceleration.z - disturbance.z ) / ESO[2].b;
    return motor_out;
}

// reset controller to avoid build up on the ground
// or to provide bumpless transfer from arducopter main controller
void AC_CustomControl_ADRC::reset(void)
{
    Target_tracker[0].reset();
    Target_tracker[1].reset();
    Target_tracker[2].reset();
    ESO[0].reset();
    ESO[1].reset();
    ESO[2].reset();
}

#endif
