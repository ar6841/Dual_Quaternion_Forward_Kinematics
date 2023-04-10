#pragma once
/**
 * This header file provides functions that return pose in dual quaternion form
 * The DH conventions and equations make sure that the quaternion is a unit dual quaternion
 * 
 * NOTES: THE CORRECT CONJUGATE TO USE ACCORDING TO THE PAPER is quaternion_conjugate() within dualquat ->dualquat_common.h
*/
#include "dualquat/dualquat.h"
#include "DH_convention.h"
#define degree_to_radian(deg) ((deg*EIGEN_PI)/180.0)
#define radian_to_degree(rad) ((rad*180.0)/EIGEN_PI)

namespace pose_dualquat
{
template<typename T>
dualquat::DualQuaternion<T>   
Pose_frame_i_iprev(DH::DH_joint<T> joint_i)
{ 
    T alpha_i_rad_half = (degree_to_radian(joint_i.alpha_i))/2;
    T theta_i_rad_half = (degree_to_radian(joint_i.theta_i))/2;

    // Elements of R8 representation of dual quaternion
    T q_1 = cos(alpha_i_rad_half)*cos(theta_i_rad_half);
    T q_2 = sin(alpha_i_rad_half)*cos(theta_i_rad_half);
    T q_3 = sin(alpha_i_rad_half)*sin(theta_i_rad_half);
    T q_4 = cos(alpha_i_rad_half)*sin(theta_i_rad_half);
    T q_5 = (-(joint_i.a_i*q_2)/2 -(joint_i.d_i*q_4)/2);
    T q_6 = ((joint_i.a_i*q_1)/2 -(joint_i.d_i*q_3)/2);
    T q_7 = ((joint_i.a_i*q_4)/2 +(joint_i.d_i*q_2)/2);
    T q_8 = (-(joint_i.a_i*q_3)/2 +(joint_i.d_i*q_1)/2);

    //Convert R8 to two quaternions
    Eigen::Quaternion<T> q_real(q_1,q_2,q_3,q_4), q_dual(q_5,q_6,q_7,q_8);

    //Convert two quaternions to dual quaternion and return
    dualquat::DualQuaternion<T> q_pose_i_iprev(q_real,q_dual);
    return q_pose_i_iprev;
}

} //namespace pose_dualquat