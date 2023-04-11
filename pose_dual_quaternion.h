#pragma once
/**
 * This header file provides functions that return pose in dual quaternion form
 * The DH conventions and equations make sure that the quaternion is a unit dual quaternion
 * 
 * NOTES: THE CORRECT CONJUGATE TO USE ACCORDING TO THE PAPER is quaternion_conjugate() within dualquat -> dualquat_common.h
 * 
 * IMORTANT: The norm for shifting frames from the joint 0 to end enffector is to pre-multiply the change of frame DualQuaternions,
 *           assuming that you're calculating change of frame from 0 frame to 1st frame initially. (0_T_1) then (1_T_2).. and so on.
 *           For example, for a serial RR configuration:      q_0_endeffector = q_2_endeffector*q_1_2*q_0_1
 *                                                            q_endeffector_0 = q_1_0*q_2_1*q_endeffector_2
*/
#include "dualquat/dualquat.h"
#include "DH_convention.h"

#define degree_to_radian(deg) ((deg*EIGEN_PI)/180.0)
#define radian_to_degree(rad) ((rad*180.0)/EIGEN_PI)

namespace pose_dualquat
{

// Function that returns transofrmation from frame i to iprev (Equivalent to i_T_iprev)
template<typename T>
dualquat::DualQuaternion<T>   
Pose_frame_i_iprev(const DH::DH_joint<T>& joint_i) // changed to 'const DH::DH_joint<T>' to pass by refrence and keep data safe
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

// Function that returns transofrmation from frame iprev to i (Equivalent to iprev_T_i)
// This one is used commonly wrt to changing the end effector frame to base frame
template<typename T>
dualquat::DualQuaternion<T>   
Pose_frame_iprev_i(const DH::DH_joint<T>& joint_i)
{ 
    return dualquat::quaternion_conjugate(Pose_frame_i_iprev(joint_i)); //q_pose_iprev_i
}

} //namespace pose_dualquat