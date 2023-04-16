#pragma once

/*
    Functions to compute the jacobian matrix using dual quaternion algebra 
    
    Inputs  -> (joint rates, DELTA_T)
    Outputs -> (angular and linear velocity of frames)

    REFERENCE: Check section 2.3.2.2 of [https://personalpages.manchester.ac.uk/staff/Bruno.Adorno/publications/phd_thesis_final_version.pdf]
*/

#include "forward_kinematics_dualquaternion.h"
#include "dualquat/Eigen/Dense"


namespace Kinematics
{

/*
    Function that returns the jacobian matrix
    Inputs  -> (a linked Robot)
    Outputs -> (Matrix of size (8,NumJoints))
*/
template<typename T>
Eigen::Matrix<T,8,Eigen::Dynamic> // Default storage order is ColumnMajour
ComputeJacobian(RobotLinks<T>& Robot)
{
    dualquat::DualQuaternion<T> ForwardK_dq = Robot.ComputeForwardKinematics();
    
    T type;

    //Initialize the Jacobian
    const int N = Robot.getNumJointsTotal();
    Eigen::Matrix< T, 8, Eigen::Dynamic> Jacobian(8,N);

    dualquat::DualQuaternion<T> q_0_i = dualquat::identity(type);

    dualquat::DualQuaternion<T> z_i;

    dualquat::DualQuaternion<T> jacobian_i;



    for(int i=0; i<N; i++) // i in the equations represents the joint i
    {
        z_i = dualquat::DualQuaternion<T>(

        T(0),
        q_0_i.real().x()*q_0_i.real().z() + q_0_i.real().w()*q_0_i.real().y(),
        q_0_i.real().y()*q_0_i.real().z() - q_0_i.real().w()*q_0_i.real().x(),
        (q_0_i.real().z()*q_0_i.real().z() - q_0_i.real().y()*q_0_i.real().y() - q_0_i.real().x()*q_0_i.real().x() + q_0_i.real().w()*q_0_i.real().w())/2,
        T(0),
        q_0_i.real().x()*q_0_i.dual().z() + q_0_i.dual().x()*q_0_i.real().z() + q_0_i.real().w()*q_0_i.dual().y() + q_0_i.dual().w()*q_0_i.real().y(),
        q_0_i.real().y()*q_0_i.dual().z() + q_0_i.dual().y()*q_0_i.real().z() - q_0_i.real().w()*q_0_i.dual().x() - q_0_i.dual().w()*q_0_i.real().x(),
        q_0_i.real().z()*q_0_i.dual().z() - q_0_i.real().y()*q_0_i.dual().y() - q_0_i.real().x()*q_0_i.dual().x() + q_0_i.real().w()*q_0_i.dual().w());

        jacobian_i = z_i*ForwardK_dq;

        Jacobian.col(i) = Eigen::Matrix<T,8,1>(
        
        jacobian_i.real().w(),
        jacobian_i.real().x(),
        jacobian_i.real().y(),
        jacobian_i.real().z(),
        jacobian_i.dual().w(),
        jacobian_i.dual().x(),
        jacobian_i.dual().y(),
        jacobian_i.dual().z());

        q_0_i = q_0_i*Pose_frame_iprev_i(*Robot.getJoint(i));
        
    }

    return Jacobian;
}
/*
    Function that returns the pose_dot quaternion members as vector
    Inputs  -> (a linked Robot)
    Outputs -> (Matrix of size (8,1))
*/
template<typename T>
Eigen::Matrix<T,8,1> // Default storage order is ColumnMajour
compute_pose_dot(RobotLinks<T>& Robot)
{
    //const int N = Robot.getNumJointsTotal();
    return Eigen::Matrix<T,8,1>(ComputeJacobian(Robot)*Robot.getThetaDotVec());
}

} // namespace Jacobian
