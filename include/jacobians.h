#pragma once

/*
    Functions to compute the jacobian matrix using dual quaternion algebra 
    
    Inputs  -> (joint rates, DELTA_T)
    Outputs -> (Jacobians, pose rates)

    REFERENCE: Check section 2.3.2.2 of [https://personalpages.manchester.ac.uk/staff/Bruno.Adorno/publications/phd_thesis_final_version.pdf]
*/

#include "forward_kinematics_dualquaternion.h"
#include "dualquat/Eigen/Dense"


namespace Kinematics
{

/*
    Function that returns the Kinematic Jacobian matrix
    To convert joint rate to pose vector rate
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
    Function that returns the Kinematic Jacobian matrix
    To convert joint rate to pose vector rate
    Inputs  -> (Linked robot and Robots current end effector pose)
    Outputs -> (Matrix of size (8,NumJoints))
*/
template<typename T>
Eigen::Matrix<T,8,Eigen::Dynamic> // Default storage order is ColumnMajour
ComputeJacobian(RobotLinks<T>& Robot, const dualquat::DualQuaternion<T>& ForwardK_dq)
{   
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
    Function that returns the Generalized Jacobian
    To convert pose vector rate to twist
    Inputs  -> (a linked Robot)
    Outputs -> (Matrix of size (8,8))
*/
template<typename T>
Eigen::Matrix<T,8,8>
GenerailzedJacobian(RobotLinks<T>& Robot)
{
    // Just copy the values into h1, h2, h3.. better than calling dq.real.w() 9 times and multiplying it with 2 every time
    dualquat::DualQuaternion<T> ForwardK_dq = Robot.ComputeForwardKinematics();
    T h1 = 2*ForwardK_dq.real.w();
    T h2 = 2*ForwardK_dq.real.x();
    T h3 = 2*ForwardK_dq.real.y();
    T h4 = 2*ForwardK_dq.real.z();
    T h5 = 2*ForwardK_dq.dual.w();
    T h6 = 2*ForwardK_dq.dual.x();
    T h7 = 2*ForwardK_dq.dual.y();
    T h8 = 2*ForwardK_dq.dual.z();
    T zero = T(0); // avoid calling this too many times

    return (Eigen::Matrix<T,8,8>()<<h5,h6,h7,h8,h1,h2,h3,h4,
                                    h6,-h5,h8,-h7,-h2,h1,-h4,h3,
                                    h7,-h8,-h5,h6,-h3,h4,h1,-h2,
                                    h8,h7,-h6,-h5,-h4,-h3,h2,h1,
                                    h1,h2,h3,h4,zero,zero,zero,zero,
                                    -h2,h1,-h4,h3,zero,zero,zero,zero,
                                    -h3,h4,h1,-h2,zero,zero,zero,zero
                                    -h4,-h3,h2,h1,zero,zero,zero,zero).finished();

}
/*
Return rows swapped generalized jacobian. The angular velocity and linear velocity terms are swapped
*/
template<typename T>
Eigen::Matrix<T,8,8>
GenerailzedJacobian_swapped(RobotLinks<T>& Robot)
{
    // Just copy the values into h1, h2, h3.. better than calling dq.real.w() 9 times and multiplying it with 2 every time
    dualquat::DualQuaternion<T> ForwardK_dq = Robot.ComputeForwardKinematics();
    T h1 = 2*ForwardK_dq.real.w();
    T h2 = 2*ForwardK_dq.real.x();
    T h3 = 2*ForwardK_dq.real.y();
    T h4 = 2*ForwardK_dq.real.z();
    T h5 = 2*ForwardK_dq.dual.w();
    T h6 = 2*ForwardK_dq.dual.x();
    T h7 = 2*ForwardK_dq.dual.y();
    T h8 = 2*ForwardK_dq.dual.z();
    T zero = T(0); // avoid calling this too many times

    return (Eigen::Matrix<T,8,8>()<<h1,h2,h3,h4,zero,zero,zero,zero,
                                    -h2,h1,-h4,h3,zero,zero,zero,zero,
                                    -h3,h4,h1,-h2,zero,zero,zero,zero
                                    -h4,-h3,h2,h1,zero,zero,zero,zero,
                                    h5,h6,h7,h8,h1,h2,h3,h4,
                                    h6,-h5,h8,-h7,-h2,h1,-h4,h3,
                                    h7,-h8,-h5,h6,-h3,h4,h1,-h2,
                                    h8,h7,-h6,-h5,-h4,-h3,h2,h1).finished();

}

/*
Return rows swapped generalized jacobian. The angular velocity and linear velocity terms are swapped
*/
template<typename T>
Eigen::Matrix<T,8,8>
GenerailzedJacobian_swapped(const dualquat::DualQuaternion<T> dq)
{
    // Just copy the values into h1, h2, h3.. better than calling dq.real.w() 9 times and multiplying it with 2 every time
    T h1 = 2*dq.real.w();
    T h2 = 2*dq.real.x();
    T h3 = 2*dq.real.y();
    T h4 = 2*dq.real.z();
    T h5 = 2*dq.dual.w();
    T h6 = 2*dq.dual.x();
    T h7 = 2*dq.dual.y();
    T h8 = 2*dq.dual.z();
    T zero = T(0); // avoid calling this too many times

    return (Eigen::Matrix<T,8,8>()<<h1,h2,h3,h4,zero,zero,zero,zero,
                                    -h2,h1,-h4,h3,zero,zero,zero,zero,
                                    -h3,h4,h1,-h2,zero,zero,zero,zero
                                    -h4,-h3,h2,h1,zero,zero,zero,zero,
                                    h5,h6,h7,h8,h1,h2,h3,h4,
                                    h6,-h5,h8,-h7,-h2,h1,-h4,h3,
                                    h7,-h8,-h5,h6,-h3,h4,h1,-h2,
                                    h8,h7,-h6,-h5,-h4,-h3,h2,h1).finished();

}

/*
    Function that returns the pose_dot quaternion members as vector
    Inputs  -> (a linked Robot)
    Outputs -> (Matrix of size (8,1))
*/
template<typename T>
Eigen::Matrix<T,8,1> 
compute_pose_dot(RobotLinks<T>& Robot){ return Eigen::Matrix<T,8,1>(ComputeJacobian(Robot)*Robot.getJointDotVec());}

/*
    Function that returns the twist of an end effector
    Inputs  -> (a linked Robot)
    Outputs -> (Matrix of size (8,1))
*/
template<typename T>
Eigen::Matrix<T,8,1> 
compute_twist(RobotLinks<T>& Robot){ return Eigen::Matrix<T,8,1>(GenerailzedJacobian_swapped(Robot)*compute_pose_dot(Robot));}

template<typename T>
Eigen::Matrix<T,8,1> 
compute_twist(const dualquat::DualQuaternion<T>& pose,const dualquat::DualQuaternion<T>& pose_dot)
{ 
    return Eigen::Matrix<T,8,1>(GenerailzedJacobian_swapped(pose)*pose_dot);
}

/*
    Function that returns the generalized forces of an end effector
    Inputs  -> (a linked Robot, dual quaternion force at end effector)
    Outputs -> Generalized forces (Matrix of size (8,1))
*/
template<typename T>
Eigen::Matrix<T,8,1>
compute_generalized_forces(RobotLinks<T>& Robot, const dualquat::DualQuaternion<T>& force_end_effector)
{ return Eigen::Matrix<T,8,1>((ComputeGenerailzedJacobian(Robot).transpose())*force_end_effector);}

} // namespace Kinematics