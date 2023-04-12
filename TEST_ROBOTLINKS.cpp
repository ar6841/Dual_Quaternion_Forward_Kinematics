#include <iostream>
#include "dualquaternion_forward_kinematics.h"
using namespace std;
int main()
{
    forwardk_dualquat::RobotLinks<double> R3_robot;
    DH::DH_joint<double> J1(33.33,0.0,10.0,0.0);
    DH::DH_joint<double> J2(0,0.0,5.0,0.0);
    DH::DH_joint<double> J3(0.22,0.0,2.0,0.0);

    R3_robot.addJoint(J1);
    R3_robot.addJoint(J2);
    R3_robot.addJoint(J3);

    cout<<"\n"<<"NumLinks = "<<R3_robot.getNumJointsTotal();

    //Test dynamic mempry allocation

    cout<<"\n"<<R3_robot.getJoint(1)->joint_data->theta_i;

    J1.theta_i = 992.2423;
    cout<<"\n"<<R3_robot.getJoint(2)->joint_data->theta_i;
    cout<<"\n"<<R3_robot.getJoint(1)->joint_data->theta_i;

    // Test kinematics result
    dualquat::DualQuaternion<double> Q1 = pose_dualquat::Pose_frame_i_iprev(J1)*pose_dualquat::Pose_frame_i_iprev(J2)*pose_dualquat::Pose_frame_i_iprev(J3);
    dualquat::DualQuaternion<double> Q2 = R3_robot.ComputeForwardKinematics();

    cout<<"\n"<<"Equality? : "<<dualquat::almost_equal(Q1,Q2,0.001); //Its working!
    cout<<"\n"<<"Is unit Q1? : "<<dualquat::is_unit(Q1,0.001);
    cout<<"\n"<<"Is unit Q2? : "<<dualquat::is_unit(Q2,0.001);


    // 
    return 0;
}