#include <iostream>
#include "forward_kinematics_dualquaternion.h"
#include "jacobians.h"

using namespace std;
int main()
{
    Kinematics::RobotLinks<double> R3_robot;
    DH::DH_joint<double> J1(3.33,2.0,10.0,6.1);
    DH::DH_joint<double> J2(99,0.0,5.0,0.0);
    DH::DH_joint<double> J3(0.22,0.0,2.0,0.0);


    R3_robot.addJoint(J1);
    R3_robot.addJoint(J2);
    R3_robot.addJoint(J3);

    cout<<"\n"<<"NumLinks = "<<R3_robot.getNumJointsTotal();

    //Test dynamic mempry allocation

    cout<<"\n"<<R3_robot.getJoint(0)->theta_i;

    J1.theta_i = 992.2423;
    
    cout<<"\n"<<R3_robot.getJoint(0)->theta_i;

    J1.theta_i = 60.23;
    cout<<"\n"<<R3_robot.getJoint(0)->theta_i;

    // Test kinematics result
    dualquat::DualQuaternion<double> Q1 = Kinematics::Pose_frame_iprev_i(J3)*Kinematics::Pose_frame_iprev_i(J2)*Kinematics::Pose_frame_iprev_i(J1);
    dualquat::DualQuaternion<double> Q2 = R3_robot.ComputeForwardKinematics();

    cout<<"\n"<<"Equality? : "<<dualquat::almost_equal(Q1,Q2,0.001); //Its working!
    cout<<"\n"<<"Is unit Q1? : "<<dualquat::is_unit(Q1,0.001);
    cout<<"\n"<<"Is unit Q2? : "<<dualquat::is_unit(Q2,0.001);

    // Test frame quaternions

    // Test Jacobians

    cout<<"\n Jacobian Matrix:"<<Kinematics::ComputeJacobian(R3_robot)<<endl;
    cout<<"\n Theta dot:"<<R3_robot.getThetaDotVec()<<endl;
    cout<<"\n pose_dot:"<<Kinematics::compute_pose_dot(R3_robot)<<endl;

    // 
    return 0;
}