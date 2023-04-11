#include <iostream>
#include "dualquaternion_forward_kinematics.h"
int main()
{
    forwardk_dualquat::RobotLinks<double> R3_robot;
    DH::DH_joint<double> *J1 = DH::CreateNewJoint<double>(0,0.0,10.0,0.0);
    DH::DH_joint<double> *J2 = new DH::DH_joint<double>(0,0,10.0,0);
    DH::DH_joint<double> *J3 = new DH::DH_joint<double>(0,0,10.0,0);
   // DH::DH_joint<double>* J2(0,0,10.0,0);
   // DH::DH_joint<double>* J3(0,0,10.0,0);

   // R3_robot.addJoint(J1);
   // R3_robot.addJoint(J2);
    //R3_robot.addJoint(J3);
    return 0;
}