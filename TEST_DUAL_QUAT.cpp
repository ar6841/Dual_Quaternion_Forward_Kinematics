#include <iostream>
#include "dualquat/dualquat.h"
#include "pose_dual_quaternion.h"
#include "DH_convention.h"
using namespace Eigen;


//using namespace dualquat;
int main() {
  Quaternion q1(1.0, 9.0, 0.0,0.0), q2(2.0,3.0, 0.0, 1.0), q3(6.0,2.0,1.2,2.4), q4(7.0,2.2,1.2,2.4);
   // w, x, y, z
    dualquat::DualQuaternion qd1(q1,q2),qd2(q3,q4);
    dualquat::DualQuaternion<double> qd3,qd4;
    qd3 = qd1+qd2;
    qd4 = qd1*qd2;
    std::pair<double, double>  result = dualquat::norm(qd4);
    std::cout<<norm(qd4).second;

    DH::DH_joint j1(30.0,20.2,1.5,5.5);
    dualquat::DualQuaternion q_i_iprev = pose_dualquat::Pose_frame_i_iprev(j1);
    dualquat::DualQuaternion q_iprev_i = quaternion_conjugate(q_i_iprev);
    dualquat::DualQuaternion result2 = q_i_iprev*q_iprev_i;
    std::cout<<"\n"<<(dualquat::is_unit(result2,0.00001));
    double x;

    dualquat::DualQuaternion<double> QI = dualquat::identity(x);
    std::cout<<"\n"<<QI.real().w();



  // std::cout << "Quaternion: " << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << std::endl;
  return 0;
}
