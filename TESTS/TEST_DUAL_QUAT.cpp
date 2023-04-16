#include <iostream>
#include "dualquat/dualquat.h"
#include "pose_dual_quaternion.h"
#include "DH_parameters.h"

int main() {
  Eigen::Quaternion q1(1.0, 9.0, 0.0,0.0), q2(2.0,3.0, 0.0, 1.0), q3(6.0,2.0,1.2,2.4), q4(7.0,2.2,1.2,2.4);

   // w, x, y, z
  dualquat::DualQuaternion qd1(q1,q2),qd2(q3,q4);
  dualquat::DualQuaternion<double> qd3,qd4;
  qd3 = qd1+qd2;
  qd4 = qd1*qd2;
  std::pair<double, double>  result = dualquat::norm(qd4);

  std::cout<<"\n"<<norm(qd4).second;

  DH::DH_joint j1(30.0,20.2,1.5,5.5);

  std::cout<<"\n"<<j1.theta_i;
  
  dualquat::DualQuaternion q_i_iprev = Kinematics::Pose_frame_iprev_i(j1);
  dualquat::DualQuaternion q_iprev_i = dualquat::quaternion_conjugate(q_i_iprev);
  dualquat::DualQuaternion result2 = q_i_iprev*q_iprev_i;
  std::cout<<"\n"<<(dualquat::is_unit(result2,0.01));
  double x;

  dualquat::DualQuaternion<double> QI = dualquat::identity(x);
  std::cout<<"\n"<<QI.real().w();

  dualquat::DualQuaternion<double> QN(1,2,3,4,5,6,7,8);

  std::cout<<"\n"<<QN.real().w()<<std::endl;

  std::cout<<"\n"<<QN.dual().w()<<std::endl;

  


  std::cout<<"\ndone";





  return 0;
}
