# Dual_Quaternion_Forward_Kinematics
C++ header files for performing forward kinematics using dual quaternion algebra.
Dual quaternions provide a more stable and compact form for representing rigid body motion, this library should help you model the kinemaics of your robot using only dual quaternions. The underlying assumption is that you know the DH parameters of each joint on your robot. Support will be added soon for parameterization using screw theory.


TODO: Add the equations and explanations


## Dependencies

1. [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
2. [dualquat](https://github.com/Hasenpfote/dualquat)

*Some changes have been made to the dualquat library so I recommend using my version, all the dependencies are included in this file*.

## Usage
Here are some short examples of some of the functions included and how to use them,

1. Create a robot joint using the `DH_joints` class 

`DH::DH_joint<double> J1(theta,alpha,a,d, DH::REVOLUTE, offset);`

2. Add all the joints in the order you need using the `RobotLinks` class

`MyRobot.addJoint(J1);`

`MyRobot.addJoint(J2);`

`MyRobot.addJoint(J3);`

3. Use `ComputeForwardKinematics()` to compute the pose of the end frame wrt to the inerial frame. (Equivalent to 
${ }^0 T_{N} $
)

`dualquat::DualQuaternion<T> pose = MyRobot.ComputeForwardKinematics()`

4. You can also compute the the Jacobian using `ComputeJacobian()`

`Eigen::Matrix<double,8,Eigen::Dynamic> J(ComputeJacobian(MyRobot));`

5. Compute rate of change of pose in dualquat coordinates using `compute_pose_dot()`

`Eigen::Matrix<T,8,1> pose_vec(compute_pose_dot(MyRobot));`

## Explanation
A dual quaternion:

$$
\underline{q}=\mathcal{P}(\underline{q})+\varepsilon \mathcal{D}(\underline{q})
$$


Where $\mathcal{P}(\underline{q})$ is the primary quaternion, $\mathcal{D}(\underline{q})$ is the dual quaternion, and $\varepsilon^2=0$ 


The relative pose from frame i to frame i-1 in dual quaternion space is given by:

$$
{ }^i \underline{q}_{i-1}=\left[\begin{array}{c}
\cos (\alpha / 2) \cos (\theta / 2) \\
\sin (\alpha / 2) \cos (\theta / 2) \\
\sin (\alpha / 2) \sin (\theta / 2) \\
\cos (\alpha / 2) \sin (\theta / 2) \\
-\frac{1}{2} a_i \sin \left(\alpha_i / 2\right) \cos \left(\theta_i / 2\right)-\frac{1}{2} d_i \cos \left(\alpha_i / 2\right) \sin \left(\theta_i / 2\right) \\
\frac{1}{2} a_i \cos \left(\alpha_i / 2\right) \cos \left(\theta_i / 2\right)-\frac{1}{2} d_i \sin \left(\alpha_i / 2\right) \sin \left(\theta_i / 2\right) \\
\frac{1}{2} a_i \cos \left(\alpha_i / 2\right) \sin \left(\theta_i / 2\right)+\frac{1}{2} d_i \sin \left(\alpha_i / 2\right) \cos \left(\theta_i / 2\right) \\
\frac{1}{2} d_i \cos \left(\alpha_i / 2\right) \cos \left(\theta_i / 2\right)-\frac{1}{2} a_i \sin \left(\alpha_i / 2\right) \sin \left(\theta_i / 2\right)
\end{array}\right]
$$

Which in cartesian coordinates is represented by the Transformation matrix:

$$
{ }^i T_{i-1}=\left[\begin{array}{ccc:c}
\cos \theta_i & \sin \theta_i & 0 & -a_i \\
-\cos \alpha_i \sin \theta_i & \cos \alpha_i \cos \theta_i & \sin \alpha_i & -d_i \sin \alpha_i \\
\sin \alpha_i \sin \theta_i & -\sin \alpha_i \cos \theta_i & \cos \alpha_i & -d_i \cos \alpha_i \\
\hdashline 0 & 0 & 0 & 1
\end{array}\right]
$$

Composition transformations

$$
{ }^0 \underline{q}_{N}= { }^2 \underline{q}_{3} { }^1 \underline{q}_{2} { }^0 \underline{q}_{1}. . .{ }^N{ }^-{ }^1 \underline{q}_{N} 
$$

Jacobians:

Mapping from joint rates to pose rate:


## Compatibility

Supports C++ 11 or higher.

| Compiler | Version           | Remarks |
| -------- | ----------------- | ------- |
| gcc      | 5.5.0 or higher.  |         |
| clang    | 7.0.0 or higher.  |         |
| msvc     | 16.5.4 or higher. |         |


## References

1. [https://dcsl.gatech.edu/papers/mdpi18%20(Printed).pdf]
2. [https://hal.science/hal-01478225/file/Robot_Kinematic_Modeling_and_Control_Based_on_Dual_Quaternion_Algebra_Part_I_Fundamentals_28Feb2017.pdf]

## Notes


