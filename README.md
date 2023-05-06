# Dual_Quaternion_Forward_Kinematics
C++ header files for performing forward kinematics using dual quaternion algebra.

Dual quaternions are extentions of quaternions as a dual number, just as quaternions of unit length can be used to represent 3D rotations, dual quaternions of unit length can be used to represent 3D rigid motions. Dual quaternions provide a more stable and compact form for representing rigid body motion, and a unified space for performing modelling, control and planning when compared to classical tranformation methods. 

This library should help you model the kinemaics of your robot using only dual quaternions. The underlying assumption is that you know the DH parameters of each joint on your robot. 

Support will be added soon for parameterization using screw theory and pose transforms of 6DOF joints.


## Dependencies

1. [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)
2. [dualquat](https://github.com/Hasenpfote/dualquat)

*Some changes have been made to the dualquat library (including new operators!) so I recommend using my version, all the dependencies are included in this file*.

## Usage
Here are some short examples of some of the functions included and how to use them,

1. Create a robot joint using the `DH_joints` class 

```c++
DH::DH_joint<double> J1(theta,alpha,a,d, DH::REVOLUTE, offset);
```

2. Add all the joints in the order you need using the `RobotLinks` class

```c++

MyRobot.addJoint(J1);
MyRobot.addJoint(J2);
MyRobot.addJoint(J3);

```

3. Use `ComputeForwardKinematics()` to compute the pose of the end frame wrt to the inerial frame. (Equivalent to 
${ }^0 T_{N} $
)

```c++
dualquat::DualQuaternion<T> pose = MyRobot.ComputeForwardKinematics()
```

4. You can also compute the the Jacobian using `ComputeJacobian()`

```c++
Eigen::Matrix<double,8,Eigen::Dynamic> J(ComputeJacobian(MyRobot));
```

5. Compute rate of change of pose in dualquat coordinates using `compute_pose_dot()`

```c++
Eigen::Matrix<T,8,1> pose_vec(compute_pose_dot(MyRobot));
```

## Explanation

### A dual quaternion:

$$
\underline{q}=\mathcal{P}(\underline{q})+\varepsilon \mathcal{D}(\underline{q})
$$


Where $\mathcal{P}(\underline{q})$ is the primary part (quaternion), $\mathcal{D}(\underline{q})$ is the dual part (quaternion), and $\varepsilon^2=0$ while $\varepsilon \neq 0$


### Pose transform

The relative pose from frame i to frame i-1 in dual quaternion space is given by:

$$
{ }^{i-1} \underline{q}_{ i}=\left[\begin{array}{c}
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
{ }^{i-1} T_{i}=\left[\begin{array}{cccc}
\cos \theta_i & -\sin \theta_i \cos \alpha_i & \sin \theta_i \sin \alpha_i & a_i \cos \theta_i \\
\sin \theta_i & \cos \theta_i \cos \alpha_i & -\cos \theta_i \sin \alpha_i & a_i \sin \theta_i \\
0 & \sin \alpha_i & \cos \alpha_i & d_i \\
0 & 0 & 0 & 1
\end{array}\right]
$$


### Composition transformations

$$
{ }^0 \underline{q_N} =  { }^0 \underline{q_1}{ }^1 \underline{q_2} { }^2 \underline{q_3}  . . .{ }^N{ }^-{ }^1 \underline{q_N} 
$$

### Jacobians:

$$
J(\underline{\mathbf{q}})=\frac{d \overrightarrow{\mathbf{q}}}{d \vec{\theta}}=\frac{d}{d \theta}\left(\begin{array}{c}
q_1 \\
\vdots \\
q_4 \\
q_5 \\
\vdots \\
q_8
\end{array}\right)=\left(\begin{array}{ccc}
\frac{\partial q_1}{\partial \theta_1} & \ldots & \frac{\partial q_1}{\partial \theta_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial q_8}{\partial \theta_1} & \ldots & \frac{\partial q_8}{\partial \theta_n}
\end{array}\right)
$$

$J(\underline{\mathbf{q}})$ has been analytically derived for any robot using the alorithm presented in [[1]](https://hal.science/hal-01478225/file/Robot_Kinematic_Modeling_and_Control_Based_on_Dual_Quaternion_Algebra_Part_I_Fundamentals_28Feb2017.pdf)

### Mapping from joint rates to pose rate:

$$
\mathrm{vec} {}^0\underline{\dot{x}_{N}} = \mathrm{J} ({\underline{x}}) \dot{\theta}
$$

Where $\dot{\theta}$ is the joint velocity a vector. This Jacobian represents a mapping from joint rate to a frame velocity


## Compatibility

Supports C++ 11 or higher.

| Compiler | Version           | Remarks |
| -------- | ----------------- | ------- |
| gcc      | 5.5.0 or higher.  |         |
| clang    | 7.0.0 or higher.  |         |
| msvc     | 16.5.4 or higher. |         |


## References

[1] Adorno, B.V. (2017) *Robot Kinematic Modeling and Control Based on Dual
Quaternion Algebra Part I: Fundamentals*, *ReasearchGate*. Available at:
https://hal.science/hal-01478225v1 (Accessed: April 9, 2023).

[2] Valverde, A. (2018) *Spacecraft Robot Kinematics Using Dual
Quaternions*, *MDPI*. Available at:
https://www.dcsl.gatech.edu/papers/mdpi18%20(Printed).pdf (Accessed:
April 9, 2023).

## Notes


