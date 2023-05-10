#pragma once
/**
 * This header file provides functions and objects related to the Screw Theory approach to computing dual quaternions
 * 
 * This approach proposes optimizations beyond the DH approach.
 * 
 * Angles are stored in degrees here
 * 
 * TODO: add mass, link COM, 
 * 
 * TODO: There is an important assumption made that the joint variables have a home position of 0,
 * They stayrt from the identity unit dual quaternion.
 * The joint variables are being calculated with countercloclwise theta, and +z direction d
 * 
*/

// TODO: use convert_to_dualquat() in dualquat_helper which converts from screw paramers to dual quaternion.
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "dualquat/dualquat_helper.h"

namespace Kinematics
{

template<typename T>
struct ScrewParameters
{
    T theta,d;
    Vector3<T> l, const Vector3<T> m;
    dualquat::DualQuaternion<T> return_pose(){return convert_to_dualquat(const Vector3<T>& l, const Vector3<T>& m, T theta, T d)}
    
};

}