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

namespace Kinematics
{

template<typename T>
struct ScrewParameters
{
    T theta, d;

    const Eigen::Vector3<T> l;
    const Eigen::Vector3<T> m;

};

}