#pragma once
/**
 * This header file provides functions and objects related to the DH approach to computing dual quaternions
 * 
 * Angles are stored in degrees here
 * 
 * TODO: add mass, link COM, 
 * 
 * TODO: There is an important assumption made that the joint variables have a home position of 0,
 * They start from the identity unit dual quaternion (1,0).
 * The joint variables are being calculated with countercloclwise theta, and +z direction d
 * 
*/
namespace DH
{
    
enum JointType{REVOLUTE,PRISMATIC,MIX};
template<typename T>
class DH_joint
{
    private:
        const JointType type_i;

        // Only constuctors can call these functions
        void setJointVar(const T& theta, const T& d);

    public:
        T theta_i,d_i;

        T *joint_var_i, *joint_var_dot_i; // points to theta_i/d_i depending on Revolute or Prismatic

        T theta_dot_i, d_dot_i; //joint velocity

        T theta_2dot_i, d_2dot_i; // joint Acceleration

        const T alpha_i, a_i, joint_offset; //Joint constraints

        DH_joint()
        {}

        explicit DH_joint(const T& theta, const T& alpha, const T& a, const T& d) : 
        
        theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), 
        theta_dot_i(T(0)), d_dot_i(T(0)), 
        theta_2dot_i(T(0)), d_2dot_i(T(0)),
        type_i(REVOLUTE), joint_offset(T(0))
        {
            setJointVar(theta_i,d_i); //add the offset to the correct joint variable
        }

        explicit DH_joint(const T& theta, const T& alpha, const T& a, const T& d, const T& offset) : 
        
        theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), 
        theta_dot_i(T(0)), d_dot_i(T(0)), 
        theta_2dot_i(T(0)), d_2dot_i(T(0)),
        type_i(REVOLUTE), joint_offset(T(offset))
        {
            //constructor to initialize the DH joint object
            setJointVar(theta_i,d_i);
        }

        explicit DH_joint(const JointType& type, const T& theta, const T& alpha, const T& a, const T& d, const T& offset) : 
        theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), 
        theta_dot_i(T(0)), d_dot_i(T(0)), 
        theta_2dot_i(T(0)), d_2dot_i(T(0)),
        type_i(type), joint_offset(T(offset))
        {
            //constructor to initialize the DH joint object
            setJointVar(theta_i,d_i);
        }
        

        //Member functions
        T getJointVar(); // Avoid the unecessary calculation to imporve performance
        T getJointVar(const bool& offset);

        void setJointVar(const T& q); //asume no offset or offset is off
        void setJointVar(const T& q, const bool& offset);
        
}; // class DH_joint

template<typename T>
inline T DH_joint<T>::getJointVar()
{
    return *joint_var_i;
}

template<typename T>
inline T DH_joint<T>::getJointVar(const bool& offset)
{
    T result = offset ? T(1):T(0);
    return *joint_var_i - result*joint_offset;
}
/*
    Setter functions
*/
template<typename T>
inline void DH_joint<T>::setJointVar(const T& q)
{
  *joint_var_i = q;
}

template<typename T>
inline void DH_joint<T>::setJointVar(const T& q, const bool& offset)
{
    T result = offset ? T(1):T(0);
    *joint_var_i = q + result*joint_offset;
}

template<typename T>
inline void DH_joint<T>::setJointVar(const T& theta, const T& d)
{
    switch(type_i)
    {
        case REVOLUTE:
            theta_i = theta+joint_offset;
            joint_var_i = &theta_i;
            joint_var_dot_i = &theta_dot_i;
        break;

        case PRISMATIC:
            d_i = d + joint_offset;
            joint_var_i = &d_i;
            joint_var_dot_i = &d_dot_i;
        break;

        case MIX: //Add support for mixed joints
        break;

        default:
        break;
        
    }
}



} // namespace DH_joint