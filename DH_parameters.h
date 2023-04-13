#pragma once
/**
 * This header file provides functions and objects related to the DH approach to computing dual quaternions
 * 
 * Angles are stored in degrees here
 * 
 * TODO: add thet_dot initialization
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
    // ASSUMPTION: Only revolute types for now
    template<typename T>
    class DH_joint
    {
        private:
            JointType type_i;

        public:
            T theta_i,d_i; //Joint variables

            T theta_dot_i, d_dot_i; //joint velocity

            T theta_2dot_i, d_2dot_i; // joint Acceleration

            const T alpha_i, a_i, joint_offset; //Joint constraints

            DH_joint()
            {}

            DH_joint(const T& theta, const T& alpha, const T& a, const T& d) : 
            theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), 
            theta_dot_i(T(0)), d_dot_i(T(0)), 
            theta_2dot_i(T(0)), d_2dot_i(T(0)),
            type_i(REVOLUTE), joint_offset(T(0))
            {
                //constructor to initialize the DH joint object
            }

            DH_joint(const T& theta, const T& alpha, const T& a, const T& d, const T& theta_dot) : 
            theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), 
            theta_dot_i(theta_dot), d_dot_i(T(0)), 
            theta_2dot_i(T(0)), d_2dot_i(T(0)),
            type_i(REVOLUTE), joint_offset(T(0))
            {

            }

            DH_joint(const T& theta, const T& alpha, const T& a, const T& d, const T& theta_dot, 
            const T& d_dot) : 
            theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), 
            theta_dot_i(theta_dot), d_dot_i(d_dot), 
            theta_2dot_i(T(0)), d_2dot_i(T(0)),
            type_i(MIX), joint_offset(T(0))
            {
                //constructor to initialize the DH joint object given velocities
            }

            DH_joint(const T& theta, const T& alpha, const T& a, const T& d, const T& theta_dot, 
            const T& d_dot, const T& theta_2dot, const T& d_2dot) : 
            theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), 
            theta_dot_i(theta_dot), d_dot_i(d_dot), 
            theta_2dot_i(theta_2dot), d_2dot_i(d_2dot),
            type_i(MIX), joint_offset(T(0))
            {
                //constructor to initialize the DH joint object given velocities and acc
            }
            
    }; // class DH_joint

} // namespace DH_joint