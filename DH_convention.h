#pragma once
/**
 * This header file provides functions anf objects related to the DH convention
 * 
 * Angles are stored in degrees here
 * 
 * TODO: add mass, link COM, 
*/
namespace DH
{
    //TODO: Add prismatic and revolute types
    template<typename T>
    class DH_joint
    {
        public:
            T theta_i,d_i; //Joint variables
            T theta_dot_i, d_dot_i; //joint Rates
            T theta_2dot_i, d_2dot_i; // joint Acceleration

            const T alpha_i,a_i; //Joint parameters (contraints)

            //const int joint_ID; //experimental 

            DH_joint()
            {}

            DH_joint(const T& theta,const T& alpha,const T& a,const T& d) : theta_i(theta), alpha_i(alpha), a_i(a), d_i(d)
            {
                //constructor to initialize the DH joint object
            }

            DH_joint(const T& theta, const T& alpha, const T& a, const T& d, const T& theta_dot, 
            const T& d_dot) : theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), theta_dot_i(theta_dot), d_dot_i(d_dot)
            {
                //constructor to initialize the DH joint object given velocities
            }

            DH_joint(const T& theta, const T& alpha, const T& a, const T& d, const T& theta_dot, 
            const T& d_dot, const T& theta_2dot, const T& d_2dot) : theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), theta_dot_i(theta_dot), d_dot_i(d_dot), theta_2dot_i(theta_2dot), d_2dot_i(d_2dot)
            {
                //constructor to initialize the DH joint object given velocities and acc
            }
            
    }; // class DH_joint

    // Function to create a pointer variable of type DH joint
    template<typename T>
    DH_joint<T>*
    CreateNewJoint(const T& theta,const T& alpha,const T& a,const T& d)
    {
        return new DH::DH_joint<T>(theta,alpha,a,d);
    }

} // namespace DH_joint