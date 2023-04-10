#pragma once
/**
 * This header file provides functions anf objects related to the DH convention
 * 
 * Angles are stored in degrees here
*/
namespace DH
{
    //TODO: Add prismatic and revolute types
    template<typename T>
    class DH_joint
    {
        public:
            T theta_i,d_i; //Joint variables
            const T alpha_i,a_i; //Joint parameters (contraints)
            DH_joint()
            {}

            DH_joint(T theta,T alpha,T a,T d) : theta_i(theta), alpha_i(alpha), a_i(a), d_i(d)
            {
                //constructor to initialize the DH joint object
            }
            
    }; // class DH_joint
} // namespace DH_joint