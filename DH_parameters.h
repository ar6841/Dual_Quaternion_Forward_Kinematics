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
            T theta_i,d_i; //Joint variables (Don't make this private its too annoying)

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
                //constructor to initialize the DH joint object
            }

            explicit DH_joint(const T& theta, const T& alpha, const T& a, const T& d, const JointType& jointType, const T& offset) : 
            
            theta_i(theta), alpha_i(alpha), a_i(a), d_i(d), 
            theta_dot_i(T(0)), d_dot_i(T(0)), 
            theta_2dot_i(T(0)), d_2dot_i(T(0)),
            type_i(REVOLUTE), joint_offset(T(offset))
            {
                //constructor to initialize the DH joint object
            }
            // TODO: add more constructors
            T getJointVar();
            void setJointVar(const T& q);
            
    }; // class DH_joint

    template<typename T>
    inline T DH_joint<T>::getJointVar()
    {
        switch(type_i)
        {
            case REVOLUTE:
                return theta_i - joint_offset;
                break;

            case PRISMATIC:
                return d_i - joint_offset;
                break;
                
            case MIX:
                return NULL; //Add support for mixed joints
            default:
                return NULL;
                break;
            
        }
    
    }
    template<typename T>
    inline void DH_joint<T>::setJointVar(const T& q)
    {
        switch(type_i)
        {
            case REVOLUTE:
                theta_i = q+joint_offset;
            break;

            case PRISMATIC:
                d_i = q + joint_offset;
            break;

            case MIX: //Add support for mixed joints
            break;

            default:
            break;
            
        }
    }



} // namespace DH_joint