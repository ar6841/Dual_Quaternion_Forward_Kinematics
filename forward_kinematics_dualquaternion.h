#pragma once
#include "pose_dual_quaternion.h"
#include <vector>

/*

Class RobotLinks that contains the joint information as a Vector
The joint_data is passed by refrence so it can be constantly changing

NOTE: All functions were kept 'inline', this reduces readability but improves performance.

TODO: Add d_dot vector function
*/ 
namespace ForwardKinematics
{

template<typename T>
class RobotLinks
{
    static_assert(std::is_floating_point<T>::value,
        "Template parameter T must be floating_point type.");
        
    private:
        std::vector<DH::DH_joint<T>*> LinkedJoints;

        /* 
            Index out of bounds error
        */ 
        void Check_ERROR_OUTOFBOUNDS(const std::string& error, const int& pos)
        {
            if(pos>LinkedJoints.size()-1)
            {
                throw std::invalid_argument(("\n Position argument out of bounds in function : " + error));
            }
        }

    public:

        void addJoint(DH::DH_joint<T>& newJoint)
        {
            LinkedJoints.push_back(&newJoint); //Push the refrence to the joint created.
        }

        void deleteJoint(const int& pos)
        {
            Check_ERROR_OUTOFBOUNDS("deleteJoint()", pos);
            delete LinkedJoints[pos];
            LinkedJoints.erase(LinkedJoints.begin() + pos);
        }

        void replaceJoint(DH::DH_joint<T>& newJoint, const int& pos)
        {
            Check_ERROR_OUTOFBOUNDS("replaceJoint()", pos);
            delete LinkedJoints[pos];
            LinkedJoints[pos] = &newJoint;
        }

        /*
            Function to return a pointer to the joint @pos index
        */ 

        DH::DH_joint<T>* getJoint(const int& pos)
        { 
            Check_ERROR_OUTOFBOUNDS("getJoint()", pos);
            return LinkedJoints[pos];
        }

        int getNumJointsTotal()
        {
            return LinkedJoints.size();
        }

        /*
            Function to return the tranformation between the end effector and the inertial frame
            Calculate forward kinematics UPTO joint NumJoints
        */ 
        dualquat::DualQuaternion<T>
        ComputeForwardKinematics(const int& NumJoints = -1)
        {
            int joint_i = ((NumJoints == -1)? LinkedJoints.size()-1 : NumJoints-1); 

            Check_ERROR_OUTOFBOUNDS("ComputeForwardKinematics()",joint_i);
            
            T type; //detect type for identety function (just a placeholder)
            
            dualquat::DualQuaternion<T> q_curr = dualquat::identity(type);

            for(int i = 0; i<=joint_i; i++)
            {
                q_curr = q_curr*Pose_frame_iprev_i(*LinkedJoints[i]);
            }

            return q_curr;
        }
        /*
            Function to return a column vector of (joint variables)_dot
        */ 
        Eigen::Matrix<T,Eigen::Dynamic,1>
        getThetaDotVec()
        {   
            // TODO: account for prismatic joints
            Eigen::Matrix<T,Eigen::Dynamic,1> theta_dot_vec(LinkedJoints.size(),1);

            for(int i=0; i<LinkedJoints.size() ;i++)
            {
                    theta_dot_vec(i,0) = LinkedJoints[i]->theta_dot_i;
            }

            return theta_dot_vec;
        }
};


} //namespace ForwardKinematics