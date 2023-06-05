#pragma once
#include "pose_dual_quaternion.h"
#include <vector>

/*

Class RobotLinks that contains the joint information as a Vector
The joint_data is passed by refrence so it can be constantly changing

NOTE: All functions were kept 'inline', this reduces readability but improves performance.

*/ 
namespace Kinematics
{

template<typename T>
class RobotLinks
{
    static_assert(std::is_floating_point<T>::value,
        "Template parameter T must be floating_point type.");
        
    private:
        std::vector<DH::DH_joint<T>*> LinkedJoints;

        void Check_ERROR_OUTOFBOUNDS(const std::string& error, const int& pos); //Index out of bounds error

    public:

        void addJoint(DH::DH_joint<T>& newJoint){ LinkedJoints.push_back(&newJoint);} //Push the refrence to the joint created.

        void deleteJoint(const int& pos); //Remove a joint from the linkage and memory

        void replaceJoint(DH::DH_joint<T>& newJoint, const int& pos); //Replace a joint and put another in its

        DH::DH_joint<T>* getJoint(const int& pos); //Function to return a pointer to the joint @pos index

        int getNumJointsTotal() { return LinkedJoints.size();}

        /*
            Function to return the tranformation between the end effector and the inertial frame
            Calculate forward kinematics UPTO joint NumJoints
        */ 
        dualquat::DualQuaternion<T> ComputeForwardKinematics(const int& NumJoints = -1);

        dualquat::DualQuaternion<T> ComputeForwardKinematics(const Eigen::Matrix<T,Eigen::Dynamic,1>& joint_vec, const int& NumJoints=-1);

        Eigen::Matrix<T,Eigen::Dynamic,1> getJointDotVec(); //Function to return a column vector of (joint variables)_dot

        Eigen::Matrix<T,Eigen::Dynamic,1> getJointVec(); //Function to return a column vector of (joint variables)

        /*
            Function to return a column vector of (joint variables) and swtich bool on or off
        */
        Eigen::Matrix<T,Eigen::Dynamic,1> getJointVec(const bool& offset);

        /*
            Function to set the joint variables, given a vector of joint variables
        */
        void setJointVec(const Eigen::Matrix<T,Eigen::Dynamic,1>& joint_new);

        /*
            Function to set the joint variables, given a vector of joint variables and swtich bool on or off
        */ 
        void setJointVec(const Eigen::Matrix<T,Eigen::Dynamic,1>& joint_new, const bool& offset);

        
}; // class RobotLinks

template<typename T>
inline void RobotLinks<T>::Check_ERROR_OUTOFBOUNDS(const std::string& error, const int& pos)
{
            if(pos>LinkedJoints.size()-1)
            {
                throw std::invalid_argument(("\n Position argument out of bounds in function : " + error));
            }
}

template<typename T>
inline void RobotLinks<T>::deleteJoint(const int& pos)
{
    Check_ERROR_OUTOFBOUNDS("deleteJoint()", pos);
    delete LinkedJoints[pos];
    LinkedJoints.erase(LinkedJoints.begin() + pos);
}

template<typename T>
inline void RobotLinks<T>::replaceJoint(DH::DH_joint<T>& newJoint, const int& pos)
{
    Check_ERROR_OUTOFBOUNDS("replaceJoint()", pos);
    delete LinkedJoints[pos];
    LinkedJoints[pos] = &newJoint;
}

template<typename T>
inline DH::DH_joint<T>* RobotLinks<T>::getJoint(const int& pos) //Function to return a pointer to the joint @pos index
{ 
    Check_ERROR_OUTOFBOUNDS("getJoint()", pos);
    return LinkedJoints[pos];
}

/*
    Function to return the tranformation between the end effector and the inertial frame
    Calculate forward kinematics UPTO joint NumJoints
*/ 
template<typename T>
inline dualquat::DualQuaternion<T> RobotLinks<T>::ComputeForwardKinematics(const int& NumJoints)
{
    int joint_n = ((NumJoints == -1)? LinkedJoints.size()-1 : NumJoints-1); 

    Check_ERROR_OUTOFBOUNDS("ComputeForwardKinematics()",joint_n);

    T type; //detect type for identety function (just a placeholder)

    dualquat::DualQuaternion<T> q_curr = dualquat::identity(type);

    for(int i = 0; i<=joint_n; i++)
    {
        q_curr = q_curr*Pose_frame_iprev_i(*LinkedJoints[i]);
    }

    return q_curr;
}
/*
    Function to calculate the pose in dualquat space, given a robot linkage and a joint vector
*/
template<typename T>
inline dualquat::DualQuaternion<T> RobotLinks<T>::ComputeForwardKinematics(
    const Eigen::Matrix<T,Eigen::Dynamic,1>& joint_vec, const int& NumJoints)
{
    int joint_n = ((NumJoints == -1)? LinkedJoints.size()-1 : NumJoints-1); 

    Check_ERROR_OUTOFBOUNDS("ComputeForwardKinematics()",joint_n);

    T type; //detect type for identety function (just a placeholder)

    dualquat::DualQuaternion<T> q_curr = dualquat::identity(type);

    for(int i = 0; i<=joint_n; i++)
    {
        q_curr = q_curr*Pose_frame_iprev_i(DH::DH_joint<T>(*LinkedJoints[i],joint_vec(i,0)));
    }

    return q_curr;
}

/*
    Function to return a column vector of (joint variables)_dot
*/ 
template<typename T>
inline Eigen::Matrix<T,Eigen::Dynamic,1> RobotLinks<T>::getJointDotVec()
{   
    Eigen::Matrix<T,Eigen::Dynamic,1> joint_dot_vec(LinkedJoints.size(),1);

    for(int i=0; i<LinkedJoints.size(); i++)
    {
        joint_dot_vec(i,0) = *(LinkedJoints[i]->joint_var_dot_i); //joint_var_dot is a pointer as well
    }

    return joint_dot_vec;
}

/*
    Function to return a column vector of (joint variables)
*/ 
template<typename T>
inline Eigen::Matrix<T,Eigen::Dynamic,1> RobotLinks<T>::getJointVec()
{   
    Eigen::Matrix<T,Eigen::Dynamic,1> joint_vec(LinkedJoints.size(),1);

    for(int i=0; i<LinkedJoints.size() ;i++)
    {
        joint_vec(i,0) = *(LinkedJoints[i]->joint_var_i); //joint_var is a pointer as well
    }

    return joint_vec;
}

template<typename T>
inline Eigen::Matrix<T,Eigen::Dynamic,1> RobotLinks<T>::getJointVec(const bool& offset)
{   
    Eigen::Matrix<T,Eigen::Dynamic,1> joint_vec(LinkedJoints.size(),1);

    for(int i=0; i<LinkedJoints.size() ;i++)
    {
        joint_vec(i,0) = LinkedJoints[i]->getJointVar(offset);
    }

    return joint_vec;
}
/*
    Function to pass a column vector of joint variables (assuming the offset is included) and set each joint
*/ 
template<typename T>
inline void RobotLinks<T>::setJointVec(const Eigen::Matrix<T,Eigen::Dynamic,1>& joint_new)
{   
    // Assumes that the joint offset is included
    for(int i=0; i<LinkedJoints.size() ;i++)
    {
        *(LinkedJoints[i]->joint_var_i) = joint_new(i,0); //joint_var is a pointer as well
    }

}

template<typename T>
inline void RobotLinks<T>::setJointVec(const Eigen::Matrix<T,Eigen::Dynamic,1>& joint_new, const bool& offset)
{   
    for(int i=0; i<LinkedJoints.size() ;i++)
    {
        LinkedJoints[i]->setJointVar(joint_new, offset);
    }
}

} //namespace Kinematics