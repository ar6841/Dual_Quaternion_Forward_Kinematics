#pragma once
#include "pose_dual_quaternion.h"

// What does this file contain?
// Create a dynamically linked list of the joints in the robot
// Create a class RobotLinks that contains the joint information as a linked list
// The joint_data is passed by refrence so it can be constantly changing

namespace forwardk_dualquat
{

template<typename T>
class JointNode
{
    public:
        DH::DH_joint<T>& joint_data;
        JointNode<T>* next;
       
        JointNode()
        {
            //Constructor for JointNode
        }
        // Function to return the current pose tranformation DualQuaternion between current and prev
        dualquat::DualQuaternion<T> pose_current_prev()
        {
            return pose_dualquat::Pose_frame_i_iprev(joint_data);
        }
};

template<typename T>
class RobotLinks
{
    static_assert(std::is_floating_point<T>::value,
        "Template parameter T must be floating_point type.");

    private:
        JointNode<T>* head;

    public:
    //Construct the link
        RobotLinks() 
        {
            head = nullptr;
        }

    //Delete the link
      /*  ~RobotForward()
        {
            //ADD destructor logic
        }*/

    // method to add a new JointNode to the list
        void addJoint(const DH::DH_joint<T>& newJoint) 
        {
            JointNode<T>* newJointNode = new JointNode<T>;
            newJointNode->joint_data = newJoint;
            newJointNode->next = head;
            head = newJointNode;
        }

        // EXPERIMENTAL method to remove a JointNode from the list 
        // IMPORTANT: Make sure to avoid dangling refrence error
        void removeJoint(const DH::DH_joint<T>& jointToRemove) 
        {
            JointNode<T>* currentJointNode = head;
            JointNode<T>* previousJointNode = nullptr;

            while (currentJointNode != nullptr) {
                if (currentJointNode->joint_data->joint_ID == jointToRemove->joint_ID) {
                    if (previousJointNode == nullptr) 
                    {
                        head = currentJointNode->next;
                    } 
                    else 
                    {
                        previousJointNode->next = currentJointNode->next;
                    }
                    delete jointToRemove;
                    delete currentJointNode;
                    break;
                }
                previousJointNode = currentJointNode;
                currentJointNode = currentJointNode->next;
            }
        } // End of experimental

        // method to access a JointNode in the list by index
        DH::DH_joint<T> getJoint(const int& index) 
        {
            JointNode<T>* currentJointNode = head;
            for(int i=0; i<index; i++)
            {
                if(currentJointNode != nullptr)
                {
                    currentJointNode = currentJointNode->next;
                }
                else
                {
                    break;
                }

            }
            return currentJointNode->joint_data;
        }
        // EXPERIMENTAL method to access a JointNode in the list by ID
        /*
        DH::DH_joint<T> getJoint(const DH::DH_joint<T>& jointToSearch) 
        {
            JointNode<T>* currentJointNode = head;
            while (currentJointNode != nullptr && ID != currentJointNode->joint_data->joint_ID) 
            {
                currentJointNode = currentJointNode->next;
            }
            return currentJointNode->joint_data;
        } // end of EXPERIMENTAL
        */
};

}