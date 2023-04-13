#pragma once
#include "pose_dual_quaternion.h"

/*
Create a dynamically linked list of the joints in the robot
Create a class RobotLinks that contains the joint information as a linked list
The joint_data is passed by refrence so it can be constantly changing

NOTE: All functions were kept 'inline', this reduces readability but improves performance in this case.

TODO: Add d_dot vector function
*/ 
namespace forwardk_dualquat
{

template<typename T>
class JointNode
{
    public:
        DH::DH_joint<T>* joint_data;
        JointNode<T>* next;
       
        JointNode()
        {
            //Constructor for JointNode
        }

        // Function to return the current pose tranformation DualQuaternion between current and prev
        dualquat::DualQuaternion<T> pose_current_prev()
        {
            return Pose_frame_i_iprev(*joint_data); //Pass by refrence
        }
};

template<typename T>
class RobotLinks
{
    static_assert(std::is_floating_point<T>::value,
        "Template parameter T must be floating_point type.");

    private: // Data about this linkage
        JointNode<T>* head;
        int NUM_JOINTS; 

    public:
    // Construct the link
        RobotLinks() 
        {
            head = nullptr;
            NUM_JOINTS = 0; // let 0 represent the inertial frame
        }

    // Destructor to delete all the nodes in the list
        ~RobotLinks() 
        {
       
            JointNode<T>* current = head;
            while (current != NULL) 
            {
                JointNode<T>* next = current->next;  // save a pointer to the next node
                delete current;              // delete the current node
                current = next;              // move to the next node
            }
            head = NULL;    // set the head pointer to NULL to indicate the list is empty
        }
    //Delete the link
      /*  ~RobotForward()
        {
            //ADD destructor logic
        }*/

    // Method to add a new JointNode to the list
        void addJoint(DH::DH_joint<T>& newJoint) 
        {
            JointNode<T>* newJointNode = new JointNode<T>;
            newJointNode->joint_data = &newJoint;
            newJointNode->next = head;
            head = newJointNode;
            NUM_JOINTS++;
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

        // Method to access a JointNode in the list by index
        JointNode<T>* getJoint(const int& index) 
        {
            JointNode<T>* currentJointNode = head;
            int i=NUM_JOINTS;

            while(i>index && currentJointNode != nullptr)
            {
                currentJointNode = currentJointNode->next;
                i--;
            }
            return currentJointNode;
        }

        /* Return the number of total joints
        */
        int getNumJointsTotal() { return NUM_JOINTS; }  

        // Call this function to return the tranformation between the end effector and the inertial frame
        dualquat::DualQuaternion<T>
        ComputeForwardKinematics(const int& NumJoints = -1)
        {
            JointNode<T>* currentJointNode = head;
            
            int i = ((NumJoints == -1)? NUM_JOINTS : NumJoints); // DEFAULT VALUE = NUM_JOINTS

            T type; //detect type for identety function (just a placeholder)

            dualquat::DualQuaternion<T> q_curr = dualquat::identity(type);

            while(i>0 && currentJointNode != nullptr)
            {
                /*
                    IMPORTANT: Here we pre-multiply the DualQuaternions q_i_iprev
                    giving the final result when the loop ends as : q_EndEffector_intertialFrame

                    TODO: There is an important assumption made that the joint variables have a home position of 0,
                    They stayrt from the identity unit dual quaternion.
                    The joint variables are being calculated with countercloclwise theta, and +z direction d
                */
               q_curr = currentJointNode->pose_current_prev()*q_curr;
               currentJointNode = currentJointNode->next;
               i--;
            }
            return q_curr;
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