#ifndef _MOTION_PLANNING_LIBRARIES_STATE_HPP_
#define _MOTION_PLANNING_LIBRARIES_STATE_HPP_

#include <string>
#include <sstream>
#include <vector>

#include <base/samples/RigidBodyState.hpp>

namespace motion_planning_libraries
{

enum StateType {STATE_EMPTY, STATE_POSE, STATE_ARM};

/**
 * Defines the state of the system which could be the pose (xytheta) 
 * OR the joint angles of the arm. Angles have always be defined in rad from -M_PI to M_PI.
 */
struct State {

 private:
    enum StateType mStateType;
    base::samples::RigidBodyState mPose;
    std::vector<double> mJointAngles;
    
 public:
    State() {
        mPose.initUnknown();
        // Invalid position is used to check if this state contains a rbs.
        mPose.invalidatePosition();
        mStateType = STATE_EMPTY;
    }

    State(base::samples::RigidBodyState rbs) : mPose(rbs) {
        mStateType = STATE_POSE;
    }
    
    State(std::vector<double> joint_angles) : mJointAngles(joint_angles) {
        mPose.initUnknown();
        // Invalid position is used to check if this state contains a rbs.
        mPose.invalidatePosition();        
        mStateType = STATE_ARM;
    }
    
    base::samples::RigidBodyState getPose() {
        return mPose;
    }
    
    std::vector<double> getJointAngles() {
        return mJointAngles;
    }
    
    int getNumJoints() {
        return mJointAngles.size();
    }
    
    enum StateType getStateType() {
        return mStateType;
    }
    
    void setPose(base::samples::RigidBodyState pose) {
        mPose = pose;
        mStateType = STATE_POSE;
    }
    
    void setJointAngles(std::vector<double>& joint_angles) {
        mJointAngles = joint_angles;
        mStateType = STATE_ARM;
    }
     
    // Returns a string of the contained data.
    std::string getString() {
        std::stringstream ss;
        switch(mStateType) {
            case STATE_EMPTY: {
                ss << "Empty";
                break;
            }   
            case STATE_POSE: {
                ss << "x y z theta: " << mPose.position[0] << " " << 
                        mPose.position[1] << " " << 
                        mPose.position[2] << " " << 
                        mPose.getYaw();
                break;
            }  
            case STATE_ARM: {
                ss << "Joint angles: ";
                for(unsigned int i=0; i < mJointAngles.size(); ++i) {
                    ss << mJointAngles[i] << " ";
                }
                break;
            }  
        }
        return ss.str();
    }
};

} // end namespace motion_planning_libraries

#endif
