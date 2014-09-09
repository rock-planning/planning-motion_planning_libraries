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

 public:
    enum StateType mStateType;
    base::samples::RigidBodyState mPose;
    std::vector<double> mJointAngles;
    double mLength;
    double mWidth;
    
    State() {
        mPose.initUnknown();
        // Invalid position is used to check if this state contains a rbs.
        mPose.invalidatePosition();
        mStateType = STATE_EMPTY;
        mLength = 0.0;
        mWidth = 0.0;
    }

    State(base::samples::RigidBodyState rbs) : mPose(rbs) {
        mStateType = STATE_POSE;
        mLength = 0.0;
        mWidth = 0.0;
    }
    
    State(base::samples::RigidBodyState rbs, double length, double width) : 
            mPose(rbs), mLength(length), mWidth(width) {
        mStateType = STATE_POSE;
    }
    
    State(std::vector<double> joint_angles) : mJointAngles(joint_angles) {
        mPose.initUnknown();
        // Invalid position is used to check if this state contains a rbs.
        mPose.invalidatePosition();        
        mStateType = STATE_ARM;
    }
    
    enum StateType getStateType() {
        return mStateType;
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
    
    double getLength() {
        return mLength;
    }

    double getWidth() {
        return mWidth;
    }
    
    void setPose(base::samples::RigidBodyState pose) {
        mPose = pose;
        mStateType = STATE_POSE;
    }
    
    void setJointAngles(std::vector<double>& joint_angles) {
        mJointAngles = joint_angles;
        mStateType = STATE_ARM;
    }
    
    void setFootprint(double length, double width) {
        mLength = length;
        mWidth = width;
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
                ss << "x y z theta length width: " << mPose.position[0] << " " << 
                        mPose.position[1] << " " << 
                        mPose.position[2] << " " << 
                        mPose.getYaw() << " " << 
                        mLength << " " <<
                        mWidth;
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
