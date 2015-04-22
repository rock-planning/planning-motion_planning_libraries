#ifndef _MOTION_PLANNING_LIBRARIES_STATE_HPP_
#define _MOTION_PLANNING_LIBRARIES_STATE_HPP_

#include <string>
#include <sstream>
#include <vector>

#include <base/samples/RigidBodyState.hpp>

#include "Config.hpp"

namespace motion_planning_libraries
{

enum StateType {STATE_EMPTY, STATE_POSE, STATE_ARM};

/**
 * Defines the state of the system which could be the pose (xytheta) 
 * OR the joint angles of the arm. Angles have always be defined in rad from -M_PI to M_PI.
 * Regarding SBPL XYTHETA planning: To each primitive type specific 
 * speed values has been assigned. This speeds are copied to this state structure,
 * but only for the first state of each primitive (not for the following intermediate 
 * points and the endpoint).
 */
struct State {

 public:
    // Has to be public to be used in ROCK, should not be modified directly.
    enum StateType mStateType;
    base::samples::RigidBodyState mPose;
    std::vector<double> mJointAngles;
    // Currently used to represent width and length.
    double mFootprintRadius;
    // Currently just used by SBPL
    // The first state of each primitive receives a prim id
    // and its speeds. The id of all other states stays -1.
    int mSBPLPrimId;
    // Contains the speeds which belong to the primitive.
    // Like the id only used for the first state of each primitive.
    Speeds mSpeeds;
    
    State() {
        mPose.initUnknown();
        // Invalid position is used to check if this state contains a rbs.
        mPose.invalidatePosition();
        mStateType = STATE_EMPTY;
        mFootprintRadius = 0.0;
        mSBPLPrimId = -1;
    }

    State(base::samples::RigidBodyState rbs) : mPose(rbs) {
        mStateType = STATE_POSE;
        mFootprintRadius = 0.0;
        mSBPLPrimId = -1;
    }
    
    State(base::samples::RigidBodyState rbs, double footprint_radius) : 
            mPose(rbs), mFootprintRadius(footprint_radius) {
        mStateType = STATE_POSE;
        mSBPLPrimId = -1;
    }
    
    State(std::vector<double> joint_angles) : mJointAngles(joint_angles) {
        mPose.initUnknown();
        // Invalid position is used to check if this state contains a rbs.
        mPose.invalidatePosition();
        mFootprintRadius = 0.0;
        mStateType = STATE_ARM;
        mSBPLPrimId = -1;
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
    
    double getFootprintRadius() {
        return mFootprintRadius;
    }
    
    unsigned int getFootprintClass(double fp_radius_min, double fp_radius_max, 
            unsigned int num_fp_classes) {
        assert (fp_radius_min <= fp_radius_max);
        assert (num_fp_classes > 0);
        return ((mFootprintRadius - fp_radius_min) / (fp_radius_max - fp_radius_min)) *  
                (num_fp_classes - 1) + 0.5; 
    }
    
    void setPose(base::samples::RigidBodyState pose) {
        mPose = pose;
        mStateType = STATE_POSE;
    }
    
    void setJointAngles(std::vector<double>& joint_angles) {
        mJointAngles = joint_angles;
        mStateType = STATE_ARM;
    }
    
    void setFootprintRadius(double radius) {
        mFootprintRadius = radius;
    }
    
    void setFootprintClass(double fp_radius_min, double fp_radius_max, 
            unsigned int num_fp_classes, unsigned int fp_class) {
        assert (fp_radius_min <= fp_radius_max);
        assert (num_fp_classes > 0);
        mFootprintRadius = fp_radius_min + (fp_radius_max - fp_radius_min) * 
                fp_class / (num_fp_classes - 1);
    }

    /**
     * Returns the abs-distance between both states.
     * If the states are not pose-states a negative number will be returned.
     */
    double dist(State state) {
        if(this->mStateType != STATE_POSE || state.mStateType != STATE_POSE) {
            return -1;
        }
        return (this->mPose.position - state.mPose.position).norm();
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
                ss << "x y z theta footprint-radius prim-id: " << mPose.position[0] << " " << 
                        mPose.position[1] << " " << 
                        mPose.position[2] << " " << 
                        mPose.getYaw() << " " << 
                        mFootprintRadius << " " <<
                        mSBPLPrimId;
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
