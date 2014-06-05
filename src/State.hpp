#ifndef _MOTION_PLANNING_LIBRARIES_STATE_HPP_
#define _MOTION_PLANNING_LIBRARIES_STATE_HPP_

#include <vector>

#include <base/samples/RigidBodyState.hpp>

namespace motion_planning_libraries
{

/**
 * Defines the state of the system which could be the pose (xytheta) 
 * or the joint angles of the arm.
 */
struct State {
    State(base::samples::RigidBodyState rbs) : mPose(rbs) {
    }
    
    State(std::vector<double> joint_angles) : mJointAngles(joint_angles) {
    }

    base::samples::RigidBodyState mPose;
    std::vector<double> mJointAngles;
};

} // end namespace motion_planning_libraries

#endif
