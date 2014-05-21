#ifndef _MOTION_PLANNING_LIBRARIES_CONFIG_HPP_
#define _MOTION_PLANNING_LIBRARIES_CONFIG_HPP_

#include <string>

namespace motion_planning_libraries
{

enum PlanningLibraryType {LIB_SBPL, LIB_OMPL};    
enum EnvType {ENV_XY, ENV_XYTHETA};

/**
 *  Contains the configuration for all planning libraries.
 *  The non relevant configurations can just be ignored.
 */
struct Config {
    Config() : mPlanningLibType(LIB_SBPL), 
            mEnvType(ENV_XY),
            mRobotWidth(0.5), 
            mRobotLength(0.5), 
            mRobotForwardVelocity(0.4), 
            mRobotBackwardVelocity(0.4),
            mRobotRotationalVelocity(1.0),
            mSearchUntilFirstSolution(false), 
            // SBPL specific configuration
            mSBPLEnvFile(),
            mSBPLMotionPrimitivesFile(), 
            mSBPLForwardSearch(true) {
    }
    
    enum PlanningLibraryType mPlanningLibType;
    enum EnvType mEnvType; // currently 2D and 2D + orientation.
    double mRobotWidth; // along the y-axis, sideward
    double mRobotLength; // along the x-axis, forward
    double mRobotForwardVelocity; // m/sec.
    double mRobotBackwardVelocity; // m/sec.
    double mRobotRotationalVelocity; // rad/sec. 
    bool mSearchUntilFirstSolution;
    
    // SBPL specific configuration
    std::string mSBPLEnvFile;
    std::string mSBPLMotionPrimitivesFile;
    bool mSBPLForwardSearch;
};

} // end namespace motion_planning_libraries

#endif
