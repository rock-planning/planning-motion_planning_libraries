#ifndef _MOTION_PLANNING_LIBRARIES_CONFIG_HPP_
#define _MOTION_PLANNING_LIBRARIES_CONFIG_HPP_

#include <math.h>

#include <string>

#include <base/Logging.hpp>

namespace motion_planning_libraries
{

enum PlanningLibraryType {
    LIB_SBPL, 
    LIB_OMPL
}; 
   
enum EnvType {
    ENV_XY, 
    ENV_XYTHETA, 
    ENV_ARM
};

/**
 *  Contains the configuration for all planning libraries.
 *  Just ignore the non relevant parameters.
 */
struct Config {
    Config() : mPlanningLibType(LIB_SBPL), 
            mEnvType(ENV_XY),
            mSearchUntilFirstSolution(false),
            mRobotWidth(0.5), 
            mRobotLength(0.5), 
            mRobotForwardVelocity(0.4), 
            mRobotBackwardVelocity(0.4),
            mRobotRotationalVelocity(1.0),
            mSBPLEnvFile(),
            mSBPLMotionPrimitivesFile(), 
            mSBPLForwardSearch(true),
            mJointBorders() {
    }
    
    // GENERAL
    enum PlanningLibraryType mPlanningLibType;
    enum EnvType mEnvType;
    bool mSearchUntilFirstSolution;
    
    // NAVIGATION
    double mRobotWidth; // along the y-axis, sideward
    double mRobotLength; // along the x-axis, forward
    double mRobotForwardVelocity; // m/sec.
    double mRobotBackwardVelocity; // m/sec. positive value
    double mRobotRotationalVelocity; // rad/sec. positive value 
    // Sbpl specific configuration
    std::string mSBPLEnvFile;
    std::string mSBPLMotionPrimitivesFile;
    bool mSBPLForwardSearch;
    
    // ARM MOTION PLANNING
    // defines the number and borders of the joints.  <low,high> in rad
    std::vector< std::pair<double,double> > mJointBorders; 
    
    void setJoints(int num, double lower_border=-M_PI, double upper_border=M_PI) {
        mJointBorders.empty();
        for(int i=0; i<num; ++i) {
            mJointBorders.push_back(std::pair<double,double>(lower_border, upper_border));
        }
    }
};

} // end namespace motion_planning_libraries

#endif
