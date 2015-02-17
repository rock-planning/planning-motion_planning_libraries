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
    ENV_ARM,
    ENV_SHERPA
};

enum MovementType {
    MOV_UNDEFINED,
    MOV_FORWARD,
    MOV_BACKWARD,
    MOV_TURN,
    MOV_POINTTURN,
    MOV_LATERAL
};

/**
 * All speeds should be positive values, multipliers have to be >=1.
 * 
 */
struct Speeds {
    Speeds() : 
           mSpeedForward(0.4),
           mSpeedBackward(0.4),
           mSpeedLateral(0.0),
           mSpeedTurn(0.1),
           mSpeedPointTurn(0.1),
           mMultiplierForward(1),
           mMultiplierBackward(1),
           mMultiplierLateral(1),
           mMultiplierTurn(1),
           mMultiplierPointTurn(1) {
    }
            
    double mSpeedForward; // m/sec.
    double mSpeedBackward; // m/sec.
    double mSpeedLateral; // m/sec.
    double mSpeedTurn; // rad/sec.
    double mSpeedPointTurn; // rad/sec.
    unsigned int mMultiplierForward;
    unsigned int mMultiplierBackward;
    unsigned int mMultiplierLateral;
    unsigned int mMultiplierTurn;
    unsigned int mMultiplierPointTurn;
};

/**
 *  Contains the configuration for all planning libraries.
 *  Just ignore the non relevant parameters.
 * TODO: Split into PlanningConfig and RobotConfig
 */
struct Config {
    Config() : mPlanningLibType(LIB_SBPL), 
            mEnvType(ENV_XY),
            mSearchUntilFirstSolution(false),
            mReplanDuringEachUpdate(false),
            mReplanOnNewStartPose(false),
            mMaxAllowedSampleDist(-1),
            mSpeeds(),
            mFootprintRadiusMinMax(0.5,0.5),  
            mNumFootprintClasses(10),
            mTimeToAdaptFootprint(40.0),
            mAdaptFootprintPenalty(20.0),
            mSBPLEnvFile(),
            mSBPLMotionPrimitivesFile(), 
            mSBPLForwardSearch(true),
            mUseIntermediatePoints(false),
            mNumPrimPartition(2),
            mJointBorders() {
       if(mNumPrimPartition < 1) {
           LOG_WARN("Number of sub-primitives (mNumPrimPartition) has to be at least 1 ");
           mNumPrimPartition = 1;
       }
    }
    
    // GENERAL
    enum PlanningLibraryType mPlanningLibType;
    enum EnvType mEnvType;
    // Defines whether the planner accepts the first found solution or uses the
    // complete available time.
    bool mSearchUntilFirstSolution; 
    // If set to true each call to plan() will try to solve the current problem / improve
    // the solution.
    bool mReplanDuringEachUpdate;
    bool mReplanOnNewStartPose;
    // If mSearchUntilFirstSolution is set to false this parameter can be used to 
    // define the maximal allowed distance between two samples.
    // If it is set to a negative value or nan ot will be ignored.
    double mMaxAllowedSampleDist;
    
    // NAVIGATION
    struct Speeds mSpeeds;
    
    // FOOTPRINT
    std::pair<double,double> mFootprintRadiusMinMax; // Minimal / maximal footprint radius.
    // Defines number of different footprint radii from min to max, used to discretize the search space.
    unsigned int mNumFootprintClasses;
    // Time in seconds to change the footprint from min to max.
    double mTimeToAdaptFootprint;
    // If the footprint is changed this time (sec) will be added to the costs.
    double mAdaptFootprintPenalty;
    
    // SBPL
    std::string mSBPLEnvFile;
    std::string mSBPLMotionPrimitivesFile;
    bool mSBPLForwardSearch;
    // Use SBPL MPrims Intermediate Points (in general ten per motion primitive).
    bool mUseIntermediatePoints;
    // Each primitive is divided into sub-movements.
    unsigned int mNumPrimPartition;
    
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
