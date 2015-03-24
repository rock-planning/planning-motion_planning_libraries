#ifndef _MOTION_PLANNING_LIBRARIES_CONFIG_HPP_
#define _MOTION_PLANNING_LIBRARIES_CONFIG_HPP_

#include <math.h>

#include <string>
#include <iomanip> 

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
    MOV_FORWARD_TURN,
    MOV_BACKWARD_TURN,
    MOV_POINTTURN,
    MOV_LATERAL
};

// TODO Add OMPL planners
enum Planners {
    UNDEFINED_PLANNER, // Let the environment decide.
    ANYTIME_DSTAR, // AD*
    ANYTIME_NONPARAMETRIC_ASTAR, // ANA*
    ANYTIME_ASTAR // ARA*
};

enum MplErrors {
    MPL_ERR_NONE,
    MPL_ERR_UNDEFINED,
    MPL_ERR_MISSING_START,
    MPL_ERR_MISSING_GOAL,
    MPL_ERR_MISSING_TRAV,
    MPL_ERR_MISSING_START_GOAL,
    MPL_ERR_MISSING_START_TRAV,
    MPL_ERR_MISSING_GOAL_TRAV,
    MPL_ERR_MISSING_START_GOAL_TRAV=9,
    MPL_ERR_PLANNING_FAILED,
    MPL_ERR_WRONG_STATE_TYPE,
    MPL_ERR_INITIALIZE_MAP,
    MPL_ERR_SET_START_GOAL,
    MPL_ERR_START_ON_OBSTACLE,
    MPL_ERR_GOAL_ON_OBSTACLE,
    MPL_ERR_START_GOAL_ON_OBSTACLE
};

/**
 * All speeds should be >= 0, multipliers have to be >=1.
 * 
 */
struct Speeds {
    
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
    
    Speeds() : 
           mSpeedForward(0.0),
           mSpeedBackward(0.0),
           mSpeedLateral(0.0),
           mSpeedTurn(0.0),
           mSpeedPointTurn(0.0),
           mMultiplierForward(1),
           mMultiplierBackward(1),
           mMultiplierLateral(1),
           mMultiplierTurn(1),
           mMultiplierPointTurn(1) {
    }
    
    Speeds(double forward, double backward, double lateral, double turn, double pointturn) :
            mSpeedForward(forward), mSpeedBackward(backward), mSpeedLateral(lateral), 
            mSpeedTurn(turn), mSpeedPointTurn(pointturn) {
    }
    
    // Checks if any speed value has been set.
    bool isSet() {
        return (mSpeedForward != 0 || mSpeedBackward != 0 || 
                mSpeedLateral != 0 || mSpeedTurn != 0 || 
                mSpeedPointTurn != 0);
    }
    
    std::string toString() {
        if(!isSet()) {
            return "";
        } else {
            std::stringstream ss;
            ss << "F B L T PT: " << std::setw(8) << std::setprecision(3) << mSpeedForward << " " <<
                    std::setw(8) << std::setprecision(3) << mSpeedBackward << " " << 
                    std::setw(8) << std::setprecision(3) << mSpeedLateral << " " << 
                    std::setw(8) << std::setprecision(3) << mSpeedTurn << " " << 
                    std::setw(8) << std::setprecision(3) << mSpeedPointTurn;
            return ss.str();
        }
    }
};

/**
 *  Contains the configuration for all planning libraries.
 *  Just ignore the non relevant parameters.
 * TODO: Split into PlanningConfig and RobotConfig
 */
struct Config {
    Config() : mPlanningLibType(LIB_SBPL), 
            mEnvType(ENV_XY),
            mPlanner(UNDEFINED_PLANNER),
            mSearchUntilFirstSolution(false),
            mReplanDuringEachUpdate(false),
            mReplanOnNewStartPose(false),
            mMaxAllowedSampleDist(-1),
            mSpeeds(),
            mFootprintRadiusMinMax(0,0),  
            mFootprintLengthMinMax(0,0),
            mFootprintWidthMinMax(0,0),
            mNumFootprintClasses(10),
            mTimeToAdaptFootprint(40.0),
            mAdaptFootprintPenalty(20.0),
            mSBPLEnvFile(),
            mSBPLMotionPrimitivesFile(), 
            mSBPLForwardSearch(true),
            mNumIntermediatePoints(0),
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
    enum Planners mPlanner;
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
    // Either a radius or a rectangle has to be defined. If the system cannot adapt it's
    // footprint use the same value for first and second.
    std::pair<double,double> mFootprintRadiusMinMax; // Minimal / maximal footprint radius.
    std::pair<double,double> mFootprintLengthMinMax; // Size along the x-axis. 
    std::pair<double,double> mFootprintWidthMinMax; // Size along the y-axis.
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
    // Can be used to create and use intermediate points for each motion primitive.
    // E.g. if you want to get 10 points per primitive, you have
    // to set this variable to 8 (8 + start and end point).
    unsigned int mNumIntermediatePoints;
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
