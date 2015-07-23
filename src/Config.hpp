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

/// \todo "Add OMPL planners"
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
    MPL_ERR_START_ON_OBSTACLE,
    MPL_ERR_GOAL_ON_OBSTACLE,
    MPL_ERR_START_GOAL_ON_OBSTACLE=21, // Do not enter another error before this entry!
    MPL_ERR_PLANNING_FAILED,
    MPL_ERR_WRONG_STATE_TYPE,
    MPL_ERR_INITIALIZE_MAP,
    MPL_ERR_SET_START_GOAL,
    MPL_ERR_REPLANNING_NOT_REQUIRED
};

/**
 * Describes the mobility of the system.
 * If a speed has been defined it will be used to define the 
 * forward/backward speed for the trajectory.
 * The minimal turning radius is used to create valid curves.
 * The multipliers are used by SBPL during planning. In addition
 * if a multiplier is set to 0 this movement will be deactivated.
 */
struct Mobility {
    // Defines the forward/backward speed of the system which will be assigned
    // to the trajectory.
    double mSpeed; // m/sec.
    // Used for cost calculations.
    double mTurningSpeed; // rad/sec
    // If > 0 allows to specify the minimal turning radius of the system in meter.
    // Without this not valid curves may be created.
    double mMinTurningRadius; 
    // Multipliers: Allows to define multipliers for each movement (used by SBPL).
    // If a multiplier is set to 0, this movement type will be deaAbstractMotionPlanningLibraryctivated.
    unsigned int mMultiplierForward;
    unsigned int mMultiplierBackward;
    unsigned int mMultiplierLateral;
    unsigned int mMultiplierForwardTurn;
    unsigned int mMultiplierBackwardTurn;
    unsigned int mMultiplierPointTurn;
    
    Mobility() : 
           mSpeed(0.0),
           mTurningSpeed(0.0),
           mMinTurningRadius(0.0),
           mMultiplierForward(0),
           mMultiplierBackward(0),
           mMultiplierLateral(0),
           mMultiplierForwardTurn(0),
           mMultiplierBackwardTurn(0),
           mMultiplierPointTurn(0) {
    }
    
    Mobility(double speed, double turning_speed, double min_turning_radius, 
             unsigned int mult_forward=0, 
             unsigned int mult_backward=0, 
             unsigned int mult_lateral=0, 
             unsigned int mult_forward_turn=0, 
             unsigned int mult_backward_turn=0, 
             unsigned int mult_pointturn=0) :
            mSpeed(speed), 
            mTurningSpeed(turning_speed),
            mMinTurningRadius(min_turning_radius),
            mMultiplierForward(mult_forward), 
            mMultiplierBackward(mult_backward),
            mMultiplierLateral(mult_lateral), 
            mMultiplierForwardTurn(mult_forward_turn), 
            mMultiplierBackwardTurn(mult_backward_turn),
            mMultiplierPointTurn(mult_pointturn) {
    }
    
    // Checks if any movement type has been set.
    bool isSet() {
        return (mMultiplierForward || mMultiplierBackward || mMultiplierLateral || 
                mMultiplierForwardTurn || mMultiplierBackwardTurn || mMultiplierPointTurn);
    }
    
    std::string toString() {
        std::stringstream ss;
        ss << "Speed: " << mSpeed << ", Min Turning Radius: " << mMinTurningRadius << 
                ", Multipliers (F B L FT BT PT): " << 
                mMultiplierForward << " " << mMultiplierBackward << " " << 
                mMultiplierLateral << " " << mMultiplierForwardTurn << " " << 
                mMultiplierBackwardTurn << " " << mMultiplierPointTurn << std::endl; 

        return ss.str();
    }
};

struct Replanning {
    // If set to true each call to plan() will try to solve the current problem / improve
    // the solution.
    bool mReplanDuringEachUpdate;
    bool mReplanOnNewStartPose;
    bool mReplanOnNewGoalPose;
    bool mReplanOnNewMap;
    // If > 0 this describes the minimal distance in the world frame in meter between start 
    // and goal to initiate a replanning if a new trav map or start pose has been received.
    // In other words: This parameter allows you to define a area around the
    // goal which prevents replanning (except a new goal pose has been received).
    // Especially for SBPL XYTHETA this parameter is meaningful by preventing unwanted 
    // circle paths.
    double mReplanMinDistStartGoal;
    
    Replanning() :
        mReplanDuringEachUpdate(false),
        mReplanOnNewStartPose(false),
        mReplanOnNewGoalPose(true),
        mReplanOnNewMap(true),
        mReplanMinDistStartGoal(0.0) {
    }
};

/**
 *  Contains the configuration for all planning libraries.
 *  Just ignore the non relevant parameters.
 * \todo "Split into PlanningConfig and RobotConfig?"
 */
struct Config {
    Config() : mPlanningLibType(LIB_SBPL), 
            mEnvType(ENV_XY),
            mPlanner(UNDEFINED_PLANNER),
            mSearchUntilFirstSolution(false), // use to 'just provide ptimal trajectories'?
            mReplanning(),
            mMobility(),
            mFootprintRadiusMinMax(0,0),  
            mFootprintLengthMinMax(0,0),
            mFootprintWidthMinMax(0,0),
            mNumFootprintClasses(10),
            mTimeToAdaptFootprint(40.0),
            mAdaptFootprintPenalty(20.0),
            mMaxAllowedSampleDist(-1),
            mSBPLEnvFile(),
            mSBPLMotionPrimitivesFile(), 
            mSBPLForwardSearch(true),
            mNumIntermediatePoints(0),
            mNumPrimPartition(2),
            mPrimAccuracy(0.25),
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
    struct Replanning mReplanning;
    
    // NAVIGATION
    struct Mobility mMobility;
    
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
    
    // OMPL
    // If mSearchUntilFirstSolution is set to false this parameter can be used to 
    // define the maximal allowed distance between two samples.
    // If it is set to a negative value or nan it will be ignored.
    double mMaxAllowedSampleDist;
     
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
    // Max distance in grids from the reached end position to the next discrete one.
    double mPrimAccuracy;
    
    // ARM MOTION PLANNING
    // defines the number and borders of the joints.  <low,high> in rad
    std::vector< std::pair<double,double> > mJointBorders; 
    
    void setJoints(int num, double lower_border=-M_PI, double upper_border=M_PI) {
        mJointBorders.empty();
        for(int i=0; i<num; ++i) {
            mJointBorders.push_back(std::pair<double,double>(lower_border, upper_border));
        }
    }
    
    // Calculates the max radius in meter using the specified footprint radius and rectangle.
    double getMaxRadius() {
        double max_radius = std::max(mFootprintRadiusMinMax.first, mFootprintRadiusMinMax.second);
        double max_length = std::max(mFootprintLengthMinMax.first, mFootprintLengthMinMax.second);
        double max_width = std::max(mFootprintWidthMinMax.first, mFootprintWidthMinMax.second);
        
        // Calculates radius using width and length.
        double max_radius2 = sqrt(pow(max_length/2.0, 2) + pow(max_width/2.0, 2));
        return std::max(max_radius, max_radius2);
    }
};

} // end namespace motion_planning_libraries

#endif
