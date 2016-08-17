#include "Config.hpp"
#include <base-logging/Logging.hpp>

namespace motion_planning_libraries {

std::string MovementTypesString[] = {
    "Undefined", 
    "Forward", 
    "Backward", 
    "Forward Turn", 
    "Backward Turn", 
    "Pointturn", 
    "Lateral",
    "Lateral Turn"
};  

std::string MplErrorsString[] = {
    "No error", 
    "Undefined error", 
    "Missing: Start", 
    "Missing: Goal", 
    "Missing: Trav-Map", 
    "Missing: Start, Goal", 
    "Missing: Start, Trav-Map", 
    "Missing: Goal, Trav-Map", 
    "Not used",
    "Missing: Start, Goal, Trav-Map", 
    "Start lies on an obstacle", 
    "Goal lies on an obstacle", 
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Not used",
    "Start and Goal lie on an obstacle", 
    "Planning failed", 
    "Wrong state type (start/goal correct?)", 
    "Map initialization failed", 
    "Start/goal could not be set", 
    "Replanning not required",
    "End of trajectory does not reach the goal position"
}; 


Config::Config() : mPlanningLibType(LIB_SBPL), 
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
                   mEscapeTrajRadiusFactor(1.0),
                   mJointBorders()
{
    if(mNumPrimPartition < 1) {
        LOG_WARN("Number of sub-primitives (mNumPrimPartition) has to be at least 1 ");
        mNumPrimPartition = 1;
    }
}


} // end namespace motion_planning_libraries
