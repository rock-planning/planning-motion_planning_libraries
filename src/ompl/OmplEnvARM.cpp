#include "OmplEnvARM.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <motion_planning_libraries/ompl/validators/TravMapValidator.hpp>
#include <motion_planning_libraries/ompl/objectives/TravGridObjective.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning_libraries
{
    
// PUBLIC
OmplEnvARM::OmplEnvARM(Config config) : Ompl(config) {
}
 
bool OmplEnvARM::initialize_arm() {

    if(mConfig.mJointBorders.size() == 0) {
        LOG_WARN("No joints/joint borders defined, arm cannot be initialized");
    }

    LOG_INFO("Create OMPL RealVector(%d) environment", mConfig.mJointBorders.size());
    
    mpStateSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(mConfig.mJointBorders.size()));
    ob::RealVectorBounds bounds(mConfig.mJointBorders.size());
    
    std::vector< std::pair<double,double> >::iterator it = mConfig.mJointBorders.begin();
    for(int i=0; it != mConfig.mJointBorders.end(); it++, i++) {
        bounds.setLow(i, it->first);   
        bounds.setHigh(i, it->second); 
    }
    
    mpStateSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    
    // Which value for arm planning?
    //mpStateSpace->setLongestValidSegmentFraction(1/(double)grid_width);
    
    mpSpaceInformation = ob::SpaceInformationPtr(new ob::SpaceInformation(mpStateSpace));
    // Add your validity checker here (arm collision).
    //mpTravMapValidator = ob::StateValidityCheckerPtr(new TravMapValidator(
    //            mpSpaceInformation, grid_width, grid_height, grid_data, mConfig.mEnvType));
    //mpSpaceInformation->setStateValidityChecker(mpTravMapValidator);
    mpSpaceInformation->setup();
        
    // Create problem definition. By default path length optimization will be used.    
    mpProblemDefinition = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(mpSpaceInformation));
   
    if(mConfig.mSearchUntilFirstSolution) { // Not optimizing planner, 
        mpPlanner = ob::PlannerPtr(new og::RRTConnect(mpSpaceInformation));
    } else { // Optimizing planners use all the available time to improve the solution.
        mpPlanner = ob::PlannerPtr(new og::RRTstar(mpSpaceInformation));
    }

    // Set the problem instance for our planner to solve
    mpPlanner->setProblemDefinition(mpProblemDefinition);
    mpPlanner->setup();
    
    return true;
}

bool OmplEnvARM::setStartGoal(struct State start_state, struct State goal_state) {
    
    assert(start_state.getJointAngles().size() == goal_state.getJointAngles().size());
    assert(start_state.getJointAngles().size() == mConfig.mJointBorders.size());
    
    if(start_state.getJointAngles().size() == 0) {
        LOG_WARN("Start and goal state do not contain any joint angles");
        return false;
    }
    
    ob::ScopedState<> start_ompl(mpStateSpace);
    ob::ScopedState<> goal_ompl(mpStateSpace);
    
    for(int i=0; i<start_state.getJointAngles().size(); ++i) {
        start_ompl->as<ob::RealVectorStateSpace::StateType>()->values[i] = 
                start_state.getJointAngles()[i];
        goal_ompl->as<ob::RealVectorStateSpace::StateType>()->values[i] = 
                goal_state.getJointAngles()[i];
    }
            
    mpProblemDefinition->setStartAndGoalStates(start_ompl, goal_ompl);
    
    return true;
}

bool OmplEnvARM::solve(double time) {
    return Ompl::solve(time);
}
    
bool OmplEnvARM::fillPath(std::vector<struct State>& path) {
    // Downcast from Path to PathGeometric is valid.
    std::vector<ompl::base::State*> path_states = 
            boost::static_pointer_cast<og::PathGeometric>(mpPathInGridOmpl)->getStates();
    std::vector<ompl::base::State*>::iterator it = path_states.begin();
    int counter = 0;
    for(;it != path_states.end(); ++it) { 
        const ompl::base::RealVectorStateSpace::StateType* state = 
            (*it)->as<ompl::base::RealVectorStateSpace::StateType>();
        
        State s;
        std::vector<double> joint_angles;
        for(int i=0; i<mConfig.mJointBorders.size(); ++i) {
            joint_angles.push_back(state->values[i]);
        }
        s.setJointAngles(joint_angles);
        path.push_back(s);
        counter++;
    }
    LOG_INFO("Trajectory contains %d states", counter); 
    return true;
}

} // namespace motion_planning_libraries
