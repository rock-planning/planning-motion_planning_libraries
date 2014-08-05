#include "OmplEnvSHERPA.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <motion_planning_libraries/ompl/validators/TravMapValidator.hpp>
#include <motion_planning_libraries/ompl/objectives/TravGridObjective.hpp>
#include <motion_planning_libraries/ompl/spaces/SherpaStateSpace.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning_libraries
{
    
// PUBLIC
OmplEnvSHERPA::OmplEnvSHERPA(Config config) : Ompl(config) {
}
 
bool OmplEnvSHERPA::initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data) { 

    LOG_INFO("Create OMPL SHERPA environment");
    
    SherpaStateSpace* sherpa_state_space = new SherpaStateSpace();
    ob::RealVectorBounds bounds(2);
    bounds.setLow (0, 0);
    bounds.setHigh(0, trav_grid->getCellSizeX());
    bounds.setLow (1, 0);
    bounds.setHigh(1, trav_grid->getCellSizeY());
    sherpa_state_space->setBounds(bounds);
    mpStateSpace = ob::StateSpacePtr(sherpa_state_space);
    
    //mpStateSpace->setLongestValidSegmentFraction(1/100.0);
    
    mpSpaceInformation = ob::SpaceInformationPtr(
            new ob::SpaceInformation(mpStateSpace));
 
    mpTravMapValidator = ob::StateValidityCheckerPtr(new TravMapValidator(
                mpSpaceInformation, trav_grid, grid_data, mConfig));
    mpSpaceInformation->setStateValidityChecker(mpTravMapValidator);
    // 1/mpStateSpace->getMaximumExtent() (max dist between two states) -> resolution of one meter.
    mpSpaceInformation->setStateValidityCheckingResolution (1/mpStateSpace->getMaximumExtent());
    mpSpaceInformation->setup();
        
    // Create problem definition.        
    mpProblemDefinition = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(mpSpaceInformation));
    // Create optimizations and balance them in getBalancedObjective().
    mpPathLengthOptimization = ob::OptimizationObjectivePtr(
        new ob::PathLengthOptimizationObjective(mpSpaceInformation));
    mpTravGridObjective = ob::OptimizationObjectivePtr(new TravGridObjective(mpSpaceInformation, false,
            trav_grid, grid_data, mConfig));
    mpProblemDefinition->setOptimizationObjective(getBalancedObjective(mpSpaceInformation));

    if(mConfig.mSearchUntilFirstSolution) { // Not optimizing planner, 
        mpPlanner = ob::PlannerPtr(new og::RRTConnect(mpSpaceInformation));
    } else { // Optimizing planners use all the available time to improve the solution.
        mpPlanner = ob::PlannerPtr(new og::RRTstar(mpSpaceInformation));
        // Allows to configure the max allowed dist between two samples.
        ompl::base::ParamSet param_set = mpPlanner->params();
        param_set.setParam("range", "1.0");
    }

    // Set the problem instance for our planner to solve
    mpPlanner->setProblemDefinition(mpProblemDefinition);
    mpPlanner->setup(); // Calls mpSpaceInformation->setup() as well.
    
    return true;
}

bool OmplEnvSHERPA::setStartGoal(struct State start_state, struct State goal_state) {
    
    double start_x = start_state.getPose().position[0];
    double start_y = start_state.getPose().position[1];
    double start_yaw = start_state.getPose().getYaw();
    double start_length = start_state.getLength();
    double start_width = start_state.getLength();
    double goal_x = goal_state.getPose().position[0];
    double goal_y = goal_state.getPose().position[1];
    double goal_yaw = start_state.getPose().getYaw();
    double goal_length = goal_state.getLength();
    double goal_width = goal_state.getLength();
    
    ob::ScopedState<> start_ompl(mpStateSpace);
    ob::ScopedState<> goal_ompl(mpStateSpace);
    
    start_ompl->as<SherpaStateSpace::StateType>()->setX(start_x);
    start_ompl->as<SherpaStateSpace::StateType>()->setY(start_y);
    start_ompl->as<SherpaStateSpace::StateType>()->setYaw(start_yaw);
    start_ompl->as<SherpaStateSpace::StateType>()->setLength(start_length);
    start_ompl->as<SherpaStateSpace::StateType>()->setWidth(start_width);
    
    goal_ompl->as<SherpaStateSpace::StateType>()->setX(goal_x);
    goal_ompl->as<SherpaStateSpace::StateType>()->setY(goal_y);
    goal_ompl->as<SherpaStateSpace::StateType>()->setYaw(goal_yaw);
    goal_ompl->as<SherpaStateSpace::StateType>()->setLength(goal_length);
    goal_ompl->as<SherpaStateSpace::StateType>()->setWidth(goal_width);
            
    mpProblemDefinition->setStartAndGoalStates(start_ompl, goal_ompl);
     
    return true;
}
    
bool OmplEnvSHERPA::fillPath(std::vector<struct State>& path) {
    // Downcast from Path to PathGeometric is valid.
    std::vector<ompl::base::State*> path_states = 
            boost::static_pointer_cast<og::PathGeometric>(mpPathInGridOmpl)->getStates();
    std::vector<ompl::base::State*>::iterator it = path_states.begin();

    int counter = 0;
    for(;it != path_states.end(); ++it) { 
        const SherpaStateSpace::StateType* state = 
            (*it)->as<SherpaStateSpace::StateType>();
            
        // Convert back to world coordinates.
        base::samples::RigidBodyState grid_pose;
        grid_pose.position[0] = state->getX();
        grid_pose.position[1] = state->getY();
        grid_pose.position[2] = 0;
        grid_pose.orientation = Eigen::AngleAxis<double>(state->getYaw(), 
                base::Vector3d(0,0,1));
        
        path.push_back(State(grid_pose, state->getLength(), state->getWidth()));
        counter++;
    }
    LOG_INFO("Trajectory contains %d states", counter); 
    return true;
}

// PROTECTED
ompl::base::OptimizationObjectivePtr OmplEnvSHERPA::getBalancedObjective(
    const ompl::base::SpaceInformationPtr& si) {

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    //opt->addObjective(mpPathLengthOptimization, 1.0);
    opt->addObjective(mpTravGridObjective, 1.0);
    mpMultiOptimization = ompl::base::OptimizationObjectivePtr(opt);

    return mpMultiOptimization;
}

} // namespace motion_planning_libraries
