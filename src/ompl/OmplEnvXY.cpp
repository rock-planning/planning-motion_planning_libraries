#include "OmplEnvXY.hpp"

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
OmplEnvXY::OmplEnvXY(Config config) : Ompl(config) {
}
 
bool OmplEnvXY::initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data) { 

    LOG_INFO("Create OMPL RealVector(2) environment");
    
    mpStateSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow (0, 0);
    bounds.setHigh(0, trav_grid->getCellSizeX());
    bounds.setLow (1, 0);
    bounds.setHigh(1, trav_grid->getCellSizeY());
    mpStateSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    // Defines the divisor for each dimension.
    // E.g. width = 100, divisor 0.01 -> if dist(x1,x2) >= 1 motion validation required
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
        param_set.setParam("range", "0.5");
    }

    // Set the problem instance for our planner to solve
    mpPlanner->setProblemDefinition(mpProblemDefinition);
    mpPlanner->setup(); // Calls mpSpaceInformation->setup() as well.
    
    return true;
}

bool OmplEnvXY::setStartGoal(struct State start_state, struct State goal_state) {
    
    ob::ScopedState<> start_ompl(mpStateSpace);
    ob::ScopedState<> goal_ompl(mpStateSpace);
    
    double start_x = start_state.getPose().position[0];
    double start_y = start_state.getPose().position[1];
    double goal_x = goal_state.getPose().position[0];
    double goal_y = goal_state.getPose().position[1];
    
    start_ompl->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_x;
    start_ompl->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_y;
    
    goal_ompl->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x;
    goal_ompl->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y;
            
    mpProblemDefinition->setStartAndGoalStates(start_ompl, goal_ompl);
    
    // Stop if the found solution is nearly a straight line.
    double dist_start_goal = sqrt((start_x - goal_x) * (start_x - goal_x) + 
            (start_y - goal_y) * (start_y - goal_y)) * 1.1;
    LOG_INFO("Set cost threshold to %4.2f", dist_start_goal);
    mpPathLengthOptimization->setCostThreshold(ob::Cost(dist_start_goal));
    
    return true;
}

bool OmplEnvXY::solve(double time) {
    return Ompl::solve(time);
}
    
bool OmplEnvXY::fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid) {
    std::vector<ompl::base::State*> path_states = getPathStates(mpPathInGridOmpl);
    std::vector<ompl::base::State*>::iterator it = path_states.begin();

    int counter = 0;
    for(;it != path_states.end(); ++it) { 
        const ompl::base::RealVectorStateSpace::StateType* state = 
            (*it)->as<ompl::base::RealVectorStateSpace::StateType>();
            
        // Convert back to world coordinates.
        base::samples::RigidBodyState grid_pose;
        grid_pose.position[0] = state->values[0];
        grid_pose.position[1] = state->values[1];
        grid_pose.position[2] = 0;
        grid_pose.orientation = Eigen::Quaterniond::Identity();
        
        path.push_back(State(grid_pose));
        counter++;
    }
    LOG_INFO("Trajectory contains %d states", counter); 
    return true;
}

// PROTECTED
ompl::base::OptimizationObjectivePtr OmplEnvXY::getBalancedObjective(
    const ompl::base::SpaceInformationPtr& si) {

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(mpPathLengthOptimization, 1.0);
    opt->addObjective(mpTravGridObjective, 1.0);
    mpMultiOptimization = ompl::base::OptimizationObjectivePtr(opt);

    return mpMultiOptimization;
}

} // namespace motion_planning_libraries
