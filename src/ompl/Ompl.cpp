#include "Ompl.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/config.h>

#include <global_path_planner/validators/TravMapValidator.hpp>
#include <global_path_planner/objectives/TravGridObjective.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace global_path_planner
{

Ompl::Ompl() : GlobalPathPlanner() {
}
 
bool Ompl::initialize() {
    mpStateSpace = ob::StateSpacePtr(new ob::SE2StateSpace());
    ob::RealVectorBounds bounds(2);
    bounds.setLow (0, 0);
    bounds.setHigh(0, mpTravGrid->getCellSizeX());
    bounds.setLow (1, 0);
    bounds.setHigh(1, mpTravGrid->getCellSizeY());
    mpStateSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    // TODO Is this the correct 'longest valid segment fraction'?
    mpStateSpace->setLongestValidSegmentFraction(1/(double)mpTravGrid->getCellSizeX());
      
    // Create a space information object using the traversability map validator.
    mpSpaceInformation = ob::SpaceInformationPtr(new ob::SpaceInformation(mpStateSpace));
    mpTravMapValidator = ob::StateValidityCheckerPtr(new TravMapValidator(mpSpaceInformation, mpTravGrid));
    mpSpaceInformation->setStateValidityChecker(mpTravMapValidator);
        
    // Create problem definition.        
    mpProblemDefinition = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(mpSpaceInformation));
    mpProblemDefinition->setOptimizationObjective(getBalancedObjective(mpSpaceInformation));
    
    // Set start and goal.  
    setStartGoal(mStartGrid, mGoalGrid, mpStateSpace, mpProblemDefinition);
    
    // Stop if the found solution is nearly a straight line.
    double dist_start_goal = (mStartGrid.position - mGoalGrid.position).norm() * 1.1;
    LOG_INFO("Set cost threshold to %4.2f", dist_start_goal);
    mpPathLengthOptimization->setCostThreshold(ob::Cost(dist_start_goal));  
        
    mpPlanner = ob::PlannerPtr(new og::RRTstar(mpSpaceInformation));
    // Set the problem instance for our planner to solve
    mpPlanner->setProblemDefinition(mpProblemDefinition);
    mpPlanner->setup(); // Calls mpSpaceInformation->setup() as well.
    
    return true;
}

bool Ompl::solve(double time) {
    ob::PlannerStatus solved = mpPlanner->solve(time);

    if (solved)
    {
        mpPathInGridOmpl = mpProblemDefinition->getSolutionPath();
        return true;
    } else {
        return false;
    }
}
    
bool Ompl::fillPath(std::vector<base::samples::RigidBodyState>& path) {
    // Downcast from Path to PathGeometric is valid.
    std::vector<ompl::base::State*> path_states = 
            boost::static_pointer_cast<og::PathGeometric>(mpPathInGridOmpl)->getStates();
    std::vector<ompl::base::State*>::iterator it = path_states.begin();

    int counter = 0;
    for(;it != path_states.end(); ++it) {
        const ompl::base::SE2StateSpace::StateType* state = 
            (*it)->as<ompl::base::SE2StateSpace::StateType>();
            
        // Convert back to world coordinates.
        base::samples::RigidBodyState grid_pose;
        grid_pose.position[0] = state->getX();
        grid_pose.position[1] = state->getY();
        grid_pose.position[2] = 0;
        grid_pose.orientation = Eigen::AngleAxis<double>(state->getYaw(), 
                base::Vector3d(0,0,1));
        
        path.push_back(grid_pose);
        counter++;
    }
    LOG_INFO("Trajectory contains %d states", counter); 
    return true;
}

// PRIVATE
ompl::base::OptimizationObjectivePtr Ompl::getBalancedObjective(
    const ompl::base::SpaceInformationPtr& si) {

    mpPathLengthOptimization = ob::OptimizationObjectivePtr(
            new ob::PathLengthOptimizationObjective(si));
    mpTravGridOjective = ob::OptimizationObjectivePtr(
            new TravGridObjective(si, mpTravGrid));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(mpPathLengthOptimization, 1.0);
    opt->addObjective(mpTravGridOjective, 1.0);
    mpMultiOptimization = ompl::base::OptimizationObjectivePtr(opt);

    return mpMultiOptimization;
}

void Ompl::setStartGoal(base::samples::RigidBodyState& start_in_grid, 
        base::samples::RigidBodyState& goal_in_grid, 
        ompl::base::StateSpacePtr state_space, 
        ompl::base::ProblemDefinitionPtr& problem_definition) {
    
    ob::ScopedState<> start_ompl(state_space);
    start_ompl->as<ob::SE2StateSpace::StateType>()->setX(start_in_grid.position.x());
    start_ompl->as<ob::SE2StateSpace::StateType>()->setY(start_in_grid.position.y());
    start_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(start_in_grid.getYaw());
    
    ob::ScopedState<> goal_ompl(state_space);
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setX(goal_in_grid.position.x());
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setY(goal_in_grid.position.y());
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(goal_in_grid.getYaw());
    
    LOG_INFO("Planning from (x,y,yaw) (%4.2f, %4.2f, %4.2f) to (%4.2f, %4.2f, %4.2f)", 
            start_in_grid.position.x(), start_in_grid.position.y(), start_in_grid.getYaw(),
            goal_in_grid.position.x(), goal_in_grid.position.y(), goal_in_grid.getYaw());
    
    problem_definition->setStartAndGoalStates(start_ompl, goal_ompl);
}

} // namespace global_path_planner
