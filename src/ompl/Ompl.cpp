#include "Ompl.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/config.h>

#include <global_path_planner/ompl/validators/TravMapValidator.hpp>
#include <global_path_planner/ompl/objectives/TravGridObjective.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace global_path_planner
{

Ompl::Ompl() : GlobalPathPlanner(), mGridWidth(0), mGridHeight(0) {
}
 
bool Ompl::initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            boost::shared_ptr<TravData> grid_data) {
    
    mGridWidth = grid_width;
    mGridHeight = grid_height;
    
    mpStateSpace = ob::StateSpacePtr(new ob::SE2StateSpace());
    ob::RealVectorBounds bounds(2);
    bounds.setLow (0, 0);
    bounds.setHigh(0, grid_width);
    bounds.setLow (1, 0);
    bounds.setHigh(1, grid_height);
    mpStateSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    // TODO Is this the correct 'longest valid segment fraction'?
    mpStateSpace->setLongestValidSegmentFraction(1/(double)grid_width);
      
    // Create a space information object using the traversability map validator.
    mpSpaceInformation = ob::SpaceInformationPtr(new ob::SpaceInformation(mpStateSpace));
    mpTravMapValidator = ob::StateValidityCheckerPtr(new TravMapValidator(
                mpSpaceInformation, grid_width, grid_height, grid_data));
    mpSpaceInformation->setStateValidityChecker(mpTravMapValidator);
        
    // Create problem definition.        
    mpProblemDefinition = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(mpSpaceInformation));
    mpProblemDefinition->setOptimizationObjective(getBalancedObjective(mpSpaceInformation));
    
    // Uses the currently set start and goal.
    setStartGoal(mStartGrid.position.x(), mStartGrid.position.y(), mStartGrid.getYaw(), 
            mGoalGrid.position.x(), mGoalGrid.position.y(), mGoalGrid.getYaw());
    
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

bool Ompl::setStartGoal(int start_x, int start_y, double start_yaw, 
        int goal_x, int goal_y, double goal_yaw) {
    
    ob::ScopedState<> start_ompl(mpStateSpace);
    start_ompl->as<ob::SE2StateSpace::StateType>()->setX(start_x);
    start_ompl->as<ob::SE2StateSpace::StateType>()->setY(start_y);
    start_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(start_yaw);

    ob::ScopedState<> goal_ompl(mpStateSpace);
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setX(goal_x);
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setY(goal_y);
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(goal_yaw);

    mpProblemDefinition->setStartAndGoalStates(start_ompl, goal_ompl);
    
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
            new TravGridObjective(si, mpTravData, mGridWidth, mGridHeight));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(mpPathLengthOptimization, 1.0);
    opt->addObjective(mpTravGridOjective, 1.0);
    mpMultiOptimization = ompl::base::OptimizationObjectivePtr(opt);

    return mpMultiOptimization;
}

} // namespace global_path_planner
