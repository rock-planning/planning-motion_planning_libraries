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

double OmplEnvXY::mCarWidth = 2.0; 
    
// PUBLIC
OmplEnvXY::OmplEnvXY(Config config) : Ompl(config), 
        mGridWidth(0), 
        mGridHeight(0) {
    mCarWidth = mConfig.mRobotLength;
}
 
bool OmplEnvXY::initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data) {
    
    mGridWidth = grid_width;
    mGridHeight = grid_height;  

    LOG_INFO("Create OMPL RealVector(2) environment");
    
    mpStateSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow (0, 0);
    bounds.setHigh(0, grid_width);
    bounds.setLow (1, 0);
    bounds.setHigh(1, grid_height);
    mpStateSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    mpStateSpace->setLongestValidSegmentFraction(1/(double)grid_width);
    
    mpSpaceInformation = ob::SpaceInformationPtr(
            new ob::SpaceInformation(mpStateSpace));
 
    mpTravMapValidator = ob::StateValidityCheckerPtr(new TravMapValidator(
                mpSpaceInformation, grid_width, grid_height, trav_grid, grid_data, mConfig.mEnvType));
    mpSpaceInformation->setStateValidityChecker(mpTravMapValidator);
    mpSpaceInformation->setup();
        
    // Create problem definition.        
    mpProblemDefinition = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(mpSpaceInformation));
    // Create optimizations and balance them in getBalancedObjective().
    mpPathLengthOptimization = ob::OptimizationObjectivePtr(
        new ob::PathLengthOptimizationObjective(mpSpaceInformation));
    mpTravGridObjective = ob::OptimizationObjectivePtr(new TravGridObjective(mpSpaceInformation, 
            trav_grid, grid_data, grid_width, grid_height, mConfig.mEnvType));
    mpProblemDefinition->setOptimizationObjective(getBalancedObjective(mpSpaceInformation));

    if(mConfig.mSearchUntilFirstSolution) { // Not optimizing planner, 
        mpPlanner = ob::PlannerPtr(new og::RRTConnect(mpSpaceInformation));
    } else { // Optimizing planners use all the available time to improve the solution.
        mpPlanner = ob::PlannerPtr(new og::RRTstar(mpSpaceInformation));
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
    
bool OmplEnvXY::fillPath(std::vector<struct State>& path) {
    // Downcast from Path to PathGeometric is valid.
    std::vector<ompl::base::State*> path_states = 
            boost::static_pointer_cast<og::PathGeometric>(mpPathInGridOmpl)->getStates();
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
