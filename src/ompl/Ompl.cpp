#include "Ompl.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>

#include <motion_planning_libraries/ompl/validators/TravMapValidator.hpp>
#include <motion_planning_libraries/ompl/objectives/TravGridObjective.hpp>
#include <base/Logging.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning_libraries
{

Ompl::Ompl(Config config) : MotionPlanningLibraries(config), mGridWidth(0), mGridHeight(0) {
}
 
bool Ompl::initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            boost::shared_ptr<TravData> grid_data) {
    
    mGridWidth = grid_width;
    mGridHeight = grid_height;
    
    // Defines a geometric problem in XY.
    switch (mConfig.mEnvType) {
        case ENV_XY: {
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
            break;
        }
        
        // Will define a control problem in SE2 (X, Y, THETA).
        case ENV_XYTHETA: {
            mpStateSpace = ompl::base::StateSpacePtr(new ob::SE2StateSpace());
            ob::RealVectorBounds bounds(2);
            bounds.setLow (0, 0);
            bounds.setHigh(0, grid_width);
            bounds.setLow (1, 0);
            bounds.setHigh(1, grid_height);
            mpStateSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
            mpStateSpace->setLongestValidSegmentFraction(1/(double)grid_width);
            
            mpControlSpace = ompl::control::ControlSpacePtr(
                    new ompl::control::RealVectorControlSpace(mpStateSpace, 2));
            ompl::base::RealVectorBounds cbounds(2);
            cbounds.setLow(0, -mConfig.mRobotForwardVelocity);
            cbounds.setHigh(0, mConfig.mRobotForwardVelocity);
            cbounds.setLow(1, -mConfig.mRobotRotationalVelocity);
            cbounds.setHigh(1, mConfig.mRobotRotationalVelocity);
            mpControlSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);
            
            ompl::control::SpaceInformationPtr control_space_inf = ompl::control::SpaceInformationPtr(
                    new ompl::control::SpaceInformation(mpStateSpace,mpControlSpace));
            mpODESolver = ompl::control::ODESolverPtr(
                    new ompl::control::ODEBasicSolver<> (control_space_inf, &kinematic_car_ode));
            // setStatePropagator can also be used to define a post-propagator.
            control_space_inf->setStatePropagator(
                    ompl::control::ODESolver::getStatePropagator(mpODESolver));
            
            // Control space information inherits from base space informartion.
            mpSpaceInformation = control_space_inf;
            
            break;
        }
    }
    
    mpTravMapValidator = ob::StateValidityCheckerPtr(new TravMapValidator(
                mpSpaceInformation, grid_width, grid_height, grid_data, mConfig.mEnvType));
    mpSpaceInformation->setStateValidityChecker(mpTravMapValidator);
    mpSpaceInformation->setup();
        
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

bool Ompl::setStartGoal(int start_x, int start_y, double start_yaw, 
        int goal_x, int goal_y, double goal_yaw) {
    
    ob::ScopedState<> start_ompl(mpStateSpace);
    ob::ScopedState<> goal_ompl(mpStateSpace);
    
    switch (mConfig.mEnvType) {
        case ENV_XY: {
            start_ompl->as<ob::RealVectorStateSpace::StateType>()->values[0] = start_x;
            start_ompl->as<ob::RealVectorStateSpace::StateType>()->values[1] = start_y;
            
            goal_ompl->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_x;
            goal_ompl->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_y;
            break;
        }
        case ENV_XYTHETA: {
            start_ompl->as<ob::SE2StateSpace::StateType>()->setX(start_x);
            start_ompl->as<ob::SE2StateSpace::StateType>()->setY(start_y);
            start_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(start_yaw);
            
            goal_ompl->as<ob::SE2StateSpace::StateType>()->setX(goal_x);
            goal_ompl->as<ob::SE2StateSpace::StateType>()->setY(goal_y);
            goal_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(goal_yaw);
            break;
        }
    }
    
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
        switch (mConfig.mEnvType) {
            case ENV_XY: {
                const ompl::base::RealVectorStateSpace::StateType* state = 
                    (*it)->as<ompl::base::RealVectorStateSpace::StateType>();
                    
                // Convert back to world coordinates.
                base::samples::RigidBodyState grid_pose;
                grid_pose.position[0] = state->values[0];
                grid_pose.position[1] = state->values[1];
                grid_pose.position[2] = 0;
                grid_pose.orientation = Eigen::Quaterniond::Identity();
                
                path.push_back(grid_pose);
                counter++;
                break;
            }
            case ENV_XYTHETA: {
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
                break;
            }
        }
    }
    LOG_INFO("Trajectory contains %d states", counter); 
    return true;
}

// PRIVATE
ompl::base::OptimizationObjectivePtr Ompl::getBalancedObjective(
    const ompl::base::SpaceInformationPtr& si) {

    mpPathLengthOptimization = ob::OptimizationObjectivePtr(
            new ob::PathLengthOptimizationObjective(si));
    
    mpTravGridOjective = ob::OptimizationObjectivePtr(new TravGridObjective(si, 
            mpTravGrid, mpTravData, mGridWidth, mGridHeight, mConfig.mEnvType));

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(mpPathLengthOptimization, 1.0);
    opt->addObjective(mpTravGridOjective, 1.0);
    mpMultiOptimization = ompl::base::OptimizationObjectivePtr(opt);

    return mpMultiOptimization;
}

} // namespace motion_planning_libraries
