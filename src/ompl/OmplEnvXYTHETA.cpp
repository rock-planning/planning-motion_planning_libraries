#include "OmplEnvXYTHETA.hpp"

#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/control/planners/rrt/RRT.h>

#include <motion_planning_libraries/ompl/validators/TravMapValidator.hpp>
#include <motion_planning_libraries/ompl/objectives/TravGridObjective.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace motion_planning_libraries
{ 
  
double OmplEnvXYTHETA::mCarLength = 2.0;   
    
// PUBLIC
OmplEnvXYTHETA::OmplEnvXYTHETA(Config config) : Ompl(config) {
    double length = std::max(mConfig.mFootprintLengthMinMax.first, mConfig.mFootprintLengthMinMax.second);
    if(length == 0) {
        length = std::max(mConfig.mFootprintRadiusMinMax.first, 
            mConfig.mFootprintRadiusMinMax.second);
        if(length == 0) {
            LOG_WARN("No length has been defined, use default %4.2f instead", OmplEnvXYTHETA::mCarLength);
            length = OmplEnvXYTHETA::mCarLength;
        } else {
            LOG_WARN("No length has been defined, use max radius %4.2f instead", length);
        }
    }
    mCarLength = length;
}
 
bool OmplEnvXYTHETA::initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data) {
  
    // Will define a control problem in SE2 (X, Y, THETA).
    LOG_INFO("Create OMPL SE2 environment");
    
    mpStateSpace = ompl::base::StateSpacePtr(new ob::SE2StateSpace());
    ob::RealVectorBounds bounds(2);
    bounds.setLow (0, 0);
    bounds.setHigh(0, trav_grid->getCellSizeX());
    bounds.setLow (1, 0);
    bounds.setHigh(1, trav_grid->getCellSizeY());
    mpStateSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    mpStateSpace->setLongestValidSegmentFraction(1/(double)trav_grid->getCellSizeX());
    
    mpControlSpace = ompl::control::ControlSpacePtr(
            new ompl::control::RealVectorControlSpace(mpStateSpace, 2));
    ompl::base::RealVectorBounds cbounds(2);
    // Requires a control-planner.
    
    // Uses the average turning speed.
    double turning_speed = (mConfig.mSpeeds.mSpeedTurn + mConfig.mSpeeds.mSpeedPointTurn) / 2.0;
    if(turning_speed == 0) {
        LOG_WARN("Rotational velocity of zero is not allowed, abort");
        return false;
    }
    cbounds.setLow(0, -mConfig.mSpeeds.mSpeedBackward);
    cbounds.setHigh(0, mConfig.mSpeeds.mSpeedForward);
    cbounds.setLow(1, -turning_speed);
    cbounds.setHigh(1, turning_speed);
    mpControlSpace->as<ompl::control::RealVectorControlSpace>()->setBounds(cbounds);
    
    // Control space information inherits from base space informartion.
    mpControlSpaceInformation = ompl::control::SpaceInformationPtr(
            new ompl::control::SpaceInformation(mpStateSpace,mpControlSpace));
    // TODO Test the other ODE solvers.
    mpODESolver = ompl::control::ODESolverPtr(
            new ompl::control::ODEAdaptiveSolver<> (mpControlSpaceInformation, &simpleOde));
    // setStatePropagator can also be used to define a post-propagator.
    mpControlSpaceInformation->setStatePropagator(
            ompl::control::ODESolver::getStatePropagator(mpODESolver, &postPropagate));
            
    // TODO What does these methods do?
    mpControlSpaceInformation->setPropagationStepSize(4);
    mpControlSpaceInformation->setMinMaxControlDuration(1,10);

    mpTravMapValidator = ob::StateValidityCheckerPtr(new TravMapValidator(
                mpControlSpaceInformation, trav_grid, grid_data, mConfig));
    mpControlSpaceInformation->setStateValidityChecker(mpTravMapValidator);
    mpControlSpaceInformation->setup();
        
    // Create problem definition.        
    mpProblemDefinition = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(mpControlSpaceInformation));
    // Create optimizations and balance them in getBalancedObjective().
    mpPathLengthOptimization = ob::OptimizationObjectivePtr(
        new ob::PathLengthOptimizationObjective(mpControlSpaceInformation));
    mpTravGridObjective = ob::OptimizationObjectivePtr(new TravGridObjective(mpControlSpaceInformation, false,
            trav_grid, grid_data, mConfig));
    mpProblemDefinition->setOptimizationObjective(getBalancedObjective(mpControlSpaceInformation));
    
    // Control based planner, optimization is not supported by OMPL.
    mpPlanner = ob::PlannerPtr(new ompl::control::RRT(mpControlSpaceInformation));

    // Set the problem instance for our planner to solve
    mpPlanner->setProblemDefinition(mpProblemDefinition);
    mpPlanner->setup(); // Calls mpSpaceInformation->setup() as well.
    
    return true;
}

bool OmplEnvXYTHETA::setStartGoal(struct State start_state, struct State goal_state) {
    
    ob::ScopedState<> start_ompl(mpStateSpace);
    ob::ScopedState<> goal_ompl(mpStateSpace);
    
    double start_x = start_state.getPose().position[0];
    double start_y = start_state.getPose().position[1];
    double start_yaw = start_state.getPose().getYaw();
    double goal_x = goal_state.getPose().position[0];
    double goal_y = goal_state.getPose().position[1];
    double goal_yaw = goal_state.getPose().getYaw();
    
    start_ompl->as<ob::SE2StateSpace::StateType>()->setX(start_x);
    start_ompl->as<ob::SE2StateSpace::StateType>()->setY(start_y);
    start_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(start_yaw);
    
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setX(goal_x);
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setY(goal_y);
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(goal_yaw);

    mpProblemDefinition->setStartAndGoalStates(start_ompl, goal_ompl);
    
    return true;
}

bool OmplEnvXYTHETA::solve(double time) {
    return Ompl::solve(time);
}
    
bool OmplEnvXYTHETA::fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid) {
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
        
        path.push_back(State(grid_pose));
        counter++;
    }
    LOG_INFO("Trajectory contains %d states", counter); 
    return true;
}

// PROTECTED
ompl::base::OptimizationObjectivePtr OmplEnvXYTHETA::getBalancedObjective(
    const ompl::base::SpaceInformationPtr& si) {

    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(mpPathLengthOptimization, 1.0);
    opt->addObjective(mpTravGridObjective, 1.0);
    mpMultiOptimization = ompl::base::OptimizationObjectivePtr(opt);

    return mpMultiOptimization;
}

} // namespace motion_planning_libraries
