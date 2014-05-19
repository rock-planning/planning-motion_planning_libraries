#ifndef _MOTION_PLANNING_LIBRARIES_OMPL_HPP_
#define _MOTION_PLANNING_LIBRARIES_OMPL_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/State.h>

#include <motion_planning_libraries/MotionPlanningLibraries.hpp>

namespace motion_planning_libraries
{

/**
 * Finds the path with minimal cost from start to goal using a traversability map. 
 * The orientation of the robot cannot be regarded, because
 * controll problems cannot be optimized in OMPL yet. Using SE2StateSpace with
 * a MotionValidator for the orientation does not create driveable trajectories as well.
 */
class Ompl : public AbstractMotionPlanningLibrary
{
 private: 
    ompl::base::StateSpacePtr mpStateSpace;
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    ompl::base::StateValidityCheckerPtr mpTravMapValidator;
    ompl::base::ProblemDefinitionPtr mpProblemDefinition;
    ompl::base::PlannerPtr mpPlanner;
    ompl::base::OptimizationObjectivePtr mpPathLengthOptimization;
    ompl::base::OptimizationObjectivePtr mpPathClearanceOptimization;
    ompl::base::OptimizationObjectivePtr mpMultiOptimization;
    ompl::base::OptimizationObjectivePtr mpMaxMinClearance;
    ompl::base::OptimizationObjectivePtr mpTravGridObjective;
    ompl::base::PathPtr mpPathInGridOmpl;
    ompl::control::ODESolverPtr mpODESolver;
    ompl::control::ControlSpacePtr mpControlSpace;
    
    size_t mGridWidth;
    size_t mGridHeight;
      
 public: 
    Ompl(Config config = Config());

    /**
     * (Re-)creates the complete ompl environment.
     */
    virtual bool initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data);
    
    /**
     * Sets the global start and goal poses (in grid coordinates) in OMPL.
     */ 
    virtual bool setStartGoal(int start_x, int start_y, double start_yaw, 
            int goal_x, int goal_y, double goal_yaw);
    
    /**
     * Tries to find a valid path for \a time seconds.
     * If this method is called several times it will optimize the found solution.
     */
    virtual bool solve(double time);
        
    /**
     * Converts the ompl path to an rigid body state path (both in grid coordinates).
     */
    virtual bool fillPath(std::vector<base::samples::RigidBodyState>& path);
    
 private:
    /**
     * Creates a combined obtimization objective which tries to minimize the
     * costs of the trav grid.
     */ 
    ompl::base::OptimizationObjectivePtr getBalancedObjective(
        const ompl::base::SpaceInformationPtr& si);
    
        // Definition of the ODE for the kinematic car.
    static const void kinematicCarOde (const ompl::control::ODESolver::StateType& q, 
            const ompl::control::Control* control, 
            ompl::control::ODESolver::StateType& qdot)
    {
        const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        const double theta = q[2];
        double carLength = 0.2;

        // Zero out qdot
        qdot.resize (q.size (), 0);

        qdot[0] = u[0] * cos(theta);
        qdot[1] = u[0] * sin(theta);
        qdot[2] = u[0] * tan(u[1]) / carLength;
    }
    
    static const void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, 
            const double duration, ompl::base::State* result)
    {
        ompl::base::SO2StateSpace SO2;
        // Ensure that the car's resulting orientation lies between 0 and 2*pi.
        ompl::base::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
        SO2.enforceBounds(s[1]);
    }
};

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_OMPL_HPP_
