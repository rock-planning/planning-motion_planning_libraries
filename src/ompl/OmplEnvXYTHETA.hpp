#ifndef _MOTION_PLANNING_LIBRARIES_OMPL_ENVXYTHETA_HPP_
#define _MOTION_PLANNING_LIBRARIES_OMPL_ENVXYTHETA_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "Ompl.hpp"

namespace motion_planning_libraries
{

class OmplEnvXYTHETA : public Ompl
{
 private: 
    ompl::control::ControlSpacePtr mpControlSpace;
    ompl::control::SpaceInformationPtr mpControlSpaceInformation;
    ompl::base::StateValidityCheckerPtr mpTravMapValidator;
    ompl::base::OptimizationObjectivePtr mpPathLengthOptimization;
    ompl::base::OptimizationObjectivePtr mpMultiOptimization;
    ompl::base::OptimizationObjectivePtr mpTravGridObjective;
    ompl::control::ODESolverPtr mpODESolver;
      
    // Used in static member functions.
    static double mCarLength;  
      
 public: 
    OmplEnvXYTHETA(Config config = Config());

    /**
     * (Re-)creates the complete ompl environment.
     */
    virtual bool initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data);
    
    /**
     * Sets the global start and goal poses (in grid coordinates) in OMPL.
     */ 
    virtual bool setStartGoal(struct State start_state, struct State goal_state);
    
    /**
     * Tries to find a valid path for \a time seconds.
     * If this method is called several times it will optimize the found solution.
     */
    virtual bool solve(double time);
        
    /**
     * Converts the ompl path to an rigid body state path (both in grid coordinates).
     */
    virtual bool fillPath(std::vector<struct State>& path);
    
 private:
    // Definition of the ODE for the kinematic car. Calculates the delt/opt/software_transterra/install/log/planning/motion_planning_libraries-build.loga.
    static const void kinematicCarOde (const ompl::control::ODESolver::StateType& q, 
            const ompl::control::Control* control, 
            ompl::control::ODESolver::StateType& qdot)
    {
        
        // u contains [0] forward velocity and [1] rotational velocity
        const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        // q contains [0] x, [1] y and [2] current orientation. 
        const double theta = q[2];
        
        // Zero out qdot
        qdot.resize (q.size (), 0);

        // Calculates delta.
        qdot[0] = u[0] * cos(theta);
        qdot[1] = u[0] * sin(theta);
        qdot[2] = u[0] * tan(u[1]) / mCarLength;
    }
    
    // Just sets the delta using the velocities.
    static const void simpleOde (const ompl::control::ODESolver::StateType& q, 
            const ompl::control::Control* control, 
            ompl::control::ODESolver::StateType& qdot)
    {
        // u contains [0] forward velocity and [1] rotational velocity
        const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
        const double theta = q[2];
        
        // Zero out qdot
        qdot.resize (q.size (), 0);
        
        // Calculates the deltas XY and just set the rotational speed.
        qdot[0] = u[0] * cos(theta);
        qdot[1] = u[0] * sin(theta);
        qdot[2] = u[1];
    }
    
    static const void postPropagate(const ompl::base::State* state, const ompl::control::Control* control, 
            const double duration, ompl::base::State* result)
    {
        ompl::base::SO2StateSpace SO2;
        // Ensure that the car's resulting orientation lies between 0 and 2*pi.
        ompl::base::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
        SO2.enforceBounds(s[1]);
    }
    
 protected:  
    /**
     * Creates a combined optimization objective which tries to minimize the
     * costs of the trav grid.
     */ 
    ompl::base::OptimizationObjectivePtr getBalancedObjective(
        const ompl::base::SpaceInformationPtr& si);
};

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_OMPL_HPP_
