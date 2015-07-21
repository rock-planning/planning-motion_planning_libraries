#ifndef _MOTION_PLANNING_LIBRARIES_OMPL_HPP_
#define _MOTION_PLANNING_LIBRARIES_OMPL_HPP_

#include <ompl/base/StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>

#include <motion_planning_libraries/AbstractMotionPlanningLibrary.hpp>

namespace motion_planning_libraries
{

/**
 * Finds the path with minimal cost from start to goal using a traversability map. 
 * The orientation of the robot cannot be regarded, because
 * controll problems cannot be optimized in OMPL yet. Using SE2StateSpace with
 * a MotionValidator for the orientation does not create driveable trajectories as well.
 * 
 * \todo "For XYTHETA: A steering angle of 0.04 matches a turning velocity of 45° per sec.
 *      Higher values allows nearly every turning. Why? Something to do with step size?"
 * \todo "Add collision checking for arm movement."
 */
class Ompl : public AbstractMotionPlanningLibrary
{
 protected: 
    ompl::base::StateSpacePtr mpStateSpace;
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    ompl::base::ProblemDefinitionPtr mpProblemDefinition;
    ompl::base::PlannerPtr mpPlanner;
    ompl::base::OptimizationObjectivePtr mpMultiOptimization;
    ompl::base::PathPtr mpPathInGridOmpl;
      
 public: 
    Ompl(Config config = Config());

    /**
     * Tries to find a valid path for \a time seconds.
     * If this method is called several times it will optimize the found solution.
     */
    virtual bool solve(double time);
};

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_OMPL_HPP_
