#ifndef _GLOBAL_PATH_PLANNER_OMPL_HPP_
#define _GLOBAL_PATH_PLANNER_OMPL_HPP_

#include <global_path_planner/GlobalPathPlanner.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>

namespace global_path_planner
{

/**
 * Finds the path with minimal cost from start to goal using a traversability map. 
 * The orientation of the robot cannot be regarded, because (it seems as if)
 * controll problems cannot be optimized in OMPL. 
 */
class Ompl : public GlobalPathPlanner
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
    ompl::base::OptimizationObjectivePtr mpTravGridOjective;
    ompl::base::PathPtr mpPathInGridOmpl;
      
 public: 
    Ompl();
 
 protected:

    /**
     * (Re-)creates the complete ompl environment.
     */
    virtual bool initialize();
    
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
        
    /**
     * Sets the global start and goal poses (in grid coordinates) in OMPL.
     */    
    void setStartGoal(base::samples::RigidBodyState& start_in_grid, 
            base::samples::RigidBodyState& goal_in_grid, 
            ompl::base::StateSpacePtr state_space, 
            ompl::base::ProblemDefinitionPtr& problem_definition);
};

} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_OMPL_HPP_
