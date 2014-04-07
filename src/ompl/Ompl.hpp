#ifndef _GLOBAL_PATH_PLANNER_OMPL_HPP_
#define _GLOBAL_PATH_PLANNER_OMPL_HPP_

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>

#include <global_path_planner/GlobalPathPlanner.hpp>

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
    
    size_t mGridWidth;
    size_t mGridHeight;
      
 public: 
    Ompl();
 
 protected:

    /**
     * (Re-)creates the complete ompl environment.
     */
    virtual bool initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
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
};

} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_OMPL_HPP_
