#ifndef _MOTION_PLANNING_LIBRARIES_OMPL_ENVXY_HPP_
#define _MOTION_PLANNING_LIBRARIES_OMPL_ENVXY_HPP_

#include "Ompl.hpp"

namespace motion_planning_libraries
{

class OmplEnvXY : public Ompl
{
 private: 
    ompl::base::StateValidityCheckerPtr mpTravMapValidator;
    ompl::base::OptimizationObjectivePtr mpPathLengthOptimization;
    ompl::base::OptimizationObjectivePtr mpMultiOptimization;
    ompl::base::OptimizationObjectivePtr mpTravGridObjective;
      
 public: 
    OmplEnvXY(Config config = Config());

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
    virtual bool fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid);
    
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
