#ifndef _GLOBAL_PATH_PLANNER_HPP_
#define _GLOBAL_PATH_PLANNER_HPP_

#include <base/samples/RigidBodyState.hpp>

#include <ompl/geometric/PathGeometric.h>

#include <global_path_planner/validators/TravMapValidator.hpp>

namespace global_path_planner
{

/**
 * Can be used to plan 2D trajectories considering the orientation.
 * The planner is meant to work on a single, local traversability map.
 * The start/goal poses are transformed from world to local grid and the
 * resulting trajectory from local grid to world.
 */
class GlobalPathPlanner
{
 private:
    envire::TraversabilityGrid* mpTravGrid;
    // Contains the grid coordinates and the orientation.
    base::samples::RigidBodyState mStartGrid, mGoalGrid;
    TravMapValidator* mpTravMapValidator;
    bool mInitialized;
    std::vector<base::Vector3d> mPath;
    
 public: 
    GlobalPathPlanner();
    ~GlobalPathPlanner();

    bool init(envire::Environment* env, std::string trav_map_id);
 
    bool setStartWorld(base::samples::RigidBodyState& start_world);
    base::samples::RigidBodyState getStartGrid() const;
    bool setGoalWorld(base::samples::RigidBodyState& goal_world);
    base::samples::RigidBodyState getGoalGrid() const;
    
    void getTrajectorie();
    
 private:
    envire::TraversabilityGrid* requestTravGrid(envire::Environment* env, 
            std::string trav_map_id);
    
    /**
     * Converts the world pose to grid coordinates including the transformed orientation.
     */        
    bool world2grid(base::samples::RigidBodyState const& rbs_world, 
        base::samples::RigidBodyState& rbs_grid);
};

} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_HPP_
