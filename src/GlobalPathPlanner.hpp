#ifndef _GLOBAL_PATH_PLANNER_HPP_
#define _GLOBAL_PATH_PLANNER_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Trajectory.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>

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
    std::vector<base::Vector3d> mPath;
    ompl::base::StateSpacePtr mpStateSpace;
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    ompl::base::ProblemDefinitionPtr mpProblemDefinition;
    ompl::base::PlannerPtr mpOptimizingPlanner;
    bool mOMPLObjectsCreated;
    
 public: 
    GlobalPathPlanner();
    ~GlobalPathPlanner();
 
    bool setTravGrid(envire::Environment* env, std::string trav_map_id);
    bool setStartWorld(base::samples::RigidBodyState& start_world);
    bool setGoalWorld(base::samples::RigidBodyState& goal_world);
    bool plan(double max_time=1.0);
    std::vector<base::Vector3d> getPath();
    base::Trajectory getTrajectory(double speed);
                
    base::samples::RigidBodyState getStartGrid() const;
    base::samples::RigidBodyState getGoalGrid() const;
    
 private:
    /**
     * Extracts the traversability map \a trav_map_id from the passed environment.
     * If the id is not available, the first traversability map will be used.
     */
    envire::TraversabilityGrid* extractTravGrid(envire::Environment* env, 
            std::string trav_map_id);
    
    /**
     * Converts the world pose to grid coordinates including the transformed orientation.
     */        
    bool world2grid(base::samples::RigidBodyState const& rbs_world, 
        base::samples::RigidBodyState& rbs_grid);

    /**
     * Creates all OMPL objects for planning.
     */        
    bool createOMPLObjects();
};

} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_HPP_
