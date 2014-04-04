#ifndef _GLOBAL_PATH_PLANNER_HPP_
#define _GLOBAL_PATH_PLANNER_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>

#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/core/Environment.hpp>

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
 protected:
    static const double REPLANNING_DIST_THRESHOLD = 0.05;
    static const double REPLANNING_TURN_THRESHOLD = 0.017;
    
    envire::TraversabilityGrid* mpTravGrid;
    base::samples::RigidBodyState mStartWorld, mGoalWorld;
    base::samples::RigidBodyState mStartGrid, mGoalGrid;
    std::vector<base::samples::RigidBodyState> mPathInGrid;
    bool mReplanningRequired;
    
    double mRobotWidth;
    double mRobotHeight;
    double mRobotForwardVelocity; // m/sec
    double mRobotRotationalVelocity; // Time in seconds to turn 45°. 
    
 public: 
    GlobalPathPlanner();
    ~GlobalPathPlanner();
    
    /**
     * Sets the traversability map to plan on.
     * The pose, scale and size of the map are used for the world2grid and
     * grid2world transformation.
     */
    bool setTravGrid(envire::Environment* env, std::string trav_map_id);
    
    /**
     * Converts the world pose to the traversability grid and 
     * sets mStartWorld and mStartGrid.
     */
    bool setStartPoseInWorld(base::samples::RigidBodyState& start_world);
    
    /**
     * Converts the world pose to the traversability grid and 
     * sets mGoalWorld and mGoalGrid.
     */
    bool setGoalPoseInWorld(base::samples::RigidBodyState& goal_world);
    
    /**
     * Tries to find a trajectory within the passed time.
     * If this method is called several times (with the same start, goal and map),
     * the planner will try to improve the found path. Otherwise a new planning
     * will be initiated. Threshold are used to decide whether a pose is new and
     * each received map will initiate a replan.
     */
    bool plan(double max_time=1.0); 
    
    /** Returns the path stored in mPath as a list of waypoints. */
    std::vector<base::Waypoint> getPathInWorld();
    
    /** Returns the path stored in mPath as a trajectory (spline). */
    base::Trajectory getTrajectoryInWorld(double speed);
    
    /** Returns the start pose within the grid. */
    base::samples::RigidBodyState getStartPoseInGrid();
    
    /** Returns the goal pose within the grid. */
    base::samples::RigidBodyState getGoalPoseInGrid();
    
    /**
     * Converts the world pose to grid coordinates including the transformed orientation.
     */        
    static bool world2grid(envire::TraversabilityGrid const* trav, 
        base::samples::RigidBodyState const& world_pose, 
        base::samples::RigidBodyState& grid_pose);
        
    /**
     * Transforms the grid pose to a world pose.
     */
    static bool grid2world(envire::TraversabilityGrid const* trav,
            base::samples::RigidBodyState const& grid_pose, 
            base::samples::RigidBodyState& world_pose);
            
 protected:
    /**
     * Initialize the complete planning environment.
     * It has to be possible to call this methods several times, which should
     * remove the old environment and create a new one.
     * The current traversability map, start and goal are set and should be used.
     */
    virtual bool initialize() = 0;
    virtual bool solve(double time) = 0;
    virtual bool fillPath(std::vector<base::samples::RigidBodyState>& path) = 0;
     
 private:
    /**
     * Extracts the traversability map \a trav_map_id from the passed environment.
     * If the id is not available, the first traversability map will be used.
     */
    envire::TraversabilityGrid* extractTravGrid(envire::Environment* env, 
            std::string trav_map_id);
};

} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_HPP_
