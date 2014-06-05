#ifndef _MOTION_PLANNING_LIBRARIES_HPP_
#define _MOTION_PLANNING_LIBRARIES_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>

#include <envire/maps/TraversabilityGrid.hpp>

#include "Config.hpp"
#include "AbstractMotionPlanningLibrary.hpp"

namespace motion_planning_libraries
{

typedef envire::TraversabilityGrid::ArrayType TravData;
    
/**
 * Can be used to plan 2D trajectories considering the orientation.
 * The planner is meant to work on a single, local traversability map.
 * The start/goal poses are transformed from world to local grid and the
 * resulting trajectory from local grid to world.
 * 
 * TODO: Integrate: clear start, use next reachable goal
 */
class MotionPlanningLibraries
{
 protected:
    static const double REPLANNING_DIST_THRESHOLD = 0.05;
    static const double REPLANNING_TURN_THRESHOLD = 0.017;
    static const double REPLANNING_JOINT_ANGLE_THRESHOLD = 0.017;
    
    boost::shared_ptr<AbstractMotionPlanningLibrary> mpPlanningLib;
    
    envire::TraversabilityGrid* mpTravGrid;
    boost::shared_ptr<TravData> mpTravData;
    struct State mStartState, mGoalState; // Pose in world coordinates.
    base::samples::RigidBodyState mStartGrid, mGoalGrid;
    std::vector<State> mPlannedPath; // Pose in grid coordinates.
    bool mReceivedNewTravGrid;
    bool mReceivedNewStartGoal;
    
 public: 
    MotionPlanningLibraries(Config config = Config());
    ~MotionPlanningLibraries();
    
    /**
     * Sets the traversability map to plan on.
     * The pose, scale and size of the map are used for the world2grid and
     * grid2world transformation.
     */
    bool setTravGrid(envire::Environment* env, std::string trav_map_id);
    
    /**
     * Sets the start state. If the start state contains a pose it has to be defined 
     * within the world frame. This pose is transformed to the traversability 
     * grid and is used to set mStartGrid.
     */
    bool setStartState(struct State start_state);
    
    /**
     * Sets the goal state. If the goal state contains a pose it has to be defined 
     * within the world frame. This pose is transformed to the traversability 
     * grid and is used to set mGoalGrid.
     */
    bool setGoalState(struct State goal_state);
    
    /**
     * Tries to find a trajectory within the passed time.
     * If this method is called several times (with the same start, goal and map),
     * the planner will try to improve the found path. Otherwise a new planning
     * will be initiated. Threshold are used to decide whether a pose is new and
     * each received map will initiate a replan.
     */
    bool plan(double max_time=1.0); 
    
    /**
     * Returns the found path containing poses or joint angles (depending on the
     * defined start states). Resulting poses are defined within the world frame.
     */
    std::vector<struct State> getPath();
    
    // POSE SPECIFIC METHODS.
    /** Returns the path stored in mPath as a list of waypoints. */
    std::vector<base::Waypoint> getPathInWorld();
    
    /** Returns the path stored in mPath as a trajectory (spline). */
    base::Trajectory getTrajectoryInWorld(double speed);
    
    /** Prints the current path to the console. */
    void printPathInWorld();
    
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
     
 private:
    /**
     * Extracts the traversability map \a trav_map_id from the passed environment.
     * If the id is not available, the first traversability map will be used.
     */
    envire::TraversabilityGrid* extractTravGrid(envire::Environment* env, 
            std::string trav_map_id);
};

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
