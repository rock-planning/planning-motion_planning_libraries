#ifndef _GLOBAL_PATH_PLANNER_HPP_
#define _GLOBAL_PATH_PLANNER_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/Planner.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>

namespace envire {
class TraversabilityGrid;
class Environment;
}

namespace global_path_planner
{

class TravMapValidator;
class TurningValidator;

/**
 * Can be used to plan 2D trajectories considering the orientation.
 * The planner is meant to work on a single, local traversability map.
 * The start/goal poses are transformed from world to local grid and the
 * resulting trajectory from local grid to world.
 */
class GlobalPathPlanner
{
 private:
    static const double REPLANNING_DIST_THRESHOLD = 0.05;
    static const double REPLANNING_TURN_THRESHOLD = 0.017;
    double mMaxSpeed;
    double mMaxTurningSpeed;
 
    envire::TraversabilityGrid* mpTravGrid;
    // Contains the grid coordinates and the orientation.
    base::samples::RigidBodyState mStartGrid, mGoalGrid;
    base::samples::RigidBodyState mStartWorld, mGoalWorld;
    std::vector<base::samples::RigidBodyState> mPath;
    
    ompl::base::StateSpacePtr mpStateSpace;
    ompl::base::SpaceInformationPtr mpSpaceInformation;
    ompl::base::StateValidityCheckerPtr mpTravMapValidator;
    ompl::base::MotionValidatorPtr mpTurningValidator;
    ompl::base::ProblemDefinitionPtr mpProblemDefinition;
    ompl::base::PlannerPtr mpOptimizingPlanner;
    ompl::base::OptimizationObjectivePtr mpPathLengthOptimization;
    ompl::base::OptimizationObjectivePtr mpPathClearanceOptimization;
    ompl::base::OptimizationObjectivePtr mpMultiOptimization;
    ompl::base::OptimizationObjectivePtr mpMaxMinClearance;
    ompl::base::OptimizationObjectivePtr mpTravGridOjective;
    ompl::control::ControlSpacePtr mpControlSpace;
    
    bool mReplanningRequired;
    
 public: 
    GlobalPathPlanner();
    ~GlobalPathPlanner();
 
    bool setTravGrid(envire::Environment* env, std::string trav_map_id);
    bool setStartWorld(base::samples::RigidBodyState& start_world);
    bool setGoalWorld(base::samples::RigidBodyState& goal_world);
    
    /**
     * Tries to find a trajectory within the passed time.
     * If this method is called several times (with the same start, goal and map),
     * the planner will try to improve the found path. Otherwise a new planning
     * will be initiated. Threshold are used to decide whether a pose is new and
     * each received map will initiate a replan.
     */
    bool plan(double max_time=1.0);
    std::vector<base::Waypoint> getPath();
    base::Trajectory getTrajectory(double speed);
                
    base::samples::RigidBodyState getStartGrid() const;
    base::samples::RigidBodyState getGoalGrid() const;
    
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
    
    /**
     * Used to check the sampling.
     */
    std::vector<base::Waypoint> getSamples();
    
 private:
    /**
     * Extracts the traversability map \a trav_map_id from the passed environment.
     * If the id is not available, the first traversability map will be used.
     */
    envire::TraversabilityGrid* extractTravGrid(envire::Environment* env, 
            std::string trav_map_id);
   
    /**
     * Creates all OMPL objects for planning.
     */        
    bool createOMPLObjects();
    
    /**
     * Converts the OMPL path to a RigidBodyState vector.
     */
    std::vector<base::samples::RigidBodyState> convertPath(ompl::base::PathPtr path_ompl);
    
    /**
     * Set the start and goal pose in OMPL using mStartGrid and mGoalGrid.
     */
    void setStartGoalOMPL(base::samples::RigidBodyState const& start,
        base::samples::RigidBodyState const& goal);
        
    /**
     * Combines several objectives like path clearance and path length.
     */
    ompl::base::OptimizationObjectivePtr getBalancedObjective(const ompl::base::SpaceInformationPtr& si);
};

} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_HPP_
