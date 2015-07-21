#ifndef _MOTION_PLANNING_LIBRARIES_HPP_
#define _MOTION_PLANNING_LIBRARIES_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>

#include <envire/maps/TraversabilityGrid.hpp>

#include "Config.hpp"
#include "State.hpp"
#include "AbstractMotionPlanningLibrary.hpp"

namespace motion_planning_libraries
{

typedef envire::TraversabilityGrid::ArrayType TravData;
    
/**
 * \mainpage MPL - Motion Planning Libraries
 * \section Introduction
 * This library integrates motion planning libraries mainly for navigation.
 * In general it realizes path planning on a 2D Envire traversability maps. 
 * If you want to add your own planning library you have to inheret from 
 * AbstractMotionPlanningLibrary and implement the core virtual functions.
 * See Config.hpp to get a list of the available planning libraries, planners and 
 * environments. See the module documentation how to set up a small navigation 
 * stack in ROCK. In the \ref sec_libraries section the available planning libraries
 * and their configuration are described.
 * 
 * \section Frames
 * Three types of coordinate frames are used within this library: 
 * - First the world frame which defines the position of the system in meter 
 *   within the root node of the received traversability map. 
 * - Second the grid local frame which defines the position within the
 *   traversability map in meters.
 * - And third the grid frame in which the position is described 
 *   within the traversability map in grid cells.
 * 
 * In general the user just works with the world frame. The following code example
 * shows the complete transformation from world2grid. 
 * \code{.cpp}
 *       Eigen::Affine3d world2local = trav->getEnvironment()->relativeTransform(
 *           trav->getEnvironment()->getRootNode(),
 *           trav->getFrameNode());
 *       local_pose.setTransform(world2local * world_pose.getTransform());
 *       trav->toGrid(local_pose.position.x(), local_pose.position.y(), x_grid, y_grid, ...);
 * \endcode
 * After the traversability map has been set (MotionPlanningLibraries::setTravGrid) 
 * the start and goal position/pose within the world can be defined 
 * (MotionPlanningLibraries::setStartState, MotionPlanningLibraries::setGoalState). 
 * The integrated planning libraries will just work within the grid frame.
 * If the planning library requires grid local cordinates (e.g. SBPL)
 * the cell size in meters can be extracted from the received traversability map.
 * Each library has to implement AbstractMotionPlanningLibrary::fillPath to
 * support the found path which can be specified within the grid or the grid local 
 * frame.
 * 
 * \section sec_libraries Libraries
 * \subsection ompl OMPL Sample-Based Planning
 * http://ompl.kavrakilab.org/
 * OMPL is a very flexible library for Sample-Based Planning
 * which is mainly be used for high-dimensional planning. Additional spaces, validators and 
 * objectives (costs) can be added without any great effort. The following environments
 * have been implemted:
 * - ENV_XY: Simple point-based optimizing planning.
 * - ENV_XYTHETA: Control based planning using a simple ODE or a car like ODE to 
 * build up the navigation tree. OMPL does not support optimization for this environment,
 * so no usable results are produced yet. Instead SBPL ENV_XYTHETA 
 * should be used for planning which regards the orientation of the system.
 * - ENV_SHERPA: This environment allows circle based planning regarding an adaptive footprint.
 * The planner tries to maximise the footprint for a safe stand, just shrinks it 
 * for narrow passages. This is a first test implementation for body embedded planning which
 * will be extended in the future.
 * - ENV_ARM: In general OMPL is used for manipulation / arm movement. This has been integrated
 * just for testing. For a meaningful use a robot model and octomaps have to be integrated into MPL.
 * 
 * \subsection spbl SBPL Search-Based Planning
 * SBPL is a library mainly used for orientation based navigation within a discrete map supporting powerful
 * planning algorithms like Anytime-DStar. This is done by using so called motion primitives.
 * This primitives are small defined motions which are combined and optimized to the overall 
 * trajectory by the planner. The SBPL library supports a MATLAB script to generate 
 * the motion primitives. For that the user has to define all motion primites for the first
 * three discrete angles by hand (if overall 16 discrete angles are used) which are used
 * to create all primitives for all angles. This has been replaced by a complete automatic
 * primitive generation for which only the speed, the multipliers and the turning radius 
 * of the system have to be defined. Each movement type (forward, backward, lateral, forward-turn, backward-turn, pointturn) 
 * got its own multiplier.
 * - ENV_XY: Simple point-based optimizing planning.
 * - ENV_XYTHETA: Orientation based optimized planning. The forward- and turning-speed, 
 * the turning radius and the multipliers have to be defined which are used to generate
 * the primitive which are used to generate the final path. An internal epsilion is used 
 * by SBPL to describe the optimality of the current solution. An optimal path 
 * has been found if the epsilon reaches 1.0.
 * 
 * \section Configuration
 * The following section describes the minimal required configuration for each environment.
 * \subsection General
 * | Parameter        | Description |
 * | ---------------- | ----------- |
 * | mPlanningLibType | Defines the planning library, see motion_planning_libraries::PlanningLibraryType |
 * | mEnvType         | Defines the environment, see motion_planning_libraries::EnvType | 
 * \subsection OMPL
 * | Environment | Parameter              | Description |
 * | ----------- | ---------------------- | ----------- |
 * | ENV_XYTHETA | mMobilty               | mSpeed and mTurningSpeed are used to define the control space and to calculate the cost traversing a grid cell. |
 * |             | mFootprintLengthMinMax | Used for the car ODE. | 
 * | ENV_SHERPA  | mFootprintRadiusMinMax | Footprint is defined as a circle here. |
 * |             | mNumFootprintClasses   | To reduce the plannign dimension the footprint radius is descretized. |
 * |             | mTimeToAdaptFootprint  | Time to change the system from min to max footprint. |
 * |             | mAdaptFootprintPenalty | Additional costs which are added if the footprint changes between two states. | 
 * | ENV_ARM     | mJointBorders          | Borders of the arm joints. |
 * \subsection SBPL
 * | Environment | Parameter | Description |
 * | ----------- | ------------------------- | ----------- |
 * | ENV_XY      | mSBPLEnvFile              | (optional) Allows to load an SBPL environment instead of using the Envire traversability map. | 
 * | ENV_XYTHETA | mMobilty                  | Speeds are used together with the multipliers for the cost calculation. In addition the multipliers are used to activates the movement types (>0). mMinTurnignRadius takes care that the curve primitives are driveable for the system. | 
 * |             | mSBPLEnvFile              | (optional) Allows to load an SBPL environment instead of using the Envire traversability map. | 
 * |             | mSBPLMotionPrimitivesFile | (optional) Allows to use an existing SBPL primitive file instead of creating one based on the mMobility parameters. |
 * |             | mFootprintLengthMinMax    | The max value is used to define the robot length in SBPL. |
 * |             | mFootprintWidthMinMax     | The max value is used to define the robot width in SBPL. |
 * |             | mNumIntermediatePoints    | Sets the number of intermediate points which are added to each primitive to create smoother trajectories. |
 * |             | mNumPrimPartition         | Defines how much primitive for each movement type should be created. More primitives will optimize the result but increase the planning time. |
 * |             | mPrimAccuracy             | Defines how close a primitive has to reach a discrete end position. If this parameter is reduced towards 0, the discretization error will be reduced but the length of the primitives will be increased and the overall number of primitive for each movement type could also be reduced. | 
 * 
 * \section TODOs
 * \todo "Adds method to remove obstacles within the start pose."
 * \todo "Optional: If the goal lies on an obstacle use next valid goal position."
 * \todo "Split State and Config to classes using inheritance. Possible?"
 * \todo "Calculate correct center of rotation in SBPL MotionPrimitives."
 * \todo "Add struct to combine all replanning parameters."
 * \todo "Config: Add boolean whether a new map should initiate a replanning."
 * \todo "Config: Add boolean whether a new goal pose should initiate a replanning."
 * \todo "Config: Add boolean if only optimal trajectories (epsilon == 1) should be supported."
 * \todo "Adds new trajectory format to integrate all type of movements."
 */
class MotionPlanningLibraries
{
 protected:
    static const double REPLANNING_DIST_THRESHOLD;
    static const double REPLANNING_TURN_THRESHOLD;
    static const double REPLANNING_JOINT_ANGLE_THRESHOLD;
    
    Config mConfig;
    
    boost::shared_ptr<AbstractMotionPlanningLibrary> mpPlanningLib;
    
    envire::TraversabilityGrid* mpTravGrid;
    boost::shared_ptr<TravData> mpTravData;
    struct State mStartState, mGoalState; // Pose in world coordinates.
    struct State mStartStateGrid, mGoalStateGrid;
    std::vector<State> mPlannedPathInWorld; // Pose in world coordinates.
    bool mReceivedNewTravGrid;
    bool mReceivedNewStart;
    bool mReceivedNewGoal;
    bool mGoalReached;
    bool mArmInitialized;
    bool mReplanRequired;
    double mLostX; // Used to trac discretization error.
    double mLostY;
    enum MplErrors mError;
    
 public: 
    MotionPlanningLibraries(Config config = Config());
    ~MotionPlanningLibraries();
    
    /**
     * Sets the traversability map to plan on. Required for robot navigation.
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
     * If this method is called several times (with the same configurations),
     * the planner will try to improve the found path. Otherwise a new planning
     * will be initiated. Thresholds define whether a state is new one.
     * For navigation: A new traversability map will initiate a reinitialization.
     * TODO: A new trav map should not require a complete reinitialization.
     * \return True if a planning has been successfully executed. 
     * If an error has occurred or replanning is not required (MPL_ERR_REPLANNING_NOT_REQUIRED) 
     * false will be returned and the error state will be set accordingly. 
     * isPlanningRequired() can be used before planning to check whether a
     * replanning is necessary. 
     */
    bool plan(double max_time, double& cost); 
    
    /**
     * Like getStates() but with world coordinates.
     */
    std::vector<struct State> getStatesInWorld();
    
    // POSE SPECIFIC METHODS.
    /** Returns the path stored in mPath as a list of waypoints. */
    std::vector<base::Waypoint> getPathInWorld();
    
    /** 
     * Returns the path stored in mPath as a trajectory (spline). 
     * If the speed parameter is set it will be used, otherwise
     * the speed which have been defined within the mobility struct.
     * If no speed has been defined
     * 
     * If speed values are available within the state for each new
     * speed a new trajectory will be created. Otherwise a single 
     * trajectory will be created using the passed speed value.
     * Takes care that each trajectory contains at least two different
     * positions and also handles equal positions and equal positions but 
     * different speeds. 
     */
    std::vector<base::Trajectory> getTrajectoryInWorld();
    
    /**
     * Inverts the last trajectory and searches from goal to start for the next valid pose.
     * This can be used if the robot stucks within an obstacle.
     * Returns an empty vector if an error occurred.
     */
    std::vector<base::Trajectory> getEscapeTrajectoryInWorld();
    
    /** Prints the current path to the console. */
    void printPathInWorld();
    
    /** Returns the start pose within the grid. */
    base::samples::RigidBodyState getStartPoseInGrid();
    
    /** Returns the goal pose within the grid. */
    base::samples::RigidBodyState getGoalPoseInGrid();
    
    /** If an error occurred during planning this can be used to get more informations. */
    inline enum MplErrors getError() {
        return mError;
    }
    
    bool getSbplMotionPrimitives(struct SbplMotionPrimitives& prims);
    
    /**
     * Converts the world pose to grid coordinates including the transformed orientation.
     */        
    static bool world2grid(envire::TraversabilityGrid const* trav, 
        base::samples::RigidBodyState const& world_pose, 
        base::samples::RigidBodyState& grid_pose,
        double* lost_x = NULL,
        double* lost_y = NULL);
        
    /**
     * Transforms the grid-coordinates to grid local to world.
     */
    bool grid2world(envire::TraversabilityGrid const* trav,
            base::samples::RigidBodyState const& grid_pose, 
            base::samples::RigidBodyState& world_pose);
     
    /**
     * Transforms the pose defined in the grid local frame to world.
     * Poses within the grid local frame use meter and radians (-PI, PI] instead of
     * discrete grid coordinates.
     */
    bool gridlocal2world(envire::TraversabilityGrid const* trav,
        base::samples::RigidBodyState const& grid_local_pose,
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
