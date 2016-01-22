#ifndef _ABSTRACT_MOTION_PLANNING_LIBRARY_HPP_
#define _ABSTRACT_MOTION_PLANNING_LIBRARY_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>
#include <base/Logging.hpp>

#include <envire/maps/TraversabilityGrid.hpp>

#include "Config.hpp"
#include "State.hpp"

namespace motion_planning_libraries
{

typedef envire::TraversabilityGrid::ArrayType TravData;

struct CellUpdate {
    size_t x;
    size_t y;
    uint8_t klass;
    double probability;
    double driveability;
    
    CellUpdate() : x(0), y(0), klass(0), probability(0.0), driveability(0.0) {
    }
    
    CellUpdate(size_t x_, size_t y_, uint8_t klass_, double probability_, double driveability_) : 
            x(x_), y(y_), klass(klass_), probability(probability_), driveability(driveability_) {
    }
};

/**
 * Base class for a motion planning library.
 */
class AbstractMotionPlanningLibrary
{
 protected: 
    Config mConfig;
    double mPathCost;
        
 public: 
    AbstractMotionPlanningLibrary(Config config = Config());
    ~AbstractMotionPlanningLibrary();
                
    /**
     * Implement for robot navigation: 
     * (Re-)initializes the complete planning environment using the passed 
     * traversability map / map data.
     */
    virtual bool initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data);
            
    /**
     * Contains a partial map update to avoid a full reinitialisation. 
     * This method will be called instead of 'initialize' 
     * if the size of the map has not changed.
     * \return By default false is returned which leads to a complete 
     * reinitialization. In addition true has to be returned if
     * the passed vector is empty. This is used to check if the
     * method has been overwritten.
     */
    virtual bool partialMapUpdate(std::vector<CellUpdate>& cell_updates);
    
    /**
     * Implement for arm motion planning.
     */
    virtual bool initialize_arm();
    
    /**
     * Sets the start and the goal within the planning environment.
     * This method is only called if a new pose has been received 
     * (REPLANNING_XXXX_THRESHOLDs are used).
     */
    virtual bool setStartGoal(struct State start_state, struct State goal_state) = 0;
    
    /**
     * Tries to find a solution (if the environment has just been initialized) 
     * or to improve the existing solution.
     */
    virtual bool solve(double time) = 0;
    
    /**
     * Fill the passed vector with the found path. By default the 
     * position have to be defined within the grid (grid-coordinates).
     * If the positions have already been converted to meter (local_grid)
     * the passed bolean has to be set to true.
     * Converting grid to grid_local: Do not move the positions 
     * to the center of a cell. This will be added automatically regrading to 
     * discretization error of the the goal pose. This takes care that the path 
     * directly ends at the goal pose.
     * E.g. SBPL XYTHETA uses intermediate points which are defined
     * in the local grid already.
     * The orientation/theta should be (-PI,PI] (according to OMPL).
     */
    virtual bool fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid) = 0;
    
    /**
     * If path-cost available within the planner they can be returned here. 
     */
    virtual double getCost() {
        return mPathCost;
    }
    
    /**
     * Can be implemented to check whether the start and/or goal state
     * are not valid. In this case MPL_ERR_START_ON_OBSTACLE,
     * MPL_ERR_GOAL_ON_OBSTACLE or MPL_ERR_START_GOAL_ON_OBSTACLE
     * should be returned.
     */
    virtual enum MplErrors isStartGoalValid() {
        return MPL_ERR_NONE;
    }
    
    /**
     * Can be used to tell whether the passed solution is the optimal one.
     * E.g. in SBPL epsilon == 1.0 identifies an optimal solution.
     * By default this method returns true, so it does not influence
     * replanning.
     */
    virtual bool foundFinalSolution() {
        return true;
    }
};

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
