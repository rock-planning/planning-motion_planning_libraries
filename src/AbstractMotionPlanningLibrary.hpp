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
    
/**
 * Base class for a motion planning library.
 */
class AbstractMotionPlanningLibrary
{
 protected: 
    Config mConfig;
        
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
     * E.g. SBPL XYTHETA uses intermediate points which are defined
     * in the local grid already.
     * The orientation/theta should be (-PI,PI] (according to OMPL).
     */
    virtual bool fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid) = 0;
};

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
