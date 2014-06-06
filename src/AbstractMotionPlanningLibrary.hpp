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
    virtual bool initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            envire::TraversabilityGrid* trav_grid,
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
     * Fills the passed vector with the found path.
     */
    virtual bool fillPath(std::vector<struct State>& path) = 0;
};

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
