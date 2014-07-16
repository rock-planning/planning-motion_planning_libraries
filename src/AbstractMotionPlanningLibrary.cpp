#include "AbstractMotionPlanningLibrary.hpp"

namespace motion_planning_libraries
{

// PUBLIC
AbstractMotionPlanningLibrary::AbstractMotionPlanningLibrary(Config config) : 
        mConfig(config) 
{
}

AbstractMotionPlanningLibrary::~AbstractMotionPlanningLibrary() {
}

bool AbstractMotionPlanningLibrary::initialize(envire::TraversabilityGrid* trav_grid,
        boost::shared_ptr<TravData> grid_data) {
    return false;
}
        
bool AbstractMotionPlanningLibrary::initialize_arm() {
    return false;
}

} // namespace motion_planning_libraries
