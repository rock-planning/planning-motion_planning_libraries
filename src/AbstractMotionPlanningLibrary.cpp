#include "AbstractMotionPlanningLibrary.hpp"

namespace motion_planning_libraries
{

// PUBLIC
AbstractMotionPlanningLibrary::AbstractMotionPlanningLibrary(Config config) : 
        mConfig(config),
        mPathCost(nan(""))
{
}

AbstractMotionPlanningLibrary::~AbstractMotionPlanningLibrary() {
}

bool AbstractMotionPlanningLibrary::initialize(envire::TraversabilityGrid* trav_grid,
        boost::shared_ptr<TravData> grid_data) {
    LOG_WARN("Abstract initialization is used");
    return false;
}
        
bool AbstractMotionPlanningLibrary::initialize_arm() {
    LOG_WARN("Abstract arm initialization is used");
    return false;
}

} // namespace motion_planning_libraries
