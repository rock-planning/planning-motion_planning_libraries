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

} // namespace motion_planning_libraries
