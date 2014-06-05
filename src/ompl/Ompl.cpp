#include "Ompl.hpp"

namespace motion_planning_libraries
{    
    
// PUBLIC
Ompl::Ompl(Config config) : AbstractMotionPlanningLibrary(config) {
}

bool Ompl::solve(double time) {
    ompl::base::PlannerStatus solved = mpPlanner->solve(time);

    if (solved)
    {
        mpPathInGridOmpl = mpProblemDefinition->getSolutionPath();
        return true;
    } else {
        return false;
    }
}

} // namespace motion_planning_libraries
