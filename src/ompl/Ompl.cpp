#include "Ompl.hpp"
#include <ompl/geometric/PathGeometric.h>

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

std::vector<ompl::base::State*> Ompl::getPathStates()
{
#if OMPL_VERSION_VALUE < 1001000
    // Downcast from Path to PathGeometric is valid.
    return boost::static_pointer_cast<og::PathGeometric>(mpPathInGridOmpl)->getStates();
#else
    std::shared_ptr<ompl::geometric::PathGeometric> pathGeometric = std::static_pointer_cast<ompl::geometric::PathGeometric>(mpPathInGridOmpl);
    if(!pathGeometric)
    {
        throw std::runtime_error("motion_planning_libraries::OmplEnvXYTHETA::fillPath: could not cast path to geometric path");
    }
    return pathGeometric->getStates();
#endif
}

} // namespace motion_planning_libraries
