#include "GlobalPathPlanner.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace global_path_planner
{

GlobalPathPlanner::GlobalPathPlanner() : mpTravMapValidator(NULL) {
}

GlobalPathPlanner::~GlobalPathPlanner() {
    if(mpTravMapValidator) {
        delete mpTravMapValidator;
        mpTravMapValidator = NULL;
    }
}

void GlobalPathPlanner::init() {
    ob::StateSpacePtr space(new ob::SE2StateSpace());
    
    // Set bounds.
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);
    
    // Create a space information object.
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    
    if(mpTravMapValidator) {
        delete mpTravMapValidator;
        mpTravMapValidator = NULL;
    }
    mpTravMapValidator = new TravMapValidator(si);
    si->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(mpTravMapValidator));
    
    ob::ScopedState<> start(space);
    start.random();
    ob::ScopedState<> goal(space);
    goal.random();

    // Problem definition.
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    pdef->setStartAndGoalStates(start, goal);
 
     // Planner.
    ob::PlannerPtr planner(new og::RRTConnect(si));
    planner->setProblemDefinition(pdef);
    planner->setup();
    ob::PlannerStatus solved = planner->solve(1.0);
    
    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        path->print(std::cout);
    }
}

} // namespace global_path_planner
