#include "GlobalPathPlanner.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace global_path_planner
{

// PUBLIC
GlobalPathPlanner::GlobalPathPlanner() : mpTravGrid(NULL), mStartGrid(), mGoalGrid(), 
        mpTravMapValidator(NULL), mInitialized(false) {
}

GlobalPathPlanner::~GlobalPathPlanner() {
    if(mpTravMapValidator) {
        delete mpTravMapValidator;
        mpTravMapValidator = NULL;
    }
}

bool GlobalPathPlanner::init(envire::Environment* env, std::string trav_map_id) {
    if(mInitialized) {
        return true;
    }
    
    mpTravGrid = requestTravGrid(env, trav_map_id);
    if(mpTravGrid == NULL) {
        LOG_WARN("Environment does not contain a traversability map");
        return false;
    }

    ob::StateSpacePtr space(new ob::SE2StateSpace());
    
    // Set bounds. Cannot be set for x,y independently?
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(std::max(mpTravGrid->getCellSizeX(), mpTravGrid->getCellSizeY()));
    space->as<ob::SE2StateSpace>()->setBounds(bounds);
    
    // Create a space information object.
    ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
    
    if(mpTravMapValidator) {
        delete mpTravMapValidator;
        mpTravMapValidator = NULL;
    }
    mpTravMapValidator = new TravMapValidator(si, mpTravGrid);
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
        // get the goal representation from the problem definition 
        // (not the same as the goal state) and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        path->print(std::cout);
        
        // Convert state-path to vector3d-path and store it to mPath.
        // Valid downcast from Path to PathGeometric.
        std::vector<ompl::base::State*> path_states = 
                boost::static_pointer_cast<og::PathGeometric>(path)->getStates();
        std::vector<ompl::base::State*>::iterator it = path_states.begin();
        for(;it != path_states.end(); ++it) {
            const ompl::base::SE2StateSpace::StateType* state = 
                (*it)->as<ompl::base::SE2StateSpace::StateType>();
            mPath.push_back(base::Vector3d(state->getX(), state->getY(), 0.0));
        }
    }
    
    mInitialized = true;
    return true;
}

bool GlobalPathPlanner::setStartWorld(base::samples::RigidBodyState& start_world) {
    return world2grid(start_world, mStartGrid);
}

base::samples::RigidBodyState GlobalPathPlanner::getStartGrid() const {
    return mStartGrid;
}

bool GlobalPathPlanner::setGoalWorld(base::samples::RigidBodyState& goal_world) {
    return world2grid(goal_world, mGoalGrid);
}

base::samples::RigidBodyState GlobalPathPlanner::getGoalGrid() const {
    return mGoalGrid;
} 

void GlobalPathPlanner::getTrajectorie() {

}

// PRIVATE
envire::TraversabilityGrid* GlobalPathPlanner::requestTravGrid(envire::Environment* env, 
        std::string trav_map_id) {
    typedef envire::TraversabilityGrid e_trav;

    // Extract traversability map from evironment.
    e_trav* trav_map = env->getItem< e_trav >(trav_map_id).get();
    if(trav_map) {
        return trav_map;
    }
    
    LOG_INFO("No traversability map with id '%s' available, first trav map will be used", 
            trav_map_id.c_str());
          
    std::vector<e_trav*> maps = env->getItems<e_trav>();
    if(maps.size() < 1) {
        LOG_WARN("Environment does not contain any traversability grids");
        return NULL;
    } else {
        std::vector<e_trav*>::iterator it = maps.begin();
        LOG_INFO("Traversability map '%s' wil be used", (*it)->getUniqueId().c_str());
        return *it;
    }
}

bool GlobalPathPlanner::world2grid(base::samples::RigidBodyState const& rbs_world, 
        base::samples::RigidBodyState& rbs_grid) {

    // Transforms from world to traversability map frame / local.
    base::samples::RigidBodyState rbs_local;
    Eigen::Affine3d t = mpTravGrid->getEnvironment()->relativeTransform(
            mpTravGrid->getEnvironment()->getRootNode(),
            mpTravGrid->getFrameNode());
    rbs_local.setTransform((rbs_world.getTransform() * t));
    
    // Calculate and set grid coordinates (and orientation).
    size_t grid_x = 0, grid_y = 0;
    if(!mpTravGrid->toGrid(rbs_local.position.x(), rbs_local.position.y(), 
            grid_x, grid_y))
    {
        LOG_ERROR("Position (%d,%d) is out of grid", grid_x, grid_y);
        return false;
    }
    rbs_grid.position.x() = grid_x;
    rbs_grid.position.y() = grid_y;
    rbs_grid.position.z() = 0;
    rbs_grid.orientation = rbs_local.orientation;
        
    return true;
}

} // namespace global_path_planner
