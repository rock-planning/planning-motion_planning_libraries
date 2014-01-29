#include "GlobalPathPlanner.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/config.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace global_path_planner
{

// PUBLIC
GlobalPathPlanner::GlobalPathPlanner() : mpTravGrid(NULL), mStartGrid(), mGoalGrid(), 
        mpTravMapValidator(NULL), mOMPLObjectsCreated(false) {
        
    mStartGrid.invalidatePosition();
    mStartGrid.invalidateOrientation();        
    mGoalGrid.invalidatePosition();
    mGoalGrid.invalidateOrientation();  
}

GlobalPathPlanner::~GlobalPathPlanner() {
}

bool GlobalPathPlanner::setTravGrid(envire::Environment* env, std::string trav_map_id) {
    envire::TraversabilityGrid* trav_grid = extractTravGrid(env, trav_map_id);
    if(trav_grid == NULL) {
        LOG_WARN("Traversability map could not be set");
        return false;
    } else {
        mpTravGrid = trav_grid;
        return true;
    }
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


bool GlobalPathPlanner::plan(double max_time) {
    if(mpTravGrid == NULL) {
        LOG_WARN("No traversability map available, planning cannot be executed");
        return false;
    } 
    
    if(!(mStartGrid.hasValidPosition() && mStartGrid.hasValidOrientation() &&
            mGoalGrid.hasValidPosition() && mGoalGrid.hasValidOrientation())) {
        LOG_WARN("Start/Goal has not been set, planning can not be executed"); 
        return false;
    }
    
    if(!mOMPLObjectsCreated) {
        bool ret = createOMPLObjects();
        if(!ret) {
            LOG_WARN("OMPL objects could not be created, planning can not be executed"); 
            return false;
        } else {
            mOMPLObjectsCreated = true;
        }
    }
    
    // Set start and goal in OMPL.
    ob::ScopedState<> start(mpStateSpace);
    // start[0] = 1; ?
    start->as<ob::SE2StateSpace::StateType>()->setX(mStartGrid.position.x());
    start->as<ob::SE2StateSpace::StateType>()->setY(mStartGrid.position.y());
    start->as<ob::SE2StateSpace::StateType>()->setYaw(mStartGrid.getYaw());
    
    ob::ScopedState<> goal(mpStateSpace);
    goal->as<ob::SE2StateSpace::StateType>()->setX(mGoalGrid.position.x());
    goal->as<ob::SE2StateSpace::StateType>()->setY(mGoalGrid.position.y());
    goal->as<ob::SE2StateSpace::StateType>()->setYaw(mGoalGrid.getYaw());
    
    mpProblemDefinition->setStartAndGoalStates(start, goal);
 
    // Start planning.
    // Setup: Once during creation or each time because of the changed start/goal?
    mpOptimizingPlanner->setup();
    ob::PlannerStatus solved = mpOptimizingPlanner->solve(1.0);
    
    if (solved)
    {
        // get the goal representation from the problem definition 
        // (not the same as the goal state) and inquire about the found path
        ob::PathPtr path = mpProblemDefinition->getSolutionPath();
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
    
    return true;
}

std::vector<base::Vector3d> GlobalPathPlanner::getPath() {
    return mPath;
}

base::Trajectory GlobalPathPlanner::getTrajectory(double speed) {
    base::Trajectory trajectory;
    trajectory.speed = speed;
    trajectory.spline.interpolate(mPath);
    return trajectory;
}

// PRIVATE
envire::TraversabilityGrid* GlobalPathPlanner::extractTravGrid(envire::Environment* env, 
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
        LOG_WARN("Position (%d,%d) is out of grid", grid_x, grid_y);
        return false;
    }
    rbs_grid.position.x() = grid_x;
    rbs_grid.position.y() = grid_y;
    rbs_grid.position.z() = 0;
    rbs_grid.orientation = rbs_local.orientation;
        
    return true;
}

bool GlobalPathPlanner::createOMPLObjects() {
    if(mpTravGrid == NULL) {
        LOG_WARN("No traversability map available, OMPL objects cannot be constructed");
        return false;
    }

    mpStateSpace = ompl::base::StateSpacePtr(new ob::SE2StateSpace());
    
    // Set bounds. Cannot be set for x,y independently?
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(std::max(mpTravGrid->getCellSizeX(), mpTravGrid->getCellSizeY()));
    mpStateSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    
    // Create a space information object and the validator using the traversability map.
    mpSpaceInformation = ob::SpaceInformationPtr(new ob::SpaceInformation(mpStateSpace));
    mpSpaceInformation->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
            new TravMapValidator(mpSpaceInformation, mpTravGrid)));
            
    // Create problem definition.        
    mpProblemDefinition = ob::ProblemDefinitionPtr(
            new ob::ProblemDefinition(mpSpaceInformation));
        
    // Construct our optimizing planner using the RRTstar algorithm.
    // TODO: RTTstar will be deleted by the planner object?
    mpOptimizingPlanner = ob::PlannerPtr(new og::RRTstar(mpSpaceInformation));
    // Set the problem instance for our planner to solve
    mpOptimizingPlanner->setProblemDefinition(mpProblemDefinition);
    
    return true;
}

} // namespace global_path_planner
