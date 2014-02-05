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
GlobalPathPlanner::GlobalPathPlanner() : mpTravGrid(NULL), 
        mStartGrid(), 
        mGoalGrid(), 
        mpTravMapValidator(NULL), 
        mPath(),
        mOMPLObjectsCreated(false) {
        
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
        mOMPLObjectsCreated = false;
        return true;
    }
}

bool GlobalPathPlanner::setStartWorld(base::samples::RigidBodyState& start_world) {
    mOMPLObjectsCreated = false;
    return world2grid(mpTravGrid, start_world, mStartGrid);
}

base::samples::RigidBodyState GlobalPathPlanner::getStartGrid() const {
    return mStartGrid;
}

bool GlobalPathPlanner::setGoalWorld(base::samples::RigidBodyState& goal_world) {
    mOMPLObjectsCreated = false;
    return world2grid(mpTravGrid, goal_world, mGoalGrid);
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
    
    LOG_INFO("Planning from (x,y,yaw) (%4.2f, %4.2f, %4.2f) to (%4.2f, %4.2f, %4.2f)", 
            mStartGrid.position.x(), mStartGrid.position.y(), mStartGrid.getYaw(),
            mGoalGrid.position.x(), mGoalGrid.position.y(), mGoalGrid.getYaw());
    
    mpProblemDefinition->setStartAndGoalStates(start, goal);
    mpOptimizingPlanner->setup();
 
    // Start planning.
    // Setup: Once during creation or each time because of the changed start/goal?
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
        // Downcast from Path to PathGeometric is valid.
        std::vector<ompl::base::State*> path_states = 
                boost::static_pointer_cast<og::PathGeometric>(path)->getStates();
        std::vector<ompl::base::State*>::iterator it = path_states.begin();
        // Clear current path.
        mPath.clear();
        int counter = 0;
        for(;it != path_states.end(); ++it) {
            const ompl::base::SE2StateSpace::StateType* state = 
                (*it)->as<ompl::base::SE2StateSpace::StateType>();
                
            // Convert back to world coordinates.
            base::samples::RigidBodyState grid_pose;
            grid_pose.position[0] = state->getX();
            grid_pose.position[1] = state->getY();
            grid_pose.position[2] = 0;
            grid_pose.orientation = Eigen::AngleAxis<double>(state->getYaw(), 
                    base::Vector3d(0,0,1));
            base::samples::RigidBodyState world_pose;
            grid2world(mpTravGrid, grid_pose, world_pose);
            
            // Add positions to path. TODO: orientation?
            mPath.push_back(world_pose.position);
            counter++;
        }
        LOG_INFO("Trajectory contains %d states", counter);
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

bool GlobalPathPlanner::world2grid(envire::TraversabilityGrid const* trav,
        base::samples::RigidBodyState const& world_pose, 
        base::samples::RigidBodyState& grid_pose) {

    // Transforms from world to local.
    base::samples::RigidBodyState local_pose;
    Eigen::Affine3d world2local = trav->getEnvironment()->relativeTransform(
            trav->getEnvironment()->getRootNode(),
            trav->getFrameNode());
    local_pose.setTransform(world2local * world_pose.getTransform());
    
    // Calculate and set grid coordinates (and orientation).
    size_t x_grid = 0, y_grid = 0;
    if(!trav->toGrid(local_pose.position.x(), local_pose.position.y(), 
            x_grid, y_grid)) 
    {
        LOG_WARN("Position (%4.2f,%4.2f) / cell (%d,%d) is out of grid", 
                local_pose.position.x(), local_pose.position.y(), x_grid, y_grid);
        return false;
    }
    grid_pose = local_pose; 
    grid_pose.position.x() = x_grid;
    grid_pose.position.y() = y_grid;
    grid_pose.position.z() = 0;
    
    return true;
}

void GlobalPathPlanner::grid2world(envire::TraversabilityGrid const* trav,
        base::samples::RigidBodyState const& grid_pose,
        base::samples::RigidBodyState& world_pose) {
        
    double x_grid = grid_pose.position[0], y_grid = grid_pose.position[1]; 
    double x_local = 0, y_local = 0; 
    
    // Transformation GRID2LOCAL       
    trav->fromGrid(x_grid, y_grid, x_local, y_local);
    base::samples::RigidBodyState local_pose = grid_pose;
    local_pose.position[0] = x_local;
    local_pose.position[1] = y_local;
    local_pose.position[2] = 0.0;
    
    // Transformation LOCAL2WOLRD
    Eigen::Affine3d local2world = trav->getEnvironment()->relativeTransform(
        trav->getFrameNode(),
        trav->getEnvironment()->getRootNode());
    world_pose.setTransform(local2world * local_pose.getTransform() );
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
    
    // Create a space information object using the traversability map validator.
    mpSpaceInformation = ob::SpaceInformationPtr(new ob::SpaceInformation(mpStateSpace));
    mpSpaceInformation->setStateValidityChecker(ompl::base::StateValidityCheckerPtr(
            new TravMapValidator(mpSpaceInformation, mpTravGrid)));
    mpSpaceInformation->setup();
            
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
