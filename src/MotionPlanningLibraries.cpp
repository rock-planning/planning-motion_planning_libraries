#include "MotionPlanningLibraries.hpp"
#include <base/Logging.hpp>

namespace motion_planning_libraries
{

// PUBLIC
MotionPlanningLibraries::MotionPlanningLibraries(Config config) : mConfig(config),
        mpTravGrid(NULL), 
        mpTravData(),
        mStartWorld(), mGoalWorld(), 
        mStartGrid(), mGoalGrid(), 
        mPathInGrid(),
        mReceivedNewTravGrid(false),
        mReceivedNewStartGoal(false) {
        
    mStartGrid.invalidatePosition();
    mStartGrid.invalidateOrientation();        
    mGoalGrid.invalidatePosition();
    mGoalGrid.invalidateOrientation();  
}

MotionPlanningLibraries::~MotionPlanningLibraries() {
}

bool MotionPlanningLibraries::setTravGrid(envire::Environment* env, std::string trav_map_id) {
    envire::TraversabilityGrid* trav_grid = extractTravGrid(env, trav_map_id);
    if(trav_grid == NULL) {
        LOG_WARN("Traversability map could not be set");
        return false;
    } else {
        mpTravGrid = trav_grid;
        // Creates a copy of the current grid data.
        mpTravData = boost::shared_ptr<TravData>(new TravData(
            mpTravGrid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY)));
        mReceivedNewTravGrid = true;
        return true;
    }
}

bool MotionPlanningLibraries::setStartPoseInWorld(base::samples::RigidBodyState& start_world) {
    base::samples::RigidBodyState start_grid_new;
    if(!world2grid(mpTravGrid, start_world, start_grid_new)) {
        LOG_WARN("Start pose could not be set");
        return false;
    }

    if(mStartWorld.hasValidPosition() && mStartWorld.hasValidOrientation()) {
        // Replan if there is a difference between the old and the new start pose.
        double dist = (mStartWorld.position - start_world.position).norm();
        double turn = fabs(mStartWorld.getYaw() - start_world.getYaw());
        if (dist > REPLANNING_DIST_THRESHOLD || turn > REPLANNING_TURN_THRESHOLD) {
            mReceivedNewStartGoal = true;
        }
    } else {
        mReceivedNewStartGoal = true;
    }
    
    mStartGrid = start_grid_new;
    mStartWorld = start_world;    
    
    return true;
}

bool MotionPlanningLibraries::setGoalPoseInWorld(base::samples::RigidBodyState& goal_world) {
    base::samples::RigidBodyState goal_grid_new;
    if(!world2grid(mpTravGrid, goal_world, goal_grid_new)) {
        LOG_WARN("Goal pose could not be set");
        return false;
    }

    if(mGoalWorld.hasValidPosition() && mGoalWorld.hasValidOrientation()) {
        // Replan if there is a difference between the old and the new goal pose.
        double dist = (mGoalWorld.position - goal_world.position).norm();
        double turn = fabs(mGoalWorld.getYaw() - goal_world.getYaw());
        if (dist > REPLANNING_DIST_THRESHOLD || turn > REPLANNING_TURN_THRESHOLD) {
            mReceivedNewStartGoal = true;
        }
    } else {
        mReceivedNewStartGoal = true;
    }
    
    mGoalGrid = goal_grid_new;
    mGoalWorld = goal_world;    
    
    return true;
}

bool MotionPlanningLibraries::plan(double max_time) {

    if(mpTravGrid == NULL) {
        LOG_WARN("No traversability map available, planning cannot be executed");
        return false;
    } 
    
    if(!(mStartGrid.hasValidPosition() && mStartGrid.hasValidOrientation() &&
            mGoalGrid.hasValidPosition() && mGoalGrid.hasValidOrientation())) {
        LOG_WARN("Start/Goal has not been set, planning can not be executed"); 
        return false;
    }
    
    if(mReceivedNewTravGrid) {
        LOG_INFO("Replanning initiated");
        
        if(!initialize(mpTravGrid->getCellSizeX(), mpTravGrid->getCellSizeY(),
                mpTravGrid->getScaleX(), mpTravGrid->getScaleY(), mpTravData)) {
            LOG_WARN("Initialization of the planning library failed"); 
            return false;
        }
    }
    
    if(mReceivedNewStartGoal) {
        LOG_INFO("Planning from (%4.2f, %4.2f, %4.2f) to (%4.2f, %4.2f, %4.2f)",
                mStartGrid.position.x(), mStartGrid.position.y(), mStartGrid.getYaw(), 
                mGoalGrid.position.x(), mGoalGrid.position.y(), mGoalGrid.getYaw());
        
        if(!setStartGoal(mStartGrid.position.x(), mStartGrid.position.y(), mStartGrid.getYaw(), 
                mGoalGrid.position.x(), mGoalGrid.position.y(), mGoalGrid.getYaw())) {
            LOG_WARN("Start/goal could not be set");
            return false;
        }
    }
    
    if(!mReceivedNewTravGrid && !mReceivedNewStartGoal) {
        LOG_INFO("Try to improve the current solution"); 
    }
    
    // Try to solve the problem / improve the solution.
    bool solved = solve(max_time);
    
    if (solved)
    {
        LOG_INFO("Solution found");
        mPathInGrid.clear();
        fillPath(mPathInGrid);
        mReceivedNewTravGrid = false;
        mReceivedNewStartGoal = false;
        return true;
    } else {
        LOG_WARN("No solution found");        
        return false;
    }
}

std::vector<base::Waypoint> MotionPlanningLibraries::getPathInWorld() {
    std::vector<base::Waypoint> path;
    std::vector<base::samples::RigidBodyState>::iterator it = mPathInGrid.begin();
    base::samples::RigidBodyState pose_in_world;
    for(;it != mPathInGrid.end(); it++) {
        if (grid2world(mpTravGrid, *it, pose_in_world)) {
            base::Waypoint waypoint;
            waypoint.position = pose_in_world.position;
            waypoint.heading = pose_in_world.getYaw();
            path.push_back(waypoint);
        }
    }
    return path;
}

base::Trajectory MotionPlanningLibraries::getTrajectoryInWorld(double speed) {
    base::Trajectory trajectory;
    trajectory.speed = speed;
    
    std::vector<base::Vector3d> path;
    std::vector<double> parameters;
    std::vector<base::geometry::SplineBase::CoordinateType> coord_types;
    std::vector<base::samples::RigidBodyState>::iterator it = mPathInGrid.begin();
    base::samples::RigidBodyState pose_in_world;    
    for(;it != mPathInGrid.end(); it++) {
        if (grid2world(mpTravGrid, *it, pose_in_world)) {
            path.push_back(pose_in_world.position);
            coord_types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
        }
    }
    try {
        trajectory.spline.interpolate(path, parameters, coord_types);
    } catch (std::runtime_error& e) {
        LOG_ERROR("Spline exception: %s", e.what());
    }
    return trajectory;
}

base::samples::RigidBodyState MotionPlanningLibraries::getStartPoseInGrid() {
    return mStartGrid;
}

base::samples::RigidBodyState MotionPlanningLibraries::getGoalPoseInGrid() {
    return mGoalGrid;
} 

bool MotionPlanningLibraries::world2grid(envire::TraversabilityGrid const* trav,
        base::samples::RigidBodyState const& world_pose, 
        base::samples::RigidBodyState& grid_pose) {
        
    if(trav == NULL) {
        LOG_WARN("world2grid transformation requires a traversability map");
        return false;
    }

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

bool MotionPlanningLibraries::grid2world(envire::TraversabilityGrid const* trav,
        base::samples::RigidBodyState const& grid_pose,
        base::samples::RigidBodyState& world_pose) {
        
    if(trav == NULL) {
        LOG_WARN("grid2world transformation requires a traversability map");
        return false;
    }
        
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
    
    return true;
}

// PRIVATE
envire::TraversabilityGrid* MotionPlanningLibraries::extractTravGrid(envire::Environment* env, 
        std::string trav_map_id) {
    typedef envire::TraversabilityGrid e_trav;

    // Extract traversability map from evironment (as an intrusive_pointer).
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

} // namespace motion_planning_libraries
