#include "MotionPlanningLibraries.hpp"

#include <motion_planning_libraries/sbpl/SbplEnvXY.hpp>
#include <motion_planning_libraries/sbpl/SbplEnvXYTHETA.hpp>
#include <motion_planning_libraries/ompl/OmplEnvXY.hpp>
#include <motion_planning_libraries/ompl/OmplEnvXYTHETA.hpp>

namespace motion_planning_libraries
{

// PUBLIC
MotionPlanningLibraries::MotionPlanningLibraries(Config config) : mpTravGrid(NULL), 
        mpTravData(),
        mStartState(), mGoalState(), 
        mStartGrid(), mGoalGrid(), 
        mPlannedPath(),
        mReceivedNewTravGrid(false),
        mReceivedNewStartGoal(false) {

    // Creates the requested planning library.  
    switch(config.mPlanningLibType) {
        case LIB_SBPL: {
            switch(config.mEnvType) {
                case ENV_XY: {
                    mpPlanningLib = boost::shared_ptr<AbstractMotionPlanningLibrary>
                            (new SbplEnvXY(config));    
                    break;
                }
                case ENV_XYTHETA: {
                    mpPlanningLib = boost::shared_ptr<AbstractMotionPlanningLibrary>
                            (new SbplEnvXYTHETA(config)); 
                    break;
                }
                default: {
                    LOG_ERROR("Environment is not available in SBPL");
                    throw new std::runtime_error("Environment not available in SBPL");
                }
            }
            break;    
        }    
        case LIB_OMPL: {
            switch(config.mEnvType) {
                case ENV_XY: {
                    mpPlanningLib = boost::shared_ptr<AbstractMotionPlanningLibrary>
                            (new OmplEnvXY(config));    
                    break;
                }
                case ENV_XYTHETA: {
                    mpPlanningLib = boost::shared_ptr<AbstractMotionPlanningLibrary>
                            (new OmplEnvXYTHETA(config));    
                    break;
                }
                //mpPlanningLib = boost::shared_ptr<AbstractMotionPlanningLibrary>(new Ompl(config));
                default: {
                    LOG_ERROR("Environment is not available in OMPL");
                    throw new std::runtime_error("Environment not available in OMPL");
                }
            }
            break;
        }
    }
        
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

bool MotionPlanningLibraries::setStartState(struct State start_state) {

    switch (start_state.getStateType()) {
        case STATE_EMPTY: {
            LOG_WARN("Start state contains no valid values and could not be set");
            return false;
        }
        case STATE_POSE: {  // If a pose is defined convert it to the grid.
            base::samples::RigidBodyState start_grid_new;
            if(!world2grid(mpTravGrid, start_state.getPose(), start_grid_new)) {
                LOG_WARN("Start pose could not be set");
                return false;
            }
                    
            // Replan if there is a difference between the old and the new start pose.
            if(mStartState.getStateType() == STATE_POSE) {
                double dist = (mStartState.getPose().position - start_state.getPose().position).norm();
                double turn = fabs(mStartState.getPose().getYaw() - start_state.getPose().getYaw());
                if (dist > REPLANNING_DIST_THRESHOLD || turn > REPLANNING_TURN_THRESHOLD) {
                    mReceivedNewStartGoal = true;
                }
            } else {
                mReceivedNewStartGoal = true;
            }
            
            mStartState = start_state; 
            mStartGrid = start_grid_new; 
            
            break;
        }
        case STATE_ARM: {
            // Only initiate a replanning if a single joint angle exceeds the threshold.
            if(mStartState.getStateType() == STATE_ARM) {
                std::vector<double>::iterator it = mStartState.getJointAngles().begin();
                std::vector<double>::iterator it_new = start_state.getJointAngles().begin();
                assert(mStartState.getJointAngles().size() == start_state.getJointAngles().size());
                for(; it != start_state.getJointAngles().end(); it++, it_new++) {
                    if(fabs(*it - *it_new) > REPLANNING_JOINT_ANGLE_THRESHOLD) {
                       mReceivedNewStartGoal = true;
                       break; 
                    }
                }
            } else {
                mReceivedNewStartGoal = true;
            }
            
            mStartState = start_state;
            break;
        }
    }    
    return true;
}

bool MotionPlanningLibraries::setGoalState(struct State goal_state) {

    switch(goal_state.getStateType()) {
        case STATE_EMPTY: {
            LOG_WARN("Goal state contains no valid values and could not be set");
            return false;
        }
        case STATE_POSE: { // If a pose is defined convert it to the grid.
            base::samples::RigidBodyState goal_grid_new;
            if(!world2grid(mpTravGrid, goal_state.getPose(), goal_grid_new)) {
                LOG_WARN("Goal pose could not be set");
                return false;
            }
                    
            // Replan if there is a difference between the old and the new goal pose.
            if(mGoalState.getStateType() == STATE_POSE) {
                double dist = (mGoalState.getPose().position - goal_state.getPose().position).norm();
                double turn = fabs(mGoalState.getPose().getYaw() - goal_state.getPose().getYaw());
                if (dist > REPLANNING_DIST_THRESHOLD || turn > REPLANNING_TURN_THRESHOLD) {
                    mReceivedNewStartGoal = true;
                }
            } else {
                mReceivedNewStartGoal = true;
            }
            
            mGoalState = goal_state; 
            mGoalGrid = goal_grid_new; 
            break;
        }
        case STATE_ARM: {
            // Only initiate a replanning if a single joint angle exceeds the threshold.
            if(mGoalState.getStateType() == STATE_ARM) {
                std::vector<double>::iterator it = mGoalState.getJointAngles().begin();
                std::vector<double>::iterator it_new = goal_state.getJointAngles().begin();
                assert(mGoalState.getJointAngles().size() == goal_state.getJointAngles().size());
                for(; it != goal_state.getJointAngles().end(); it++, it_new++) {
                    if(fabs(*it - *it_new) > REPLANNING_JOINT_ANGLE_THRESHOLD) {
                       mReceivedNewStartGoal = true;
                       break; 
                    }
                }
            } else {
                mReceivedNewStartGoal = true;
            }
            
            mGoalState = goal_state;
            break;
        }  
    }
    
    return true;
}

bool MotionPlanningLibraries::plan(double max_time) {

    assert(mStartState.getStateType() == mGoalState.getStateType());
    
    if(mStartState.getStateType() == STATE_EMPTY) {
        LOG_WARN("Start/goal states have not been set yet, planning can not be executed"); 
        return false;
    }

    // Required checks for path planning (not required for arm movement).
    if(mStartState.getStateType() == STATE_POSE) {
        if(mpTravGrid == NULL) {
            LOG_WARN("No traversability map available, planning cannot be executed");
            return false;
        } 
        
        if(!(mStartGrid.hasValidPosition() && mGoalGrid.hasValidPosition())) {
            LOG_WARN("Start/Goal has not been set, planning can not be executed"); 
            return false;
        }
        
        if(mReceivedNewTravGrid) {
            LOG_INFO("Replanning initiated");
            
            if(!mpPlanningLib->initialize(mpTravGrid->getCellSizeX(), 
                    mpTravGrid->getCellSizeY(),
                    mpTravGrid->getScaleX(), 
                    mpTravGrid->getScaleY(),
                    mpTravGrid, 
                    mpTravData)) {
                LOG_WARN("Initialization of the planning library failed"); 
                return false;
            } else {
                mReceivedNewTravGrid = false;    
            }
        }
    }
    
    if(mReceivedNewStartGoal) {
        State start_state = mStartState;
        State goal_state = mGoalState;
        
        // Create a start state using the grid poses.
        if(mStartState.getStateType() == STATE_POSE) {
            start_state = State(mStartGrid);
            goal_state = State(mGoalGrid);
        }
        
        
        LOG_INFO("Planning from %s to %s", 
                start_state.getString().c_str(), goal_state.getString().c_str());
       
        if(!mpPlanningLib->setStartGoal(start_state, goal_state)) {
            LOG_WARN("Start/goal state could not be set");
            return false;
        } else {
            mReceivedNewStartGoal = false;
        }
    }
    
    // No new informations since last planning.
    if((mStartState.getStateType() == STATE_POSE && !mReceivedNewTravGrid && !mReceivedNewStartGoal) || 
            (mStartState.getStateType() == STATE_ARM && !mReceivedNewStartGoal)) {
        LOG_INFO("Try to improve the current solution"); 
    }
    
    // Try to solve the problem / improve the solution.
    bool solved = mpPlanningLib->solve(max_time);
    
    if (solved)
    {
        LOG_INFO("Solution found");
        mPlannedPath.clear();
        mpPlanningLib->fillPath(mPlannedPath);
        return true;
    } else {
        LOG_WARN("No solution found");        
        return false;
    }
}

std::vector<base::Waypoint> MotionPlanningLibraries::getPathInWorld() {
    std::vector<base::Waypoint> path;
    std::vector<State>::iterator it = mPlannedPath.begin();
    base::samples::RigidBodyState pose_in_world;
    for(;it != mPlannedPath.end(); it++) {
        if (grid2world(mpTravGrid, it->getPose(), pose_in_world)) {
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
    base::Vector3d last_position;
    last_position[0] = last_position[1] = last_position[2] = nan("");
    std::vector<base::geometry::SplineBase::CoordinateType> coord_types;
    std::vector<State>::iterator it = mPlannedPath.begin();
    base::samples::RigidBodyState pose_in_world;    
    for(;it != mPlannedPath.end(); it++) {
        if (grid2world(mpTravGrid, it->getPose(), pose_in_world)) {
            // Prevents to add the same position consecutively, otherwise
            // the spline creation fails.
            if(pose_in_world.position != last_position) {
                path.push_back(pose_in_world.position);
                coord_types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
            } 
            last_position = pose_in_world.position;
        }
    }
    try {
        trajectory.spline.interpolate(path, parameters, coord_types);
    } catch (std::runtime_error& e) {
        LOG_ERROR("Spline exception: %s", e.what());
    }
    return trajectory;
}

void MotionPlanningLibraries::printPathInWorld() {
    std::vector<base::Waypoint> waypoints = getPathInWorld();
    std::vector<base::Waypoint>::iterator it = waypoints.begin();
    int counter = 1;
    printf("%s %s %s %s %s\n", "       #", "       X", "       Y", "       Z", "   THETA");
    for(; it != waypoints.end(); it++, counter++) {
        printf("%8d %8.2f %8.2f %8.2f %8.2f\n", counter, it->position[0], 
                it->position[1], it->position[2], it->heading);
    }
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
