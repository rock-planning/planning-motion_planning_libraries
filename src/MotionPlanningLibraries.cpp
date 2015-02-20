#include "MotionPlanningLibraries.hpp"

#include <motion_planning_libraries/sbpl/SbplEnvXY.hpp>
#include <motion_planning_libraries/sbpl/SbplEnvXYTHETA.hpp>
#include <motion_planning_libraries/ompl/OmplEnvXY.hpp>
#include <motion_planning_libraries/ompl/OmplEnvXYTHETA.hpp>
#include <motion_planning_libraries/ompl/OmplEnvARM.hpp>
#include <motion_planning_libraries/ompl/OmplEnvSHERPA.hpp>

namespace motion_planning_libraries
{

// PUBLIC
MotionPlanningLibraries::MotionPlanningLibraries(Config config) : 
        mConfig(config),
        mpTravGrid(NULL), 
        mpTravData(),
        mStartState(), mGoalState(), 
        mStartStateGrid(), mGoalStateGrid(), 
        mPlannedPathInWorld(),
        mReceivedNewTravGrid(false),
        mReceivedNewStart(false),
        mReceivedNewGoal(false),
        mArmInitialized(false),
        mReplanRequired(false),
        mError(MPL_ERR_NONE) {
            
    // Do some checks.
    // Footprint, need radius or rectangle
    if( isnan(mConfig.mFootprintRadiusMinMax.first) && 
        isnan(mConfig.mFootprintRadiusMinMax.second) &&
        isnan(mConfig.mFootprintLengthMinMax.first) &&
        isnan(mConfig.mFootprintLengthMinMax.second) && 
        isnan(mConfig.mFootprintWidthMinMax.first) && 
        isnan(mConfig.mFootprintWidthMinMax.second)) {
        LOG_ERROR("No footprint available, either a radius or width/length have to be defined");
        throw new std::runtime_error("No footprint has been defined");
    }

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
                case ENV_ARM: {
                    mpPlanningLib = boost::shared_ptr<AbstractMotionPlanningLibrary>
                            (new OmplEnvARM(config));    
                    break;
                }
                case ENV_SHERPA: {
                    mpPlanningLib = boost::shared_ptr<AbstractMotionPlanningLibrary>
                            (new OmplEnvSHERPA(config));    
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
}

MotionPlanningLibraries::~MotionPlanningLibraries() {
}

bool MotionPlanningLibraries::setTravGrid(envire::Environment* env, std::string trav_map_id) {
    mReceivedNewTravGrid = false;
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
    mReceivedNewStart = false; 
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
                    mReceivedNewStart = true;
                }
            } else {
                mReceivedNewStart = true;
            }
            
            mStartState = start_state; 
            mStartStateGrid = State(start_grid_new, start_state.getFootprintRadius()); 
            
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
                       mReceivedNewStart = true;
                       break; 
                    }
                }
            } else {
                mReceivedNewStart = true;
            }
            
            mStartState = start_state;
            break;
        }
    }    
    return true;
}

bool MotionPlanningLibraries::setGoalState(struct State goal_state) {
    mReceivedNewGoal = false;
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
                    mReceivedNewGoal = true;
                }
            } else {
                mReceivedNewGoal = true;
            }
            
            mGoalState = goal_state; 
            mGoalStateGrid = State(goal_grid_new, goal_state.getFootprintRadius()); 
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
                       mReceivedNewGoal = true;
                       break; 
                    }
                }
            } else {
                mReceivedNewGoal = true;
            }
            
            mGoalState = goal_state;
            break;
        }  
    }
    
    return true;
}

bool MotionPlanningLibraries::plan(double max_time) {
    
    mError = MPL_ERR_NONE;

    if(mStartState.getStateType() == STATE_EMPTY) {
        LOG_WARN("Start states have not been set yet, planning can not be executed"); 
        mError = static_cast<MplErrors>((int)mError + (int)MPL_ERR_MISSING_START);
    }

    if(mGoalState.getStateType() == STATE_EMPTY) {
        LOG_WARN("Goal states have not been set yet, planning can not be executed"); 
        mError = static_cast<MplErrors>((int)mError + (int)MPL_ERR_MISSING_GOAL);
    }

    // Arm planning environment does not need a trav map.
    if(mConfig.mEnvType != ENV_ARM && mpTravGrid == NULL) {
        LOG_WARN("No traversability map available, planning cannot be executed");
        mError = static_cast<MplErrors>((int)mError + (int)MPL_ERR_MISSING_TRAV);
    } 
    
    if(mError != MPL_ERR_NONE) {
        return false;
    }

    assert(mStartState.getStateType() == mGoalState.getStateType());

    // Required checks for path planning (not required for arm movement).
    if(mConfig.mEnvType != ENV_ARM ) {
        if(mStartStateGrid.getStateType() != STATE_POSE || 
                mGoalStateGrid.getStateType() != STATE_POSE) {
            LOG_WARN("Start/Goal (grid) contains no pose, planning can not be executed"); 
            mError = MPL_ERR_WRONG_STATE_TYPE;
            return false;
        }
   
        // Currently the complete environment will be reinitialized 
        // if a new trav map has been received.
        if(mReceivedNewTravGrid) {
            LOG_INFO("Received a new trav grid, reinitializes the environment");
            
            if(!mpPlanningLib->initialize(mpTravGrid, mpTravData)) {
                LOG_WARN("Initialization (navigation) failed"); 
                mError = MPL_ERR_INITIALIZE_MAP;
                return false;
            } else {
                mReplanRequired = true;
                mReceivedNewTravGrid = false;
            }
            
            // Reset current start and goal state within the new environment!
            if(!mpPlanningLib->setStartGoal(mStartStateGrid, mGoalStateGrid)) {
                LOG_WARN("Start/goal state could not be set after reinitialization");
                mError = MPL_ERR_SET_START_GOAL;
                return false;
            }
        }
    }
    
    // Currently the arm environment will be initialized just once.
    // Later changes in the environment may require a reinitialization similar 
    // to the current implementation of the robot navigation.
    if(mConfig.mEnvType == ENV_ARM) {
        if(!mArmInitialized) {
            if(!mpPlanningLib->initialize_arm()) {
                LOG_WARN("Initialization (arm motion planning) failed"); 
                mError = MPL_ERR_UNDEFINED;
                return false;
            } else {
                mArmInitialized = true;   
            }
        }
    }
    
    // Updates the start/goal pose within the planner.
    if(mReceivedNewStart || mReceivedNewGoal) {       
        if(!mpPlanningLib->setStartGoal(mStartStateGrid, mGoalStateGrid)) {
            LOG_WARN("Start/goal state could not be set");
            mError = MPL_ERR_SET_START_GOAL;
            return false;
        } else {
            if(mReceivedNewGoal ||
                (mConfig.mReplanOnNewStartPose && mReceivedNewStart)) {
                mReplanRequired = true;
            }
            
            mReceivedNewStart = false;
            mReceivedNewGoal = false;
        }
    }
    
    // Replanning if a new goal, start (if mReplanningOnNewStartPose is true),
    // trav map (navigation planning) have been received or mReplanDuringEachUpdate
    // has been set to true.
    bool solved = false;
    if(mReplanRequired || mConfig.mReplanDuringEachUpdate) { 
                  
        LOG_INFO("Planning from %s (Grid %s) to %s (Grid %s)", 
                mStartState.getString().c_str(), mStartStateGrid.getString().c_str(),
                mGoalState.getString().c_str(), mGoalStateGrid.getString().c_str());    
            
        // Try to solve the problem / improve the solution.
        solved = mpPlanningLib->solve(max_time);
    } else {
        LOG_INFO("Replanning not required");
        return true;
    }
    // At the moment (if mReplanDuringEachUpdate is set to false) the planner just
    // tries to solve the problem once and no optimizations are made.
    // And if max_time is too small no solutions will be found.
    // So actually for navigation mConfig.mReplanDuringEachUpdate should be set to true.
    mReplanRequired = false;
    
    if (solved)
    {
        LOG_INFO("Solution found");
        
        // TODO Not required?
        /*
        mReceivedNewStart = false;
        mReceivedNewGoal = false;
        mReceivedNewTravGrid = false;
        */
        
        // By default grid coordinates are expeted.
        std::vector<State> planned_path;
        bool pos_defined_in_local_grid = false;
        mpPlanningLib->fillPath(planned_path, pos_defined_in_local_grid);
        
        if(planned_path.size() == 0) {
            LOG_WARN("Planned path does not contain any states!");
            mError = MPL_ERR_UNDEFINED;
            return false;
        }
        
        // Convert path from grid or grid-local to world.
        mPlannedPathInWorld.clear();
        std::vector<State>::iterator it = planned_path.begin();
        base::samples::RigidBodyState rbs_world;
        for(; it != planned_path.end(); it++) {
            if(pos_defined_in_local_grid) {
                gridlocal2world(mpTravGrid, it->getPose(), rbs_world);
            } else {
                grid2world(mpTravGrid, it->getPose(), rbs_world);
            }
            it->setPose(rbs_world);
            mPlannedPathInWorld.push_back(*it);
        }
        return true;
    } else {
        LOG_WARN("No solution found");     
        mError = MPL_ERR_PLANNING_FAILED;
        return false;
    }
}

std::vector<struct State> MotionPlanningLibraries::getStatesInWorld() {
    return mPlannedPathInWorld;
}

std::vector<base::Waypoint> MotionPlanningLibraries::getPathInWorld() {
    std::vector<base::Waypoint> path;
    std::vector<State>::iterator it = mPlannedPathInWorld.begin();
    base::Waypoint waypoint;
    for(;it != mPlannedPathInWorld.end(); it++) {
        waypoint.position = it->getPose().position;
        waypoint.heading = it->getPose().getYaw();
        path.push_back(waypoint);  
    }
    return path;
}

std::vector<base::Trajectory> MotionPlanningLibraries::getTrajectoryInWorld(double speed) {
    
    std::vector<base::Trajectory> trajectories;
   
    double use_this_speed = 0.0;
    double last_speed = nan("");
    std::vector<base::Vector3d> path;
    std::vector<double> parameters;
    std::vector<base::geometry::SplineBase::CoordinateType> coord_types;
    base::Vector3d last_position;
    last_position[0] = last_position[1] = last_position[2] = nan("");
    
    std::vector<State>::iterator it = mPlannedPathInWorld.begin();  
    // If the states contain speed values they will be used (can be checked by checking the first state).
    // So for each different forward and backward speed a single tranjectory will be created.
    // Otherwise just a single trajectory will be created using the passed speed parameter.
    bool create_one_trajectory_with_one_speed = !it->mSpeeds.isSet();
    if( create_one_trajectory_with_one_speed) {
        LOG_INFO("Create one trajectory with speed %4.2f", speed);
    } else {
        LOG_INFO("Create trajectories using the speed of the planner library");
    }
    
    for(;it != mPlannedPathInWorld.end(); it++) {
        if(create_one_trajectory_with_one_speed) {
            use_this_speed = speed;
        } else if(it->mSpeeds.isSet()) { // Just regards states with new, not-empty speeds. 
            // Forward and backward speed are not allowed to be set at the same time.
            if(it->mSpeeds.mSpeedForward && it->mSpeeds.mSpeedBackward) {
                LOG_ERROR("Trajetory cannot be created properly, forward and backward speed are set (%4.2f, %4.2f)!", 
                        it->mSpeeds.mSpeedForward, it->mSpeeds.mSpeedBackward);
                return std::vector<base::Trajectory>();
            }
            use_this_speed = it->mSpeeds.mSpeedForward + it->mSpeeds.mSpeedBackward;
            LOG_INFO("New trajectory speed: %4.2f", use_this_speed);
        }
        
        // Add positions to path.
        base::Vector3d position = it->getPose().position;
        // Prevents to add the same position consecutively, otherwise
        // the spline creation fails.
        if(position != last_position) {
            path.push_back(position);
            coord_types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
        } 
        last_position = position;
        
        // For each new speed a new trajectory will be created (if already points have been added)
        if(use_this_speed != last_speed && path.size() > 1) {
            LOG_INFO("Add trajectory with speed: %4.2f", last_speed);
            base::Trajectory trajectory;
            trajectory.speed = last_speed;  
            try {
                trajectory.spline.interpolate(path, parameters, coord_types);
            } catch (std::runtime_error& e) {
                LOG_ERROR("Spline exception: %s", e.what());
                return std::vector<base::Trajectory>();
            }
            trajectories.push_back(trajectory);
            path.clear();
            coord_types.clear();
            
            // Re-add current point to be the starting point of the next trajectory.
            path.push_back(position);
            coord_types.push_back(base::geometry::SplineBase::ORDINARY_POINT);

            last_position = position;
        }
        last_speed = use_this_speed;
    }   
    
    // Create a trajectory with all remaining points or actually all points
    // if only one speed has been used.
    // Last point does not contain any speed (endpoint), so the path should at
    // least contain two points, so the trajectory should be valid.
    if(path.size() > 1) {
        base::Trajectory trajectory;
        trajectory.speed = use_this_speed;  
        LOG_INFO("Add trajectory with speed: %4.2f", use_this_speed);
        try {
            trajectory.spline.interpolate(path, parameters, coord_types);
        } catch (std::runtime_error& e) {
            LOG_ERROR("Spline exception: %s", e.what());
        }
        trajectories.push_back(trajectory);
    }
            
    return trajectories;
}

void MotionPlanningLibraries::printPathInWorld() {
    std::vector<base::Waypoint> waypoints = getPathInWorld();
    std::vector<base::Waypoint>::iterator it = waypoints.begin();
    std::vector<State>::iterator it_state = mPlannedPathInWorld.begin();
    
    int counter = 1;
    
    if(mConfig.mEnvType == ENV_SHERPA) 
    {
        printf("%s %s %s %s %s %s\n", "       #", "       X", "       Y",
                "       Z", "   THETA", "  RADIUS");
        for(; it != waypoints.end() && it_state != mPlannedPathInWorld.end(); it++, counter++, it_state++) {
            printf("%8d %8.2f %8.2f %8.2f %8.2f %8.2f\n", counter, 
                    it->position[0], it->position[1], it->position[2], 
                    it->heading, it_state->getFootprintRadius());
        }
    } 
    else if(mConfig.mEnvType == ENV_XYTHETA && mConfig.mPlanningLibType == LIB_SBPL) 
    {
        printf("%s %s %s %s %s %s %s\n", "       #", "       X", "       Y",
                "       Z", "   THETA", " PRIM ID", "  SPEEDS");
        for(; it != waypoints.end() && it_state != mPlannedPathInWorld.end(); it++, counter++, it_state++) {
            printf("%8d %8.2f %8.2f %8.2f %8.2f %8.2d %s\n", counter, 
                    it->position[0], it->position[1], it->position[2], 
                    it->heading, it_state->mSBPLPrimId, it_state->mSpeeds.toString().c_str()
                  );
        }
    } 
    else 
    {
        printf("%s %s %s %s %s\n", "       #", "       X", "       Y", "       Z", "   THETA");
        for(; it != waypoints.end(); it++, counter++) {
            printf("%8d %8.2f %8.2f %8.2f %8.2f\n", counter, it->position[0], 
                    it->position[1], it->position[2], it->heading);
        }
    }
}

base::samples::RigidBodyState MotionPlanningLibraries::getStartPoseInGrid() {
    return mStartStateGrid.getPose();
}

base::samples::RigidBodyState MotionPlanningLibraries::getGoalPoseInGrid() {
    return mGoalStateGrid.getPose();
} 

bool MotionPlanningLibraries::getSbplMotionPrimitives(struct SbplMotionPrimitives& prims) {

    SbplEnvXYTHETA* sbpl_xytheta = dynamic_cast<SbplEnvXYTHETA*>(mpPlanningLib.get());
    if(sbpl_xytheta) {
        struct SbplMotionPrimitives* prims_ptr = sbpl_xytheta->getMotionPrimitives();
        if(prims_ptr == NULL) {
            LOG_WARN("Automatically generated motion primitives are not available");
            return false;
        } else { // Copy primitives and return.
            prims = *prims_ptr;
            return true;
        }
    } else {
        LOG_WARN("Current environment is not SbplEnvXYTHETA");
        return false;
    }
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
    //trav->fromGrid(x_grid, y_grid, x_local, y_local);
    // TODO Is it ok: Do not use fromGrid() to avoid shifting to the cell center.
    x_local = x_grid * trav->getScaleX() + trav->getOffsetX();
    y_local = y_grid * trav->getScaleY() + trav->getOffsetY();

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

bool MotionPlanningLibraries::gridlocal2world(envire::TraversabilityGrid const* trav,
        base::samples::RigidBodyState const& grid_local_pose,
        base::samples::RigidBodyState& world_pose) {
        
    if(trav == NULL) {
        LOG_WARN("grid2world transformation requires a traversability map");
        return false;
    }
        
    // Transformation LOCAL2WOLRD
    Eigen::Affine3d local2world = trav->getEnvironment()->relativeTransform(
        trav->getFrameNode(),
        trav->getEnvironment()->getRootNode());
    world_pose.setTransform(local2world * grid_local_pose.getTransform() );
    
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
