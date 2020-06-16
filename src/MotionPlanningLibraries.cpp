#include "MotionPlanningLibraries.hpp"

#include "Helpers.hpp"

#include <motion_planning_libraries/sbpl/SbplEnvXY.hpp>
#include <motion_planning_libraries/sbpl/SbplEnvXYTHETA.hpp>
#include <motion_planning_libraries/ompl/OmplEnvXY.hpp>
#include <motion_planning_libraries/ompl/OmplEnvXYTHETA.hpp>
#include <motion_planning_libraries/ompl/OmplEnvARM.hpp>
#include <motion_planning_libraries/ompl/OmplEnvSHERPA.hpp>

namespace motion_planning_libraries
{

Eigen::Affine3d MotionPlanningLibraries::mWorld2Local = Eigen::Affine3d::Identity();

// PUBLIC
MotionPlanningLibraries::MotionPlanningLibraries(Config config) : 
        mConfig(config),
        mpEnv(nullptr),
        mpTravGrid(NULL), 
        mpTravData(),
        mpLastTravGrid(NULL),
        mStartState(), mGoalState(), 
        mStartStateGrid(), mGoalStateGrid(), 
        mPlannedPathInWorld(),
        mReplanRequired(false),
        mNewGoalReceived(false),
        mLostX(0.0),
        mLostY(0.0),
        mError(MPL_ERR_NONE) {
            
    // Do some checks.
    // Footprint, need radius or rectangle
    if( std::isnan(mConfig.mFootprintRadiusMinMax.first) &&
        std::isnan(mConfig.mFootprintRadiusMinMax.second) &&
        std::isnan(mConfig.mFootprintLengthMinMax.first) &&
        std::isnan(mConfig.mFootprintLengthMinMax.second) &&
        std::isnan(mConfig.mFootprintWidthMinMax.first) &&
        std::isnan(mConfig.mFootprintWidthMinMax.second)) {
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
    
    // Currently the arm environment will be initialized just once.
    // Later changes in the environment may require a reinitialization similar 
    // to the current implementation of the robot navigation.
    if(mConfig.mEnvType == ENV_ARM) {
        if(!mpPlanningLib->initialize_arm()) {
            throw std::runtime_error("Arm environment culd not be initialized");
        }
    }
}

MotionPlanningLibraries::~MotionPlanningLibraries() {
}

bool MotionPlanningLibraries::setTravGrid(envire::Environment* env, std::string trav_map_id) {

    if(mpPlanningLib == NULL) {
        LOG_WARN("Planning library has not been allocated yet");
        return false;
    }
    envire::TraversabilityGrid* trav_grid = extractTravGrid(env, trav_map_id);
    if(trav_grid == NULL) {
        LOG_WARN("Traversability map could not be extracted");
        return false;
    } 
    
    LOG_INFO("Received Trav Map: Number of cells (%d, %d), cell size in meter (%4.2f, %4.2f), offset (%4.2f, %4.2f)", 
            trav_grid->getCellSizeX(), trav_grid->getCellSizeY(), 
            trav_grid->getScaleX(), trav_grid->getScaleY(), 
            trav_grid->getOffsetX(), trav_grid->getOffsetY());
    
    // Currently if you start the grap-slam-module and do not wait a few seconds
    // you get a map with sizex/sizey 0.1.
    if(trav_grid->getSizeX() < 1 || trav_grid->getSizeY() < 1) {
        LOG_ERROR("Size of the extracted map is incorrect (%4.2f, %4.2f)", trav_grid->getSizeX(), trav_grid->getSizeY());
        return false;
    }
    
    // If the map size has not changed, partial updates are possible.
    bool different_map_size = true;
    if(mpLastTravGrid != NULL) {
        different_map_size = mpTravGrid->getWidth() != mpLastTravGrid->getWidth() ||
            mpTravGrid->getHeight() != mpLastTravGrid->getHeight();
        LOG_INFO("Trav map sizes are different: %s", different_map_size ? "true" : "false");
    }
    
    std::vector<CellUpdate> cell_updates;
    // Tests if partialUpdates are supported by the planning library (empty vector should return true).
    bool partial_update_implemented = mpPlanningLib->partialMapUpdate(cell_updates);
    bool partial_update_successful = false;
    // Execute the partial update.
    if(!different_map_size && partial_update_implemented) {
        collectCellUpdates(mpLastTravGrid, mpTravGrid, cell_updates);
        partial_update_successful = mpPlanningLib->partialMapUpdate(cell_updates);
        if(!partial_update_successful) {
             LOG_WARN("A complete initialization will be executed, a partial update failed");
        }
    }
    
    mpTravGrid = trav_grid;
    // Creates a copy of the new grid data.
    mpTravData = boost::shared_ptr<TravData>(new TravData(
            trav_grid->getGridData(envire::TraversabilityGrid::TRAVERSABILITY)));
    
    // Stores last trav map for partial update testing.
    if(mpLastTravGrid != NULL) {
        delete mpLastTravGrid;
        mpLastTravGrid = NULL;
    }
    mpLastTravGrid = new envire::TraversabilityGrid(*mpTravGrid);
    //envire::TraversabilityGrid grid = *mpTravGrid;
    
    
    // Reinitialize the complete planning environment.
    // Will be used if the partial update has not been implemented or could not be executed.
    if(!partial_update_successful && !mpPlanningLib->initialize(mpTravGrid, mpTravData)) {
        LOG_WARN("Initialization (navigation) failed"); 
        mError = MPL_ERR_INITIALIZE_MAP;
        return false;
    }
    
    // Reset current start and goal state within the new environment if they are valid!
    // The new trav grid can contain another transformation, so 
    // the old start and goal pose have to be transformed into the grid again.
    // The boolean in setGoalState() prevents an unwanted replanning.
    if(mStartState.hasValidPosition() && mGoalState.hasValidPosition()) {
        if(!setStartState(mStartState) || !setGoalState(mGoalState, true)) {
            LOG_ERROR("Old start and goal pose could not be transformed into the new environment");
            mError = MPL_ERR_SET_START_GOAL;
            return false;
        }
        
        if(!mpPlanningLib->setStartGoal(mStartStateGrid, mGoalStateGrid)) {
            LOG_WARN("Start/goal state could not be set after reinitialization");
            mError = MPL_ERR_SET_START_GOAL;
            return false;
        }
        
        // Replanning without valid start/goal is not necessary.
        if(mConfig.mReplanning.mReplanOnNewMap) {
            mReplanRequired = true;
        }
    }
    return true;
}

bool MotionPlanningLibraries::setTravGrid(const maps::grid::TraversabilityGrid& trav)
{
    boost::intrusive_ptr<envire::TraversabilityGrid> envireTrav = convertToEnvire(trav);
    std::string uuid = "/trav_map";
    envireTrav->setUniqueId(uuid);

    if (!mpEnv)
        mpEnv = boost::shared_ptr<envire::Environment>(new envire::Environment);

    // Detach item if there is already a map in the environment.
    // Not storing the returned intrusive_ptr causes the item to self delete.
    boost::intrusive_ptr<envire::TraversabilityGrid> trav_ptr = mpEnv->getItem<envire::TraversabilityGrid>(uuid);
    if (trav_ptr)
        mpEnv->detachItem(trav_ptr.get());

    // Check for a frame node to reuse.
    std::list<envire::FrameNode*> frameNodes = mpEnv->getRootNode()->getChildren();
    envire::FrameNode* travFrameNode;
    if (frameNodes.empty())
        travFrameNode = new envire::FrameNode();
    else
        travFrameNode = frameNodes.front();

    mpEnv->attachItem(envireTrav.get());
    mpEnv->getRootNode()->addChild(travFrameNode);
    envireTrav->setFrameNode(travFrameNode);

    // Set transform in case it was updated or hasn't been set yet.
    travFrameNode->setTransform(mWorld2Local);

    return setTravGrid(mpEnv.get(), uuid);
}

bool MotionPlanningLibraries::setStartState(struct State new_state) {
    if(mpPlanningLib == NULL) {
        LOG_WARN("Planning library has not been allocated yet");
        return false;
    }
    
    bool new_state_received = mStartState.differs(new_state);
    
    switch (new_state.getStateType()) {
        case STATE_EMPTY: {
            LOG_WARN("States contain no valid values and could not be set");
            return false;
        }
        case STATE_POSE: {  // If a pose is defined convert it to the grid.
            if(!travGridAvailable()) {
                LOG_WARN("A traversability map is required to set the start/goal pose.");
                return false;
            }
            
            if(!new_state.mPose.hasValidPosition()) {
                LOG_WARN("Received start pose does not contain a valid position");
            }
            
            // Start
            base::samples::RigidBodyState new_grid;
            if(!world2grid(mpTravGrid, new_state.getPose(), new_grid)) {
                LOG_WARN("Start pose could not be transformed into the grid");
                return false;
            }
            mStartState = new_state; 
            mStartStateGrid = new_state;
            mStartStateGrid.mPose = new_grid;
            //State(new_grid, new_state.getFootprintRadius()); 
            //mStartStateGrid.mSpeed = new_state.mSpeed;
            break;
        }
        case STATE_ARM: {
            mStartState = mStartStateGrid = new_state;
            break;
        }
    }
    
    // If required create dummy goal state (a start copy).
    State goal_state_grid;
    if(goalStateAvailable()) {
        goal_state_grid = mGoalStateGrid;
    } else {
        goal_state_grid = mStartStateGrid;
    }
    
    if(!mpPlanningLib->setStartGoal(mStartStateGrid, goal_state_grid)) {
            LOG_WARN("Start/goal state could not be set");
            mError = MPL_ERR_SET_START_GOAL;
            return false;
    }
    
    if(mConfig.mReplanning.mReplanOnNewStartPose && new_state_received) {
        mReplanRequired = true;
        
    }
    
    return true;
}

bool MotionPlanningLibraries::setGoalState(struct State new_state, bool reset) {
    if(mpPlanningLib == NULL) {
        LOG_WARN("Planning library has not been allocated yet");
        return false;
    }
    
    switch (new_state.getStateType()) {
        case STATE_EMPTY: {
            LOG_WARN("States contain no valid values and could not be set");
            return false;
        }
        case STATE_POSE: {  // If a pose is defined convert it to the grid.
            if(!travGridAvailable()) {
                LOG_WARN("A traversability map is required to set the start/goal pose.");
                return false;
            }
            
            if(!new_state.mPose.hasValidPosition()) {
                LOG_WARN("Received goal pose does not contain a valid position");
            }
            
            // Start
            base::samples::RigidBodyState new_grid;
            if(!world2grid(mpTravGrid, new_state.getPose(), new_grid, &mLostX, &mLostY)) {
                LOG_WARN("Goal pose could not be transformed into the grid");
                return false;
            }
            mGoalState = new_state; 
            mGoalStateGrid = new_state;
            mGoalStateGrid.mPose = new_grid;
            //State(new_grid, new_state.getFootprintRadius()); 
            //mGoalStateGrid.mSpeed = new_state.mSpeed;
            break;
        }
        case STATE_ARM: {
            mGoalState = mGoalStateGrid = new_state;
            break;
        }
    }
    
    // If required create dummy start state (a goal copy).
    State start_state_grid;
    if(startStateAvailable()) {
        start_state_grid = mStartStateGrid;
    } else {
        start_state_grid = mGoalStateGrid;
    }
    
    if(!mpPlanningLib->setStartGoal(start_state_grid, mGoalStateGrid)) {
            LOG_WARN("Start/goal state could not be set");
            mError = MPL_ERR_SET_START_GOAL;
            return false;
    }
    
    if(!reset) {
        if(mConfig.mReplanning.mReplanOnNewGoalPose) {
            mReplanRequired = true;
        }  
        mNewGoalReceived = true;
    }
        
    return true;
}

bool MotionPlanningLibraries::allInputsAvailable(enum MplErrors& err) {
    int err_i = (int)MPL_ERR_NONE;
    // TODO CHECK
    if(!travGridAvailable()) {
        LOG_INFO("No traversability map available");
        err_i += (int)MPL_ERR_MISSING_TRAV;
    }
    
    if(!startStateAvailable()) {
        LOG_INFO("No start state available");
        err_i += (int)MPL_ERR_MISSING_START;
    }
    
    if(!goalStateAvailable()) {
        LOG_INFO("No goal state available");
        err_i += (int)MPL_ERR_MISSING_GOAL;
    }
    
    err = (enum MplErrors)err_i;
    
    if(err != MPL_ERR_NONE) {
        LOG_WARN("Input is missing, planning cannot be executed");
        return false;
    }
    return true;
}

bool MotionPlanningLibraries::replanningRequired() {
    // TODO If foundFinalSolution is not implemented it will always return true.
    // So we only plan once even the solution may not be the final one.
    bool ret = mReplanRequired || 
                    mConfig.mReplanning.mReplanDuringEachUpdate || 
                    !mpPlanningLib->foundFinalSolution();
    
    double dist = mStartState.dist(mGoalState);
    if(dist < mConfig.mReplanning.mReplanMinDistStartGoal &&
            !mNewGoalReceived) {
        LOG_INFO("Distance %4.2f to goal prevents replanning, a new goal is required", dist);
        ret = false;
    }
    
    if(ret) {
        LOG_INFO("Replanning required");
    } else {
        LOG_INFO("Replanning not required");
    }
    
    return ret;
}

bool MotionPlanningLibraries::foundFinalSolution()
{
    return mpPlanningLib->foundFinalSolution();
}


bool MotionPlanningLibraries::plan(double max_time, double& cost) {
    
    if(mpPlanningLib == NULL) {
        LOG_WARN("Planning library has not been allocated yet");
        return false;
    }
    
    if(max_time <= 0) {
        LOG_WARN("Max allowed planning time must exceed 0, set to 1");
        max_time = 1.0;
    }
    
    if(!allInputsAvailable(mError)){
        return false;
    }
    
    assert(mStartState.getStateType() == mGoalState.getStateType());
    
    // Replanning required?
    // Distance start goal can prevent replanning and 
    // mReplanDuringEachUpdate always initiates a replanning.
    if(!replanningRequired()) { 
        mError = MPL_ERR_REPLANNING_NOT_REQUIRED;
        return false;
    }
    
    // Check whether start and/or goal lies on an obstacle.
    mError = mpPlanningLib->isStartGoalValid();
    if(mError != MPL_ERR_NONE) {
        mReplanRequired = false;
        return false;
    }
    
    // Planning
    LOG_INFO("Planning from \n%s (Grid %s) \nto \n%s (Grid %s)", 
        mStartState.getString().c_str(), mStartStateGrid.getString().c_str(),
        mGoalState.getString().c_str(), mGoalStateGrid.getString().c_str());    
    bool solved = mpPlanningLib->solve(max_time);
    mReplanRequired = false;
    mNewGoalReceived = false;
    
    if (!solved) {
        LOG_WARN("No solution found");   
        mError = MPL_ERR_PLANNING_FAILED;
        return false;
    }

    // Solution found.
    LOG_INFO("Solution found");
    
    // Request costs if available, otherwise nan is returned.
    cost = mpPlanningLib->getCost();
    
    // By default grid coordinates are expected.
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
    
    // Calculate distance between goal pose and end of trajectory.
    // Currently with OMPL the trajectory may not reach the goal pose.
    if(mPlannedPathInWorld.size() > 0) {
        base::samples::RigidBodyState end_pose_trajectory = (mPlannedPathInWorld.end()-1)->mPose;
        double dist = (end_pose_trajectory.position - mGoalState.getPose().position).head(2).norm();
        double max_allowed_dist = 0.2;
        LOG_INFO("Distance end of trajectory to goal position in world: %4.2f", dist);
        if(dist > max_allowed_dist) {
            LOG_WARN("Goal position could only be reached imprecisely (>%4.2f m)", max_allowed_dist);
            mError = MPL_ERR_GOAL_COULD_ONLY_BE_REACHED_IMPRECISELY;
            mReplanRequired = true; // TODO Does this cause any troubles?
            return false;
        }
    }
    
    return true;
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

std::vector<base::Trajectory> MotionPlanningLibraries::getTrajectoryInWorld() {
    
    std::vector<base::Trajectory> trajectories;
    
    if(mConfig.mMobility.mSpeed == 0) {
        LOG_WARN("No speed has been defined within the mobility struct, trajectory will be empty");
        return trajectories;
    }
   
    double use_this_speed = 0.0;
    double last_speed = nan("");
    std::vector<base::Vector3d> path;
    std::vector<double> parameters;
    std::vector<base::geometry::SplineBase::CoordinateType> coord_types;
    base::Vector3d last_position;
    last_position[0] = last_position[1] = last_position[2] = nan("");
     
    std::vector<State>::iterator it = mPlannedPathInWorld.begin();  
    LOG_DEBUG("mPlannedPathInWorld size %d", mPlannedPathInWorld.size()); 
    for(;it != mPlannedPathInWorld.end(); it++) {
        if(!std::isnan(it->mSpeed)) {
            use_this_speed = it->mSpeed;
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
        
        // Last point does not contain any speed (endpoint), so the path should at
        // least contain two points, so the trajectory should be valid.
        if(path.size() > 1){
            // For each new speed a new trajectory will be created (if already more than one point have been added)
            // If the last point have been reached a trajectory will be created with all the remaining points
            // (or all points if only one speed has been used).
            if((use_this_speed != last_speed) || it+1 == mPlannedPathInWorld.end()) {
                base::Trajectory trajectory;
                trajectory.speed = last_speed;
                LOG_DEBUG("Adds trajectory with speed %4.2f, path contains %d coordinates",
                        last_speed, path.size());
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
        }
        last_speed = use_this_speed;
    }   
    
    /*
    // Create a trajectory with all remaining points or actually all points
    // if only one speed has been used.
    // Last point does not contain any speed (endpoint), so the path should at
    // least contain two points, so the trajectory should be valid.
    if(path.size() > 1) {
        base::Trajectory trajectory;
        trajectory.speed = use_this_speed;  
        LOG_DEBUG("Add trajectory with speed: %4.2f", use_this_speed);
        try {
            trajectory.spline.interpolate(path, parameters, coord_types);
        } catch (std::runtime_error& e) {
            LOG_ERROR("Spline exception: %s", e.what());
        }
        trajectories.push_back(trajectory);
    }
    */      
    return trajectories;
}

std::vector<base::Trajectory> MotionPlanningLibraries::getEscapeTrajectoryInWorld() {
    
    if(mPlannedPathInWorld.size() == 0) {
        LOG_WARN("No path available, escape trajectory cannot be created");
        return std::vector<base::Trajectory>();
    }
    
    if(mpTravGrid == NULL) {
        LOG_WARN("No traversability map available, escape trajectory cannot be created");
        return std::vector<base::Trajectory>();
    }
    
    std::vector<base::Trajectory> trajectories = getTrajectoryInWorld();
    std::vector<base::Trajectory> inverted_trajectories;
    GridCalculations grid_calc;
    grid_calc.setTravGrid(mpTravGrid, mpTravData);
    double max_radius = mConfig.getMaxRadius();
    double min_cell_size = std::min(mpTravGrid->getScaleX(), mpTravGrid->getScaleY());
    double robot_max_radius_in_grid =   max_radius / min_cell_size; 
    // Checks all cells within the radius.. can be very expensive.
    // Footprint radius is increased a little bit to add some extra safety distance.
    robot_max_radius_in_grid *= mConfig.mEscapeTrajRadiusFactor;
    /// \todo "Could use false here, so that just the center and the border of the circle are used."
    grid_calc.setFootprintCircleInGrid(robot_max_radius_in_grid, true);
    LOG_DEBUG("Robot max radius %4.2f, min cell size %4.2f, robot max radius in grid %4.2f\n", 
            max_radius, min_cell_size, robot_max_radius_in_grid);
    
    bool free_point_found = false;
    base::samples::RigidBodyState rbs_world, rbs_grid;
    rbs_world.orientation.setIdentity();
    
    if(trajectories.size() == 0) {
        LOG_ERROR("Trajectories size is 0, escape trajectory could not be created");
        return std::vector<base::Trajectory>();
    }
    
    for(int i=(int)(trajectories.size())-1; i>=0; i--) {
        LOG_DEBUG("Trajectory %u", i);
        double inverted_speed = -trajectories[i].speed; // Invert speed.
        base::geometry::Spline<3> spline = trajectories[i].spline;
        
        // Creates a point every 0.1m.
        double dist = 0.1;
        double division = spline.getCurveLength() / dist;
        if(division < 2) {
            division = 2; // Divides each spline at least into two pieces (each spline requires at least two points).
        }
        double stepSize = (spline.getEndParam() - spline.getStartParam()) / division;
        std::vector<base::Vector3d> inverted_points;
        std::vector<base::geometry::SplineBase::CoordinateType> coord_types;
        base::Vector3d point;
        LOG_DEBUG("Spline %d: Start %4.2f End %4.2f Step %4.2f", i, spline.getStartParam(), spline.getEndParam(), stepSize);
        
        // Extract points from spline from end to start.
        for(double p = spline.getEndParam(); p >= spline.getStartParam(); p -= stepSize ) { 
            point = spline.getPoint(p);
            inverted_points.push_back(point);
            coord_types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
            LOG_DEBUG("Adds point (%4.2f, %4.2f) to the escape trajectory", point[0], point[1]);
            // Stops if the first point does not lie on an obstacle anymore.
            // Transforms to the grid and uses the max radius of the system.
            rbs_world.position = point;
            world2grid(mpTravGrid, rbs_world, rbs_grid, NULL, NULL); 
            grid_calc.setFootprintPoseInGrid(rbs_grid.position[0], rbs_grid.position[1], 0);
            try {
                free_point_found = grid_calc.isValid();
                LOG_DEBUG("Free point found: %s", free_point_found ? "true" : "false");
            } catch (std::runtime_error& e) {
                LOG_ERROR("Exception in isValid: %s, escape trajectory cannot be created", e.what());
                return std::vector<base::Trajectory>();
            }
            if(free_point_found && inverted_points.size() >= 2) {
                LOG_DEBUG("Free point found and at least two inverted points are available");
                break;
            }
        }
        base::Trajectory inverted_trajectory;
        try {
            inverted_trajectory.speed = inverted_speed;
            std::vector<double> parameters;
            inverted_trajectory.spline.interpolate(inverted_points, parameters, coord_types);
        } catch (std::runtime_error& e) {
            LOG_ERROR("Spline exception: %s", e.what());
            return std::vector<base::Trajectory>();
        }
        inverted_trajectories.push_back(inverted_trajectory);
        if(free_point_found) {
            break;
        }
    }
    if(free_point_found) {
        LOG_INFO("Escape trajectory contains %d splines\n", inverted_trajectories.size());
        LOG_INFO("First safe point is at %4.2f %4.2f\n", rbs_world.position[0], rbs_world.position[1]);
        //return inverted_trajectories;
    } else {
        LOG_INFO("Escape trajectory could NOT be found, empty trajectory will be returned");
        //return std::vector<base::Trajectory>();
    }
    // TODO Currently we support an escape trajectory even if all positions are invalid.
    // We need sth. to escape.. 
    return inverted_trajectories;
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
        printf("%s %s %s %s %s %s %s %s\n", "       #", "       X", "       Y",
                "       Z", "   THETA", " PRIM ID", "  SPEEDS", "MOVEMENT TYPE");
        for(; it != waypoints.end() && it_state != mPlannedPathInWorld.end(); it++, counter++, it_state++) {
            printf("%8d %8.2f %8.2f %8.2f %8.2f %8.2d %8.2f %s\n", counter, 
                    it->position[0], it->position[1], it->position[2], 
                    it->heading, it_state->mSBPLPrimId, it_state->mSpeed,
                    MovementTypesString[it_state->mMovType].c_str()
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

void MotionPlanningLibraries::setWorld2Local(Eigen::Affine3d world2local)
{
    mWorld2Local = world2local;
}

Eigen::Affine3d MotionPlanningLibraries::getWorld2Local()
{
    return mWorld2Local;
}

bool MotionPlanningLibraries::world2grid(envire::TraversabilityGrid const* trav,
        base::samples::RigidBodyState const& world_pose, 
        base::samples::RigidBodyState& grid_pose,
        double* lost_x,
        double* lost_y) {
        
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
    bool ret = true;
    double xmod = 0.0, ymod = 0.0;
    if(lost_x != NULL && lost_y != NULL) {
        ret = trav->toGrid(local_pose.position.x(), local_pose.position.y(), 
            x_grid, y_grid,
            xmod, ymod);
        *lost_x = xmod;
        *lost_y = ymod;
    } else {
        ret = trav->toGrid(local_pose.position.x(), local_pose.position.y(), 
            x_grid, y_grid);
    }
    if(!ret) 
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
    /// \todo "Check if it is it ok: Do not use fromGrid() to avoid shifting to the cell center."
    x_local = x_grid * trav->getScaleX() + trav->getOffsetX();
    y_local = y_grid * trav->getScaleY() + trav->getOffsetY();
    
    // Readds discretization error based on the set goal pose.
    x_local += mLostX;
    y_local += mLostY;

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
    
    base::samples::RigidBodyState grid_local_pose_tmp = grid_local_pose;
    
    // We got a grid local pose, but the trav map offset is still missing.
    grid_local_pose_tmp.position[0] += trav->getOffsetX();
    grid_local_pose_tmp.position[1] += trav->getOffsetY();
    
    // Readds discretization error based on the set goal pose.
    grid_local_pose_tmp.position[0] += mLostX;
    grid_local_pose_tmp.position[1] += mLostY;
        
    // Transformation LOCAL2WOLRD
    Eigen::Affine3d local2world = trav->getEnvironment()->relativeTransform(
        trav->getFrameNode(),
        trav->getEnvironment()->getRootNode());
    world_pose.setTransform(local2world * grid_local_pose_tmp.getTransform() );
    
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

void MotionPlanningLibraries::collectCellUpdates( envire::TraversabilityGrid* old_map,
        envire::TraversabilityGrid* new_map,
        std::vector<CellUpdate>& cell_updates) {
    
    assert(old_map->getCellSizeX() == new_map->getCellSizeX());
    assert(old_map->getCellSizeY() == new_map->getCellSizeY());

    base::Time start_t = base::Time::now();
    
    int cell_counter = 0;
    int cell_counter_same = 0;
    double driveability = 0.0;
    double probability = 0.0;
    
    envire::Grid<uint8_t>::ArrayType& trav_old = old_map->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);
    envire::Grid<uint8_t>::ArrayType& prob_old = old_map->getGridData(envire::TraversabilityGrid::PROBABILITY);
    envire::Grid<uint8_t>::ArrayType& trav_new = new_map->getGridData(envire::TraversabilityGrid::TRAVERSABILITY);
    envire::Grid<uint8_t>::ArrayType& prob_new = new_map->getGridData(envire::TraversabilityGrid::PROBABILITY);
    
    uint8_t* trav_old_p = trav_old.origin();
    uint8_t* prob_old_p = prob_old.origin();
    uint8_t* trav_new_p = trav_new.origin();
    uint8_t* prob_new_p = prob_new.origin();
    
    // TODO Probability relevant? Currently not used as double.
    for(unsigned int y=0; y < new_map->getCellSizeY(); ++y) {
        for(unsigned int x=0; x < new_map->getCellSizeX(); ++x) {
            if(*trav_old_p != *trav_new_p || *prob_old_p != *prob_new_p) {
                driveability = new_map->getTraversabilityClass(*trav_new_p).getDrivability();
                // Does the same conversion which is done in TraversabilityGrid.
                probability = ((double)*prob_new_p) /std::numeric_limits< uint8_t >::max();
                cell_updates.push_back(CellUpdate(x, y, *trav_new_p, probability, driveability));
                cell_counter++;
            } else {
                cell_counter_same++;
            }
            trav_old_p++;
            prob_old_p++;
            trav_new_p++;
            prob_new_p++;
        }
    }
    
    LOG_INFO("%d different cells collected within %4.4f sec.", cell_counter, 
            (base::Time::now() - start_t).toSeconds());
    LOG_INFO("Number of unchanged cells %d", cell_counter_same);
}

boost::intrusive_ptr<envire::TraversabilityGrid> MotionPlanningLibraries::convertToEnvire(const maps::grid::TraversabilityGrid& traversabilityGrid) const
{
    maps::grid::Vector2ui numCells = traversabilityGrid.getNumCells();
    maps::grid::Vector2d resolution = traversabilityGrid.getResolution();
    maps::grid::Vector3d localFrame = traversabilityGrid.translation();

    boost::intrusive_ptr<envire::TraversabilityGrid> envTravGrid(new envire::TraversabilityGrid(numCells.x(), numCells.y(),
                                                                                                resolution.x(), resolution.y(),
                                                                                                localFrame.x(), localFrame.y()));

    std::vector<maps::grid::TraversabilityClass> travClasses = traversabilityGrid.getTraversabilityClasses();
    envire::TraversabilityClass travClass;

    for (size_t travClassId = 0; travClassId < travClasses.size(); ++travClassId)
    {
        travClass = envire::TraversabilityClass(travClasses.at(travClassId).getDrivability());
        envTravGrid->setTraversabilityClass(travClassId, travClass);
    }

    uint8_t travClassId;
    double probability;

    uint8_t* trav_ptr = envTravGrid->getGridData(envTravGrid->TRAVERSABILITY).origin();
    uint8_t* prob_ptr = envTravGrid->getGridData(envTravGrid->PROBABILITY).origin();
    maps::grid::TraversabilityGrid::const_iterator it = traversabilityGrid.begin();
    for (; it < traversabilityGrid.end(); ++it, ++trav_ptr, ++prob_ptr)
    {
        travClassId = it->getTraversabilityClassId();
        probability = it->getProbability();
        *trav_ptr = travClassId;
        *prob_ptr = probability;
    }

    return envTravGrid;
}

} // namespace motion_planning_libraries
