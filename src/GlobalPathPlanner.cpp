#include "GlobalPathPlanner.hpp"

#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/core/Environment.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/config.h>

#include <global_path_planner/validators/TravMapValidator.hpp>
#include <global_path_planner/validators/TurningValidator.hpp>
#include <global_path_planner/objectives/PathClearance.hpp>
#include <global_path_planner/objectives/TravGridObjective.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace global_path_planner
{

// PUBLIC
GlobalPathPlanner::GlobalPathPlanner() : mpTravGrid(NULL), 
        mStartGrid(), 
        mGoalGrid(), 
        mPath(),
        mReplanningRequired(true) {
        
    mStartGrid.invalidatePosition();
    mStartGrid.invalidateOrientation();        
    mGoalGrid.invalidatePosition();
    mGoalGrid.invalidateOrientation();  
    mMaxSpeed = 0.5; // m/s
    mMaxTurningSpeed = M_PI / 5; // rad/s
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
        mReplanningRequired = true;
        return true;
    }
}

bool GlobalPathPlanner::setStartWorld(base::samples::RigidBodyState& start_world) {
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
            mReplanningRequired = true;
        }
    } else {
        mReplanningRequired = true;
    }
    
    mStartGrid = start_grid_new;
    mStartWorld = start_world;    
    
    return true;
}

base::samples::RigidBodyState GlobalPathPlanner::getStartGrid() const {
    return mStartGrid;
}

bool GlobalPathPlanner::setGoalWorld(base::samples::RigidBodyState& goal_world) {
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
            mReplanningRequired = true;
        }
    } else {
        mReplanningRequired = true;
    }
    
    mGoalGrid = goal_grid_new;
    mGoalWorld = goal_world;    
    
    return true;
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
    
    if(mReplanningRequired) {
        LOG_INFO("Replanning initiated");
        mPath.clear();
        
        bool ret = createOMPLObjects();
        if(!ret) {
            LOG_WARN("OMPL objects could not be created, planning can not be executed"); 
            return false;
        }
        // mpOptimizingPlanner->clear() + mpOptimizingPlanner->setup() seems to work not so well
    } else {
        LOG_INFO("Try to improve the current trajectory");
    }
    
    setStartGoalOMPL(mStartGrid, mGoalGrid);
    
    mpProblemDefinition->print();
 
    // Start planning.
    // Setup: Once during creation or each time because of the changed start/goal?
    ob::PlannerStatus solved = mpOptimizingPlanner->solve(max_time);
    
    if (solved)
    {
        // get the goal representation from the problem definition 
        // (not the same as the goal state) and inquire about the found path
        ob::PathPtr path = mpProblemDefinition->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        path->print(std::cout);
        
        mPath.clear();
        mPath = convertPath(path);
        mReplanningRequired = false;
    }
    
    return true;
}

std::vector<base::Waypoint> GlobalPathPlanner::getPath() {
    std::vector<base::Waypoint> path;
    std::vector<base::samples::RigidBodyState>::iterator it = mPath.begin();
    for(;it != mPath.end(); it++) {
        base::Waypoint waypoint;
        waypoint.position = it->position;
        waypoint.heading = it->getYaw();
        path.push_back(waypoint);
    }
    return path;
}

base::Trajectory GlobalPathPlanner::getTrajectory(double speed) {
    base::Trajectory trajectory;
    trajectory.speed = speed;
    
    std::vector<base::Vector3d> path;
    std::vector<double> parameters;
    std::vector<base::geometry::SplineBase::CoordinateType> coord_types;
    std::vector<base::samples::RigidBodyState>::iterator it = mPath.begin();
    for(;it != mPath.end(); it++) {
        path.push_back(it->position);
        //parameters.push_back(it->getYaw()); // TODO: What are the parameters for?
        coord_types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
    }
    trajectory.spline.interpolate(path, parameters, coord_types);
    return trajectory;
}

bool GlobalPathPlanner::world2grid(envire::TraversabilityGrid const* trav,
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

bool GlobalPathPlanner::grid2world(envire::TraversabilityGrid const* trav,
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

std::vector<base::Waypoint> GlobalPathPlanner::getSamples() {
    if(mpTravMapValidator == NULL) {
        std::cout << "mpTravMapValidator is empty" << std::endl; 
        return std::vector<base::Waypoint>();
    } else {
        //return mpTravMapValidator->getSamples();
    }    
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
    // Set the bounds for the RealVectorStateSpace (x,y), 
    // not required/possible for orientation (default bound?)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0); // for all dimensions.
    bounds.setHigh(0,mpTravGrid->getCellSizeX());
    bounds.setHigh(1,mpTravGrid->getCellSizeY());
    mpStateSpace->as<ob::SE2StateSpace>()->setBounds(bounds);
    mpStateSpace->setLongestValidSegmentFraction(1/(double)mpTravGrid->getCellSizeX()); // TODO What does LongestValidSegmentFraction mean? Optimal value here?
    std::cout << "GET LONGEST VALID SEGMENT FRACTION: " << mpStateSpace->getLongestValidSegmentFraction() << std::endl;
    std::cout << "GET MAX EXTENT: " << mpStateSpace->getMaximumExtent() << std::endl;   
      
    // Create the control  space.
//    mpControlSpace = ompl::control::ControlSpacePtr(
//            new ompl::control::RealVectorControlSpace(mpStateSpace));
    
    
    // Create a space information object using the traversability map validator.
    mpSpaceInformation = ob::SpaceInformationPtr(new ob::SpaceInformation(mpStateSpace));
    mpTravMapValidator = ompl::base::StateValidityCheckerPtr(new TravMapValidator(mpSpaceInformation, mpTravGrid));
    mpSpaceInformation->setStateValidityChecker(mpTravMapValidator);
    double max_grid_sec = mMaxSpeed / mpTravGrid->getScaleX();
    mpTurningValidator = ompl::base::MotionValidatorPtr(new TurningValidator(mpSpaceInformation, max_grid_sec, mMaxTurningSpeed));
    mpSpaceInformation->setMotionValidator(mpTurningValidator);
    mpSpaceInformation->setup();
            
    // Create problem definition.        
    mpProblemDefinition = ob::ProblemDefinitionPtr(
            new ob::ProblemDefinition(mpSpaceInformation));
    //mpPathLengthOptimization = ompl::base::OptimizationObjectivePtr(
    //        new ob::PathLengthOptimizationObjective(mpSpaceInformation));
    mpProblemDefinition->setOptimizationObjective(getBalancedObjective(mpSpaceInformation));
        
    // Construct our optimizing planner using the RRTstar algorithm.
    // TODO: RTTstar will be deleted by the planner object?
    mpOptimizingPlanner = ob::PlannerPtr(new og::PRMstar(mpSpaceInformation));
    // Set the problem instance for our planner to solve
    mpOptimizingPlanner->setProblemDefinition(mpProblemDefinition);
    mpOptimizingPlanner->setup();
    
    return true;
}

std::vector<base::samples::RigidBodyState> GlobalPathPlanner::convertPath(
        ob::PathPtr path_ompl) {
    std::vector<base::samples::RigidBodyState> path_rbs;

    // Downcast from Path to PathGeometric is valid.
    std::vector<ompl::base::State*> path_states = 
            boost::static_pointer_cast<og::PathGeometric>(path_ompl)->getStates();
    std::vector<ompl::base::State*>::iterator it = path_states.begin();

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
        path_rbs.push_back(world_pose);
        counter++;
    }
    LOG_INFO("Trajectory contains %d states", counter); 
    return path_rbs;  
}

void GlobalPathPlanner::setStartGoalOMPL(base::samples::RigidBodyState const& start,
        base::samples::RigidBodyState const& goal) {
        
    // Set start and goal in OMPL.
    ob::ScopedState<> start_ompl(mpStateSpace);
    // start[0] = 1; ?
    start_ompl->as<ob::SE2StateSpace::StateType>()->setX(start.position.x());
    start_ompl->as<ob::SE2StateSpace::StateType>()->setY(start.position.y());
    start_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(start.getYaw());
    
    ob::ScopedState<> goal_ompl(mpStateSpace);
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setX(goal.position.x());
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setY(goal.position.y());
    goal_ompl->as<ob::SE2StateSpace::StateType>()->setYaw(goal.getYaw());
    
    LOG_INFO("Planning from (x,y,yaw) (%4.2f, %4.2f, %4.2f) to (%4.2f, %4.2f, %4.2f)", 
            start.position.x(), start.position.y(), start.getYaw(),
            goal.position.x(), goal.position.y(), goal.getYaw());
    
    mpProblemDefinition->setStartAndGoalStates(start_ompl, goal_ompl);
    // Stop if the found solution is nearly a straight line.
    double dist_start_goal = (start.position - goal.position).norm() * 1.1;
    LOG_INFO("Set cost threshold to %4.2f", dist_start_goal);
    mpPathLengthOptimization->setCostThreshold(ob::Cost(dist_start_goal));
}

ompl::base::OptimizationObjectivePtr GlobalPathPlanner::getBalancedObjective(
        const ompl::base::SpaceInformationPtr& si) {
    /*    
    mpPathLengthOptimization = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    mpPathClearanceOptimization = ob::OptimizationObjectivePtr(new PathClearance(si));
    return mpPathLengthOptimization + mpPathClearanceOptimization;
    */
    
    mpPathLengthOptimization = ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    mpPathClearanceOptimization = ob::OptimizationObjectivePtr(new PathClearance(si));
    mpMaxMinClearance = ob::OptimizationObjectivePtr(new ob::MaximizeMinClearanceObjective(si));
    mpTravGridOjective = ob::OptimizationObjectivePtr(new TravGridObjective(si, mpTravGrid));
    
    //ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
    //ob::OptimizationObjectivePtr clearObj(new PathClearance(si));
    ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
    opt->addObjective(mpPathLengthOptimization, 1.0);
    //opt->addObjective(mpPathClearanceOptimization, 1.0);
    //opt->addObjective(mpMaxMinClearance, 0.0);
    //opt->addObjective(mpTravGridOjective, 1.0);
    mpMultiOptimization = ompl::base::OptimizationObjectivePtr(opt);

    return mpMultiOptimization;
}

} // namespace global_path_planner
