#include "SbplEnvXYTHETA.hpp"

#include <exception>

#include <sbpl/headers.h>
#include <sbpl/sbpl_exception.h>

namespace motion_planning_libraries
{

// PUBLIC
SbplEnvXYTHETA::SbplEnvXYTHETA(Config config) : Sbpl(config), 
        mSBPLScaleX(0), mSBPLScaleY(0) {
    LOG_DEBUG("SbplEnvXYTHETA constructor");
}

bool SbplEnvXYTHETA::initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data) { 
    
    LOG_DEBUG("SBPLEnvXYTHETA initialize");
    
    size_t grid_width = trav_grid->getCellSizeX();
    size_t grid_height = trav_grid->getCellSizeY();
    double scale_x = trav_grid->getScaleX();
    double scale_y = trav_grid->getScaleY();
    
    mSBPLScaleX = scale_x;
    mSBPLScaleY = scale_y;
       
    // Create and fill SBPL environment.
    mpSBPLEnv = boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT>(
            new EnvironmentNAVXYTHETAMLEVLAT());


    // Use the sbpl-env file if path is given.
    if(!mConfig.mSBPLEnvFile.empty()) {
        LOG_INFO("Load SBPL environment '%s'", mConfig.mSBPLEnvFile.c_str());
        
        try {
            mpSBPLEnv->InitializeEnv(mConfig.mSBPLEnvFile.c_str());
        
            // Request loaded cellsize / scale for environment SBPL_XYTHETA.
            boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
                    boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
            mSBPLScaleX = mSBPLScaleY = env_xytheta->GetEnvNavConfig()->cellsize_m;
        } catch (SBPL_Exception& e) {
            LOG_ERROR("EnvironmentNAVXYTHETAMLEVLAT could not be initialized");
            return false;
        }
        
    // Create an sbpl-environment.
    } else {
        createSBPLMap(trav_grid, grid_data);
       
        LOG_INFO("Create SBPL EnvironmentNAVXYTHETAMLEVLAT environment");
        boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
                boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
        try {
            // SBPL does not allow the definition of forward AND backward velocity.
            double speed = fabs(mConfig.mRobotForwardVelocity);
            
            if(mConfig.mRobotForwardVelocity != mConfig.mRobotBackwardVelocity) {
                LOG_WARN("SBPL does not use backward velocity, only forward velocity will be used");
            }
            
            if(speed == 0.0) {
                LOG_WARN("Speed of zero is not allowed, abort");
                return false;
            }
           
            if(mConfig.mRobotRotationalVelocity == 0.0) {
                LOG_WARN("Rotational velocity of zero is not allowed, abort");
                return false;
            }
            // SBPL: time in sec for a 45° turn
            double time_to_turn_45_degree = fabs((M_PI / 4.0) / mConfig.mRobotRotationalVelocity);
            double robot_width = 2 * std::max(mConfig.mFootprintRadiusMinMax.first, mConfig.mFootprintRadiusMinMax.second);
            double robot_length = robot_width;
            
            if(mConfig.mFootprintRadiusMinMax.first != mConfig.mFootprintRadiusMinMax.second) {
                LOG_WARN("SBPL does not support variable footprints, max radius will be used");
            }
            
            std::vector<sbpl_2Dpt_t> fp_vec = createFootprint(robot_width, robot_length);
            
            env_xytheta->InitializeEnv(grid_width, grid_height, 
                mpSBPLMapData, // initial map
                0,0,0, //mStartGrid.position.x(), mStartGrid.position.y(), mStartGrid.getYaw(), 
                0,0,0, //mGoalGrid.position.x(), mGoalGrid.position.y(), mGoalGrid.getYaw(),
                0.1, 0.1, 0.1, // tolerance x,y,yaw, ignored
                fp_vec, 
                scale_x,  // Size of a cell in meter => in SBPL cells have to be quadrats
                speed, 
                time_to_turn_45_degree, 
                SBPL_MAX_COST, // cost threshold
                mConfig.mSBPLMotionPrimitivesFile.c_str()); // motion primitives file
        } catch (SBPL_Exception& e) {
            LOG_ERROR("EnvironmentNAVXYTHETAMLEVLAT could not be initialized");
            return false;
        }
        std::cout << "SBPL Init done!" << std::endl;
    }
 
    // Create planner.
    //mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new ARAPlanner(mpSBPLEnv.get(), mConfig.mSBPLForwardSearch));
    mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new ADPlanner(mpSBPLEnv.get(), mConfig.mSBPLForwardSearch));
    mpSBPLPlanner->set_search_mode(mConfig.mSearchUntilFirstSolution); 
    
    // If available use the start and goal defined in the SBPL environment.
    if(!mConfig.mSBPLEnvFile.empty()) {
        LOG_INFO("Use start/goal of the loaded SBPL environment");
        MDPConfig mdp_cfg;
        
        if (! mpSBPLEnv->InitializeMDPCfg(&mdp_cfg)) {
            LOG_ERROR("InitializeMDPCfg failed, start and goal id cannot be requested yet");
            return false;
        }
           
        std::cout << "SBPL: About to set start and goal, startid" << mdp_cfg.startstateid << std::endl;
        if (mpSBPLPlanner->set_start(mdp_cfg.startstateid) == 0) {
            LOG_ERROR("Failed to set start state");
            return false;
        }

        if (mpSBPLPlanner->set_goal(mdp_cfg.goalstateid) == 0) {
            LOG_ERROR("Failed to set goal state");
            return false;
        }
    }

    return true;
}

bool SbplEnvXYTHETA::setStartGoal(struct State start_state, struct State goal_state) {
    
    LOG_DEBUG("SBPL setStartGoal");
    
    int start_id = 0;
    int goal_id = 0;
    
    // Strat/goal have to be defined in meters!
    boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
            boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
    
    double start_x = start_state.getPose().position[0];
    double start_y = start_state.getPose().position[1];
    double start_yaw = start_state.getPose().getYaw();
    double goal_x = goal_state.getPose().position[0];
    double goal_y = goal_state.getPose().position[1];
    double goal_yaw = goal_state.getPose().getYaw();
    
    start_id = env_xytheta->SetStart(start_x * mSBPLScaleX, start_y * mSBPLScaleY, start_yaw);
    goal_id = env_xytheta->SetGoal(goal_x * mSBPLScaleX, goal_y * mSBPLScaleY, goal_yaw);

    if (mpSBPLPlanner->set_start(start_id) == 0) {
        LOG_ERROR("Failed to set start state");
        return false;
    }

    if (mpSBPLPlanner->set_goal(goal_id) == 0) {
        LOG_ERROR("Failed to set goal state");
        return false;
    }
      
    return true;
}
    
bool SbplEnvXYTHETA::solve(double time) {
    return Sbpl::solve(time);
}
    
bool SbplEnvXYTHETA::fillPath(std::vector<struct State>& path) {
    
    LOG_DEBUG("SBPL fillPath");
    
    base::samples::RigidBodyState rbs;
    
    int x = 0, y = 0, theta = 0; // In SBPL theta is an integer as well.
    
    // Fill path with the found solution.
    std::vector<int>::iterator it = mSBPLWaypointIDs.begin();
    for(; it != mSBPLWaypointIDs.end(); it++) {
       
        boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
                boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
        env_xytheta->GetCoordFromState(*it, x, y, theta);

        rbs.position = base::Vector3d(x,y,0);
        rbs.orientation =  Eigen::AngleAxis<double>(theta, base::Vector3d(0,0,1));
        path.push_back(rbs);
    }
    
    return true;
}

} // namespace motion_planning_libraries
