#include "SbplEnvXYTHETA.hpp"

#include <exception>

#include <sbpl/headers.h>
#include <sbpl/sbpl_exception.h>

namespace motion_planning_libraries
{

// PUBLIC
SbplEnvXYTHETA::SbplEnvXYTHETA(Config config) : Sbpl(config), 
        mSBPLScaleX(0), mSBPLScaleY(0), mPrims(NULL) {
    LOG_DEBUG("SbplEnvXYTHETA constructor");
}

bool SbplEnvXYTHETA::initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data) { 
    
    LOG_DEBUG("SBPLEnvXYTHETA initialize");
    
    mPrims = NULL;
    
    size_t grid_width = trav_grid->getCellSizeX();
    size_t grid_height = trav_grid->getCellSizeY();
    double scale_x = trav_grid->getScaleX();
    double scale_y = trav_grid->getScaleY();
    
    mSBPLScaleX = scale_x;
    mSBPLScaleY = scale_y;
    
    assert (scale_x == scale_y);
       
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
        } catch (SBPL_Exception* e) {
            LOG_ERROR("EnvironmentNAVXYTHETAMLEVLAT could not be initialized using %s (%s)", mConfig.mSBPLEnvFile.c_str(), e->what());
            return false;
        }        
    // Create an sbpl-environment.
    } else {
        
        std::string mprim_file = mConfig.mSBPLMotionPrimitivesFile;
        
        // If no mprim file is specified it will be generated using the available
        // speed informations.
        // TODO test
        if(mprim_file.empty()) {
            LOG_INFO("No sbpl mprim file specified, it will be generated");
            assert(scale_x == scale_y);
            mprim_file = "sbpl_motion_primitives.mprim";
            MotionPrimitivesConfig mprim_config(mConfig, grid_width, grid_height, scale_x);
            mPrims = new struct SbplMotionPrimitives(mprim_config);
            mPrims->createPrimitives();
            mPrims->storeToFile(mprim_file);
        }
        
        createSBPLMap(trav_grid, grid_data);
       
        LOG_INFO("Create SBPL EnvironmentNAVXYTHETAMLEVLAT environment");
        boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
                boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
        try {
            // SBPL does not allow the definition of forward AND backward velocity.
            double speed = fabs(mConfig.mSpeeds.mSpeedForward);
            
            if(mConfig.mSpeeds.mSpeedForward != mConfig.mSpeeds.mSpeedBackward) {
                LOG_WARN("SBPL does not use backward velocity, only forward velocity will be used");
            }
            
            if(speed == 0.0) {
                LOG_WARN("Speed of zero is not allowed, abort");
                return false;
            }
          
            // The average turning speed (forward-turn and point-turn) is used for the
            // cost calculation.
            double turning_speed = (mConfig.mSpeeds.mSpeedTurn + mConfig.mSpeeds.mSpeedPointTurn) / 2.0;
            if(turning_speed == 0) {
                LOG_WARN("Rotational velocity of zero is not allowed, abort");
                return false;
            }
            
            // SBPL: time in sec for a 45° turn  
            double time_to_turn_45_degree = fabs((M_PI / 4.0) / turning_speed);
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
                mprim_file.c_str()); // motion primitives file
        } catch (SBPL_Exception* e) {
            LOG_ERROR("EnvironmentNAVXYTHETAMLEVLAT could not be created using the motion primitive file %s (%s)", 
                    mprim_file.c_str(),
                    e->what());
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
    
    // Start/goal have to be defined in meters (grid_local).
    boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
            boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
    
    double start_x = start_state.getPose().position[0] * mSBPLScaleX;
    double start_y = start_state.getPose().position[1] * mSBPLScaleY;
    double start_yaw = start_state.getPose().getYaw();
    double goal_x = goal_state.getPose().position[0] * mSBPLScaleX;
    double goal_y = goal_state.getPose().position[1] * mSBPLScaleY;
    double goal_yaw = goal_state.getPose().getYaw();
    
    start_id = env_xytheta->SetStart(start_x, start_y, start_yaw);
    goal_id = env_xytheta->SetGoal(goal_x, goal_y, goal_yaw);

    if (mpSBPLPlanner->set_start(start_id) == 0) {
        LOG_ERROR("Failed to set start state");
        return false;
    }

    if (mpSBPLPlanner->set_goal(goal_id) == 0) {
        LOG_ERROR("Failed to set goal state");
        return false;
    }
    
    // Used to add it at the end of the intermediate path.
    mGoalLocal[0] = goal_x;
    mGoalLocal[1] = goal_y;
    mGoalLocal[2] = goal_yaw;
      
    return true;
}
    
bool SbplEnvXYTHETA::solve(double time) {
    return Sbpl::solve(time);
}
    
bool SbplEnvXYTHETA::fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid) {
    
    LOG_DEBUG("SBPL fillPath");
    
    base::samples::RigidBodyState rbs;
    
    // GetCoordFromState returns the discrete position and angle!!
    int x_discrete = 0, y_discrete = 0, theta_discrete = 0; 
    
    std::vector<int> path_ids;
    std::vector<sbpl_xy_theta_pt_t> path_xytheta;
    
    // Just fill the path with the motion primitive poses (in grid coordinates).
    std::vector<int>::iterator it = mSBPLWaypointIDs.begin();
    for(; it != mSBPLWaypointIDs.end(); it++) {
        // Fill path with the found solution.
        boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
                boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
        env_xytheta->GetCoordFromState(*it, x_discrete, y_discrete, theta_discrete);

        // MotionPlanningLibraries expects grid coordinates, but a real angle in rad,
        // not the discrete one! (0-15), adapts to OMPL angles with (-PI,PI]
        rbs.position = base::Vector3d((double)x_discrete, (double)y_discrete, 0);
        double theta_rad = DiscTheta2Cont(theta_discrete, NAVXYTHETALAT_THETADIRS);
        // Converts [0,2*M_PI) to (-PI,PI].
        if(theta_rad > 180) {
            theta_rad -= 2*M_PI;
        }
        rbs.orientation =  Eigen::AngleAxis<double>(theta_rad, base::Vector3d(0,0,1));
        
        if(mConfig.mUseIntermediatePoints) {
            // Just store the path ids to request the intermediate poses.
            path_ids.push_back(*it);
        } else {
            path.push_back(rbs);
        }
    }
    
    boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> env_xytheta =
        boost::dynamic_pointer_cast<EnvironmentNAVXYTHETAMLEVLAT>(mpSBPLEnv);
    
    // Use ConvertStateIDPathintoXYThetaPath to create the path in the local grid frame
    // using the intermediate points. In this case 'pos_defined_in_local_grid' has to be set to true.
    // The intermediate poses already contain start and goal and their orientations
    // are already adapted to (-PI,PI].
    if(mConfig.mUseIntermediatePoints) {         
        // The returned path is already transformed to grid-local.   
        // TODO Does not support all states, e.g. got 16 ids but just supports 14 waypoints.
        env_xytheta->ConvertStateIDPathintoXYThetaPath(&path_ids, &path_xytheta);
        pos_defined_in_local_grid = true;
        
        sbpl_xy_theta_pt_t xyt_m_rad;
        for(unsigned int i=0; i<path_xytheta.size(); ++i) {
            xyt_m_rad = path_xytheta[i];
            xyt_m_rad.x -= mSBPLScaleX / 2.0;
            xyt_m_rad.y -= mSBPLScaleY / 2.0;
            rbs.position = base::Vector3d(xyt_m_rad.x, xyt_m_rad.y, 0);
            rbs.orientation =  Eigen::AngleAxis<double>(xyt_m_rad.theta, base::Vector3d(0,0,1));
            path.push_back(rbs);
        }
        // The goal pose is not part of the received path, so we add it manually.
        rbs.position = base::Vector3d(mGoalLocal[0], mGoalLocal[1], 0.0);
        rbs.orientation =  Eigen::AngleAxis<double>(mGoalLocal[2], base::Vector3d(0,0,1));
        path.push_back(rbs);
    }
    
    // Request and assign prim id and speed values.
    // The prim ids and the speed values are just assigned
    // to the fisrt starting state of each primitive.
    std::vector<EnvNAVXYTHETALATAction_t> action_list;
    env_xytheta->GetActionsFromStateIDPath(&path_ids, &action_list);
    std::vector<EnvNAVXYTHETALATAction_t>::iterator it_action = action_list.begin();
    std::vector<struct State>::iterator it_state = path.begin();
    unsigned int prim_id = 0;
    
    // Counter to increase the state iterator.
    int inc_state = 1;
    if(mConfig.mUseIntermediatePoints) {
        inc_state = mPrims->mConfig.mNumIntermediatePoses;
    }
    
    // Runs through all states and assigns the prim id and its speed values
    // to the first state of each primitive.
    for(;it_state != path.end() && it_action != action_list.end(); it_state += inc_state, it_action++) {
        prim_id = it_action->aind;
        Speeds speed;
        if(!mPrims->getSpeeds(prim_id, speed)) {
            LOG_WARN("Speed informations are not available for prim id %d", prim_id);
        }
        
        it_state->mSBPLPrimId = prim_id;
        it_state->mSpeeds = speed;
    }
    
    return true;
}

} // namespace motion_planning_libraries
