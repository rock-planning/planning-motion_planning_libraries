#include "Sbpl.hpp"

#include <exception>

#include <sbpl/headers.h>
#include <sbpl/sbpl_exception.h>

namespace global_path_planner
{

// PUBLIC
Sbpl::Sbpl(ConfigurationSBPL config_sbpl) : GlobalPathPlanner(),
        mConfigSBPL(config_sbpl),
        mpSBPLMapData(NULL),
        mSBPLNumElementsMap(0) {
}

// PROTECTED
bool Sbpl::initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            boost::shared_ptr<TravData> grid_data) { 
    
    LOG_DEBUG("SBPL initialize");
       
    // Create and fill SBPL environment.
    mpSBPLEnv = boost::shared_ptr<EnvironmentNAV2D>(new EnvironmentNAV2D());
    try {
        // Use the sbpl-env file if path is given.
        if(!mConfigSBPL.mSBPLEnvFile.empty()) {
            LOG_INFO("Load SBPL environment '%s'", mConfigSBPL.mSBPLEnvFile.c_str());
            mpSBPLEnv->InitializeEnv(mConfigSBPL.mSBPLEnvFile.c_str());
        // Create an sbpl-environment.
        } else {
            createSBPLMap(mpTravData);
            mpSBPLEnv->InitializeEnv(grid_width, grid_height, mpSBPLMapData, SBPL_MAX_COST);
        }
    } catch (SBPL_Exception& e) {
        LOG_ERROR("SBPL environment '%s' could not be loaded", 
                mConfigSBPL.mSBPLEnvFile.c_str());
        mpSBPLEnv.reset();
        return false;
    } catch(...) {
        LOG_ERROR("Any error");
    }
    
    /*
    mpEnv = boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT>(new EnvironmentNAVXYTHETAMLEVLAT());    
    // TODO For SBPL the cells have to be quadrats.
    mpEnv->InitializeEnv(grid_width, grid_height, 
            NULL, // initial map, use it 
            mStartGrid.position.x(), mStartGrid.position.y(), mStartGrid.getYaw(), 
            mGoalGrid.position.x(), mGoalGrid.position.y(), mGoalGrid.getYaw(),
            0.1, 0.1, 0.1, // tolerance x,y,yaw, ignored
            createFootprint(mConfig.mRobotWidth, mConfig.mRobotHeight), scale_x,
            mConfig.mRobotForwardVelocity, mConfig.mRobotRotationalVelocity,
            12, // cost threshold
            conf_sbpl->mMotionPrimitivesFile); // motion primitives file
    */
    
    // Create planner.
    mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new ARAPlanner(mpSBPLEnv.get(), mConfigSBPL.mSBPLForwardSearch));
    mpSBPLPlanner->set_search_mode(mConfigSBPL.mSBPLSearchUntilFirstSolution); 
    
    // If available use the start and goal defined in the SBPL environment.
    if(!mConfigSBPL.mSBPLEnvFile.empty()) {
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

bool Sbpl::setStartGoal(int start_x, int start_y, double start_yaw, 
    int goal_x, int goal_y, double goal_yaw) {
    
    LOG_DEBUG("SBPL setStartGoal");
    
    int start_id = mpSBPLEnv->SetStart(start_x, start_y/*, start_yaw*/);
    int goal_id = mpSBPLEnv->SetGoal(goal_x, goal_y/*, goal_yaw*/);
    
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

bool Sbpl::solve(double time) {
    
    LOG_DEBUG("SBPL solve()");
    
    mSBPLWaypointIDs.clear();
    if(mpSBPLPlanner->replan(time, &mSBPLWaypointIDs)) {
        LOG_INFO("Found solution contains %d waypoints", mSBPLWaypointIDs.size());
    } else {
        return false;
    }
    return true;
}
    
bool Sbpl::fillPath(std::vector<base::samples::RigidBodyState>& path) {
    
    LOG_DEBUG("SBPL fillPath");
    
    mPathInGrid.clear();
    base::samples::RigidBodyState rbs;
    
    int x = 0, y = 0;
    
    // Fill global grid path with the found solution.
    // env: nav2D
    std::vector<int>::iterator it = mSBPLWaypointIDs.begin();
    for(; it != mSBPLWaypointIDs.end(); it++) {
        mpSBPLEnv->GetCoordFromState(*it, x, y);
        rbs.position = base::Vector3d(x,y,0);
        mPathInGrid.push_back(rbs);
    }
    
    return true;
}

// PRIVATE
void Sbpl::createSBPLMap(boost::shared_ptr<TravData> trav_data) {
    
    LOG_DEBUG("SBPL createSBPLMap");
    
    // Create a new sbpl map if it has not been created yet or the number 
    // of elements have changed.
    if(mpSBPLMapData != NULL) {
        if(trav_data->num_elements() != mSBPLNumElementsMap) {
            free(mpSBPLMapData);
            mpSBPLMapData = NULL;
            
            mpSBPLMapData = (unsigned char*)calloc(trav_data->num_elements(), sizeof(unsigned char));
            mSBPLNumElementsMap = trav_data->num_elements();
        } else {
            memset(mpSBPLMapData, 0, mSBPLNumElementsMap);
        }
    } else {
        mpSBPLMapData = (unsigned char*)calloc(trav_data->num_elements(), sizeof(unsigned char));
        mSBPLNumElementsMap = trav_data->num_elements();
    }
      
    // Adds the costs to the sbpl map using the driveability value of the traversability classes.
    double driveability = 0.0;
    unsigned char sbpl_cost = 0;
    unsigned char* sbpl_map_p = mpSBPLMapData;
    uint8_t* stop_p = trav_data->origin() + trav_data->num_elements();
    
    //std::cout << "ORIGIN " << std::hex << trav_data->origin() << " ENDE " << std::hex << trav_data->origin() + trav_data->num_elements() << std::endl;
    for(uint8_t* p = trav_data->origin(); p < stop_p; p++, sbpl_map_p++) {
        uint8_t class_value = *p;
        driveability = (mpTravGrid->getTraversabilityClass(class_value)).getDrivability();
        sbpl_cost = SBPL_MAX_COST - (int)(driveability * (double)SBPL_MAX_COST + 0.5);
        *sbpl_map_p = sbpl_cost;
    }
}

std::vector<sbpl_2Dpt_t> Sbpl::createFootprint(double robot_width, double robot_height) {

    LOG_DEBUG("SBPL createFootprint");
    
    double half_robot_width = robot_width / 2.0;
    double half_robot_height = robot_height / 2.0;
    std::vector<sbpl_2Dpt_t> footprint;
    
    sbpl_2Dpt_t pt_m;
    pt_m.x = -half_robot_width;
    pt_m.y = -half_robot_height;
    footprint.push_back(pt_m);
    pt_m.x = half_robot_width;
    pt_m.y = -half_robot_height;
    footprint.push_back(pt_m);
    pt_m.x = half_robot_width;
    pt_m.y = half_robot_height;
    footprint.push_back(pt_m);
    pt_m.x = -half_robot_width;
    pt_m.y = half_robot_height;
    footprint.push_back(pt_m);
    
    return footprint;
}

} // namespace global_path_planner
