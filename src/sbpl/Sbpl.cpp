#include "Sbpl.hpp"

#include <exception>

#include <sbpl/headers.h>
#include <sbpl/sbpl_exception.h>

namespace global_path_planner
{

// PUBLIC
Sbpl::Sbpl(ConfigurationSBPL config_sbpl) : GlobalPathPlanner(),
        mConfigSBPL(config_sbpl) {
}

// PROTECTED
bool Sbpl::initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            boost::shared_ptr<TravData> grid_data) { 
       
    mpSBPLEnv = boost::shared_ptr<EnvironmentNAV2D>(new EnvironmentNAV2D());
 
    // Contains start and goal as well
    LOG_INFO("Load SBPL environment '%s'", mConfigSBPL.mSBPLEnvFile.c_str());
    try {
        mpSBPLEnv->InitializeEnv(mConfigSBPL.mSBPLEnvFile.c_str());
    } catch (SBPL_Exception& e) {
        LOG_ERROR("SBPL environment '%s' could not be loaded", 
                mConfigSBPL.mSBPLEnvFile.c_str());
        //mpSBPLEnv.reset();
        return false;
    }
    
    MDPConfig mdp_cfg;
    if (! mpSBPLEnv->InitializeMDPCfg(&mdp_cfg)) {
        LOG_ERROR("InitializeMDPCfg failed, start and goal id cannot be requested yet");
        return false;
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
    
    mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new ARAPlanner(mpSBPLEnv.get(), mConfigSBPL.mSBPLForwardSearch));
    mpSBPLPlanner->set_search_mode(mConfigSBPL.mSBPLSearchUntilFirstSolution); 
    
    if (mpSBPLPlanner->set_start(mdp_cfg.startstateid) == 0) {
        LOG_ERROR("Failed to set start state");
        return false;
    }

    if (mpSBPLPlanner->set_goal(mdp_cfg.goalstateid) == 0) {
        LOG_ERROR("Failed to set goal state");
        return false;
    }

    return true;
}

bool Sbpl::setStartGoal(int start_x, int start_y, double start_yaw, 
    int goal_x, int goal_y, double goal_yaw) {
    
#if 0
    
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
    
#endif
    
    return true;
}

bool Sbpl::solve(double time) {
    
    mSBPLWaypointIDs.clear();
    if(mpSBPLPlanner->replan(time, &mSBPLWaypointIDs)) {
        LOG_INFO("Solution found containing %d waypoints", mSBPLWaypointIDs.size());
    } else {
        LOG_WARN("Solution not found");
        return false;
    }
    
    return true;
}
    
bool Sbpl::fillPath(std::vector<base::samples::RigidBodyState>& path) {
    
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
std::vector<sbpl_2Dpt_t> Sbpl::createFootprint(double robot_width, double robot_height) {
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
