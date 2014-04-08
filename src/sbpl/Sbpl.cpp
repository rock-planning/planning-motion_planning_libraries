#include "Sbpl.hpp"

#include <exception>

#include <sbpl/discrete_space_information/environment_navxythetamlevlat.h>

namespace global_path_planner
{

// PUBLIC
Sbpl::Sbpl() : GlobalPathPlanner() {
}

// PROTECTED
bool Sbpl::initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            boost::shared_ptr<TravData> grid_data) { 
    
    // mpConfig should point to a sbpl config file.
    struct ConfigurationSBPL* conf_sbpl = std::dynamic_cast<struct ConfigurationSBPL*>(mpConfig);
    
    mpEnv = boost::shared_ptr<EnvironmentNAV2D>(new EnvironmentNAV2D());
    
    // Contains start and goal as well
    if(!mpEnv->InitializeEnv(conf_sbpl->mEnvFile)) {
        LOG_ERROR("SBPL environment could not be initialized");
        return false;
    }
    
    MDPConfig mdp_cfg;
    if (! mpEnv->InitializeMDPCfg(&mdp_cfg)) {
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
    
    mpPlanner = boost::shared_ptr<SBPLPlanner>(new ARAPlanner*(mpEnv, conf_sbpl->mForwardSearch));
    mpPlanner->set_search_mode(conf_sbpl->mSearchUntilFirstSolution); 
    
    if (mpPlanner->set_start(mdp_cfg.startstateid) == 0) {
        LOG_ERROR("Failed to set start state");
        return false;
    }

    if (mpPlanner->set_goal(mdp_cfg.goalstateid) == 0) {
        LOG_ERROR("Failed to set goal state");
        return false;
    }
    
    return true;
}

bool Sbpl::setStartGoal(int start_x, int start_y, double start_yaw, 
    int goal_x, int goal_y, double goal_yaw) {
    
    int start_id = mpEnv->SetStart(start_x, start_y, start_yaw);
    int goal_id = mpEnv->SetGoal(goal_x, goal_y, goal_yaw);
    
    if (mpPlanner->set_start(start_id) == 0) {
        LOG_ERROR("Failed to set start state");
        return false;
    }

    if (mpPlanner->set_goal(goal_id) == 0) {
        LOG_ERROR("Failed to set goal state");
        return false;
    }
    
    return true;
}

bool Sbpl::solve(double time) {
    
    mWaypointIDs.clear();
    if(mpPlanner->replan(time, &mWaypointIDs)) {
        LOG_INFO("Solution found containing %d waypoints", mWaypointIDs.size());
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
    std::vector<int>::iterator it = mWaypointIDs.begin();
    for(; it != mWaypointIDs.end(); it++) {
        mpEnv->GetCoordFromState(int stateID, int& x, int& y);
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
