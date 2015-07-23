#include "SbplEnvXY.hpp"

#include <sbpl/headers.h>

namespace motion_planning_libraries
{

// PUBLIC
SbplEnvXY::SbplEnvXY(Config config) : Sbpl(config) {
    LOG_DEBUG("SBPLEnvXY constructor");
}

bool SbplEnvXY::initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data) { 
    
    LOG_DEBUG("SBPLEnvXY initialize");
    
    size_t grid_width = trav_grid->getCellSizeX();
    size_t grid_height = trav_grid->getCellSizeY();

    mpSBPLEnv = boost::shared_ptr<EnvironmentNAV2D>(new EnvironmentNAV2D());
         
    try {
        // Use the sbpl-env file if path is given.
        if(!mConfig.mSBPLEnvFile.empty()) {
            LOG_INFO("Load SBPL environment '%s'", mConfig.mSBPLEnvFile.c_str());
            mpSBPLEnv->InitializeEnv(mConfig.mSBPLEnvFile.c_str());
        // Create an sbpl-environment.
        } else {
            createSBPLMap(trav_grid, grid_data);

            LOG_INFO("Create SBPL EnvironmentNAV2D environment");
            boost::shared_ptr<EnvironmentNAV2D> env_xy =
                    boost::dynamic_pointer_cast<EnvironmentNAV2D>(mpSBPLEnv);
            env_xy->InitializeEnv(grid_width, grid_height, mpSBPLMapData, SBPL_MAX_COST + 1);
        }
    } catch (SBPL_Exception* e) {
        LOG_ERROR("SBPL environment '%s' could not be loaded (%s)", 
                mConfig.mSBPLEnvFile.c_str(),
                e->what());
        mpSBPLEnv.reset();
        return false;
    } 
      
    // Create planner.
    switch(mConfig.mPlanner) {
        case UNDEFINED_PLANNER: {
        }
        case ANYTIME_DSTAR: {
            mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new ADPlanner(mpSBPLEnv.get(), 
                    mConfig.mSBPLForwardSearch));
            break;
        }
        case ANYTIME_NONPARAMETRIC_ASTAR: {
            mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new anaPlanner(mpSBPLEnv.get(), 
                    mConfig.mSBPLForwardSearch));
            break;
        }
        case ANYTIME_ASTAR: {
            mpSBPLPlanner = boost::shared_ptr<SBPLPlanner>(new ARAPlanner(mpSBPLEnv.get(), 
                    mConfig.mSBPLForwardSearch));
        }
        default: {
            LOG_ERROR("Planner %d is not available for this environment", (int)mConfig.mPlanner);
            return false;
        }
    }
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

bool SbplEnvXY::setStartGoal(struct State start_state, struct State goal_state) {
    
    LOG_DEBUG("SBPLEnvXY setStartGoal");
    
    int start_id = 0;
    int goal_id = 0;
    
    boost::shared_ptr<EnvironmentNAV2D> env_xy =
            boost::dynamic_pointer_cast<EnvironmentNAV2D>(mpSBPLEnv);
            
    start_id = env_xy->SetStart(start_state.getPose().position[0], 
            start_state.getPose().position[1]);
    goal_id = env_xy->SetGoal(goal_state.getPose().position[0], 
            goal_state.getPose().position[1]);

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

bool SbplEnvXY::solve(double time) {
    return Sbpl::solve(time);
}
    
bool SbplEnvXY::fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid) {
    
    LOG_DEBUG("SBPL fillPath");
    
    base::samples::RigidBodyState rbs;
    
    int x = 0, y = 0, theta = 0; // In SBPL theta is an integer as well.
    
    // Fill path with the found solution.
    std::vector<int>::iterator it = mSBPLWaypointIDs.begin();
    for(; it != mSBPLWaypointIDs.end(); it++) {
       
        boost::shared_ptr<EnvironmentNAV2D> env_xy =
                boost::dynamic_pointer_cast<EnvironmentNAV2D>(mpSBPLEnv);
        env_xy->GetCoordFromState(*it, x, y);

        rbs.position = base::Vector3d(x,y,0);
        rbs.orientation =  Eigen::AngleAxis<double>(theta, base::Vector3d(0,0,1));
        path.push_back(rbs);
    }
    
    return true;
}

enum MplErrors SbplEnvXY::isStartGoalValid() {
     LOG_INFO("Check discrete start (%d, %d) and goal position (%d, %d) for validity",
            mStartGrid[0], mStartGrid[1], mGoalGrid[0], mGoalGrid[1]);
     
    boost::shared_ptr<EnvironmentNAV2D> env_xy =
            boost::dynamic_pointer_cast<EnvironmentNAV2D>(mpSBPLEnv);
        
    int err = (int)MPL_ERR_NONE;
    
    if(!env_xy->IsObstacle(mStartGrid[0], mStartGrid[1])) {
        err += (int)MPL_ERR_START_ON_OBSTACLE;
    }
    
    if(!env_xy->IsObstacle(mGoalGrid[0], mGoalGrid[1])) {
        err += (int)MPL_ERR_GOAL_ON_OBSTACLE;
    }
    
    return (enum MplErrors)err;
}

} // namespace motion_planning_libraries
