#ifndef _GLOBAL_PATH_PLANNER_SBPL_HPP_
#define _GLOBAL_PATH_PLANNER_SBPL_HPP_

#include <vector>

#include <boost/shared_ptr.hpp>

#include <base/Logging.hpp>
#include <global_path_planner/GlobalPathPlanner.hpp>

#include <sbpl/utils/utils.h>
#include <sbpl/config.h> // here #define DEBUG 0, causes a lot of trouble
#include <sbpl/discrete_space_information/environment_nav2D.h>
#include <sbpl/planners/planner.h>
#undef DEBUG

namespace global_path_planner
{

struct ConfigurationSBPL : public ConfigurationBase {
    ConfigurationSBPL() : 
            mSBPLEnvFile(),
            mSBPLMotionPrimitivesFile(), 
            mSBPLForwardSearch(true), 
            mSBPLSearchUntilFirstSolution(false) {
    }
    
    std::string mSBPLEnvFile;
    std::string mSBPLMotionPrimitivesFile;
    bool mSBPLForwardSearch;
    bool mSBPLSearchUntilFirstSolution;
};
    
/**
 * Finds the path with minimal cost from start to goal using a traversability map. 
 * The orientation of the robot cannot be regarded, because (it seems as if)
 * controll problems cannot be optimized in OMPL. 
 */
class Sbpl : public GlobalPathPlanner
{  
private:
    ConfigurationSBPL mConfigSBPL;
    
    boost::shared_ptr<EnvironmentNAV2D> mpSBPLEnv;
    boost::shared_ptr<SBPLPlanner> mpSBPLPlanner;
    std::vector<int> mSBPLWaypointIDs;
    
 public: 
    Sbpl(ConfigurationSBPL config_sbpl = ConfigurationSBPL());
    void mpEnv();
 
 protected:
    /**
     * 
     */
    virtual bool initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            boost::shared_ptr<TravData> grid_data);
    
    /**
     * 
     */
    virtual bool setStartGoal(int start_x, int start_y, double start_yaw, 
            int goal_x, int goal_y, double goal_yaw);
    
    /**
     * 
     */
    virtual bool solve(double time);
        
    /**
     * 
     */
    virtual bool fillPath(std::vector<base::samples::RigidBodyState>& path);
    
 private:    
    std::vector<sbpl_2Dpt_t> createFootprint(double robot_width, double robot_height);
};
    
} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_SBPL_HPP_
