#ifndef _GLOBAL_PATH_PLANNER_SBPL_HPP_
#define _GLOBAL_PATH_PLANNER_SBPL_HPP_

#include <vector>

#include <boost/shared_ptr.hpp>

#include <sbpl/utils/utils.h>

#include <global_path_planner/GlobalPathPlanner.hpp>

class EnvironmentNAVXYTHETAMLEVLAT;

namespace global_path_planner
{

/**
 * Finds the path with minimal cost from start to goal using a traversability map. 
 * The orientation of the robot cannot be regarded, because (it seems as if)
 * controll problems cannot be optimized in OMPL. 
 */
class Sbpl : public GlobalPathPlanner
{  
 public: 
    Sbpl();
 
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
    boost::shared_ptr<EnvironmentNAVXYTHETAMLEVLAT> mpEnv;
    
    std::vector<sbpl_2Dpt_t> createFootprint(double robot_width, double robot_height);
};
    
} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_SBPL_HPP_
