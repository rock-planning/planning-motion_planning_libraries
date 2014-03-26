#ifndef _GLOBAL_PATH_PLANNER_SBPL_HPP_
#define _GLOBAL_PATH_PLANNER_SBPL_HPP_

#include <global_path_planner/GlobalPathPlanner.hpp>

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
    virtual bool initialize();
    
    /**
     * 
     */
    virtual bool solve(double time);
        
    /**
     * 
     */
    virtual bool fillPath(std::vector<base::samples::RigidBodyState>& path);
};
    
} // end namespace global_path_planner

#endif // _GLOBAL_PATH_PLANNER_SBPL_HPP_
