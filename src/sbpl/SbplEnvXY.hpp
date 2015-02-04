#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_ENVXY_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_ENVXY_HPP_

#include "Sbpl.hpp"

namespace motion_planning_libraries
{
    
class SbplEnvXY : public Sbpl
{      
   
 public: 
    SbplEnvXY(Config config = Config());
 
    /**
     * 
     */
    virtual bool initialize(envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data);
    
    /**
     * 
     */
    virtual bool setStartGoal(struct State start_state, struct State goal_state);
      
    /**
     * 
     */       
    virtual bool solve(double time);   
        
    /**
     * 
     */
    virtual bool fillPath(std::vector<struct State>& path, bool& pos_defined_in_local_grid);       
};
    
} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_SBPL_ENVXY_HPP_
