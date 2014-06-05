#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_

#include "Sbpl.hpp"

namespace motion_planning_libraries
{
    
class SbplEnvXYTHETA : public Sbpl
{          
 protected:
    // Required to set start/goal in ENV_XYTHETA 
    // (grid coordinates have to be converted back to meters) 
    double mSBPLScaleX, mSBPLScaleY; 

 public: 
    SbplEnvXYTHETA(Config config = Config());
 
    /**
     * 
     */
    virtual bool initialize(size_t grid_width, size_t grid_height, 
            double scale_x, double scale_y, 
            envire::TraversabilityGrid* trav_grid,
            boost::shared_ptr<TravData> grid_data);
    
    /**
     * 
     */
    virtual bool setStartGoal(struct State start_state, struct State goal_state);
        
    virtual bool solve(double time);    
        
    /**
     * 
     */
    virtual bool fillPath(std::vector<struct State>& path);
};
    
} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_
