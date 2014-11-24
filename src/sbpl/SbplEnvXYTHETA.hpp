#ifndef _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_
#define _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_

#include "Sbpl.hpp"
#include "SbplMotionPrimitives.hpp"

namespace motion_planning_libraries
{
    
class SbplEnvXYTHETA : public Sbpl
{          
 protected:
    // Required to set start/goal in ENV_XYTHETA 
    // (grid coordinates have to be converted back to meters) 
    double mSBPLScaleX, mSBPLScaleY; 
    struct SbplMotionPrimitives* mPrims;

 public: 
    SbplEnvXYTHETA(Config config = Config());
 
    /**
     * 
     */
    virtual bool initialize(envire::TraversabilityGrid* trav_grid,
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
    
    inline struct SbplMotionPrimitives* getMotionPrimitives() {
        return mPrims;
    }
};
    
} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_SBPL_XYTHETA_HPP_
